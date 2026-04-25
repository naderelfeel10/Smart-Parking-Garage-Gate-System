#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "basic_io.h"

/*
 * Smart Parking Garage Gate System
 *
 * Button connections, active low:
 * PF4 = Driver OPEN
 * PE0 = Driver CLOSE
 * PE1 = Security OPEN
 * PB0 = Security CLOSE
 * PB1 = Open limit
 * PD0 = Closed limit
 * PD1 = Obstacle
 *
 * LEDs:
 * PF3 = Green LED, gate opening
 * PF1 = Red LED, gate closing
 */

#define BTN_PF4                (1U << 4)
#define BTN_PE0                (1U << 0)
#define BTN_PE1                (1U << 1)
#define BTN_PB0                (1U << 0)
#define BTN_PB1                (1U << 1)
#define BTN_PD0                (1U << 0)
#define BTN_PD1                (1U << 1)

#define DRIVER_OPEN_MASK       0x01
#define DRIVER_CLOSE_MASK      0x02
#define SECURITY_OPEN_MASK     0x04
#define SECURITY_CLOSE_MASK    0x08
#define OPEN_LIMIT_MASK        0x10
#define CLOSED_LIMIT_MASK      0x20
#define OBSTACLE_MASK          0x40

#define PANEL_BUTTONS_MASK     (DRIVER_OPEN_MASK | DRIVER_CLOSE_MASK | SECURITY_OPEN_MASK | SECURITY_CLOSE_MASK)
#define ALL_BUTTONS_MASK       (PANEL_BUTTONS_MASK | OPEN_LIMIT_MASK | CLOSED_LIMIT_MASK | OBSTACLE_MASK)

#define RED_LED_MASK           0x02
#define GREEN_LED_MASK         0x08
#define BOTH_LEDS_MASK         (RED_LED_MASK | GREEN_LED_MASK)

#define DEBOUNCE_COUNT         3
#define INPUT_DELAY_MS         20
#define HOLD_TIME_MS           300
#define REVERSE_TIME_MS        500
#define STATUS_DELAY_MS        1000

#define MODE_STOPPED           0
#define MODE_WAITING_RELEASE   1
#define MODE_MANUAL            2
#define MODE_AUTO              3

typedef enum {
    IDLE_OPEN,
    IDLE_CLOSED,
    OPENING,
    CLOSING,
    STOPPED_MIDWAY,
    REVERSING
} states;

typedef enum {
    BTN_DRIVER_OPEN,
    BTN_DRIVER_CLOSE,
    BTN_SECURITY_OPEN,
    BTN_SECURITY_CLOSE,
    BTN_LIMIT_OPEN,
    BTN_LIMIT_CLOSED,
    BTN_OBSTACLE,
    BTN_RELEASED,
    BTN_CONFLICT,
    BTN_HOLD_TIMEOUT
} ButtonEvents;

typedef enum {
    RED_LED,
    GREEN_LED,
    OFF_LED
} LEDSEvents;

static QueueHandle_t buttonsQueue;
static QueueHandle_t ledsQueue;
static QueueHandle_t obstacleQueue;

static SemaphoreHandle_t openLimitSemaphore;
static SemaphoreHandle_t closedLimitSemaphore;
static SemaphoreHandle_t stateMutex;

static states currentState = IDLE_CLOSED;

static void vInputTask(void *pvParameters);
static void vGateCTRLTask(void *pvParameters);
static void vLEDCTRLTask(void *pvParameters);
static void vSafetyTask(void *pvParameters);
static void vStatusTask(void *pvParameters);

static void GPIO_Init(void);
static uint8_t readButtons(void);
static ButtonEvents getPanelEvent(uint8_t buttons);
static bool isMoveButton(ButtonEvents event);
static bool isOpenButton(ButtonEvents event);
static bool isCloseButton(ButtonEvents event);
static void sendButtonEvent(ButtonEvents event, BaseType_t urgent);
static void setState(states newState);
static states getState(void);
static void sendLedEvent(LEDSEvents event);
static void startOpening(void);
static void startClosing(void);
static void stopGate(states stopState);
static const char *stateName(states state);

int main(void)
{
    GPIO_Init();

    buttonsQueue = xQueueCreate(20, sizeof(ButtonEvents));
    ledsQueue = xQueueCreate(8, sizeof(LEDSEvents));
    obstacleQueue = xQueueCreate(4, sizeof(ButtonEvents));

    openLimitSemaphore = xSemaphoreCreateBinary();
    closedLimitSemaphore = xSemaphoreCreateBinary();
    stateMutex = xSemaphoreCreateMutex();

    if (buttonsQueue == NULL || ledsQueue == NULL || obstacleQueue == NULL ||
        openLimitSemaphore == NULL || closedLimitSemaphore == NULL || stateMutex == NULL) {
        vPrintString("RTOS objects failed\n");
        while (1) {
        }
    }

    if (xTaskCreate(vInputTask, "Input", 200, NULL, 4, NULL) != pdPASS ||
        xTaskCreate(vGateCTRLTask, "Gate", 220, NULL, 3, NULL) != pdPASS ||
        xTaskCreate(vLEDCTRLTask, "LED", 160, NULL, 3, NULL) != pdPASS ||
        xTaskCreate(vSafetyTask, "Safety", 180, NULL, 5, NULL) != pdPASS ||
        xTaskCreate(vStatusTask, "Status", 160, NULL, 2, NULL) != pdPASS) {
        vPrintString("Task creation failed\n");
        while (1) {
        }
    }

    vTaskStartScheduler();

    while (1) {
    }
}

static void GPIO_Init(void)
{
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;  /* Port B clock */
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;  /* Port D clock */
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;  /* Port E clock */
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;  /* Port F clock */

    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R1) == 0) {
    }
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R3) == 0) {
    }
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R4) == 0) {
    }
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0) {
    }

    GPIO_PORTB_DIR_R &= ~(BTN_PB0 | BTN_PB1);
    GPIO_PORTB_DEN_R |= (BTN_PB0 | BTN_PB1);
    GPIO_PORTB_PUR_R |= (BTN_PB0 | BTN_PB1);
    GPIO_PORTB_AFSEL_R &= ~(BTN_PB0 | BTN_PB1);
    GPIO_PORTB_AMSEL_R &= ~(BTN_PB0 | BTN_PB1);

    GPIO_PORTD_DIR_R &= ~(BTN_PD0 | BTN_PD1);
    GPIO_PORTD_DEN_R |= (BTN_PD0 | BTN_PD1);
    GPIO_PORTD_PUR_R |= (BTN_PD0 | BTN_PD1);
    GPIO_PORTD_AFSEL_R &= ~(BTN_PD0 | BTN_PD1);
    GPIO_PORTD_AMSEL_R &= ~(BTN_PD0 | BTN_PD1);

    GPIO_PORTE_DIR_R &= ~(BTN_PE0 | BTN_PE1);
    GPIO_PORTE_DEN_R |= (BTN_PE0 | BTN_PE1);
    GPIO_PORTE_PUR_R |= (BTN_PE0 | BTN_PE1);
    GPIO_PORTE_AFSEL_R &= ~(BTN_PE0 | BTN_PE1);
    GPIO_PORTE_AMSEL_R &= ~(BTN_PE0 | BTN_PE1);

    GPIO_PORTF_DIR_R |= BOTH_LEDS_MASK;
    GPIO_PORTF_DIR_R &= ~BTN_PF4;
    GPIO_PORTF_DEN_R |= (BOTH_LEDS_MASK | BTN_PF4);
    GPIO_PORTF_PUR_R |= BTN_PF4;
    GPIO_PORTF_AFSEL_R &= ~(BOTH_LEDS_MASK | BTN_PF4);
    GPIO_PORTF_AMSEL_R &= ~(BOTH_LEDS_MASK | BTN_PF4);
    GPIO_PORTF_DATA_R &= ~BOTH_LEDS_MASK;
}

static uint8_t readButtons(void)
{
    uint8_t buttons = 0;

    if ((GPIO_PORTF_DATA_R & BTN_PF4) == 0) {
        buttons |= DRIVER_OPEN_MASK;
    }
    if ((GPIO_PORTE_DATA_R & BTN_PE0) == 0) {
        buttons |= DRIVER_CLOSE_MASK;
    }
    if ((GPIO_PORTE_DATA_R & BTN_PE1) == 0) {
        buttons |= SECURITY_OPEN_MASK;
    }
    if ((GPIO_PORTB_DATA_R & BTN_PB0) == 0) {
        buttons |= SECURITY_CLOSE_MASK;
    }
    if ((GPIO_PORTB_DATA_R & BTN_PB1) == 0) {
        buttons |= OPEN_LIMIT_MASK;
    }
    if ((GPIO_PORTD_DATA_R & BTN_PD0) == 0) {
        buttons |= CLOSED_LIMIT_MASK;
    }
    if ((GPIO_PORTD_DATA_R & BTN_PD1) == 0) {
        buttons |= OBSTACLE_MASK;
    }

    return buttons;
}

static ButtonEvents getPanelEvent(uint8_t buttons)
{
    /* Security panel has priority over driver panel. */
    if ((buttons & SECURITY_OPEN_MASK) != 0 && (buttons & SECURITY_CLOSE_MASK) != 0) {
        return BTN_CONFLICT;
    }
    if ((buttons & SECURITY_OPEN_MASK) != 0) {
        return BTN_SECURITY_OPEN;
    }
    if ((buttons & SECURITY_CLOSE_MASK) != 0) {
        return BTN_SECURITY_CLOSE;
    }

    if ((buttons & DRIVER_OPEN_MASK) != 0 && (buttons & DRIVER_CLOSE_MASK) != 0) {
        return BTN_CONFLICT;
    }
    if ((buttons & DRIVER_OPEN_MASK) != 0) {
        return BTN_DRIVER_OPEN;
    }
    if ((buttons & DRIVER_CLOSE_MASK) != 0) {
        return BTN_DRIVER_CLOSE;
    }

    return BTN_RELEASED;
}

static bool isMoveButton(ButtonEvents event)
{
    return event == BTN_DRIVER_OPEN || event == BTN_DRIVER_CLOSE ||
           event == BTN_SECURITY_OPEN || event == BTN_SECURITY_CLOSE;
}

static bool isOpenButton(ButtonEvents event)
{
    return event == BTN_DRIVER_OPEN || event == BTN_SECURITY_OPEN;
}

static bool isCloseButton(ButtonEvents event)
{
    return event == BTN_DRIVER_CLOSE || event == BTN_SECURITY_CLOSE;
}

static void sendButtonEvent(ButtonEvents event, BaseType_t urgent)
{
    if (urgent == pdTRUE) {
        xQueueSendToFront(buttonsQueue, &event, portMAX_DELAY);
    } else {
        xQueueSendToBack(buttonsQueue, &event, portMAX_DELAY);
    }
}

static void setState(states newState)
{
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    currentState = newState;
    xSemaphoreGive(stateMutex);
}

static states getState(void)
{
    states state;

    xSemaphoreTake(stateMutex, portMAX_DELAY);
    state = currentState;
    xSemaphoreGive(stateMutex);

    return state;
}

static void sendLedEvent(LEDSEvents event)
{
    xQueueSendToBack(ledsQueue, &event, portMAX_DELAY);
}

static void startOpening(void)
{
    setState(OPENING);
    sendLedEvent(GREEN_LED);
    vPrintString("Gate opening\n");
}

static void startClosing(void)
{
    setState(CLOSING);
    sendLedEvent(RED_LED);
    vPrintString("Gate closing\n");
}

static void stopGate(states stopState)
{
    setState(stopState);
    sendLedEvent(OFF_LED);
    vPrintString("Gate stopped\n");
}

static const char *stateName(states state)
{
    switch (state) {
        case IDLE_OPEN:
            return "IDLE_OPEN";
        case IDLE_CLOSED:
            return "IDLE_CLOSED";
        case OPENING:
            return "OPENING";
        case CLOSING:
            return "CLOSING";
        case STOPPED_MIDWAY:
            return "STOPPED_MIDWAY";
        case REVERSING:
            return "REVERSING";
        default:
            return "UNKNOWN";
    }
}

static void vInputTask(void *pvParameters)
{
    uint8_t stableButtons = readButtons();
    uint8_t lastSample = stableButtons;
    uint8_t sample;
    uint8_t changedButtons;
    uint8_t debounceCounter = 0;
    ButtonEvents panelEvent = getPanelEvent(stableButtons);
    ButtonEvents obstacleEvent = BTN_OBSTACLE;
    TickType_t pressStartTime = xTaskGetTickCount();
    bool holdEventSent = false;

    (void)pvParameters;

    sendButtonEvent(panelEvent, pdFALSE);

    while (1) {
        sample = readButtons();

        if (sample == lastSample) {
            if (debounceCounter < DEBOUNCE_COUNT) {
                debounceCounter++;
            }
        } else {
            lastSample = sample;
            debounceCounter = 0;
        }

        if (debounceCounter >= DEBOUNCE_COUNT && sample != stableButtons) {
            changedButtons = stableButtons ^ sample;
            stableButtons = sample;

            if ((changedButtons & PANEL_BUTTONS_MASK) != 0) {
                panelEvent = getPanelEvent(stableButtons);
                sendButtonEvent(panelEvent, pdFALSE);
                pressStartTime = xTaskGetTickCount();
                holdEventSent = false;
            }

            if ((changedButtons & OPEN_LIMIT_MASK) != 0 && (stableButtons & OPEN_LIMIT_MASK) != 0) {
                xSemaphoreGive(openLimitSemaphore);
                sendButtonEvent(BTN_LIMIT_OPEN, pdTRUE);
            }

            if ((changedButtons & CLOSED_LIMIT_MASK) != 0 && (stableButtons & CLOSED_LIMIT_MASK) != 0) {
                xSemaphoreGive(closedLimitSemaphore);
                sendButtonEvent(BTN_LIMIT_CLOSED, pdTRUE);
            }

            if ((changedButtons & OBSTACLE_MASK) != 0 && (stableButtons & OBSTACLE_MASK) != 0) {
                xQueueSendToBack(obstacleQueue, &obstacleEvent, portMAX_DELAY);
            }
        }

        panelEvent = getPanelEvent(stableButtons);

        if (isMoveButton(panelEvent) && holdEventSent == false) {
            if ((xTaskGetTickCount() - pressStartTime) >= pdMS_TO_TICKS(HOLD_TIME_MS)) {
                sendButtonEvent(BTN_HOLD_TIMEOUT, pdFALSE);
                holdEventSent = true;
            }
        }

        if (panelEvent == BTN_RELEASED || panelEvent == BTN_CONFLICT) {
            holdEventSent = false;
        }

        vTaskDelay(pdMS_TO_TICKS(INPUT_DELAY_MS));
    }
}

static void vGateCTRLTask(void *pvParameters)
{
    ButtonEvents event;
    ButtonEvents activeButton = BTN_RELEASED;
    int gateMode = MODE_STOPPED;
    bool ignoreCommandsUntilRelease = false;

    (void)pvParameters;

    while (1) {
        xQueueReceive(buttonsQueue, &event, portMAX_DELAY);

        if (event == BTN_RELEASED) {
            ignoreCommandsUntilRelease = false;

            if (gateMode == MODE_WAITING_RELEASE) {
                gateMode = MODE_AUTO;
                vPrintString("Auto mode\n");
            } else if (gateMode == MODE_MANUAL) {
                stopGate(STOPPED_MIDWAY);
                gateMode = MODE_STOPPED;
            }

            continue;
        }

        if (ignoreCommandsUntilRelease == true &&
            event != BTN_LIMIT_OPEN &&
            event != BTN_LIMIT_CLOSED &&
            event != BTN_OBSTACLE) {
            continue;
        }

        switch (event) {
            case BTN_LIMIT_OPEN:
                if (xSemaphoreTake(openLimitSemaphore, 0) == pdTRUE) {
                    if (getState() == OPENING || getState() == REVERSING) {
                        stopGate(IDLE_OPEN);
                        gateMode = MODE_STOPPED;
                        activeButton = BTN_RELEASED;
                    }
                }
                break;

            case BTN_LIMIT_CLOSED:
                if (xSemaphoreTake(closedLimitSemaphore, 0) == pdTRUE) {
                    if (getState() == CLOSING) {
                        stopGate(IDLE_CLOSED);
                        gateMode = MODE_STOPPED;
                        activeButton = BTN_RELEASED;
                    }
                }
                break;

            case BTN_OBSTACLE:
                if (getState() == CLOSING) {
                    vPrintString("Obstacle detected\n");

                    ignoreCommandsUntilRelease = true;
                    gateMode = MODE_STOPPED;
                    activeButton = BTN_RELEASED;

                    stopGate(STOPPED_MIDWAY);
                    setState(REVERSING);
                    sendLedEvent(GREEN_LED);
                    vTaskDelay(pdMS_TO_TICKS(REVERSE_TIME_MS));
                    stopGate(STOPPED_MIDWAY);
                }
                break;

            case BTN_HOLD_TIMEOUT:
                if (gateMode == MODE_WAITING_RELEASE && activeButton != BTN_RELEASED) {
                    gateMode = MODE_MANUAL;
                    vPrintString("Manual mode\n");
                }
                break;

            case BTN_CONFLICT:
                stopGate(STOPPED_MIDWAY);
                gateMode = MODE_STOPPED;
                activeButton = BTN_RELEASED;
                break;

            case BTN_DRIVER_OPEN:
            case BTN_SECURITY_OPEN:
                if (getState() == IDLE_OPEN) {
                    stopGate(IDLE_OPEN);
                    gateMode = MODE_STOPPED;
                } else {
                    activeButton = event;
                    gateMode = MODE_WAITING_RELEASE;
                    startOpening();
                }
                break;

            case BTN_DRIVER_CLOSE:
            case BTN_SECURITY_CLOSE:
                if (getState() == IDLE_CLOSED) {
                    stopGate(IDLE_CLOSED);
                    gateMode = MODE_STOPPED;
                } else {
                    activeButton = event;
                    gateMode = MODE_WAITING_RELEASE;
                    startClosing();
                }
                break;

            default:
                break;
        }
    }
}

static void vLEDCTRLTask(void *pvParameters)
{
    LEDSEvents ledEvent;

    (void)pvParameters;

    while (1) {
        xQueueReceive(ledsQueue, &ledEvent, portMAX_DELAY);

        switch (ledEvent) {
            case GREEN_LED:
                GPIO_PORTF_DATA_R &= ~BOTH_LEDS_MASK;
                GPIO_PORTF_DATA_R |= GREEN_LED_MASK;
                break;

            case RED_LED:
                GPIO_PORTF_DATA_R &= ~BOTH_LEDS_MASK;
                GPIO_PORTF_DATA_R |= RED_LED_MASK;
                break;

            case OFF_LED:
            default:
                GPIO_PORTF_DATA_R &= ~BOTH_LEDS_MASK;
                break;
        }
    }
}

static void vSafetyTask(void *pvParameters)
{
    ButtonEvents obstacleEvent;

    (void)pvParameters;

    while (1) {
        xQueueReceive(obstacleQueue, &obstacleEvent, portMAX_DELAY);

        if (obstacleEvent == BTN_OBSTACLE && getState() == CLOSING) {
            sendButtonEvent(BTN_OBSTACLE, pdTRUE);
        }
    }
}

static void vStatusTask(void *pvParameters)
{
    (void)pvParameters;

    while (1) {
        vPrintString("State: ");
        vPrintString(stateName(getState()));
        vPrintString("\n");
        vTaskDelay(pdMS_TO_TICKS(STATUS_DELAY_MS));
    }
}
