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
 * Button connections -hardware wiring:
 * PF4 = Driver OPEN      pull-up,   active-low  (pressed = 0)
 * PE0 = Driver CLOSE     pull-down, active-high (pressed = 1)
 * PE1 = Security OPEN    pull-down, active-high (pressed = 1)
 * PB0 = Security CLOSE   pull-down, active-high (pressed = 1)
 * PB1 = Open limit       pull-down, active-high (pressed = 1)
 * PD0 = Closed limit     pull-down, active-high (pressed = 1)
 * PD1 = Obstacle         pull-down, active-high (pressed = 1)
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

/* Clock-gate masks for Ports B, D, E, F */
#define RCGCGPIO_B             (1U << 1)
#define RCGCGPIO_D             (1U << 3)
#define RCGCGPIO_E             (1U << 4)
#define RCGCGPIO_F             (1U << 5)
#define RCGCGPIO_USED          (RCGCGPIO_B | RCGCGPIO_D | RCGCGPIO_E | RCGCGPIO_F)

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

#define DEBOUNCE_COUNT         2
#define INPUT_DELAY_MS         10
#define HOLD_TIME_MS           300
#define REVERSE_TIME_MS        500
#define STATUS_DELAY_MS        1000

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
    BTN_ACTIVE_RELEASED,
    BTN_CONFLICT,
    BTN_HOLD_TIMEOUT
} ButtonEvents;

typedef enum {
    RED_LED,
    GREEN_LED,
    OFF_LED
} LEDSEvents;

typedef enum {
    MODE_STOPPED,
    MODE_WAITING_RELEASE,
    MODE_MANUAL,
    MODE_AUTO
} GateMode;

typedef enum {
    OWNER_NONE,
    OWNER_DRIVER,
    OWNER_SECURITY
} CommandOwner;

static QueueHandle_t buttonsQueue;
static QueueHandle_t ledsQueue;
static QueueHandle_t obstacleQueue;

static SemaphoreHandle_t openLimitSemaphore;
static SemaphoreHandle_t closedLimitSemaphore;
static SemaphoreHandle_t stateMutex;

static states currentState = IDLE_CLOSED;
static GateMode currentMode = MODE_STOPPED;
static CommandOwner currentOwner = OWNER_NONE;

static void vInputTask(void *pvParameters);
static void vGateCTRLTask(void *pvParameters);
static void vLEDCTRLTask(void *pvParameters);
static void vSafetyTask(void *pvParameters);
static void vStatusTask(void *pvParameters);

static void GPIO_Init(void);
static uint8_t readButtons(void);
static void LED_Set(uint32_t ledMask);
static void applyLedEvent(LEDSEvents event);
static ButtonEvents getPanelEvent(uint8_t buttons);
static bool isMoveButton(ButtonEvents event);
static bool isDriverEvent(ButtonEvents event);
static bool isSecurityEvent(ButtonEvents event);
static CommandOwner eventOwner(ButtonEvents event);

static void sendButtonEvent(ButtonEvents event, BaseType_t urgent);
static void setGateStatus(states newState, GateMode newMode, CommandOwner newOwner);
static void setMode(GateMode newMode);
static states getState(void);
static void getGateStatus(states *state, GateMode *mode, CommandOwner *owner);
static void sendLedEvent(LEDSEvents event);
static void startOpening(CommandOwner owner);
static void startClosing(CommandOwner owner);
static void stopGate(states stopState);
static const char *stateName(states state);
static const char *modeName(GateMode mode);
static const char *ownerName(CommandOwner owner);

int main(void)
{
    GPIO_Init();

    buttonsQueue = xQueueCreate(20, sizeof(ButtonEvents));
    ledsQueue = xQueueCreate(1, sizeof(LEDSEvents));
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
    /* Enable clocks for ports B, D, E, F and wait until all are ready. */
    SYSCTL_RCGCGPIO_R |= RCGCGPIO_USED;
    while ((SYSCTL_PRGPIO_R & RCGCGPIO_USED) != RCGCGPIO_USED) {
    }

    /* ---------- Port B: PB0/PB1 inputs, pull-down, active-high ---------- */
    GPIO_PORTB_AMSEL_R &= ~(BTN_PB0 | BTN_PB1);
    GPIO_PORTB_PCTL_R  &= ~0x000000FFU;
    GPIO_PORTB_AFSEL_R &= ~(BTN_PB0 | BTN_PB1);
    GPIO_PORTB_DIR_R   &= ~(BTN_PB0 | BTN_PB1);
    GPIO_PORTB_PUR_R   &= ~(BTN_PB0 | BTN_PB1);
    GPIO_PORTB_PDR_R   |=  (BTN_PB0 | BTN_PB1);
    GPIO_PORTB_DEN_R   |=  (BTN_PB0 | BTN_PB1);

    /* ---------- Port D: PD0/PD1 inputs, pull-down, active-high ---------- */
    GPIO_PORTD_AMSEL_R &= ~(BTN_PD0 | BTN_PD1);
    GPIO_PORTD_PCTL_R  &= ~0x000000FFU;
    GPIO_PORTD_AFSEL_R &= ~(BTN_PD0 | BTN_PD1);
    GPIO_PORTD_DIR_R   &= ~(BTN_PD0 | BTN_PD1);
    GPIO_PORTD_PUR_R   &= ~(BTN_PD0 | BTN_PD1);
    GPIO_PORTD_PDR_R   |=  (BTN_PD0 | BTN_PD1);
    GPIO_PORTD_DEN_R   |=  (BTN_PD0 | BTN_PD1);

    /* ---------- Port E: PE0/PE1 inputs, pull-down, active-high ---------- */
    GPIO_PORTE_AMSEL_R &= ~(BTN_PE0 | BTN_PE1);
    GPIO_PORTE_PCTL_R  &= ~0x000000FFU;
    GPIO_PORTE_AFSEL_R &= ~(BTN_PE0 | BTN_PE1);
    GPIO_PORTE_DIR_R   &= ~(BTN_PE0 | BTN_PE1);
    GPIO_PORTE_PUR_R   &= ~(BTN_PE0 | BTN_PE1);
    GPIO_PORTE_PDR_R   |=  (BTN_PE0 | BTN_PE1);
    GPIO_PORTE_DEN_R   |=  (BTN_PE0 | BTN_PE1);

    /* ---------- Port F: PF1/PF3 LEDs and PF4 input, pull-up, active-low ---------- */
    GPIO_PORTF_AMSEL_R &= ~(BOTH_LEDS_MASK | BTN_PF4);
    GPIO_PORTF_PCTL_R  &= ~0x000FFFF0U;   /* PF1..PF4 as GPIO */
    GPIO_PORTF_AFSEL_R &= ~(BOTH_LEDS_MASK | BTN_PF4);

    GPIO_PORTF_DIR_R   |=  BOTH_LEDS_MASK;
    GPIO_PORTF_DIR_R   &= ~BTN_PF4;

    GPIO_PORTF_PDR_R   &= ~BTN_PF4;
    GPIO_PORTF_PUR_R   |=  BTN_PF4;
    GPIO_PORTF_DEN_R   |=  (BOTH_LEDS_MASK | BTN_PF4);

    LED_Set(0);
}

static uint8_t readButtons(void)
{
    uint8_t buttons = 0;

    
    if ((GPIO_PORTF_DATA_R & BTN_PF4) == 0) {
        buttons |= DRIVER_OPEN_MASK;
    }
    if ((GPIO_PORTE_DATA_R & BTN_PE0) != 0) {
        buttons |= DRIVER_CLOSE_MASK;
    }
    if ((GPIO_PORTE_DATA_R & BTN_PE1) != 0) {
        buttons |= SECURITY_OPEN_MASK;
    }
    if ((GPIO_PORTB_DATA_R & BTN_PB0) != 0) {
        buttons |= SECURITY_CLOSE_MASK;
    }
    if ((GPIO_PORTB_DATA_R & BTN_PB1) != 0) {
        buttons |= OPEN_LIMIT_MASK;
    }
    if ((GPIO_PORTD_DATA_R & BTN_PD0) != 0) {
        buttons |= CLOSED_LIMIT_MASK;
    }
    if ((GPIO_PORTD_DATA_R & BTN_PD1) != 0) {
        buttons |= OBSTACLE_MASK;
    }

    return buttons;
}

static void LED_Set(uint32_t ledMask)
{
    GPIO_PORTF_DATA_R = (GPIO_PORTF_DATA_R & ~BOTH_LEDS_MASK) | (ledMask & BOTH_LEDS_MASK);
}

static void applyLedEvent(LEDSEvents event)
{
    switch (event) {
        case GREEN_LED:
            LED_Set(GREEN_LED_MASK);
            break;

        case RED_LED:
            LED_Set(RED_LED_MASK);
            break;

        case OFF_LED:
        default:
            LED_Set(0);
            break;
    }
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

static bool isDriverEvent(ButtonEvents event)
{
    return event == BTN_DRIVER_OPEN || event == BTN_DRIVER_CLOSE;
}

static bool isSecurityEvent(ButtonEvents event)
{
    return event == BTN_SECURITY_OPEN || event == BTN_SECURITY_CLOSE;
}

static CommandOwner eventOwner(ButtonEvents event)
{
    if (isSecurityEvent(event)) {
        return OWNER_SECURITY;
    }
    if (isDriverEvent(event)) {
        return OWNER_DRIVER;
    }
    return OWNER_NONE;
}


static void sendButtonEvent(ButtonEvents event, BaseType_t urgent)
{
    if (urgent == pdTRUE) {
        xQueueSendToFront(buttonsQueue, &event, portMAX_DELAY);
    } else {
        xQueueSendToBack(buttonsQueue, &event, portMAX_DELAY);
    }
}

static void setGateStatus(states newState, GateMode newMode, CommandOwner newOwner)
{
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    currentState = newState;
    currentMode = newMode;
    currentOwner = newOwner;
    xSemaphoreGive(stateMutex);
}

static void setMode(GateMode newMode)
{
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    currentMode = newMode;
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

static void getGateStatus(states *state, GateMode *mode, CommandOwner *owner)
{
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    *state = currentState;
    *mode = currentMode;
    *owner = currentOwner;
    xSemaphoreGive(stateMutex);
}

static void sendLedEvent(LEDSEvents event)
{
    applyLedEvent(event);
    xQueueOverwrite(ledsQueue, &event);
}

static void startOpening(CommandOwner owner)
{
    setGateStatus(OPENING, MODE_WAITING_RELEASE, owner);
    sendLedEvent(GREEN_LED);
    vPrintString("Gate opening\n");
}

static void startClosing(CommandOwner owner)
{
    setGateStatus(CLOSING, MODE_WAITING_RELEASE, owner);
    sendLedEvent(RED_LED);
    vPrintString("Gate closing\n");
}

static void stopGate(states stopState)
{
    setGateStatus(stopState, MODE_STOPPED, OWNER_NONE);
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

static const char *modeName(GateMode mode)
{
    switch (mode) {
        case MODE_STOPPED:
            return "STOPPED";
        case MODE_WAITING_RELEASE:
            return "WAITING_RELEASE";
        case MODE_MANUAL:
            return "MANUAL";
        case MODE_AUTO:
            return "AUTO";
        default:
            return "UNKNOWN";
    }
}

static const char *ownerName(CommandOwner owner)
{
    switch (owner) {
        case OWNER_DRIVER:
            return "DRIVER";
        case OWNER_SECURITY:
            return "SECURITY";
        case OWNER_NONE:
        default:
            return "NONE";
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
    ButtonEvents previousPanelEvent;
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
                previousPanelEvent = panelEvent;
                panelEvent = getPanelEvent(stableButtons);

                if (panelEvent != previousPanelEvent) {
                    if (isSecurityEvent(previousPanelEvent) && isDriverEvent(panelEvent)) {
                        sendButtonEvent(BTN_ACTIVE_RELEASED, pdFALSE);
                    }

                    sendButtonEvent(panelEvent, pdFALSE);
                    pressStartTime = xTaskGetTickCount();
                    holdEventSent = false;
                }
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
    GateMode gateMode = MODE_STOPPED;
    CommandOwner activeOwner = OWNER_NONE;
    CommandOwner newOwner;
    bool ignoreCommandsUntilRelease = false;
    bool driverLockedBySecurityUntilRelease = false;

    (void)pvParameters;

    while (1) {
        xQueueReceive(buttonsQueue, &event, portMAX_DELAY);

        if (event == BTN_RELEASED) {
            ignoreCommandsUntilRelease = false;
            driverLockedBySecurityUntilRelease = false;

            if (gateMode == MODE_WAITING_RELEASE) {
                gateMode = MODE_AUTO;
                setMode(MODE_AUTO);
                vPrintString("Auto mode\n");
            } else if (gateMode == MODE_MANUAL) {
                stopGate(STOPPED_MIDWAY);
                gateMode = MODE_STOPPED;
                activeButton = BTN_RELEASED;
                activeOwner = OWNER_NONE;
            }

            continue;
        }

        if (event == BTN_ACTIVE_RELEASED) {
            if (gateMode == MODE_WAITING_RELEASE) {
                gateMode = MODE_AUTO;
                setMode(MODE_AUTO);
                vPrintString("Auto mode\n");
            } else if (gateMode == MODE_MANUAL) {
                stopGate(STOPPED_MIDWAY);
                gateMode = MODE_STOPPED;
                activeButton = BTN_RELEASED;
                activeOwner = OWNER_NONE;
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
                        activeOwner = OWNER_NONE;
                    }
                }
                break;

            case BTN_LIMIT_CLOSED:
                if (xSemaphoreTake(closedLimitSemaphore, 0) == pdTRUE) {
                    if (getState() == CLOSING) {
                        stopGate(IDLE_CLOSED);
                        gateMode = MODE_STOPPED;
                        activeButton = BTN_RELEASED;
                        activeOwner = OWNER_NONE;
                    }
                }
                break;

            case BTN_OBSTACLE:
                if (getState() == CLOSING) {
                    vPrintString("Obstacle detected\n");

                    ignoreCommandsUntilRelease = true;
                    gateMode = MODE_STOPPED;
                    activeButton = BTN_RELEASED;
                    activeOwner = OWNER_NONE;

                    stopGate(STOPPED_MIDWAY);
                    setGateStatus(REVERSING, MODE_AUTO, OWNER_SECURITY);
                    sendLedEvent(GREEN_LED);
                    vPrintString("Gate reversing\n");
                    vTaskDelay(pdMS_TO_TICKS(REVERSE_TIME_MS));
                    stopGate(STOPPED_MIDWAY);
                }
                break;

            case BTN_HOLD_TIMEOUT:
                if (gateMode == MODE_WAITING_RELEASE && activeButton != BTN_RELEASED) {
                    gateMode = MODE_MANUAL;
                    setMode(MODE_MANUAL);
                    vPrintString("Manual mode\n");
                }
                break;

            case BTN_CONFLICT:
                ignoreCommandsUntilRelease = true;
                stopGate(STOPPED_MIDWAY);
                gateMode = MODE_STOPPED;
                activeButton = BTN_RELEASED;
                activeOwner = OWNER_NONE;
                break;

            case BTN_DRIVER_OPEN:
            case BTN_SECURITY_OPEN:
                if (isDriverEvent(event) &&
                    (driverLockedBySecurityUntilRelease == true ||
                     (activeOwner == OWNER_SECURITY && gateMode != MODE_STOPPED))) {
                    break;
                }

                newOwner = eventOwner(event);
                if (newOwner == OWNER_SECURITY) {
                    driverLockedBySecurityUntilRelease = true;
                }

                if (getState() == IDLE_OPEN) {
                    stopGate(IDLE_OPEN);
                    gateMode = MODE_STOPPED;
                    activeButton = BTN_RELEASED;
                    activeOwner = OWNER_NONE;
                } else {
                    activeButton = event;
                    activeOwner = newOwner;
                    gateMode = MODE_WAITING_RELEASE;
                    startOpening(newOwner);
                }
                break;

            case BTN_DRIVER_CLOSE:
            case BTN_SECURITY_CLOSE:
                if (isDriverEvent(event) &&
                    (driverLockedBySecurityUntilRelease == true ||
                     (activeOwner == OWNER_SECURITY && gateMode != MODE_STOPPED))) {
                    break;
                }

                newOwner = eventOwner(event);
                if (newOwner == OWNER_SECURITY) {
                    driverLockedBySecurityUntilRelease = true;
                }

                if (getState() == IDLE_CLOSED) {
                    stopGate(IDLE_CLOSED);
                    gateMode = MODE_STOPPED;
                    activeButton = BTN_RELEASED;
                    activeOwner = OWNER_NONE;
                } else {
                    activeButton = event;
                    activeOwner = newOwner;
                    gateMode = MODE_WAITING_RELEASE;
                    startClosing(newOwner);
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
        applyLedEvent(ledEvent);
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
    states state;
    GateMode mode;
    CommandOwner owner;

    (void)pvParameters;

    while (1) {
        getGateStatus(&state, &mode, &owner);

        vPrintString("State: ");
        vPrintString(stateName(state));
        vPrintString(", Mode: ");
        vPrintString(modeName(mode));
        vPrintString(", Owner: ");
        vPrintString(ownerName(owner));
        vPrintString("\n");
        vTaskDelay(pdMS_TO_TICKS(STATUS_DELAY_MS));
    }
}
