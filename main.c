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
 * Buttons are active low and connected to Port B:
 * PB0: Driver OPEN
 * PB1: Driver CLOSE
 * PB2: Security OPEN
 * PB3: Security CLOSE
 * PB4: Open limit
 * PB5: Closed limit
 * PB6: Obstacle
 *
 * LEDs:
 * PF3: Green LED for opening
 * PF1: Red LED for closing
 */

#define DRIVER_OPEN     0x01
#define DRIVER_CLOSE    0x02
#define SECURITY_OPEN   0x04
#define SECURITY_CLOSE  0x08
#define OPEN_LIMIT      0x10
#define CLOSED_LIMIT    0x20
#define OBSTACLE        0x40

#define PANEL_BUTTONS   (DRIVER_OPEN | DRIVER_CLOSE | SECURITY_OPEN | SECURITY_CLOSE)
#define ALL_BUTTONS     (PANEL_BUTTONS | OPEN_LIMIT | CLOSED_LIMIT | OBSTACLE)

#define RED_LED         0x02
#define GREEN_LED       0x08
#define BOTH_LEDS       (RED_LED | GREEN_LED)

#define DEBOUNCE_COUNT  3
#define INPUT_DELAY_MS  20
#define HOLD_TIME_MS    300
#define REVERSE_TIME_MS 500

typedef enum {
    IDLE_OPEN,
    IDLE_CLOSED,
    OPENING,
    CLOSING,
    STOPPED_MIDWAY,
    REVERSING
} GateState;

typedef enum {
    NO_DIRECTION,
    OPEN_DIRECTION,
    CLOSE_DIRECTION
} GateDirection;

typedef enum {
    NO_SOURCE,
    DRIVER_SOURCE,
    SECURITY_SOURCE
} CommandSource;

typedef enum {
    MODE_STOPPED,
    MODE_WAITING_RELEASE,
    MODE_MANUAL,
    MODE_AUTO
} ControlMode;

typedef enum {
    EVENT_BUTTONS,
    EVENT_OPEN_LIMIT,
    EVENT_CLOSED_LIMIT,
    EVENT_HOLD_TIMEOUT,
    EVENT_OBSTACLE
} EventType;

typedef enum {
    LED_OFF,
    LED_GREEN,
    LED_RED
} LedCommand;

typedef struct {
    CommandSource source;
    GateDirection direction;
    bool conflict;
} ButtonCommand;

typedef struct {
    EventType type;
    uint8_t pressedButtons;
    uint8_t changedButtons;
} GateEvent;

typedef struct {
    bool active;
    uint8_t pressedButtons;
} ObstacleEvent;

static QueueHandle_t buttonQueue;
static QueueHandle_t ledQueue;
static QueueHandle_t obstacleQueue;

static SemaphoreHandle_t openLimitSemaphore;
static SemaphoreHandle_t closedLimitSemaphore;
static SemaphoreHandle_t stateMutex;

static GateState currentState = IDLE_CLOSED;

static void inputTask(void *pvParameters);
static void gateControlTask(void *pvParameters);
static void ledTask(void *pvParameters);
static void safetyTask(void *pvParameters);
static void statusTask(void *pvParameters);

static void hardwareInit(void);
static uint8_t readButtons(void);
static ButtonCommand getButtonCommand(uint8_t buttons);
static void sendGateEvent(EventType type, uint8_t buttons, uint8_t changed, BaseType_t urgent);
static void setGateState(GateState newState);
static GateState getGateState(void);
static void sendLedCommand(LedCommand command);
static void startGate(GateDirection direction);
static void stopGate(GateState stopState);
static const char *stateToString(GateState state);

int main(void)
{
    hardwareInit();

    buttonQueue = xQueueCreate(20, sizeof(GateEvent));
    ledQueue = xQueueCreate(8, sizeof(LedCommand));
    obstacleQueue = xQueueCreate(4, sizeof(ObstacleEvent));

    openLimitSemaphore = xSemaphoreCreateBinary();
    closedLimitSemaphore = xSemaphoreCreateBinary();
    stateMutex = xSemaphoreCreateMutex();

    if (buttonQueue == NULL || ledQueue == NULL || obstacleQueue == NULL ||
        openLimitSemaphore == NULL || closedLimitSemaphore == NULL || stateMutex == NULL) {
        vPrintString("Failed to create RTOS objects\n");
        while (1) {
        }
    }

    if (xTaskCreate(inputTask, "Input", 200, NULL, 4, NULL) != pdPASS ||
        xTaskCreate(gateControlTask, "Gate", 220, NULL, 3, NULL) != pdPASS ||
        xTaskCreate(ledTask, "LED", 160, NULL, 3, NULL) != pdPASS ||
        xTaskCreate(safetyTask, "Safety", 180, NULL, 5, NULL) != pdPASS ||
        xTaskCreate(statusTask, "Status", 160, NULL, 2, NULL) != pdPASS) {
        vPrintString("Failed to create tasks\n");
        while (1) {
        }
    }

    vTaskStartScheduler();

    while (1) {
    }
}

static void hardwareInit(void)
{
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;  /* Enable Port B clock */
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;  /* Enable Port F clock */

    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R1) == 0) {
    }
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0) {
    }

    GPIO_PORTB_DIR_R &= ~ALL_BUTTONS;   /* Port B pins are inputs */
    GPIO_PORTB_DEN_R |= ALL_BUTTONS;    /* Digital enable */
    GPIO_PORTB_PUR_R |= ALL_BUTTONS;    /* Pull-up because buttons are active low */
    GPIO_PORTB_AFSEL_R &= ~ALL_BUTTONS;
    GPIO_PORTB_AMSEL_R &= ~ALL_BUTTONS;

    GPIO_PORTF_DIR_R |= BOTH_LEDS;      /* LED pins are outputs */
    GPIO_PORTF_DEN_R |= BOTH_LEDS;
    GPIO_PORTF_AFSEL_R &= ~BOTH_LEDS;
    GPIO_PORTF_AMSEL_R &= ~BOTH_LEDS;
    GPIO_PORTF_DATA_R &= ~BOTH_LEDS;
}

static uint8_t readButtons(void)
{
    return (uint8_t)((~GPIO_PORTB_DATA_R) & ALL_BUTTONS);
}

static ButtonCommand getButtonCommand(uint8_t buttons)
{
    ButtonCommand command;

    command.source = NO_SOURCE;
    command.direction = NO_DIRECTION;
    command.conflict = false;

    /* Security panel has priority over driver panel. */
    if ((buttons & (SECURITY_OPEN | SECURITY_CLOSE)) != 0) {
        command.source = SECURITY_SOURCE;

        if ((buttons & SECURITY_OPEN) != 0 && (buttons & SECURITY_CLOSE) != 0) {
            command.conflict = true;
        } else if ((buttons & SECURITY_OPEN) != 0) {
            command.direction = OPEN_DIRECTION;
        } else {
            command.direction = CLOSE_DIRECTION;
        }

        return command;
    }

    if ((buttons & (DRIVER_OPEN | DRIVER_CLOSE)) != 0) {
        command.source = DRIVER_SOURCE;

        if ((buttons & DRIVER_OPEN) != 0 && (buttons & DRIVER_CLOSE) != 0) {
            command.conflict = true;
        } else if ((buttons & DRIVER_OPEN) != 0) {
            command.direction = OPEN_DIRECTION;
        } else {
            command.direction = CLOSE_DIRECTION;
        }
    }

    return command;
}

static void sendGateEvent(EventType type, uint8_t buttons, uint8_t changed, BaseType_t urgent)
{
    GateEvent event;

    event.type = type;
    event.pressedButtons = buttons;
    event.changedButtons = changed;

    if (urgent == pdTRUE) {
        xQueueSendToFront(buttonQueue, &event, portMAX_DELAY);
    } else {
        xQueueSendToBack(buttonQueue, &event, portMAX_DELAY);
    }
}

static void setGateState(GateState newState)
{
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    currentState = newState;
    xSemaphoreGive(stateMutex);
}

static GateState getGateState(void)
{
    GateState state;

    xSemaphoreTake(stateMutex, portMAX_DELAY);
    state = currentState;
    xSemaphoreGive(stateMutex);

    return state;
}

static void sendLedCommand(LedCommand command)
{
    xQueueSendToBack(ledQueue, &command, portMAX_DELAY);
}

static void startGate(GateDirection direction)
{
    if (direction == OPEN_DIRECTION) {
        setGateState(OPENING);
        sendLedCommand(LED_GREEN);
        vPrintString("Gate opening\n");
    } else if (direction == CLOSE_DIRECTION) {
        setGateState(CLOSING);
        sendLedCommand(LED_RED);
        vPrintString("Gate closing\n");
    }
}

static void stopGate(GateState stopState)
{
    setGateState(stopState);
    sendLedCommand(LED_OFF);
    vPrintString("Gate stopped\n");
}

static const char *stateToString(GateState state)
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

static void inputTask(void *pvParameters)
{
    uint8_t stableButtons;
    uint8_t lastSample;
    uint8_t sample;
    uint8_t debounceCounter = 0;
    uint8_t changedButtons;
    TickType_t pressStartTime = 0;
    bool holdEventSent = false;
    ButtonCommand command;

    (void)pvParameters;

    stableButtons = readButtons();
    lastSample = stableButtons;
    sendGateEvent(EVENT_BUTTONS, stableButtons, stableButtons, pdFALSE);

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

            sendGateEvent(EVENT_BUTTONS, stableButtons, changedButtons, pdFALSE);

            if ((changedButtons & OPEN_LIMIT) != 0 && (stableButtons & OPEN_LIMIT) != 0) {
                xSemaphoreGive(openLimitSemaphore);
                sendGateEvent(EVENT_OPEN_LIMIT, stableButtons, changedButtons, pdTRUE);
            }

            if ((changedButtons & CLOSED_LIMIT) != 0 && (stableButtons & CLOSED_LIMIT) != 0) {
                xSemaphoreGive(closedLimitSemaphore);
                sendGateEvent(EVENT_CLOSED_LIMIT, stableButtons, changedButtons, pdTRUE);
            }

            if ((changedButtons & OBSTACLE) != 0) {
                ObstacleEvent obstacleEvent;

                obstacleEvent.active = ((stableButtons & OBSTACLE) != 0);
                obstacleEvent.pressedButtons = stableButtons;
                xQueueSendToBack(obstacleQueue, &obstacleEvent, portMAX_DELAY);
            }

            pressStartTime = xTaskGetTickCount();
            holdEventSent = false;
        }

        command = getButtonCommand(stableButtons);

        if (command.direction != NO_DIRECTION && command.conflict == false && holdEventSent == false) {
            if ((xTaskGetTickCount() - pressStartTime) >= pdMS_TO_TICKS(HOLD_TIME_MS)) {
                sendGateEvent(EVENT_HOLD_TIMEOUT, stableButtons, 0, pdFALSE);
                holdEventSent = true;
            }
        }

        if (command.direction == NO_DIRECTION || command.conflict == true) {
            holdEventSent = false;
        }

        vTaskDelay(pdMS_TO_TICKS(INPUT_DELAY_MS));
    }
}

static void gateControlTask(void *pvParameters)
{
    GateEvent event;
    ButtonCommand command;
    ControlMode mode = MODE_STOPPED;
    CommandSource activeSource = NO_SOURCE;
    GateDirection activeDirection = NO_DIRECTION;
    uint8_t blockedButtons = 0;
    uint8_t usefulButtons;

    (void)pvParameters;

    while (1) {
        xQueueReceive(buttonQueue, &event, portMAX_DELAY);

        blockedButtons &= event.pressedButtons;
        usefulButtons = event.pressedButtons & (uint8_t)(~blockedButtons);

        if (event.type == EVENT_OPEN_LIMIT) {
            if (xSemaphoreTake(openLimitSemaphore, 0) == pdTRUE) {
                if (getGateState() == OPENING || getGateState() == REVERSING) {
                    stopGate(IDLE_OPEN);
                    mode = MODE_STOPPED;
                    activeSource = NO_SOURCE;
                    activeDirection = NO_DIRECTION;
                }
            }
        } else if (event.type == EVENT_CLOSED_LIMIT) {
            if (xSemaphoreTake(closedLimitSemaphore, 0) == pdTRUE) {
                if (getGateState() == CLOSING) {
                    stopGate(IDLE_CLOSED);
                    mode = MODE_STOPPED;
                    activeSource = NO_SOURCE;
                    activeDirection = NO_DIRECTION;
                }
            }
        } else if (event.type == EVENT_OBSTACLE) {
            if (getGateState() == CLOSING) {
                vPrintString("Obstacle detected\n");

                blockedButtons |= (event.pressedButtons & PANEL_BUTTONS);
                mode = MODE_STOPPED;
                activeSource = NO_SOURCE;
                activeDirection = NO_DIRECTION;

                stopGate(STOPPED_MIDWAY);
                setGateState(REVERSING);
                sendLedCommand(LED_GREEN);
                vTaskDelay(pdMS_TO_TICKS(REVERSE_TIME_MS));
                stopGate(STOPPED_MIDWAY);
            }
        } else if (event.type == EVENT_HOLD_TIMEOUT) {
            command = getButtonCommand(usefulButtons);

            if (mode == MODE_WAITING_RELEASE &&
                command.source == activeSource &&
                command.direction == activeDirection &&
                command.conflict == false) {
                mode = MODE_MANUAL;
                vPrintString("Manual mode\n");
            }
        } else {
            command = getButtonCommand(usefulButtons);

            if (command.conflict == true) {
                stopGate(STOPPED_MIDWAY);
                mode = MODE_STOPPED;
                activeSource = NO_SOURCE;
                activeDirection = NO_DIRECTION;
            } else if (command.direction == NO_DIRECTION) {
                if (mode == MODE_WAITING_RELEASE) {
                    mode = MODE_AUTO;
                    vPrintString("Auto mode\n");
                } else if (mode == MODE_MANUAL) {
                    stopGate(STOPPED_MIDWAY);
                    mode = MODE_STOPPED;
                    activeSource = NO_SOURCE;
                    activeDirection = NO_DIRECTION;
                }
            } else {
                if (mode == MODE_STOPPED ||
                    mode == MODE_AUTO ||
                    command.source != activeSource ||
                    command.direction != activeDirection) {

                    if (command.direction == OPEN_DIRECTION && getGateState() == IDLE_OPEN) {
                        stopGate(IDLE_OPEN);
                        mode = MODE_STOPPED;
                    } else if (command.direction == CLOSE_DIRECTION && getGateState() == IDLE_CLOSED) {
                        stopGate(IDLE_CLOSED);
                        mode = MODE_STOPPED;
                    } else {
                        activeSource = command.source;
                        activeDirection = command.direction;
                        mode = MODE_WAITING_RELEASE;
                        startGate(command.direction);
                    }
                }
            }
        }
    }
}

static void ledTask(void *pvParameters)
{
    LedCommand command;

    (void)pvParameters;

    while (1) {
        xQueueReceive(ledQueue, &command, portMAX_DELAY);

        if (command == LED_GREEN) {
            GPIO_PORTF_DATA_R &= ~BOTH_LEDS;
            GPIO_PORTF_DATA_R |= GREEN_LED;
        } else if (command == LED_RED) {
            GPIO_PORTF_DATA_R &= ~BOTH_LEDS;
            GPIO_PORTF_DATA_R |= RED_LED;
        } else {
            GPIO_PORTF_DATA_R &= ~BOTH_LEDS;
        }
    }
}

static void safetyTask(void *pvParameters)
{
    ObstacleEvent obstacleEvent;

    (void)pvParameters;

    while (1) {
        xQueueReceive(obstacleQueue, &obstacleEvent, portMAX_DELAY);

        if (obstacleEvent.active == true && getGateState() == CLOSING) {
            sendGateEvent(EVENT_OBSTACLE, obstacleEvent.pressedButtons, OBSTACLE, pdTRUE);
        }
    }
}

static void statusTask(void *pvParameters)
{
    (void)pvParameters;

    while (1) {
        vPrintString("State: ");
        vPrintString(stateToString(getGateState()));
        vPrintString("\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
