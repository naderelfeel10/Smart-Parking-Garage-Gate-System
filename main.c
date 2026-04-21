#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "basic_io.h"
#include "queue.h"

void vInputTask(void* pvParameters);
void vGateCTRLTask(void* pvParameters);
void vLEDCTRLTask(void* pvParameters);
void vSafetyTask(void* pvParameters);
void vStatusTask(void* pvParameters);


const StackType_t inputTaskDepth = 200;
const StackType_t gateCTRLTaskDepth = 200;
const StackType_t LEDCTRLTaskDepth = 200;
const StackType_t safetyTaskDepth = 200;
const StackType_t StatusTaskDepth = 200;



 UBaseType_t highPriority = 4;
 UBaseType_t medPriority = 3;
 UBaseType_t lowPriority = 2;
 UBaseType_t highestPriority = 5;



 xTaskHandle* const inputTaskHandle = NULL;
 xTaskHandle* const gateCTRLTaskHandle = NULL;
 xTaskHandle* const LEDCTRLTaskHandle = NULL;
 xTaskHandle* const safetyTaskHandle = NULL;
 xTaskHandle* const StatusTaskHanlde = NULL;



typedef enum{
	IDLE_OPEN,
	IDLE_CLOSED,
	OPENING,
	CLOSING,
	STOPPED_MIDWAY,
	REVERSING
}states;



typedef enum {
   BTN_DRIVER_OPEN,
   BTN_DRIVER_CLOSE,
	
   BTN_SECURITY_OPEN,
   BTN_SECURITY_CLOSE,
	
   BTN_LIMIT_OPEN,
   BTN_LIMIT_CLOSED,
	
   BTN_OBSTACLE,
	
	 BTN_RELEASED
	
} ButtonEvents;

xQueueHandle buttonsQueue;


typedef enum {
	RED_LED,
	GREEN_LED,
	OFF_LED
	
} LEDSEvents;

xQueueHandle ledsQueue;

// to do: we need to use MUTEX here
states curr_state = IDLE_CLOSED;
int
main(void) {

		//vPrintString("Elfeel");
		xTaskCreate(vInputTask, "input task", inputTaskDepth, NULL, highPriority,&inputTaskHandle);
		xTaskCreate(vGateCTRLTask, "gate ctrl task", gateCTRLTaskDepth, NULL, medPriority,&gateCTRLTaskHandle);
		xTaskCreate(vLEDCTRLTask, "LED ctrl task", LEDCTRLTaskDepth, NULL, medPriority,&LEDCTRLTaskHandle);
		xTaskCreate(vSafetyTask, "safety task", safetyTaskDepth, NULL, highestPriority ,&safetyTaskHandle);
		xTaskCreate(vStatusTask, "status task", StatusTaskDepth, NULL, lowPriority ,&StatusTaskHanlde);
		
	
		

		vTaskStartScheduler();
    for (;;){}
}

/*Reads and debounces all buttons; sends events to a queue*/ 
void vInputTask(void* pvParameters){
	
	buttonsQueue = xQueueCreate(10, sizeof(ButtonEvents));
	
	ButtonEvents but1 = BTN_DRIVER_CLOSE;
	
	for(;;){
		
			vPrintString("vInputTask is running \n");
		  xQueueSendToBack(buttonsQueue,&but1,portMAX_DELAY);
		
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}



/*Implements the state machine; decides gate actions*/ 
void vGateCTRLTask(void* pvParameters){
	ButtonEvents received_btn;
	portBASE_TYPE xStatus;
	
	ledsQueue = xQueueCreate(10, sizeof(LEDSEvents));
	LEDSEvents led;
	
	for(;;){
		vPrintString("vGateCTRLTask is running \n");
		
		xStatus = xQueueReceive(buttonsQueue, &received_btn, portMAX_DELAY );
		
		if( xStatus == pdPASS )
		{

			vPrintStringAndNumber( "Received = ", received_btn );
			
			switch(received_btn){
				
				case BTN_DRIVER_OPEN:{
					vPrintString("open gate order from driver received\n");
					
					curr_state = OPENING;
					led = GREEN_LED;
					xQueueSendToBack(ledsQueue, &led,0);
					break;
				}
				
				case BTN_DRIVER_CLOSE:{
					vPrintString("close gate order from driver received\n");
					curr_state = CLOSING;
					led = RED_LED;
					xQueueSendToBack(ledsQueue, &led,0);
					break;
				}
				
				case BTN_SECURITY_OPEN:{
					vPrintString("open gate order from sec received\n");
					curr_state = OPENING;
					led = GREEN_LED;
					xQueueSendToBack(ledsQueue, &led,0);
					break;
				}
				
				case BTN_SECURITY_CLOSE:{
					vPrintString("close gate order from sec received\n");
					curr_state = CLOSING;
					led = RED_LED;
					xQueueSendToBack(ledsQueue, &led,0);
					break;
				}
				
				case BTN_LIMIT_OPEN:{
					vPrintString("open gate order from limit received\n");
					//to do: we need to handle this using semaphores
					curr_state = IDLE_OPEN;
					led = OFF_LED;
					xQueueSendToBack(ledsQueue, &led,0);
					break;
				}
				
				case BTN_LIMIT_CLOSED:{
					vPrintString("close gate order from limit received\n");
					//to do: we need to handle this using semaphores
					curr_state = IDLE_CLOSED;
					led = OFF_LED;
					xQueueSendToBack(ledsQueue, &led,0);
					break;
				}
				
				case BTN_OBSTACLE:{
					vPrintString("obstacle order from limit received\n");
					
					if(curr_state == CLOSING){
						
						curr_state = REVERSING;
						led = GREEN_LED;
						xQueueSendToBack(ledsQueue, &led,0);
						
						vTaskDelay(pdMS_TO_TICKS(500));
						
						curr_state = STOPPED_MIDWAY;
						led = OFF_LED;
						xQueueSendToBack(ledsQueue, &led,0);
						
					}
					break;
				}
				case BTN_RELEASED:{
						curr_state = STOPPED_MIDWAY;
						led = OFF_LED;
						xQueueSendToBack(ledsQueue, &led,0);
					
				}
				
				
			}
		}
		
		vTaskDelay(pdMS_TO_TICKS(400));
	}
	
	
}

void vLEDCTRLTask(void* pvParameters){
	
	LEDSEvents received_led;
	portBASE_TYPE xStatus;

	for(;;){
		vPrintString("vLEDTask is running \n");
		xStatus = xQueueReceive(ledsQueue, &received_led, portMAX_DELAY);
		
		if(xStatus == pdPASS ){
			
			switch(received_led){
				case GREEN_LED:{
							vPrintString("Green LED is ON \n");
							break;
			}
			case RED_LED:{
							vPrintString("RED LED is ON \n");
							break;
			}
			case OFF_LED:{
							vPrintString("both leds are OFF \n");
							break;
			}
	}
			
		vTaskDelay(pdMS_TO_TICKS(300));
	}
}
}

void vSafetyTask(void* pvParameters){
	for(;;){
		vPrintString("vSafetyTask is running \n");
		vTaskDelay(pdMS_TO_TICKS(600));
	}
}

void vStatusTask(void* pvParameters){
		for(;;){
		vPrintString("vStatusTask is running \n");
		vTaskDelay(pdMS_TO_TICKS(200));
	}
}