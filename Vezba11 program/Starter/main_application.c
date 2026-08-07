// STANDARD INCLUDES
#include <stdio.h>l.g 
#include <conio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// KERNEL INCLUDES
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "extint.h"

// HARDWARE SIMULATOR UTILITY FUNCTIONS  
#include "HW_access.h"


// SERIAL SIMULATOR CHANNEL TO USE 

#define SENSOR_IN_CH (0)
#define SENSOR_OUT_CH (1)
#define PC_CH (2)

// TASK PRIORITIES 
#define	TASK_SERIAL_SEND_PRI		( tskIDLE_PRIORITY + 2 )
#define TASK_SERIAl_REC_PRI			( tskIDLE_PRIORITY + 3 )
#define	SERVICE_TASK_PRI			( tskIDLE_PRIORITY + 1 )
#define TASK_TEMP_PROCESS_PRI       ( tskIDLE_PRIORITY + 2 )


// TASKS: FORWARD DECLARATIONS 
void LEDBar_Task(void* pvParameters);
void SerialSend_Task(void* pvParameters);
void SerialReceive_Task(void* pvParameters);
void TemperatureProcess_Task(void* pvParameters);
int8_t CalculateTemperature(uint8_t resistance);

// TRASNMISSION DATA - CONSTANT IN THIS APPLICATION 
const char trigger[] = "XYZ";
unsigned volatile t_point;

// RECEPTION DATA BUFFER - COM 0
#define R_BUF_SIZE (32)
uint8_t r_buffer[R_BUF_SIZE];
unsigned volatile r_point;


// 7-SEG NUMBER DATABASE - ALL HEX DIGITS [ 0 1 2 3 4 5 6 7 8 9 A B C D E F ]
static const char hexnum[] = { 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71 };


// GLOBAL OS-HANDLES 
SemaphoreHandle_t LED_INT_BinarySemaphore;
SemaphoreHandle_t TBE_BinarySemaphore;
SemaphoreHandle_t RXC_CH0_Semaphore;
SemaphoreHandle_t RXC_CH1_Semaphore;

QueueHandle_t LEDBar_Queue;
QueueHandle_t Sensor_Queue;

// STRUCTURES
typedef struct
{
	uint8_t channel;
	uint8_t resistance;

} SensorData; 

uint8_t sensor0_values[5];
uint8_t sensor1_values[5];

uint8_t index0 = 0;
uint8_t index1 = 0;

uint8_t resistance_ch0 = 0;
uint8_t resistance_ch1 = 0;

uint8_t digit_count_ch0 = 0;
uint8_t digit_count_ch1 = 0;

// SENSOR CALIBRATION
int8_t sensor_min_temp = 20;
int8_t sensor_max_temp = 50;

#define SENSOR_MAX_RESISTANCE (71U)

//FUNCTIONS
int8_t CalculateTemperature(uint8_t resistance)
{
	int16_t temperature;


	temperature = sensor_max_temp -
		(
			(
				(int16_t)resistance *
				(sensor_max_temp - sensor_min_temp)
				)
			/
			SENSOR_MAX_RESISTANCE
			);


	return (int8_t)temperature;
}

// INTERRUPTS //
static uint32_t OnLED_ChangeInterrupt(void) {	// OPC - ON INPUT CHANGE - INTERRUPT HANDLER 
	BaseType_t xHigherPTW = pdFALSE;

	xSemaphoreGiveFromISR(LED_INT_BinarySemaphore, &xHigherPTW);
	portYIELD_FROM_ISR(xHigherPTW);
}

static uint32_t prvProcessTBEInterrupt(void) {	// TBE - TRANSMISSION BUFFER EMPTY - INTERRUPT HANDLER 
	BaseType_t xHigherPTW = pdFALSE;

	xSemaphoreGiveFromISR(TBE_BinarySemaphore, &xHigherPTW);
	portYIELD_FROM_ISR(xHigherPTW);
}

static uint32_t prvProcessRXCInterrupt(void)
{
	BaseType_t xHigherPTW = pdFALSE;


	if (get_RXC_status(SENSOR_IN_CH))
	{
		xSemaphoreGiveFromISR(
			RXC_CH0_Semaphore,
			&xHigherPTW
		);
	}


	if (get_RXC_status(SENSOR_OUT_CH))
	{
		xSemaphoreGiveFromISR(
			RXC_CH1_Semaphore,
			&xHigherPTW
		);
	}


	portYIELD_FROM_ISR(xHigherPTW);
}


// MAIN - SYSTEM STARTUP POINT 
void main_demo(void) {
	// INITIALIZATION OF THE PERIPHERALS
	//init_7seg_comm();
	init_LED_comm();
	init_serial_uplink(SENSOR_IN_CH);		// inicijalizacija serijske TX na kanalu 0
	init_serial_downlink(SENSOR_IN_CH);	// inicijalizacija serijske RX na kanalu 0
	init_serial_uplink(SENSOR_OUT_CH);
	init_serial_downlink(SENSOR_OUT_CH);

	// INTERRUPT HANDLERS
	vPortSetInterruptHandler(portINTERRUPT_SRL_OIC, OnLED_ChangeInterrupt);		// ON INPUT CHANGE INTERRUPT HANDLER 
	vPortSetInterruptHandler(portINTERRUPT_SRL_TBE, prvProcessTBEInterrupt);	// SERIAL TRANSMITT INTERRUPT HANDLER 
	vPortSetInterruptHandler(portINTERRUPT_SRL_RXC, prvProcessRXCInterrupt);	// SERIAL RECEPTION INTERRUPT HANDLER 

	// BINARY SEMAPHORES
	LED_INT_BinarySemaphore = xSemaphoreCreateBinary();	// CREATE LED INTERRUPT SEMAPHORE 
	TBE_BinarySemaphore = xSemaphoreCreateBinary();		// CREATE TBE SEMAPHORE - SERIAL TRANSMIT COMM 
	RXC_CH0_Semaphore = xSemaphoreCreateBinary();

	RXC_CH1_Semaphore = xSemaphoreCreateBinary();

	// QUEUES
	LEDBar_Queue = xQueueCreate(2, sizeof(uint8_t));
	Sensor_Queue = xQueueCreate(10, sizeof(SensorData));
	
	// TASKS 
	xTaskCreate(SerialSend_Task, "STx", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_SEND_PRI, NULL);	// SERIAL TRANSMITTER TASK 
	xTaskCreate(SerialReceive_Task, "SRx", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAl_REC_PRI, NULL);	// SERIAL RECEIVER TASK 
	r_point = 0;
	xTaskCreate(LEDBar_Task, "ST", configMINIMAL_STACK_SIZE, NULL, SERVICE_TASK_PRI, NULL);				// CREATE LED BAR TASK  
	xTaskCreate(TemperatureProcess_Task,"Temp",configMINIMAL_STACK_SIZE,NULL,TASK_TEMP_PROCESS_PRI,NULL);

	// START SCHEDULER
	vTaskStartScheduler();
	while (1);
}

// TASKS: IMPLEMENTATIONS
void LEDBar_Task(void* pvParameters) {
	uint8_t LEDsPattern;
	while (1) {
		xQueueReceive(LEDBar_Queue, &LEDsPattern, portMAX_DELAY);		
		set_LED_BAR(0, LEDsPattern);
	}
}

void SerialSend_Task(void* pvParameters) {
	t_point = 0;
	while (1) {
		if (t_point > (sizeof(trigger) - 1))
			t_point = 0;
		send_serial_character(SENSOR_IN_CH, trigger[t_point++]);
		xSemaphoreTake(TBE_BinarySemaphore, portMAX_DELAY);// kada se koristi predajni interapt
		//vTaskDelay(pdMS_TO_TICKS(100));// kada se koristi vremenski delay
	}
}


void SerialReceive_Task(void* pvParameters)
{
	uint8_t cc = 0;
	uint8_t channel;
	uint8_t resistance = 0;


	SensorData data;

	while (1)
	{
		if (xSemaphoreTake(RXC_CH0_Semaphore, 0) == pdTRUE)
		{
			get_serial_character(
				SENSOR_IN_CH,
				&cc
			);

			channel = SENSOR_IN_CH;
		}


		else if (xSemaphoreTake(RXC_CH1_Semaphore, 0) == pdTRUE)
		{
			get_serial_character(
				SENSOR_OUT_CH,
				&cc
			);

			channel = SENSOR_OUT_CH;
		}


		else
		{
			vTaskDelay(pdMS_TO_TICKS(10));
			continue;
		}




		// ako je cifra
		if ((cc >= (uint8_t)'0') && (cc <= (uint8_t)'9'))
		{
			if (channel == SENSOR_IN_CH)
			{
				resistance_ch0 = resistance_ch0 * 10U + (cc - (uint8_t)'0');
				digit_count_ch0++;
			}
			else
			{
				resistance_ch1 = resistance_ch1 * 10U + (cc - (uint8_t)'0');
				digit_count_ch1++;
			}


			// primili smo broj (maksimalno 2 cifre)
			if ((channel == SENSOR_IN_CH && digit_count_ch0 == 2U) ||
				(channel == SENSOR_OUT_CH && digit_count_ch1 == 2U))
			{

				if (channel == SENSOR_IN_CH)
				{
					resistance = resistance_ch0;
				}
				else
				{
					resistance = resistance_ch1;
				}

				// ogranicenje senzora 0-71 ohm
				if (resistance <= 71U)
				{
					data.channel = channel;
					data.resistance = resistance;


					xQueueSend(
						Sensor_Queue,
						&data,
						portMAX_DELAY
					);
				}


				if (channel == SENSOR_IN_CH)
				{
					resistance_ch0 = 0;
					digit_count_ch0 = 0;
				}
				else
				{
					resistance_ch1 = 0;
					digit_count_ch1 = 0;
				}
			}
		}


		// ako stigne CR (ENTER)
		else if (cc == 13U)
		{
			if (channel == SENSOR_IN_CH)
			{
				if (digit_count_ch0 > 0U)
				{
					if (resistance_ch0 <= SENSOR_MAX_RESISTANCE)
					{
						data.channel = channel;
						data.resistance = resistance_ch0;

						xQueueSend(
							Sensor_Queue,
							&data,
							portMAX_DELAY
						);
					}
				}

				resistance_ch0 = 0;
				digit_count_ch0 = 0;
			}


			else
			{
				if (digit_count_ch1 > 0U)
				{
					if (resistance_ch1 <= SENSOR_MAX_RESISTANCE)
					{
						data.channel = channel;
						data.resistance = resistance_ch1;

						xQueueSend(
							Sensor_Queue,
							&data,
							portMAX_DELAY
						);
					}
				}

				resistance_ch1 = 0;
				digit_count_ch1 = 0;
			}
		}
	}
}


void TemperatureProcess_Task(void* pvParameters)
{
	SensorData data;

	int8_t temperature;


	while (1)
	{
		xQueueReceive(
			Sensor_Queue,
			&data,
			portMAX_DELAY
		);


		temperature = CalculateTemperature(data.resistance);


		printf(
			"Senzor kanal %u : R=%u ohm T=%d C\n",
			(unsigned)data.channel,
			(unsigned)data.resistance,
			(int)temperature
		);
	}
}