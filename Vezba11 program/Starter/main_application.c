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
void SensorTrigger_Task(void* pvParameters);
void PCReceive_Task(void* pvParameters);
void PCSend_Task(void* pvParameters);
void TemperatureDisplay_Task(void* pvParameters);
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
SemaphoreHandle_t TBE_PC_BinarySemaphore;
SemaphoreHandle_t TBE_CH1_BinarySemaphore;
SemaphoreHandle_t RXC_CH0_Semaphore;
SemaphoreHandle_t RXC_CH1_Semaphore;
SemaphoreHandle_t RXC_PC_Semaphore;

QueueHandle_t LEDBar_Queue;
QueueHandle_t Sensor_Queue;
QueueHandle_t PCSend_Queue;

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

uint8_t sensor0_count = 0U;
uint8_t sensor1_count = 0U;

uint8_t average_resistance_ch0 = 0U;
uint8_t average_resistance_ch1 = 0U;

int8_t temperature_ch0 = 0;
int8_t temperature_ch1 = 0;

uint8_t resistance_ch0 = 0;
uint8_t resistance_ch1 = 0;

uint8_t digit_count_ch0 = 0;
uint8_t digit_count_ch1 = 0;

// SENSOR CALIBRATION
int8_t sensor_min_temp = 20;
int8_t sensor_max_temp = 50;
int8_t temperature_high_limit = 100;
int8_t temperature_low_limit = 10;

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

int8_t ParseTemperatureValue(const char* command, uint8_t start_index
)
{
	int16_t value = 0;
	uint8_t i = start_index;

	while ((command[i] >= '0') && (command[i] <= '9'))
	{
		value = (value * 10) + (command[i] - '0');
		i++;
	}

	return (int8_t)value;
}
uint8_t CalculateAverage(uint8_t* values, uint8_t count
)
{
	uint16_t sum = 0U;
	uint8_t i;

	if (count == 0U)
	{
		return 0U;
	}

	for (i = 0U; i < count; i++)
	{
		sum += values[i];
	}

	return (uint8_t)(sum / count);
}

// INTERRUPTS //
static uint32_t OnLED_ChangeInterrupt(void) {	// OPC - ON INPUT CHANGE - INTERRUPT HANDLER 
	BaseType_t xHigherPTW = pdFALSE;

	xSemaphoreGiveFromISR(LED_INT_BinarySemaphore, &xHigherPTW);
	portYIELD_FROM_ISR(xHigherPTW);
}

static uint32_t prvProcessTBEInterrupt(void)
{
	BaseType_t xHigherPTW = pdFALSE;

	if (get_TBE_status(SENSOR_IN_CH))
	{
		xSemaphoreGiveFromISR(
			TBE_BinarySemaphore,
			&xHigherPTW
		);
	}

	if (get_TBE_status(SENSOR_OUT_CH))
	{
		xSemaphoreGiveFromISR(
			TBE_CH1_BinarySemaphore,
			&xHigherPTW
		);
	}

	if (get_TBE_status(PC_CH))
	{
		xSemaphoreGiveFromISR(
			TBE_PC_BinarySemaphore,
			&xHigherPTW
		);
	}

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

	if (get_RXC_status(PC_CH))
	{
		xSemaphoreGiveFromISR(
			RXC_PC_Semaphore,
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
	init_serial_uplink(PC_CH);
	init_serial_downlink(PC_CH);

	// INTERRUPT HANDLERS
	vPortSetInterruptHandler(portINTERRUPT_SRL_OIC, OnLED_ChangeInterrupt);		// ON INPUT CHANGE INTERRUPT HANDLER 
	vPortSetInterruptHandler(portINTERRUPT_SRL_TBE, prvProcessTBEInterrupt);	// SERIAL TRANSMITT INTERRUPT HANDLER 
	vPortSetInterruptHandler(portINTERRUPT_SRL_RXC, prvProcessRXCInterrupt);	// SERIAL RECEPTION INTERRUPT HANDLER 

	// BINARY SEMAPHORES
	LED_INT_BinarySemaphore = xSemaphoreCreateBinary();	// CREATE LED INTERRUPT SEMAPHORE 
	TBE_BinarySemaphore = xSemaphoreCreateBinary();
	TBE_CH1_BinarySemaphore = xSemaphoreCreateBinary();
	TBE_PC_BinarySemaphore = xSemaphoreCreateBinary();
	RXC_CH0_Semaphore = xSemaphoreCreateBinary();

	RXC_CH1_Semaphore = xSemaphoreCreateBinary();

	RXC_PC_Semaphore = xSemaphoreCreateBinary();

	// QUEUES
	LEDBar_Queue = xQueueCreate(2, sizeof(uint8_t));
	Sensor_Queue = xQueueCreate(10, sizeof(SensorData));
	PCSend_Queue = xQueueCreate(5, sizeof(char[32]));

	// TASKS 
	/* xTaskCreate(SerialSend_Task, "STx", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_SEND_PRI, NULL);	// SERIAL TRANSMITTER TASK */
	xTaskCreate(SensorTrigger_Task, "Trigger", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_SEND_PRI, NULL);
	xTaskCreate(SerialReceive_Task, "SRx", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAl_REC_PRI, NULL);	// SERIAL RECEIVER TASK 
	r_point = 0;
	xTaskCreate(LEDBar_Task, "ST", configMINIMAL_STACK_SIZE, NULL, SERVICE_TASK_PRI, NULL);				// CREATE LED BAR TASK  
	xTaskCreate(TemperatureProcess_Task, "Temp", configMINIMAL_STACK_SIZE, NULL, TASK_TEMP_PROCESS_PRI, NULL);
	xTaskCreate(PCReceive_Task, "PCRx", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAl_REC_PRI, NULL);
	xTaskCreate(PCSend_Task, "PCTx", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_SEND_PRI, NULL);
	xTaskCreate(TemperatureDisplay_Task, "TempDisplay", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_SEND_PRI, NULL);

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

	(void)pvParameters;

	while (1)
	{
		xQueueReceive(
			Sensor_Queue,
			&data,
			portMAX_DELAY
		);

		if (data.channel == SENSOR_IN_CH)
		{
			sensor0_values[index0] = data.resistance;

			index0++;

			if (index0 >= 5U)
			{
				index0 = 0U;
			}

			if (sensor0_count < 5U)
			{
				sensor0_count++;
			}

			average_resistance_ch0 =
				CalculateAverage(
					sensor0_values,
					sensor0_count
				);

			temperature_ch0 =
				CalculateTemperature(
					average_resistance_ch0
				);

			printf(
				"Senzor kanal 0: R=%u ohm T=%d C\n",
				(unsigned)average_resistance_ch0,
				(int)temperature_ch0
			);
		}
		else if (data.channel == SENSOR_OUT_CH)
		{
			sensor1_values[index1] = data.resistance;

			index1++;

			if (index1 >= 5U)
			{
				index1 = 0U;
			}

			if (sensor1_count < 5U)
			{
				sensor1_count++;
			}

			average_resistance_ch1 =
				CalculateAverage(
					sensor1_values,
					sensor1_count
				);

			temperature_ch1 =
				CalculateTemperature(
					average_resistance_ch1
				);

			printf(
				"Senzor kanal 1: R=%u ohm T=%d C\n",
				(unsigned)average_resistance_ch1,
				(int)temperature_ch1
			);
		}
	}
}


void SensorTrigger_Task(void* pvParameters)
{
	(void)pvParameters;

	while (1)
	{
		/* Kanal 0 */
		send_serial_character(SENSOR_IN_CH, 'X');
		xSemaphoreTake(TBE_BinarySemaphore, portMAX_DELAY);

		send_serial_character(SENSOR_IN_CH, 'Y');
		xSemaphoreTake(TBE_BinarySemaphore, portMAX_DELAY);

		send_serial_character(SENSOR_IN_CH, 'Z');
		xSemaphoreTake(TBE_BinarySemaphore, portMAX_DELAY);


		/* Kanal 1 */
		send_serial_character(SENSOR_OUT_CH, 'X');
		xSemaphoreTake(TBE_CH1_BinarySemaphore, portMAX_DELAY);

		send_serial_character(SENSOR_OUT_CH, 'Y');
		xSemaphoreTake(TBE_CH1_BinarySemaphore, portMAX_DELAY);

		send_serial_character(SENSOR_OUT_CH, 'Z');
		xSemaphoreTake(TBE_CH1_BinarySemaphore, portMAX_DELAY);


		/* Ponovi nakon 1 sekunde */
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}


void PCReceive_Task(void* pvParameters)
{
	uint8_t cc;
	char command[32];
	uint8_t index = 0U;

	(void)pvParameters;

	while (1)
	{
		/* Cekamo da stigne bar jedan znak sa PC-a */
		xSemaphoreTake(
			RXC_PC_Semaphore,
			portMAX_DELAY
		);

		/*
		 * Obradi sve znakove koji su trenutno pristigli.
		 * Ovo je bitno jer RXC semafor moze biti samo jednom
		 * "dat", iako je pristiglo vise znakova.
		 */
		do
		{
			if (get_serial_character(PC_CH, &cc) != 0)
			{
				break;
			}

			/* ENTER - prihvatamo i CR i LF */
			if ((cc == 13U) || (cc == 10U))
			{
				if (index > 0U)
				{
					command[index] = '\0';

					printf(
						"Primljena komanda: %s\n",
						command
					);

					/* MINTEMP */
					if (strncmp(command, "MINTEMP", 7U) == 0)
					{
						sensor_min_temp =
							ParseTemperatureValue(command, 7U);

						printf(
							"MINTEMP = %d C\n",
							(int)sensor_min_temp
						);

						{
							char message[32] = "OK\r\n";

							xQueueSend(
								PCSend_Queue,
								message,
								portMAX_DELAY
							);
						}
					}

					/* MAXTEMP */
					else if (strncmp(command, "MAXTEMP", 7U) == 0)
					{
						sensor_max_temp =
							ParseTemperatureValue(command, 7U);

						printf(
							"MAXTEMP = %d C\n",
							(int)sensor_max_temp
						);

						{
							char message[32] = "OK\r\n";

							xQueueSend(
								PCSend_Queue,
								message,
								portMAX_DELAY
							);
						}
					}

					/* THIGH */
					else if (strncmp(command, "THIGH", 5U) == 0)
					{
						temperature_high_limit =
							ParseTemperatureValue(command, 5U);

						printf(
							"THIGH = %d C\n",
							(int)temperature_high_limit
						);

						{
							char message[32] = "OK\r\n";

							xQueueSend(
								PCSend_Queue,
								message,
								portMAX_DELAY
							);
						}
					}

					/* TLOW */
					else if (strncmp(command, "TLOW", 4U) == 0)
					{
						temperature_low_limit =
							ParseTemperatureValue(command, 4U);

						printf(
							"TLOW = %d C\n",
							(int)temperature_low_limit
						);

						{
							char message[32] = "OK\r\n";

							xQueueSend(
								PCSend_Queue,
								message,
								portMAX_DELAY
							);
						}
					}

					index = 0U;
				}
			}

			/* Obican znak komande */
			else
			{
				if (index < (sizeof(command) - 1U))
				{
					command[index] = (char)cc;
					index++;
				}
				else
				{
					/* Overflow - resetuj komandu */
					index = 0U;
				}
			}

		} while (get_RXC_status(PC_CH) == 1);
	}
}


void TemperatureDisplay_Task(void* pvParameters)
{
	char message[32];

	(void)pvParameters;

	while (1)
	{
		sprintf(
			message,
			"TIN=%d C TOUT=%d C\r\n",
			(int)temperature_ch0,
			(int)temperature_ch1
		);

		xQueueSend(
			PCSend_Queue,
			&message,
			portMAX_DELAY
		);

		vTaskDelay(
			pdMS_TO_TICKS(1000)
		);
	}
}

void PCSend_Task(void* pvParameters)
{
	char message[32];
	uint8_t i;

	(void)pvParameters;

	while (1)
	{
		xQueueReceive(
			PCSend_Queue,
			&message,
			portMAX_DELAY
		);

		i = 0U;

		while (message[i] != '\0')
		{
			send_serial_character(
				PC_CH,
				(uint8_t)message[i]
			);

			xSemaphoreTake(
				TBE_PC_BinarySemaphore,
				portMAX_DELAY
			);

			i++;
		}
	}
}