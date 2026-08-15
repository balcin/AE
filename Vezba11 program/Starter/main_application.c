// STANDARD INCLUDES
#include <stdio.h>
#include <string.h>
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
#define TASK_LCD_PRI (tskIDLE_PRIORITY + 1)

// TASKS: FORWARD DECLARATIONS 
/* Entry point used outside this file. */
void main_demo(void);

/* Functions used only inside main_application.c. */
static void LEDBar_Task(void* pvParameters);
static void SerialReceive_Task(void* pvParameters);
static void TemperatureProcess_Task(void* pvParameters);
static void SensorTrigger_Task(void* pvParameters);
static void PCReceive_Task(void* pvParameters);
static void PCSend_Task(void* pvParameters);
static void TemperatureDisplay_Task(void* pvParameters);
static void LCDDisplay_Task(void* pvParameters);

static void LCDTimerCallback(TimerHandle_t xTimer);

static int8_t CalculateTemperature(uint8_t resistance);
static uint8_t TemperatureToLEDPattern(int8_t temperature);
static int8_t ParseTemperatureValue(
	const char* command,
	uint8_t start_index
);
static uint8_t CalculateAverage(
	const uint8_t* values,
	uint8_t count
);



// 7-SEG NUMBER DATABASE - ALL HEX DIGITS [ 0 1 2 3 4 5 6 7 8 9 A B C D E F ]
static const uint8_t hexnum[16] =
{
	0x3FU, 0x06U, 0x5BU, 0x4FU,
	0x66U, 0x6DU, 0x7DU, 0x07U,
	0x7FU, 0x6FU, 0x77U, 0x7CU,
	0x39U, 0x5EU, 0x79U, 0x71U
};


// GLOBAL OS-HANDLES 
static SemaphoreHandle_t LED_INT_BinarySemaphore;
static SemaphoreHandle_t TBE_BinarySemaphore;
static SemaphoreHandle_t TBE_PC_BinarySemaphore;
static SemaphoreHandle_t TBE_CH1_BinarySemaphore;
static SemaphoreHandle_t RXC_CH0_Semaphore;
static SemaphoreHandle_t RXC_CH1_Semaphore;
static SemaphoreHandle_t RXC_PC_Semaphore;
static SemaphoreHandle_t LCD_BinarySemaphore;

static QueueHandle_t Sensor_Queue;
/* STEP 2: Added queue for temperature data */
static QueueHandle_t Temperature_Queue;
static QueueHandle_t PCSend_Queue;
static QueueHandle_t LCD_Queue;

static TimerHandle_t LCD_Timer;

// STRUCTURES
typedef struct
{
	uint8_t channel;
	uint8_t resistance;

} SensorData;

/* STEP 1: Added structure for temperature data */
typedef struct
{
	uint8_t channel;
	int8_t temperature;

} TemperatureData;

typedef struct
{
	uint8_t resistance_inside;
	uint8_t resistance_outside;
	int8_t temperature_inside;
	int8_t temperature_outside;
} LCDData;

static uint8_t sensor0_values[5];
static uint8_t sensor1_values[5];

static uint8_t index0 = 0;
static uint8_t index1 = 0;

static uint8_t sensor0_count = 0U;
static uint8_t sensor1_count = 0U;

static uint8_t average_resistance_ch0 = 0U;
static uint8_t average_resistance_ch1 = 0U;

static int8_t temperature_ch0 = 0;
static int8_t temperature_ch1 = 0;

static uint8_t resistance_ch0 = 0;
static uint8_t resistance_ch1 = 0;

static uint8_t digit_count_ch0 = 0;
static uint8_t digit_count_ch1 = 0;

// SENSOR CALIBRATION
static int8_t sensor_min_temp = 20;
static int8_t sensor_max_temp = 50;
static int8_t temperature_high_limit = 100;
static int8_t temperature_low_limit = 10;

#define SENSOR_MAX_RESISTANCE (71U)

//FUNCTIONS
static int8_t CalculateTemperature(uint8_t resistance)
{
	int32_t resistance_value;
	int32_t minimum_temperature;
	int32_t maximum_temperature;
	int32_t temperature_range;
	int32_t maximum_resistance;
	int32_t calculated_temperature;
	int8_t result;

	resistance_value = (int32_t)resistance;
	minimum_temperature = (int32_t)sensor_min_temp;
	maximum_temperature = (int32_t)sensor_max_temp;
	maximum_resistance = (int32_t)SENSOR_MAX_RESISTANCE;

	temperature_range =
		maximum_temperature - minimum_temperature;

	calculated_temperature =
		maximum_temperature -
		(
			(resistance_value * temperature_range)
			/ maximum_resistance
			);

	/*
	 * The calibration limits are stored as int8_t, therefore the
	 * calculated linear interpolation remains inside the int8_t range.
	 */
	result = (int8_t)calculated_temperature;

	return result;
}

static uint8_t TemperatureToLEDPattern(int8_t temperature)
{
	uint8_t led_count;
	uint8_t pattern;
	uint16_t scaled_temperature = 0U;

	if (temperature <= 0)
	{
		led_count = 0U;
	}
	else if (temperature >= 80)
	{
		led_count = 8U;
	}
	else
	{
		/*
		 * U ovaj blok se ulazi samo kada je temperatura
		 * veca od 0 i manja od 80, pa je konverzija
		 * u unsigned tip bezbjedna.
		 */
		scaled_temperature = (uint16_t)temperature;
		scaled_temperature =
			(scaled_temperature * 8U) / 80U;

		led_count = (uint8_t)scaled_temperature;
	}

	if (led_count == 0U)
	{
		pattern = 0x00U;
	}
	else
	{
		pattern = (uint8_t)((1U << led_count) - 1U);
	}

	return pattern;
}

static int8_t ParseTemperatureValue(const char* command, uint8_t start_index
)
{
	uint16_t value = 0U;
	uint8_t index = start_index;
	uint8_t current_character;
	uint8_t digit;
	int8_t result;

	current_character = (uint8_t)command[index];

	while (
		(current_character >= (uint8_t)'0') &&
		(current_character <= (uint8_t)'9')
		)
	{
		digit =
			current_character - (uint8_t)'0';

		/*
		 * Najveca dozvoljena vrijednost je 127,
		 * jer se rezultat cuva kao int8_t.
		 */
		if (
			(value < 12U) ||
			((value == 12U) && (digit <= 7U))
			)
		{
			value = (value * 10U) + digit;
		}
		else
		{
			value = 127U;
		}

		index++;
		current_character = (uint8_t)command[index];
	}

	/*
	 * value je prethodno ogranicen na opseg 0-127,
	 * pa je konverzija u int8_t bezbjedna.
	 */
	result = (int8_t)value;

	return result;
}
static uint8_t CalculateAverage(
	const uint8_t* values,
	uint8_t count
)
{
	uint16_t sum = 0U;
	uint8_t index;
	uint8_t result = 0U;

	if ((values != NULL) && (count > 0U))
	{
		for (index = 0U; index < count; index++)
		{
			sum += (uint16_t)values[index];
		}

		result = (uint8_t)(
			sum / (uint16_t)count
			);
	}
	else
	{
		/*
		 * Nema dostupnih mjerenja ili pokazivac nije validan.
		 * result ostaje 0U.
		 */
		result = 0U;
	}

	return result;
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

	if (get_TBE_status(SENSOR_IN_CH)==1)
	{
		xSemaphoreGiveFromISR(
			TBE_BinarySemaphore,
			&xHigherPTW
		);
	}

	if (get_TBE_status(SENSOR_OUT_CH)==1)
	{
		xSemaphoreGiveFromISR(
			TBE_CH1_BinarySemaphore,
			&xHigherPTW
		);
	}

	if (get_TBE_status(PC_CH)==1)
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


	if (get_RXC_status(SENSOR_IN_CH)==1)
	{
		xSemaphoreGiveFromISR(
			RXC_CH0_Semaphore,
			&xHigherPTW
		);
	}


	if (get_RXC_status(SENSOR_OUT_CH)==1)
	{
		xSemaphoreGiveFromISR(
			RXC_CH1_Semaphore,
			&xHigherPTW
		);
	}

	if (get_RXC_status(PC_CH)==1)
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
	init_7seg_comm();
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

	LCD_BinarySemaphore = xSemaphoreCreateBinary();

	LCD_Timer = xTimerCreate(
		"LCDTimer",
		pdMS_TO_TICKS(100),
		pdTRUE,
		NULL,
		LCDTimerCallback
	);

	// QUEUES
	
	Sensor_Queue = xQueueCreate(10, sizeof(SensorData));
	/* STEP 3: Create queue for temperature data */
	Temperature_Queue = xQueueCreate(10, sizeof(TemperatureData));
	PCSend_Queue = xQueueCreate(5, sizeof(char[32]));
	LCD_Queue = xQueueCreate(1, sizeof(LCDData));

	// TASKS 
	
	xTaskCreate(SensorTrigger_Task, "Trigger", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_SEND_PRI, NULL);
	xTaskCreate(SerialReceive_Task, "SRx", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAl_REC_PRI, NULL);	// SERIAL RECEIVER TASK 
	xTaskCreate(LEDBar_Task, "ST", configMINIMAL_STACK_SIZE, NULL, SERVICE_TASK_PRI, NULL);				// CREATE LED BAR TASK  
	xTaskCreate(TemperatureProcess_Task, "Temp", configMINIMAL_STACK_SIZE, NULL, TASK_TEMP_PROCESS_PRI, NULL);
	xTaskCreate(PCReceive_Task, "PCRx", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAl_REC_PRI, NULL);
	xTaskCreate(PCSend_Task, "PCTx", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_SEND_PRI, NULL);
	xTaskCreate(TemperatureDisplay_Task, "TempDisplay", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_SEND_PRI, NULL);
	xTaskCreate(LCDDisplay_Task, "LCD", configMINIMAL_STACK_SIZE, NULL, TASK_LCD_PRI, NULL);
	if (LCD_Timer != NULL)
	{
		(void)xTimerStart(LCD_Timer, 0U);
	}

	// START SCHEDULER
	vTaskStartScheduler();
	for (;;) {
	}
}

// TASKS: IMPLEMENTATIONS
static void LEDBar_Task(void* pvParameters)
{
	TemperatureData data;

	int8_t temp_inside = 0;
	int8_t temp_outside = 0;

	uint8_t alarm_state = 0U;
	(void)pvParameters;

	for (;;)
	{


		if (xQueueReceive(
			Temperature_Queue,
			&data,
			0) == pdTRUE)
		{
			if (data.channel == SENSOR_IN_CH)
			{
				temp_inside = data.temperature;
			}
			else if (data.channel == SENSOR_OUT_CH)
			{
				temp_outside = data.temperature;
			}
			else
			{
				/*
				 * Podaci sa drugih kanala se ne ocekuju
				 * i namjerno se ignorisu.
				 */
			}

			/* STEP 8: Display temperatures on LED bars */
			set_LED_BAR(
				1,
				TemperatureToLEDPattern(temp_inside)
			);

			set_LED_BAR(
				2,
				TemperatureToLEDPattern(temp_outside)
			);

			printf(
				"LED TASK: unutrasnja=%d C, spoljasnja=%d C\n",
				(int)temp_inside,
				(int)temp_outside
			);
		}

		/* STEP 7: Check temperature limits and blink first LED bar */
		if ((temp_inside < temperature_low_limit) ||
			(temp_inside > temperature_high_limit) ||
			(temp_outside < temperature_low_limit) ||
			(temp_outside > temperature_high_limit))
		{
			if (alarm_state == 0U)
			{
				set_LED_BAR(0, 0xFFU);
				alarm_state = 1U;
			}
			else
			{
				set_LED_BAR(0, 0x00U);
				alarm_state = 0U;
			}
		}
		else
		{
			set_LED_BAR(0, 0x00U);
			alarm_state = 0U;
		}

		vTaskDelay(pdMS_TO_TICKS(500));
	}
}



static void SerialReceive_Task(void* pvParameters)
{
	uint8_t cc = 0;
	uint8_t channel;
	uint8_t resistance = 0;


	SensorData data;
	(void)pvParameters;

	for (;;)
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
		else
{
/*
 * Karakteri koji nisu cifra ili CR
 * namjerno se ignorisu.
 */
}
}
}


static void TemperatureProcess_Task(void* pvParameters)
{
	SensorData data;
	TemperatureData temperature_data;

	LCDData lcd_data =
	{
		0U,
		0U,
		0,
		0
	};

	(void)pvParameters;

	for (;;)
	{
		if (xQueueReceive(
			Sensor_Queue,
			&data,
			portMAX_DELAY
		) == pdTRUE)
		{
			temperature_data.channel = data.channel;

			if (data.channel == SENSOR_IN_CH)
			{
				/* Sacuvaj novo ocitavanje unutrasnjeg senzora */
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

				/* Izracunaj prosjek dostupnih ocitavanja */
				average_resistance_ch0 = CalculateAverage(
					sensor0_values,
					sensor0_count
				);

				/* Temperaturu racunamo iz prosjecne otpornosti */
				temperature_ch0 = CalculateTemperature(
					average_resistance_ch0
				);

				temperature_data.temperature = temperature_ch0;

				lcd_data.resistance_inside = average_resistance_ch0;
				lcd_data.temperature_inside = temperature_ch0;
			}
			else if (data.channel == SENSOR_OUT_CH)
			{
				/* Sacuvaj novo ocitavanje spoljasnjeg senzora */
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

				/* Izracunaj prosjek dostupnih ocitavanja */
				average_resistance_ch1 = CalculateAverage(
					sensor1_values,
					sensor1_count
				);

				/* Temperaturu racunamo iz prosjecne otpornosti */
				temperature_ch1 = CalculateTemperature(
					average_resistance_ch1
				);

				temperature_data.temperature = temperature_ch1;

				lcd_data.resistance_outside = average_resistance_ch1;
				lcd_data.temperature_outside = temperature_ch1;
			}
			else
			{
				/* Neocekivan kanal - podatak se ne obradjuje */
				continue;
			}

			printf(
				"Kanal %u: Ravg=%u ohm, T=%d C\n",
				(unsigned)data.channel,
				(unsigned)(
					(data.channel == SENSOR_IN_CH)
					? average_resistance_ch0
					: average_resistance_ch1
					),
				(int)temperature_data.temperature
			);

			/* Posalji temperaturu LED tasku */
			(void)xQueueSend(
				Temperature_Queue,
				&temperature_data,
				portMAX_DELAY
			);

			/* LCD-u je potrebna samo najnovija kompletna vrijednost */
			if (LCD_Queue != NULL)
			{
				(void)xQueueOverwrite(
					LCD_Queue,
					&lcd_data
				);
			}
		}
	}
}

static void SensorTrigger_Task(void* pvParameters)
{
	(void)pvParameters;

	for (;;)
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


static void PCReceive_Task(void* pvParameters)
{
	uint8_t cc;
	char command[32];
	uint8_t index = 0U;

	(void)pvParameters;

	for (;;)
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
					else
					{
						/*
						 * Nepodrzana komanda ne mijenja
						 * konfiguraciju sistema.
						 */
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


static void TemperatureDisplay_Task(void* pvParameters)
{
	char message[32];

	(void)pvParameters;

	for (;;)
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

static void PCSend_Task(void* pvParameters)
{
	char message[32];
	uint8_t i;

	(void)pvParameters;

	for (;;)
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

static void LCDTimerCallback(TimerHandle_t xTimer)
{
	(void)xTimer;

	if (LCD_BinarySemaphore != NULL)
	{
		(void)xSemaphoreGive(LCD_BinarySemaphore);
	}
}

static void LCDDisplay_Task(void* pvParameters)
{
	LCDData lcd_data =
	{
		0U,
		0U,
		0,
		0
	};

	uint8_t display_mode = 0U;
	uint8_t refresh_count = 0U;
	uint8_t data_available = 0U;

	uint8_t digits[4] =
	{
		0U,
		0U,
		0U,
		0U
	};

	uint8_t value = 0U;
	uint8_t i;

	(void)pvParameters;

	for (;;)
	{
		/*
		 * LCD task ceka semafor koji tajmer
		 * daje svakih 100 ms.
		 */
		if (xSemaphoreTake(
			LCD_BinarySemaphore,
			portMAX_DELAY
		) == pdTRUE)
		{
			/*
			 * Procitaj najnovije podatke iz LCD queue reda.
			 * xQueuePeek ne uklanja podatak iz reda.
			 */
			if (xQueuePeek(
				LCD_Queue,
				&lcd_data,
				0U
			) == pdTRUE)
			{
				data_available = 1U;
			}

			if (data_available == 1U)
			{
				/*
				 * display_mode:
				 *
				 * 0 - unutrasnja otpornost
				 * 1 - spoljasnja otpornost
				 * 2 - unutrasnja temperatura
				 * 3 - spoljasnja temperatura
				 */
				switch (display_mode)
				{
				case 0U:
				{
					/*
					 * IrXX
					 * Unutrasnja otpornost
					 */
					value = lcd_data.resistance_inside;

					digits[0] = 0x06U; /* I */
					digits[1] = 0x50U; /* r */

					break;
				}

				case 1U:
				{
					/*
					 * OrXX
					 * Spoljasnja otpornost
					 */
					value = lcd_data.resistance_outside;

					digits[0] = 0x3FU; /* O */
					digits[1] = 0x50U; /* r */

					break;
				}

				case 2U:
				{
					/*
					 * ItXX
					 * Unutrasnja temperatura
					 */
					if (lcd_data.temperature_inside < 0)
					{
						value = 0U;
					}
					else
					{
						value =
							(uint8_t)lcd_data.temperature_inside;
					}

					digits[0] = 0x06U; /* I */
					digits[1] = 0x78U; /* t */

					break;
				}

				case 3U:
				{
					/*
					 * OtXX
					 * Spoljasnja temperatura
					 */
					if (lcd_data.temperature_outside < 0)
					{
						value = 0U;
					}
					else
					{
						value =
							(uint8_t)lcd_data.temperature_outside;
					}

					digits[0] = 0x3FU; /* O */
					digits[1] = 0x78U; /* t */

					break;
				}

				default:
				{
					display_mode = 0U;
					value = 0U;

					digits[0] = 0x00U;
					digits[1] = 0x00U;

					break;
				}
				}

				/*
				 * Za broj koristimo poslednje dvije cifre.
				 * Najveca vrijednost koju mozemo prikazati je 99.
				 */
				if (value > 99U)
				{
					value = 99U;
				}

				/*
				 * Razlaganje vrijednosti na desetice i jedinice.
				 */
				digits[2] = hexnum[value / 10U];
				digits[3] = hexnum[value % 10U];

				/*
				 * Upis sve cetiri cifre na Seg7Mux.
				 */
				for (i = 0U; i < 4U; i++)
				{
					(void)select_7seg_digit(i);
					(void)set_7seg_digit(digits[i]);
				}
			}
			else
			{
				/*
				 * Ako jos nije stiglo nijedno mjerenje,
				 * ugasi sve cetiri cifre.
				 */
				for (i = 0U; i < 4U; i++)
				{
					(void)select_7seg_digit(i);
					(void)set_7seg_digit(0x00U);
				}
			}

			/*
			 * Task se aktivira svakih 100 ms.
			 * Nakon 10 aktiviranja prosla je jedna sekunda.
			 */
			refresh_count++;

			if (refresh_count >= 10U)
			{
				refresh_count = 0U;
				display_mode++;

				/*
				 * Poslije cetvrtog prikaza
				 * vrati se na prvi.
				 */
				if (display_mode >= 4U)
				{
					display_mode = 0U;
				}
			}
		}
	}
}