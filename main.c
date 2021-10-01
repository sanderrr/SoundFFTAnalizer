
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "math.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "ds18b20_mflib.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint16_t ADC_value[base_len];                                //буффер АЦП
uint16_t discrete_period = 1632;                             //делитель для формирования частоты дискретизации; зн-е: тактовая частота контроллера (72MHz)/dsicrete_period(1632)+1 = 44100Hz
extern uint16_t new_disc_period;                             //переменная для механизма установки новой частоты дискретизации
//uint8_t dataSend[4];                                       //байт данных для отправки по VCOM CDC
float sum_cells_value[base_len/2];                           //сумма ячеек для частотной выборки
float sum_cells_value_save[base_len/2];                      //ячейки хранения полученных выборок для последующей их отправки по Modbus
uint8_t value[12] = {0,5,13,17,22,26,33,39,60,74,88,103};    //польз.уставки, критерии выбора частот для суммирования амплитуд, первая яч. должна быть ноль для выбора данных массива начиная с яч.1
extern uint8_t DMAend;                                       //флаг прерываний DMA
uint8_t count_cycle, countdpoint = 0;                        //счётчик циклов анализа, счетчик "мёртвых точек"
uint8_t dpoint = 2;                                          //количество мертвых точек в механизме поиска максимумов
uint16_t candidate = 0;                                      //кандидат на определившийся максимум;
uint8_t n_cycle = 20;                                        //польз.уст. кол-ва циклов анализа
uint8_t mode = 3;                                            // режимы работы основного цикла; 1-сумма амплитуд гармоник в диапазонах, 2-кол-во максимумов амплитуд гармоник
//int32_t av_param[128];
//int32_t av_param_result[128];
//extern int32_t fft(uint16_t *massiv);
float In[base_len*2];                                        //входной массив анализа FFT
//short InShort[base_len*2];
//short Out[base_len*2];                                     //выходной массив анализа FFT, яч. соотв.: 0 - реальное число, 1 - мнимое и т.д. для 2,3; 4,5 ... Аналогично и входной массив
float full_analysis[base_len];                               //готовый результат после амплитудного анализа
uint8_t count_max_freq[base_len/2];                          //массив для подсчета максимумов в соответствии с ячейками амплитудного анализа
uint8_t count_max_freq_save[base_len/2];
size_t fre;
float full_analysis_save_param = 0;                          //переменная для механизма поиска максимумов
uint8_t busy = 0;
uint8_t average_factor = 5;
uint16_t button_indicate = 0;
//uint16_t ResultInject[3];
float VoltSource, VoltTenso, mVoltPerVolt, mVoltPerVolt_prev, Ftenso, InstZero = 0;
float VoltPerBit = 0.0008301691405;                          //ADC
uint8_t countFtensoAv = 0;
float FtensoAv[5];
float Ftenso_r = 0;
volatile float tensoX1 = 0;
volatile float FtensoX1 = 0;
volatile float tensoX2 = 0;
volatile float FtensoX2 = 0;
volatile uint8_t run_first = 1;
uint8_t calibration_tenso = 0;
uint8_t saveSettings = 77;
uint8_t blocked = 1;
float ResultPowerTenzo_r, ResultTenzo_r;
float ResultVref_r = 1500;
volatile uint32_t ResultVref, ResultPowerTenzo, ResultTenzo = 0;
//float InsertTemp, ResultTemp = 0;
//float V25 = 1.41;
//float avg_slope = 0.0043;
volatile uint16_t continous_i;
const uint16_t averageTenzo = 1000;
volatile uint8_t already_started = 0;
volatile uint8_t count_empty_cycles = 0;
float temperature = 0;
uint8_t thermoCompWt = 1;
uint16_t discrete = 10;
volatile float Vtenso_k = 10;
volatile float Vtenso_b = -3.3;
volatile float Thermocomp_k = 1;
volatile float Thermocomp_b = 1;
volatile float Ftenso_exp_factor = 0.5;
//float testf = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC2_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
__STATIC_INLINE void delay_us(__IO uint32_t micros)
{
	micros *= (SystemCoreClock / 1000000)/10;
	/* Wait till done */
	while (micros--) ;
}
void WriteToFlash(uint32_t addr, float data) 
{
	float dataSave = data;
	HAL_FLASH_Unlock();
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, *(uint64_t *)&dataSave);
	HAL_FLASH_Lock();
}
void FlashErase(uint32_t addr) 
{
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PageError = 0;
	HAL_FLASH_Unlock();
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = addr;
	EraseInitStruct.NbPages = 1;
	HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
	HAL_FLASH_Lock();
}
float FlashRead(uint32_t address) 
{
	uint8_t data[4];
	/*data[0] = *(__IO uint8_t*)address;
	data[1] = *(__IO uint8_t*)(address+1);
	data[2] = *(__IO uint8_t*)(address+2);
	data[3] = *(__IO uint8_t*)(address+3);*/
	for (uint8_t i=0;i<4;i++) 
	{
		data[i] = *(__IO uint8_t*)(address+i);
		HAL_Delay(1);
	}
    return (*(__IO float *)(&data));
}

extern void ModbusRTUTask(void const * argument); //задача Modbus из mbtask.c

//функция поиска максимумов
void max_search(void) 
{ 
	for (uint16_t i=1; i<base_len/2; i++) 
	{
		if (full_analysis[i] > full_analysis_save_param) 
		{
			candidate = i;
			countdpoint = 0;
			full_analysis_save_param = full_analysis[i];
		}
		else 
		{
			countdpoint++;
			full_analysis_save_param = full_analysis[i];
			if (candidate > 0 && countdpoint == dpoint) 
			{
			count_max_freq[candidate]++;
			candidate = 0;
			}
		}
	}
}
void max_search_reset(void) 
{
	candidate = 0;
	countdpoint = 0;
	full_analysis_save_param = 0;
}

//функция запуска сбора данных с АЦП (запуск ДМА и ТИМ)
void start_timdma(void) 
{
  if(HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) Error_Handler();
  HAL_ADCEx_Calibration_Start(&hadc1);
  if(HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&ADC_value,base_len) != HAL_OK) Error_Handler();
}

//функция останова сбора данных с АЦП (стоп ДМА и ТИМ)
void stop_timdma(void)
{ 
	HAL_ADC_Stop_DMA(&hadc1);
	//HAL_TIM_Base_Stop_IT(&htim4);
	HAL_TIM_Base_Stop_IT(&htim3);
}

//функция перевода из int32 в uint8
void u8from32 (uint8_t b[4], int32_t u32) 
{
    b[3] = (uint8_t)u32;
    b[2] = (uint8_t)(u32>>=8);
    b[1] = (uint8_t)(u32>>=8);
    b[0] = (uint8_t)(u32>>=8);
}

//функция Сумма ячеек (старт.яч., кон.яч.,буффер)
float sum_cells(uint8_t start, uint8_t end, float *buffer, float summ) 
{
	float sum_cell=0;
	//стартовая ячейка start - из основного алгоритма (предыдущая конечная +1)
	for (uint8_t i=start; i<end+1; i++)
	{
		sum_cell+=buffer[i]; //каждое значение суммируем в sum_cell
	}
	sum_cell+=summ;
	return sum_cell;
}
void sound_analysis(void const * argument) 
{
	HAL_ADCEx_Calibration_Start(&hadc1);
	start_timdma();
	while(1) 
	{
	//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	//массив с данными АЦП готов
		if (DMAend == 1) 
		{
			DMAend = 0;
			if (mode == 3||mode == 5) 
			{
				for (int i=0; i<base_len; i++) 
				{
					In[i*2]=ADC_value[i];  //реальную часть заполняем данными с АЦП
					//In[i*2]=0;           //для проверки, если раскомментировать, то результат БПФ - нули (это значит, что он работает верно)
					In[i*2+1]=0;           //мнимую - нулями
				}
				arm_cfft_f32(&arm_cfft_sR_f32_len256,In,0,1);
				arm_cmplx_mag_f32(In,full_analysis,base_len);
			}
			//режим 1 - сумма амплитуд заданных диапазонов ячеек
			if (mode == 1) 
			{ 
				for (uint8_t i=0; i<11; i++)
				{
					sum_cells_value[i]=sum_cells((value[i]+1),value[i+1],full_analysis,sum_cells_value[i]); //sum_cells пример:(значение параметра[0] +1, занчение параметра[1], массив с готовыми данными)
				}
			}
			if (mode == 2) 
			{
				max_search();
			}
			//режим 3 - сумма амплитуд каждой ячейки
			if (mode >= 3) 
			{
				for (uint8_t i=1; i<base_len/2; i++) 
				{
					sum_cells_value[i]=sum_cells(i,i,full_analysis,sum_cells_value[i]);
				}
			}
			count_cycle++;
			//в случае с "больше" - то это надо тогда, когда вдруг от юзера придёт уставка n_cycle меньше, а count_cycle уже перешагнёт эту уставку
			if (count_cycle == n_cycle || count_cycle > n_cycle) 
			{ 
				  /*for (uint8_t i=0; i<128; i++) {
					  av_param_result[i] = av_param[i]/n_cycle; //сумму делим на количество циклов, т.о. усредняем результат за 1с.
					  av_param[i] = 0; //обнуляем ячейки с суммами
				  }*/
				count_cycle = 0;
				if (mode == 1) 
				{
					for (uint8_t i=0; i<11; i++) 
					{
						sum_cells_value_save[i]=sum_cells_value[i];    //сохраняем полученные результаты сумм в другую переменную
						sum_cells_value[i] = 0;                        // обнуляем переменную с результатами*/
						/*u8from32(dataSend, sum_cells_value_save[i]); //переводим ячейки полученного результата в uint8 (для COM)
						HAL_Delay(10); //задержка 10мс*/
					}
				}
				if (mode == 2) 
				{
					for (uint16_t i=0; i<base_len/2; i++) 
					{
						count_max_freq_save[i] = count_max_freq[i];
						count_max_freq[i] = 0;
						max_search_reset();
					}
				}
				if (mode == 3||mode == 4) 
				{
					busy=1;
					arm_copy_f32(sum_cells_value,sum_cells_value_save,base_len/2);
					arm_fill_f32(0,sum_cells_value,base_len/2);
					busy=0;
				}
				if (mode == 5||mode == 6) 
				{
					busy=1;
					for (uint8_t i=0; i<base_len/2; i++) 
					{
						sum_cells_value_save[i] = (sum_cells_value_save[i] * (average_factor - 1) + sum_cells_value[i]) / average_factor;
					}
					arm_fill_f32(0,sum_cells_value,base_len/2);
					busy=0;
				}
			}
			start_timdma(); //запускаем опрос АЦП
		}
	}
}
void Tenso(void const * argument) {
	for(;;) 
	{
		if (continous_i < averageTenzo && already_started == 0) 
		{
			HAL_ADCEx_Calibration_Start(&hadc2);
			HAL_ADCEx_InjectedStart_IT(&hadc2);
			already_started = 1;
		}
		else 
		{
			count_empty_cycles++;
			if (count_empty_cycles == 2) already_started = 0;
		}
		if (continous_i == averageTenzo) 
		{
			HAL_ADCEx_InjectedStop_IT(&hadc2);
			busy = 1;
			continous_i = 0;
			HAL_ADCEx_Calibration_Start(&hadc1);
			HAL_ADCEx_InjectedStart(&hadc1);
			HAL_ADCEx_InjectedPollForConversion(&hadc1, 1);
			ResultVref = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);//Резльтат АЦП инжектированного канала 1 - значение опорного 1,2В
			if (ResultVref>1490) 
			{
			//ResultVref_r = ((ResultVref_r*19)+ResultVref)/20;
			ResultVref_r = ResultVref;
			//VoltPerBit = (1.2/ResultVref);
			}
			ResultPowerTenzo_r = (float)ResultPowerTenzo/averageTenzo;
			ResultTenzo_r = (float)ResultTenzo/averageTenzo;
			ResultVref = 0;
			ResultPowerTenzo = 0;
			ResultTenzo = 0;
			//VoltPerBit = (1.2/ResultVref_r);//1.00895;
			//ResultTemp = VoltPerBit*HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
			//InsertTemp = ((V25-ResultTemp)/avg_slope)+25;           //вычисляем температуру встроенного датчика.
			VoltSource = VoltPerBit*ResultPowerTenzo_r*3.844906445;   //находим U питания тензо. В конце к-т делителя питания тензо на ОУ
			//VoltTenso = (VoltPerBit*ResultTenzo_r)/100.66;          //110 - аппаратный к-т усиления на ОУ 30мВ-->3.3В
			VoltTenso = Vtenso_k*(VoltPerBit*ResultTenzo_r)+Vtenso_b; //уравнение прямой f(x) построенной по х-ке выхода платы тензо 30мВ-->3.3В. Результат мВ с тензо.
			//mVoltPerVolt = (VoltTenso/VoltSource)*1000;
			mVoltPerVolt = (VoltTenso/VoltSource); //Получаем мВ/В
			if (thermoCompWt == 1) 
			{
				mVoltPerVolt = mVoltPerVolt-(Thermocomp_k*temperature+Thermocomp_b);
			}
			mVoltPerVolt = Ftenso_exp_factor*mVoltPerVolt+(1-Ftenso_exp_factor)*mVoltPerVolt_prev;  //использована одна из реализаций экспоненциального фильтра
			mVoltPerVolt_prev = mVoltPerVolt;
			Ftenso_r = FtensoX1 + (mVoltPerVolt-tensoX1)*((FtensoX2-FtensoX1)/(tensoX2-tensoX1));
			Ftenso_r = Ftenso_r - InstZero;
			Ftenso = (roundf(Ftenso_r*discrete))/discrete;                                          //округляем к параметру дискретности
			busy = 0;
		}
		if (run_first == 1) 
		{
			HAL_GPIO_TogglePin(GPIOB, LED_MAX_PB11_Pin);
			tensoX1 = FlashRead(FLASH_VAR_X1);
			FtensoX1 = FlashRead(FLASH_VAR_FX1);
			tensoX2 = FlashRead(FLASH_VAR_X2);
			FtensoX2 = FlashRead(FLASH_VAR_FX2);
			Vtenso_k = FlashRead(FLASH_VTENSO_K);
			Vtenso_b = FlashRead(FLASH_VTENSO_B);
			Thermocomp_k = FlashRead(FLASH_THERMOCOMP_K);
			Thermocomp_b = FlashRead(FLASH_THERMOCOMP_B);
			HAL_Delay(1000);
			run_first = 0;
		}
		if (calibration_tenso == 1) 
		{
			blocked = 0;
			switch (saveSettings) 
			{
				case 0:                     //установить нулевую точку
					blocked = 1;
					tensoX1 = mVoltPerVolt;
					saveSettings = 100;
					break;
				case 1:                     //установить вторую точку
					blocked = 1;
					tensoX2 = mVoltPerVolt;
					saveSettings = 111;
					break;
				case 5:                     //прочитать данные из флеш
					run_first = 1;
					tensoX1 = FlashRead(FLASH_VAR_X1);
					FtensoX1 = FlashRead(FLASH_VAR_FX1);
					tensoX2 = FlashRead(FLASH_VAR_X2);
					FtensoX2 = FlashRead(FLASH_VAR_FX2);
					Vtenso_k = FlashRead(FLASH_VTENSO_K);
					Vtenso_b = FlashRead(FLASH_VTENSO_B);
					Thermocomp_k = FlashRead(FLASH_THERMOCOMP_K);
					Thermocomp_b = FlashRead(FLASH_THERMOCOMP_B);
					saveSettings = 55;
					blocked = 1;
					HAL_Delay(1000);
					run_first = 0;
					break;
				case 7:                     //установка нуля
					blocked = 1;
					InstZero = Ftenso;
					saveSettings = 70;
					break;
				case 8:                     //сброс InstZero - установки нуля
					blocked = 1;
					InstZero = 0;
					saveSettings = 80;
					break;
				case 10:                    //записать данные из ОЗУ во Флеш
					blocked = 1;
					FlashErase(FLASH_VAR_X1);                         //стираем всю страницу флеш
					WriteToFlash(FLASH_VAR_X1, tensoX1);              //пишем флеш
					WriteToFlash(FLASH_VAR_FX1, FtensoX1);
					WriteToFlash(FLASH_VAR_X2, tensoX2);
					WriteToFlash(FLASH_VAR_FX2, FtensoX2);
					WriteToFlash(FLASH_VTENSO_K, Vtenso_k);
					WriteToFlash(FLASH_VTENSO_B, Vtenso_b);
					WriteToFlash(FLASH_THERMOCOMP_K, Thermocomp_k);
					WriteToFlash(FLASH_THERMOCOMP_B, Thermocomp_b);
					run_first = 1;
					tensoX1 = FlashRead(FLASH_VAR_X1);                //читаем из флеш
					FtensoX1 = FlashRead(FLASH_VAR_FX1);
					tensoX2 = FlashRead(FLASH_VAR_X2);
					FtensoX2 = FlashRead(FLASH_VAR_FX2);
					Vtenso_k = FlashRead(FLASH_VTENSO_K);
					Vtenso_b = FlashRead(FLASH_VTENSO_B);
					Thermocomp_k = FlashRead(FLASH_THERMOCOMP_K);
					Thermocomp_b = FlashRead(FLASH_THERMOCOMP_B);
					saveSettings = 222;
					HAL_Delay(1000);
					run_first = 0;
					break;
			}
		}
		osDelay(500);
	}
}

void LED_ReadyOSstate(void const * argument) {
	for(;;) 
	{
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		ds18b20_init_seq();
		ds18b20_send_rom_cmd(SKIP_ROM_CMD_BYTE);
		ds18b20_send_function_cmd(CONVERT_T_CMD);
		delay_us(100);
		ds18b20_init_seq();
		ds18b20_send_rom_cmd(SKIP_ROM_CMD_BYTE);
		ds18b20_send_function_cmd(READ_SCRATCHPAD_CMD);
		temperature = ds18b20_read_temp();
		osDelay(1000);
	}
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  new_disc_period = discrete_period;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  fre=xPortGetFreeHeapSize();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(sound_analysis, sound_analysis, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);             //задача для анализа звука
  osThreadCreate(osThread(sound_analysis), NULL);
  fre=xPortGetFreeHeapSize();
  osThreadDef(ModbusRTUTask, ModbusRTUTask, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);               //задача Modbus
  osThreadCreate(osThread(ModbusRTUTask), NULL);
  fre=xPortGetFreeHeapSize();
  osThreadDef(Tenso, Tenso, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);                               //задача моргания светодиодом
  osThreadCreate(osThread(Tenso), NULL);
  fre=xPortGetFreeHeapSize();
  osThreadDef(LED_ReadyOSstate, LED_ReadyOSstate, osPriorityAboveNormal, 0, configMINIMAL_STACK_SIZE);    //задача моргания светодиодом
  osThreadCreate(osThread(LED_ReadyOSstate), NULL);
  fre=xPortGetFreeHeapSize();
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;
  ADC_InjectionConfTypeDef sConfigInjected;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Injected Channel 
    */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_VREFINT;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* ADC2 init function */
static void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;
  ADC_InjectionConfTypeDef sConfigInjected;

    /**Common config 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Injected Channel 
    */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedNbrOfConversion = 2;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Injected Channel 
    */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_5;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3599;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_MAX_PB11_Pin|DERE_PB12_Pin|DS18B20_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_PB1_Pin */
  GPIO_InitStruct.Pin = BUTTON_PB1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_PB1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_MAX_PB11_Pin DERE_PB12_Pin */
  GPIO_InitStruct.Pin = LED_MAX_PB11_Pin|DERE_PB12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DS18B20_Pin */
  GPIO_InitStruct.Pin = DS18B20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(DS18B20_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc) 
{
	continous_i++;
	ResultPowerTenzo = ResultPowerTenzo+(HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1)); //Результат АЦП инжектированного канала 2 - зачение напряжения питания тензо.
	ResultTenzo = ResultTenzo+(HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2));           //Результат АЦП инжектированного канала 3 - значение тензо.
	already_started = 0;
	count_empty_cycles = 0;
	if (continous_i<averageTenzo) 
	{
		HAL_ADCEx_Calibration_Start(&hadc2);
		HAL_ADCEx_InjectedStart_IT(&hadc2);
		already_started = 1;
	}
}
/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	if (new_disc_period != discrete_period && new_disc_period >= 499 && new_disc_period > 0)
	{
			discrete_period = new_disc_period;
			MX_TIM3_Init();
			stop_timdma();
			osDelay(10);
			DMAend=0;
			start_timdma();
	}
	//для настройки микрофона, если значение АЦП превышает заданное
	if (ADC_value[0]>=4090)
	{ 

		HAL_GPIO_WritePin(GPIOB, LED_MAX_PB11_Pin, GPIO_PIN_SET); //зажигаем светодиод
		osDelay(50);
	}
	else HAL_GPIO_WritePin(GPIOB, LED_MAX_PB11_Pin, GPIO_PIN_RESET);
	if (HAL_GPIO_ReadPin(BUTTON_PB1_GPIO_Port, BUTTON_PB1_Pin) != 1) 
	{
		button_indicate = 0xFFFF;
		HAL_GPIO_WritePin(GPIOB, LED_MAX_PB11_Pin, GPIO_PIN_SET);
		osDelay(50);
		HAL_GPIO_WritePin(GPIOB, LED_MAX_PB11_Pin, GPIO_PIN_RESET);
	}
	else button_indicate = 0;
  }
  /* USER CODE END 5 */ 
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) 
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
