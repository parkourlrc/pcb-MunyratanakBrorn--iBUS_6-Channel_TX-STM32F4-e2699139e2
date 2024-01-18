/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "IIRFilter.h"
#include "MY_NRF24.h"
#include "nRF24L01.h"
#include "stdbool.h"
#include "stdlib.h"
#include "MY_FLASH.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	uint16_t ch1;
	uint16_t ch2;
	uint16_t ch3;
	uint16_t ch4;

	uint16_t swa;
	uint16_t swb;
}TxBufferTypedef;  //Channel Tx


typedef struct {
	bool channel_reverse[4];
	bool aux_reverse[3];
}ReverseConfig_typedef ;

typedef struct{

	long min;
	long max;
	bool Is_Min_Cal;
	bool Is_Max_Cal;

}Calibrate_Typedef;

typedef struct{
	uint16_t mem_max[4];
	uint16_t mem_min[4];
}Calibrate_mem_typedef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IIR_FILTER_ALPHA_I		0.0001f
#define IIR_FILTER_ALPHA_II		0.00001f

#define CALIBRATE_SAMPLING		1000
#define MIN_VAL					1000
#define MAX_VAL					2000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint64_t TxpipeAddrs = 0x11223344AA;
uint8_t rf_channel_mem;
uint8_t rf_channel = 0;
bool AckPayload;

uint8_t toggle_sw[2];
TxBufferTypedef Tx_Buffer;

uint32_t adc_val[4];
uint16_t adc_filter_val_I[4];
uint16_t adc_filter_val_II[4];
uint16_t channel_filter_val[4];
IIRFilter adc_filter[4];
IIRFilter channel_filter[4];

uint16_t channel_map[4];
uint16_t channel_val[4];
uint8_t channel_index;

uint8_t btn_bind_state;
uint8_t btn_mode_state;

uint16_t calibrate_mem_max[4];
uint16_t calibrate_mem_min[4];

uint8_t calibrate_num;
int8_t offset_val[4];
int8_t sign[4]={1,1,1,1,};
Calibrate_Typedef calibrate[4];
Calibrate_mem_typedef buffer_write;
Calibrate_mem_typedef buffer_read;

uint32_t loop_timer;
bool Is_Bind = false;
bool Is_cal = false;
bool Op_Disable = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void switch_channel(){
	// switch A
	if ( toggle_sw[0] == 0 ) Tx_Buffer.swa = 1000;
	else Tx_Buffer.swa = 2000;

	// switch B
	if ( toggle_sw[1] == 0 ) Tx_Buffer.swb = 1000;
		else Tx_Buffer.swb = 2000;
}
void get_filter_value(){
	for ( uint8_t i=0; i<4; i++ ){
		adc_filter_val_I[i] = IIRFilter_Update(&adc_filter[i], adc_val[i]);
		adc_filter_val_II[i] = IIRFilter_Update(&adc_filter[i], adc_filter_val_I[i]);
		channel_filter_val[i] = IIRFilter_Update(&channel_filter[i], adc_filter_val_II[i]);
	}
}
void reverse(uint8_t channel_index){


	switch (channel_index) {

		case 0:
		   channel_map[0] = map(channel_filter_val[0], buffer_read.mem_min[0], buffer_read.mem_max[0], MIN_VAL, MAX_VAL)-5;  //YAW TX2
		   //channel_map[0] = map(channel_filter_val[0],buffer_read.mem_min[0],buffer_read.mem_max[0], MAX_VAL, MIN_VAL)-34;
			//channel_map[0] = map(channel_filter_val[0],5,4091, MAX_VAL, MIN_VAL)-34; //TX1
		   //channel_map[0] = map(channel_filter_val[0],buffer_read.mem_min[0],buffer_read.mem_max[0], MAX_VAL, MIN_VAL)+18; //TX3
		  // channel_map[0] = map(channel_filter_val[0],5, 4090, MIN_VAL, MAX_VAL)+18; //TX3
			//channel_map[0] = map(channel_filter_val[0],6, 4092, MIN_VAL, MAX_VAL)-19; //TX4
			//channel_map[0] = map(channel_filter_val[0],6, 4092, MIN_VAL, MAX_VAL); //TX5
			//channel_map[0] = map(channel_filter_val[0], buffer_read.mem_min[0], buffer_read.mem_max[0], MIN_VAL, MAX_VAL)-19;  //YAW TX4
			//channel_map[0] = map(channel_filter_val[0], buffer_read.mem_min[0], buffer_read.mem_max[0], MIN_VAL, MAX_VAL)-5;  //YAW TX5
			/****Left THU***/
			//channel_map[0] = map(channel_filter_val[0],5, 4092, MIN_VAL, MAX_VAL)-20; //TX1 Pitch
			//channel_map[0] = map(channel_filter_val[0],4, 4090, MIN_VAL, MAX_VAL)+7; //TX2 Pitch
			//channel_map[0] = map(channel_filter_val[0],buffer_read.mem_min[0],buffer_read.mem_max[0], MIN_VAL, MAX_VAL)-20; //TX1
		   break;

		case 1:
			//channel_map[1] = map(channel_filter_val[1],61,4049, MIN_VAL, MAX_VAL); //TX1
			//channel_map[1] = map(channel_filter_val[1], buffer_read.mem_min[1],buffer_read.mem_max[1], MIN_VAL, MAX_VAL);
			channel_map[1] = map(channel_filter_val[1], buffer_read.mem_min[1],buffer_read.mem_max[1], MIN_VAL, MAX_VAL); //TX2
			//channel_map[1] = map(channel_filter_val[1], buffer_read.mem_min[1],buffer_read.mem_max[1], MIN_VAL, MAX_VAL); //TX3
			//channel_map[1] = map(channel_filter_val[1],70, 4082, MIN_VAL, MAX_VAL); //TX3
			//channel_map[1] = map(channel_filter_val[1],10, 4091, MIN_VAL, MAX_VAL); //TX4
			//channel_map[1] = map(channel_filter_val[1],16, 4091, MIN_VAL, MAX_VAL); //TX5
			//channel_map[1] = map(channel_filter_val[1],buffer_read.mem_min[1],buffer_read.mem_max[1], MIN_VAL, MAX_VAL); //THR TX4
			//channel_map[1] = map(channel_filter_val[1],buffer_read.mem_min[1],buffer_read.mem_max[1], MIN_VAL, MAX_VAL); //THR TX5
			/****Left THU***/
			//channel_map[1] = map(channel_filter_val[1],90, 4090, MIN_VAL, MAX_VAL)+16; //TX1 Roll
			//channel_map[1] = map(channel_filter_val[1],13, 4090, MIN_VAL, MAX_VAL)+6; //TX2 Roll
			//channel_map[1] = map(channel_filter_val[1],buffer_read.mem_min[1],buffer_read.mem_max[1], MIN_VAL, MAX_VAL)+16; //TX1
			break;

		case 2:
			//channel_map[2] = map(channel_filter_val[2], 5, 4091, MIN_VAL, MAX_VAL)+7;//TX1
			//channel_map[2] = map(channel_filter_val[2],buffer_read.mem_min[2],buffer_read.mem_max[2], MIN_VAL, MAX_VAL)+7;
			channel_map[2] = map(channel_filter_val[2],buffer_read.mem_min[2],buffer_read.mem_max[2], MIN_VAL, MAX_VAL)-13; //TX2
			//channel_map[2] = map(channel_filter_val[2],5, 4091, MIN_VAL, MAX_VAL)-17; //TX3
			//channel_map[2] = map(channel_filter_val[2],5, 4090, MIN_VAL, MAX_VAL)-26; //TX4
			//channel_map[2] = map(channel_filter_val[2],4, 4091, MIN_VAL, MAX_VAL); //TX5
			//channel_map[2] = map(channel_filter_val[2],buffer_read.mem_min[2],buffer_read.mem_max[2], MIN_VAL, MAX_VAL)-17;//Pitch TX3
			//channel_map[2] = map(channel_filter_val[2],buffer_read.mem_min[2],buffer_read.mem_max[2], MIN_VAL, MAX_VAL)-26;//Pitch TX4
			//channel_map[2] = map(channel_filter_val[2],buffer_read.mem_min[2],buffer_read.mem_max[2], MIN_VAL, MAX_VAL)-8;//Pitch TX5
			/****Left THU***/
			//channel_map[2] = map(channel_filter_val[2],52, 4091, MIN_VAL, MAX_VAL)-6; //TX1 YAW
			//channel_map[2] = map(channel_filter_val[2],5, 4089, MIN_VAL, MAX_VAL)+9; //TX2 YAW
			//channel_map[2] = map(channel_filter_val[2],buffer_read.mem_min[2],buffer_read.mem_max[2], MIN_VAL, MAX_VAL)-6; //TX1
			break;
		case 3:
		  //channel_map[3] = map(channel_filter_val[3],7, 4090, MIN_VAL, MAX_VAL)-10;//TX1
		 // channel_map[3] = map(channel_filter_val[3],buffer_read.mem_min[3],buffer_read.mem_max[3], MIN_VAL, MAX_VAL)-10; //TX1 //10
		  channel_map[3] = map(channel_filter_val[3],buffer_read.mem_min[3],buffer_read.mem_max[3], MIN_VAL, MAX_VAL)-17; //TX2
		 // channel_map[3] = map(channel_filter_val[3],13, 4091, MIN_VAL, MAX_VAL)-10; //TX3
			//channel_map[3] = map(channel_filter_val[3],10, 4091, MIN_VAL, MAX_VAL)-20; //TX4
			//channel_map[3] = map(channel_filter_val[3],5, 4091, MIN_VAL, MAX_VAL); //TX5
			//channel_map[3] = map(channel_filter_val[3],buffer_read.mem_min[3],buffer_read.mem_max[3], MIN_VAL, MAX_VAL)-20; //TX4
			//channel_map[3] = map(channel_filter_val[3],buffer_read.mem_min[3],buffer_read.mem_max[3], MIN_VAL, MAX_VAL)-17; //TX5
			/****Left THU***/
			//channel_map[3] = map(channel_filter_val[3],80, 4092, MIN_VAL, MAX_VAL); //TX1 THR
			//channel_map[3] = map(channel_filter_val[3],buffer_read.mem_min[3],buffer_read.mem_max[3], MIN_VAL, MAX_VAL); //TX1
			//channel_map[3] = map(channel_filter_val[3],82, 4092, MIN_VAL, MAX_VAL); //TX2 THR
		  break;
		default:
			break;
	}

}

void channel_default(uint8_t channel_index){

	switch (channel_index) {

		case 0:
			//channel_map[0] = map(channel_filter_val[0],5, 4091, MIN_VAL, MAX_VAL)-34;//TX1
			//channel_map[0] = map(channel_filter_val[0], buffer_read.mem_min[0], buffer_read.mem_max[0], MIN_VAL, MAX_VAL)-34;  //YAW //TX1
			channel_map[0] = map(channel_filter_val[0], buffer_read.mem_min[0], buffer_read.mem_max[0], MIN_VAL, MAX_VAL)-5;  //YAW TX2
			//channel_map[0] = map(channel_filter_val[0],5, 4090, MIN_VAL, MAX_VAL)+18; //TX3
			//channel_map[0] = map(channel_filter_val[0], buffer_read.mem_min[0], buffer_read.mem_max[0], MIN_VAL, MAX_VAL)+18;  //YAW TX3
			//channel_map[0] = map(channel_filter_val[0],6, 4092, MIN_VAL, MAX_VAL)-19; //TX4
			//channel_map[0] = map(channel_filter_val[0],6, 4092, MIN_VAL, MAX_VAL)-5; //TX5
			//channel_map[0] = map(channel_filter_val[0], buffer_read.mem_min[0], buffer_read.mem_max[0], MIN_VAL, MAX_VAL)-19;  //YAW TX4
			//channel_map[0] = map(channel_filter_val[0], buffer_read.mem_min[0], buffer_read.mem_max[0], MIN_VAL, MAX_VAL)-5;  //YAW TX5
			/****Left THU***/
			//channel_map[0] = map(channel_filter_val[0],5, 4092, MIN_VAL, MAX_VAL)-20; //TX1 Pitch
			//channel_map[0] = map(channel_filter_val[0],4, 4090, MIN_VAL, MAX_VAL)+7; //TX2 Pitch
			//channel_map[0] = map(channel_filter_val[0],buffer_read.mem_min[0],buffer_read.mem_max[0], MIN_VAL, MAX_VAL)-20; //TX1
			break;
		case 1:
			//channel_map[1] = map(channel_filter_val[1],61, 4049, MIN_VAL, MAX_VAL);//TX1
			//channel_map[1] = map(channel_filter_val[1],buffer_read.mem_min[1],buffer_read.mem_max[1], MIN_VAL, MAX_VAL); //THR TX1
			channel_map[1] = map(channel_filter_val[1],buffer_read.mem_min[1],buffer_read.mem_max[1], MIN_VAL, MAX_VAL); //THR TX2
			//channel_map[1] = map(channel_filter_val[1],70, 4082, MIN_VAL, MAX_VAL); //TX3
			//channel_map[1] = map(channel_filter_val[1],buffer_read.mem_min[1],buffer_read.mem_max[1], MIN_VAL, MAX_VAL); //THR TX3
			//channel_map[1] = map(channel_filter_val[1],9, 4091, MIN_VAL, MAX_VAL); //TX4
			//channel_map[1] = map(channel_filter_val[1],16, 4091, MIN_VAL, MAX_VAL); //TX5
			//channel_map[1] = map(channel_filter_val[1],buffer_read.mem_min[1],buffer_read.mem_max[1], MIN_VAL, MAX_VAL); //THR TX4
			//channel_map[1] = map(channel_filter_val[1],buffer_read.mem_min[1],buffer_read.mem_max[1], MIN_VAL, MAX_VAL); //THR TX5
			/****Left THU***/
			//channel_map[1] = map(channel_filter_val[1],90, 4090, MIN_VAL, MAX_VAL)+16; //TX1 Roll
			//channel_map[1] = map(channel_filter_val[1],13, 4090, MIN_VAL, MAX_VAL)+6; //TX2 Roll
			//channel_map[1] = map(channel_filter_val[1],buffer_read.mem_min[1],buffer_read.mem_max[1], MIN_VAL, MAX_VAL)+16; //TX1
			break;
		case 2:
			//channel_map[2] = map(channel_filter_val[2],5, 4091, MIN_VAL, MAX_VAL)+7; //Pitch TX1
			//channel_map[2] = map(channel_filter_val[2],buffer_read.mem_min[2],buffer_read.mem_max[2], MIN_VAL, MAX_VAL)+7; //TX1
			channel_map[2] = map(channel_filter_val[2],buffer_read.mem_min[2],buffer_read.mem_max[2], MIN_VAL, MAX_VAL)-13;//Pitch TX2
			//channel_map[2] = map(channel_filter_val[2],5, 4091, MIN_VAL, MAX_VAL)-17; //TX3
			//channel_map[2] = map(channel_filter_val[2],buffer_read.mem_min[2],buffer_read.mem_max[2], MIN_VAL, MAX_VAL)-17;//Pitch TX3
			//channel_map[2] = map(channel_filter_val[2],5, 4090, MIN_VAL, MAX_VAL)-26; //TX4
			//channel_map[2] = map(channel_filter_val[2],4, 4091, MIN_VAL, MAX_VAL)-8; //TX5
			//channel_map[2] = map(channel_filter_val[2],buffer_read.mem_min[2],buffer_read.mem_max[2], MIN_VAL, MAX_VAL)-26;//Pitch TX4
			//channel_map[2] = map(channel_filter_val[2],buffer_read.mem_min[2],buffer_read.mem_max[2], MIN_VAL, MAX_VAL)-8;//Pitch TX5
			/****Left THU***/
			//channel_map[2] = map(channel_filter_val[2],52, 4091, MIN_VAL, MAX_VAL)-6; //TX1 YAW
			//channel_map[2] = map(channel_filter_val[2],5, 4089, MIN_VAL, MAX_VAL)+9; //TX2 YAW
			//channel_map[2] = map(channel_filter_val[2],buffer_read.mem_min[2],buffer_read.mem_max[2], MIN_VAL, MAX_VAL)-6; //TX1
		case 3:
		   // channel_map[3] = map(channel_filter_val[3],7, 4090, MIN_VAL, MAX_VAL)-10; //Roll TX1
			//channel_map[3] = map(channel_filter_val[3],buffer_read.mem_min[3],buffer_read.mem_max[3], MIN_VAL, MAX_VAL)-10; //TX1 //TX3
			channel_map[3] = map(channel_filter_val[3],buffer_read.mem_min[3],buffer_read.mem_max[3], MIN_VAL, MAX_VAL)-17; //TX2
			//channel_map[3] = map(channel_filter_val[3],13, 4091, MIN_VAL, MAX_VAL)-10; //TX3
			//channel_map[3] = map(channel_filter_val[3],10, 4091, MIN_VAL, MAX_VAL)-20; //TX4
			//channel_map[3] = map(channel_filter_val[3],5, 4091, MIN_VAL, MAX_VAL)-17; //TX5
			//channel_map[3] = map(channel_filter_val[3],buffer_read.mem_min[3],buffer_read.mem_max[3], MIN_VAL, MAX_VAL)-20; //TX4
			//channel_map[3] = map(channel_filter_val[3],buffer_read.mem_min[3],buffer_read.mem_max[3], MIN_VAL, MAX_VAL)-17; //TX5
			/****Left THU***/
			//channel_map[3] = map(channel_filter_val[3],80, 4092, MIN_VAL, MAX_VAL); //TX1 THR
			//channel_map[3] = map(channel_filter_val[3],82, 4092, MIN_VAL, MAX_VAL); //TX2 THR
			//channel_map[3] = map(channel_filter_val[3],buffer_read.mem_min[3],buffer_read.mem_max[3], MIN_VAL, MAX_VAL); //TX1
			break;
		default:
			break;
	}

	for (uint8_t i =0; i<4 ; i++ ){
		channel_val[i] = channel_map[i];
	}

}
void calibration_joystick(uint8_t index){

	  if ( Is_cal ){
		  if ( (adc_val[index] > 3500 && adc_val[index] < 4095 ) && calibrate[index].Is_Max_Cal == true ){
			  calibrate[index].max = 0;
			  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
			  HAL_Delay(1000);
			  for ( uint16_t i=0; i<CALIBRATE_SAMPLING; i++ ){
				  if ( i % 10 == 0 ) HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				  get_filter_value();
				  calibrate[index].max += adc_filter_val_I[index];
				  HAL_Delay(4);
			  }
			  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
			  calibrate[index].max /= CALIBRATE_SAMPLING;
			  buffer_write.mem_max[index] = calibrate[index].max;
			  calibrate[index].Is_Max_Cal = false;
			  calibrate_num++;

		  }
		  if ( (adc_val[index] > 0 && adc_val[index] < 500) && calibrate[index].Is_Min_Cal == true ) {
			  calibrate[index].min = 0;
			  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
			  HAL_Delay(1000);
			  for ( uint16_t i=0; i<CALIBRATE_SAMPLING; i++ ){
				  if ( i % 10 == 0 ) HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				  get_filter_value();
				  calibrate[index].min += adc_filter_val_I[index];
				  HAL_Delay(4);
			  }
			  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
			  calibrate[index].min /= CALIBRATE_SAMPLING;
			  buffer_write.mem_min[index] = calibrate[index].min;
			  calibrate[index].Is_Min_Cal = false;
			  calibrate_num++;
		  }
	  }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */

  //Write RF chan Value
MY_FLASH_SetSectorAddrs(4,0x08010000); //Write RF CHAN Value
MY_FLASH_ReadN(0,&rf_channel, sizeof(rf_channel), DATA_TYPE_8);
HAL_Delay(50);
  //READ CAL Value
MY_FLASH_SetSectorAddrs(5, 0x08020000); //Write CAL Value
MY_FLASH_ReadN(0,&buffer_read, sizeof(buffer_read), DATA_TYPE_8);
HAL_Delay(50);
HAL_ADC_Start_DMA(&hadc1, adc_val, 4);
HAL_Delay(50);

     for ( uint8_t i =0; i<4; i++ ){
   	  IIRFilter_Init(&adc_filter[i], IIR_FILTER_ALPHA_I);
   	  IIRFilter_Init(&channel_filter[i], IIR_FILTER_ALPHA_II);
     }

     for ( uint8_t i =0; i<4; i++ ){
       	  calibrate[i].Is_Max_Cal = true;
       	  calibrate[i].Is_Min_Cal = true;
         }

     ///nRF Stuffs///
        NRF24_begin(SPI1_CNS_GPIO_Port, SPI1_CNS_Pin, SPI1_CE_Pin, hspi1);
        NRF24_stopListening();
        NRF24_openWritingPipe(TxpipeAddrs);
        NRF24_setAutoAck(true);
        NRF24_setChannel(rf_channel);
       // NRF24_setChannel(81);
       // NRF24_setChannel(82);
       // NRF24_setChannel(83);
      // NRF24_setChannel(84);
       // NRF24_setChannel(87);
       // NRF24_setChannel(88);
       // NRF24_setChannel(90);
        NRF24_setPayloadSize(14);
        NRF24_enableAckPayload();
        NRF24_enableDynamicPayloads();
        HAL_Delay(50);

        HAL_TIM_Base_Start_IT(&htim10);
        HAL_TIM_Base_Start(&htim9);

        /////******Bind_Mode******//////
        btn_bind_state = HAL_GPIO_ReadPin(swa_GPIO_Port, swa_Pin);
         if ( btn_bind_state == 0 && adc_val[1]< 300 && adc_val[0]> 4000  ) Is_Bind = true;
         else Is_Bind = false;
         /////******Bind_Mode******////// Left THR
//              btn_bind_state = HAL_GPIO_ReadPin(swa_GPIO_Port, swa_Pin);
//               if ( btn_bind_state == 0 && adc_val[2]> 4080 && adc_val[3]< 60  ) Is_Bind = true;
//               else Is_Bind = false;
        //////******Operation disable******//// TX1
//         if (adc_val[1]>80 ||btn_mode_state ==1 ||btn_bind_state ==1) Op_Disable = true;
//         else Op_Disable = false;
        //////******Operation disable******//// TX2
         if (adc_val[1]>200 ||btn_mode_state ==1 ||btn_bind_state ==1) Op_Disable = true;
         else Op_Disable = false;
         //////******Operation disable******//// TX3
//                 if (adc_val[1]>100 ||btn_mode_state ==1 ||btn_bind_state ==1) Op_Disable = true;
//                 else Op_Disable = false;
         //////******Operation disable******//// TX4
                          //if (adc_val[1]>60 ||btn_mode_state ==1 ||btn_bind_state ==1) Op_Disable = true;
                         // else Op_Disable = false;
        //////******Operation disable******//// TX5
                          /* if (adc_val[1]>25 ||btn_mode_state ==1 ||btn_bind_state ==1) Op_Disable = true;
                              else Op_Disable = false;*/
               //////******Operation disable******//// TX1 Left THR
                          //  if (adc_val[3]>1500 ||btn_mode_state ==1 ||btn_bind_state ==1) Op_Disable = true;
                           // else Op_Disable = false;
               //////******Operation disable******//// TX2 Left THR
//                if (adc_val[3]>150 ||btn_mode_state ==1 ||btn_bind_state ==1) Op_Disable = true;
//                else Op_Disable = false;
       //////******JoyStick calibration******////TX1
//         btn_mode_state = HAL_GPIO_ReadPin(swb_GPIO_Port, swb_Pin);
//         if ( btn_mode_state ==0 && btn_bind_state == 0 && adc_val[2]<100 && adc_val[3]>4000 && adc_val[1]<80 ) Is_cal = true;
//         else Is_cal = false;

	//////******JoyStick calibration******////TX2
         btn_mode_state = HAL_GPIO_ReadPin(swb_GPIO_Port, swb_Pin);
         if (btn_mode_state ==0 && btn_bind_state ==0 && adc_val[2]<100 && adc_val[3]>4000 && adc_val[1]<170 ) Is_cal = true;
         else Is_cal = false;
	//////******JoyStick calibration******////TX3
//                   btn_mode_state = HAL_GPIO_ReadPin(swb_GPIO_Port, swb_Pin);
//                   if ( btn_mode_state ==0 && btn_bind_state == 0 && adc_val[2]<15 && adc_val[3]>4000 && adc_val[1]<80 ) Is_cal = true;
//                   else Is_cal = false;
	//////******JoyStick calibration******////TX5
	//btn_mode_state = HAL_GPIO_ReadPin(swb_GPIO_Port, swb_Pin);
	//if (btn_mode_state == 0 && btn_bind_state == 0 && adc_val[2] < 15
			//&& adc_val[3] > 4000 && adc_val[1] < 20)
		//Is_cal = true;
	//else
		//Is_cal = false;
            	//////******JoyStick calibration******////TX1 THR
//            	btn_mode_state = HAL_GPIO_ReadPin(swb_GPIO_Port, swb_Pin);
//            	if (btn_mode_state == 0 && btn_bind_state == 0 && adc_val[3] < 95 && adc_val[0] < 15 && adc_val[1] > 4080)
//            		Is_cal = true;
//            	else
//            		Is_cal = false;
//            	btn_mode_state = HAL_GPIO_ReadPin(swb_GPIO_Port, swb_Pin);
//            		if (btn_mode_state == 0 && btn_bind_state == 0 && adc_val[2] < 15
//            				&& adc_val[3] > 4000 && adc_val[1] < 20)
//            			Is_cal = true;
//            		else
//            			Is_cal = false;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  while (Is_Bind){
	 		  for ( uint8_t i=0; i<127; i++ ){
	 			  NRF24_setChannel(i);
	 				 HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	 				 HAL_Delay(50);
	 				 HAL_GPIO_TogglePin(Buzz_GPIO_Port, Buzz_Pin);
	 			  if ( NRF24_write(&Tx_Buffer,32) ){
	 				  NRF24_read(&AckPayload, 32);
	 			  }
	 			  HAL_Delay(10);
	 			  if ( AckPayload == true ){
	 				  rf_channel_mem = i;
	 				MY_FLASH_SetSectorAddrs(4,0x08010000);
	 				MY_FLASH_WriteN(0, &rf_channel_mem, sizeof(rf_channel_mem), DATA_TYPE_8);
	  			      HAL_Delay(50);
	 				  Is_Bind = false;
	 				 HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
	 				 HAL_GPIO_WritePin(Buzz_GPIO_Port, Buzz_Pin, 0);
	 				  break;
	 			  }

	 		  }

	 	  }

	  toggle_sw[0] = HAL_GPIO_ReadPin(swa_GPIO_Port, swa_Pin);
	  toggle_sw[1] = HAL_GPIO_ReadPin(swb_GPIO_Port, swb_Pin);

	while (Op_Disable){
	  		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  		  HAL_Delay(500);
	  		  HAL_GPIO_TogglePin(Buzz_GPIO_Port, Buzz_Pin);
	  		  btn_bind_state = HAL_GPIO_ReadPin(swa_GPIO_Port, swa_Pin);
	  		  HAL_Delay(25);
	  		 if(adc_val[1]< 80 && btn_mode_state ==0 && btn_bind_state ==0) { //Safe THR Detection
	  		  Op_Disable = false;
	  		 HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
	  		 HAL_GPIO_WritePin(Buzz_GPIO_Port, Buzz_Pin, 0);
	  		  break;
	  		 }
	  	  }//Unable to USE
//		while (Op_Disable){
//		  		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//		  		  HAL_Delay(500);
//		  		  HAL_GPIO_TogglePin(Buzz_GPIO_Port, Buzz_Pin);
//		  		  btn_bind_state = HAL_GPIO_ReadPin(swa_GPIO_Port, swa_Pin);
//		  		  HAL_Delay(25);
//		  		 if(adc_val[3]< 100 && btn_mode_state ==0 && btn_bind_state ==0) { //Safe THR Detection
//		  		  Op_Disable = false;
//		  		 HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
//		  		 HAL_GPIO_WritePin(Buzz_GPIO_Port, Buzz_Pin, 0);
//		  		  break;
//		  		 }
//		  	  }//Unable to USE Left THR

//	  	 HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
//	  	 HAL_GPIO_WritePin(Buzz_GPIO_Port, Buzz_Pin, 0);

		if (calibrate_num >= 8) {
			HAL_Delay(10);
			MY_FLASH_SetSectorAddrs(5, 0x08020000);
			MY_FLASH_WriteN(0, &buffer_write, sizeof(Calibrate_mem_typedef),DATA_TYPE_8);
			HAL_Delay(10);
			calibrate_num = 0;
			Is_cal = false;
		} else {
			for (uint8_t i = 0; i < 4; i++) {
				calibration_joystick(i);
			}
		}

	  	     get_filter_value();
	  	     switch_channel();
	  	     reverse(0);
	  	     reverse(1);
	  	     reverse(2);
	  	     reverse(3);
	  	     channel_default(0);
	  	     channel_default(1);
	  	     channel_default(2);
	  	     channel_default(3);

	  	   Tx_Buffer.ch3 = channel_val[1]; //THR
	  	   Tx_Buffer.ch4 = channel_val[0]; //YAW
	  	   Tx_Buffer.ch2 =channel_val[2];  //Pitch
	  	   Tx_Buffer.ch1 = channel_val[3]; //Roll

//	  	Tx_Buffer.ch3 = channel_val[3]; //THR
//		Tx_Buffer.ch4 = channel_val[2]; //YAW
//		Tx_Buffer.ch2 = channel_val[0];  //Pitch
//		Tx_Buffer.ch1 = channel_val[1]; //Roll

	  	  NRF24_write(&Tx_Buffer,sizeof(Tx_Buffer));
	  	  while ( __HAL_TIM_GET_COUNTER(&htim9) - loop_timer < 1000 );
	  	  loop_timer = __HAL_TIM_GET_COUNTER(&htim9);
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
