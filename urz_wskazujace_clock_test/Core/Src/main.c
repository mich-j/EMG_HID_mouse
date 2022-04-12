/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_hid.h"
#include "math.h"
#include "BNO055_registers.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define SCREEN_RES_X 1920
#define SCREEN_RES_Y 1080

#define DEBUG 1
#define MOUSE_CONFIG_WAIT_TIME 3000
#define RMS_THRES 70
#define RMS_MAX 500

#define EMG_ZERO_LEVEL 2000

#define RMS_FRAME_LEN 60

#define USE_USB_FS 1

typedef struct {
	uint8_t button;
	int8_t cursor_X;
	int8_t cursor_Y;
	int8_t wheel;
} USBmouse;

typedef struct {
	uint8_t sys;
	uint8_t gyr;
	uint8_t acc;
	uint8_t mag;
} CalibrationData;

typedef struct {
	float_t X;
	float_t Y;
	float_t Z;
	float_t w;
} Angles;

typedef struct {
	uint8_t X;
	uint8_t Y;
} Pixels;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim1;
ADC_HandleTypeDef hadc1;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define I2C_TIMEOUT 1000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
USBD_HandleTypeDef mouseDevice;
extern USBD_HandleTypeDef hUsbDeviceFS;
USBmouse mouse = { 0, 0, 0, 0 };
USBmouse mouse_prev = { 0, 0, 0, 0 };

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char buf[64];
volatile uint16_t emg[1];
int16_t data_raw[RMS_FRAME_LEN];
float data_rms = 0;
uint8_t counter;
uint8_t click_flg = 0;

uint8_t error_code;
uint8_t test;
uint8_t usb_status;
uint8_t calib_status;
uint8_t tim6_flg = 0;
uint8_t adc_flg = 0;

Angles Euler;
Angles Euler_prev = { 0, 0, 0 };
Angles bound_min = { -40, -6, 0 };
Angles bound_max = { 40, 30, 0 };
Angles Quaternion;

Pixels px_per_deg = { 40, 40 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

void BNO_I2C_Write(uint16_t address, uint8_t data) {
	uint8_t buf = data;
	uint8_t err;

	do {
		err = HAL_I2C_Mem_Write(&hi2c1, BNO_ADDRESS, address, 1, &buf,
				sizeof(buf),
				I2C_TIMEOUT);
	} while (err != HAL_OK);

}

uint8_t BNO_I2C_Read(uint16_t address) {
	uint8_t read;
	if (hi2c1.State == HAL_I2C_STATE_READY) {
		HAL_I2C_Mem_Read(&hi2c1, BNO_ADDRESS, address, 1, &read, 1,
		I2C_TIMEOUT); //funkcja HAL_I2C_Mem_Read wysyła do urządzenia podrzędnego adres rejestru do odczytania, a następnie odbiera wysłane dane i przekazuje do zmiennej read
	}

	return read;
}

uint8_t BNO_Init(void) {
	HAL_GPIO_WritePin(INFO_LED_1_GPIO_Port, INFO_LED_1_Pin, GPIO_PIN_SET);
	uint8_t check = 0;

	check = BNO_I2C_Read(CHIP_ID); //odczytanie identyfikatora

	BNO_I2C_Write(BNO_OPR_MODE, CONFIGMODE); //przejście do trybu konfiguracji
	BNO_I2C_Write(BNO_SYS_TRIGGER, EXT_CLK_SEL); //wybór zewnętrznego oscylatora
	BNO_I2C_Write(BNO_AXIS_MAP_CONFIG, 0b011000); //zamiana osi
	BNO_I2C_Write(BNO_AXIS_MAP_SIGN, 0b100);
	BNO_I2C_Write(BNO_OPR_MODE, IMUMODE); //przejście do trybu NDOF (Nine Degrees Of Freedom) - jednoczesna praca akcelerometru, żyroskopu, magnetometru.

	if (BNO_I2C_Read(BNO_OPR_MODE) != IMUMODE) {
		Error_Handler();
	}

	HAL_GPIO_WritePin(INFO_LED_1_GPIO_Port, INFO_LED_1_Pin, GPIO_PIN_RESET);

	return check;
}

Angles BNO_GetEuler(void) {
	uint8_t data[6];

	HAL_I2C_Mem_Read(&hi2c1, BNO_ADDRESS, EUL_DATA_X_LSB, 1, (uint8_t*) &data,
			6,
			I2C_TIMEOUT);
	Angles Eul;
	int16_t X, Y, Z;

	X = ((data[1] << 8) | data[0]);
	Y = ((data[3] << 8) | data[2]);
	Z = ((data[5] << 8) | data[4]);

	float_t X_temp = X / EUL_LSB;

	if (X_temp >= 180.0) {
		Eul.X = X_temp - 360.0; // przejście z systemu (0, 360) na (-180, 180)
	} else {
		Eul.X = X_temp;
	}

	Eul.Y = Y / EUL_LSB;
	Eul.Z = Z / EUL_LSB;

	return Eul;
}

Angles BNO_GetQuaternions(void) {
	uint8_t data[8];
	HAL_I2C_Mem_Read(&hi2c1, BNO_ADDRESS, QUA_DATA_W_LSB, 1, (uint8_t*) &data,
			8,
			I2C_TIMEOUT);

	Angles Qua;
	int16_t w, X, Y, Z;
	w = ((data[1] << 8) | data[0]);
	X = ((data[3] << 8) | data[2]);
	Y = ((data[5] << 8) | data[4]);
	Z = ((data[7] << 8) | data[6]);

	Qua.w = w / (float_t) QUA_LSB;
	Qua.X = X / (float_t) QUA_LSB;
	Qua.Y = Y / (float_t) QUA_LSB;
	Qua.Z = Z / (float_t) QUA_LSB;

	return Qua;
}

Angles ToEuler(Angles qua) {
	Angles eul;

	// roll (x-axis rotation)
	float_t sinr_cosp = 2 * (qua.w * qua.X + qua.Y * qua.Z);
	float_t cosr_cosp = 1 - 2 * (qua.X * qua.X + qua.Y * qua.Y);
	eul.X = atan2f(sinr_cosp, cosr_cosp) * 57.295;

	// pitch (y-axis rotation)
	float_t sinp = 2 * (qua.w * qua.Y - qua.Z * qua.X);
	if (abs(sinp) >= 1)
		eul.Y = copysignf(M_PI / 2, sinp) * 57.295; // use 90 degrees if out of range
	else
		eul.Y = asinf(sinp) * 57.295;

	// yaw (z-axis rotation)
	float_t siny_cosp = 2 * (qua.w * qua.Z + qua.X * qua.Y);
	float_t cosy_cosp = 1 - 2 * (qua.Y * qua.Y + qua.Z * qua.Z);
	eul.Z = atan2f(siny_cosp, cosy_cosp) * 57.295;

	return eul;
}
void BNO_CheckCalibration(uint8_t calib_thres) {
	/* Funkcja sprawdzająca status kalibracji akcelerometru, żyroskopu i magnetometru
	 * Liczba 3 (binarnie 11) oznacza pełną kalibrację komponentu, 0 - konieczność dokonania kalibracji
	 * Funkcja wstrzymuje działanie programu, jeżeli komponenty nie są skalibrowane zgodnie z progiem przekazanym w argumencie funkcji
	 */
//	__disable_irq(); //globalne wyłączenie przerwań
	uint8_t status = 0;

	do {
		CalibrationData calib;
		uint8_t data = BNO_I2C_Read(CALIB_STAT);
		calib.sys = (data >> 6) & 0b11;
		calib.gyr = (data >> 4) & 0b11;
		calib.acc = (data >> 2) & 0b11;
		calib.mag = data & 0b11;

		if ((calib.gyr >= calib_thres) && (calib.acc >= calib_thres)) {
			status = 1;
			HAL_GPIO_WritePin(INFO_LED_1_GPIO_Port, INFO_LED_1_Pin, GPIO_PIN_RESET);
		} else {
			status = 0;
			HAL_GPIO_WritePin(INFO_LED_1_GPIO_Port, INFO_LED_1_Pin, GPIO_PIN_SET); //jeśli komponenty nie są skalibrowane, zaświeć diodą
		}
//		__enable_irq();

		HAL_Delay(10);
//		__disable_irq();
	} while (status == 0);
//	__enable_irq(); //globalne włączenie przerwań
}

//uint16_t BNO_SaveOffsets(void)
//{
// SAVE_TO_EEPROM;
//}
//
//void BNO_RestoreOffsets(void)
//{
// READ_FROM_EEPROM;
//}

uint8_t HID_SendReport(USBmouse device) {
	uint8_t status = USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*) &device,
			sizeof(device));
	return status;
}

int16_t map_values(int16_t x, int16_t min_in, int16_t max_in, int16_t min_out,
		int16_t max_out) {
	return (x - min_in) * (max_out - min_out) / (max_in - min_in) + min_out;
}

void BNO_ResetSystem(void) {
	/* W trybie IMU zresetowanie BNO055 powoduje ustalenie początkowego układu współrzędnych na nowo */

	BNO_I2C_Write(BNO_SYS_TRIGGER, RST_SYS);
	HAL_Delay(800); //czas potrzebny na dokonanie POST (Power On Self Test);_
}

void Mouse_SetBoundaries(void) {

	bound_min.X = Euler.X;
	HAL_Delay(MOUSE_CONFIG_WAIT_TIME);

	bound_max.X = Euler.X;
	HAL_Delay(MOUSE_CONFIG_WAIT_TIME);

	bound_min.Y = Euler.Y;
	HAL_Delay(MOUSE_CONFIG_WAIT_TIME);

	bound_max.Y = Euler.Y;
	HAL_Delay(MOUSE_CONFIG_WAIT_TIME);

	px_per_deg.X = SCREEN_RES_X / (bound_max.X - bound_min.X);
	px_per_deg.Y = SCREEN_RES_Y / (bound_max.Y - bound_min.Y);
}

void Mouse_ConfigMode() {
	Mouse_SetBoundaries();

}


void EnableFPU(void) {
	SCB->CPACR |= ((3UL << 10 * 2) | /* set CP10 Full Access */
	(3UL << 11 * 2)); /* set CP11 Full Access */

}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) { //timer ustawia tylko flagę, aby wykonać w przerwaniu jak najmniej instrukcji
	if (htim->Instance == TIM6) {
		tim6_flg = 1;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	// przesuwanie wierszy w tabeli o 1
	adc_flg = 1;

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
}

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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

	if (BNO_Init() != 0xA0) { //sprawdzenie, czy identyfikator urządzenia jest zgodny - weryfikacja poprawności komunikacji między urządzeniami
		Error_Handler();
	}

	//inicjalizacja tablicy z pomiarami ADC zerami
	for (uint8_t i = 0; i < RMS_FRAME_LEN; i++) {
		data_raw[i] = 0;
	}

	BNO_CheckCalibration(1);
//	Mouse_SetBoundaries();
	EnableFPU();

	HAL_TIM_Base_Start_IT(&htim6); // rozpoczęcie pracy timera odpowiedzialnego za przerwania
	HAL_TIM_Base_Start(&htim1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &emg, 1); //rozpoczęcie pracy przetwornika ADC w trybie DMA (bezpośredniego dostępu do pamięci z pominięciem CPU)

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		if (tim6_flg == 1) { //realizacja ruchów kursora, kliknięć, wysłanie raportu HID

			Euler = BNO_GetEuler();

			if (Euler.X == 0 && Euler.Y == 0 && Euler.Z == 0) //jeśli wszystkie zwracane kąty są równe zero, istnieje prawdopodobieństwo że czujnik uległ rozkalibrowaniu
					{
				BNO_CheckCalibration(1);
			}

			if (Euler.Y >= bound_max.Y + 15) {
				HAL_NVIC_SetPendingIRQ(EXTI9_5_IRQn);
			}

			mouse.cursor_X = ((Euler.X - Euler_prev.X) * px_per_deg.X); //obliczenie składowych wektora ruchu kursora w stopniach i przeliczenie ich na piksele
			mouse.cursor_Y = (Euler.Z - Euler_prev.Z) * px_per_deg.Y;

			HID_SendReport(mouse); //wysłanie raportu HID
			mouse.button = 0;

			Euler_prev = Euler;

			tim6_flg = 0;
		}

		if (adc_flg == 1) {
			for (uint8_t i = 0; i < RMS_FRAME_LEN - 1; i++) { //przesunięcie elementów tablicy z pomiarami
				data_raw[i] = data_raw[i + 1];
			}
			data_raw[RMS_FRAME_LEN - 1] = emg[0] - 1800;

			uint32_t square = 0;
			float_t mean = 0.0;
			float_t square_root = 0.0;

			/* Obliczenie RMS - średniej kwadratowej */
			for (uint8_t i = 0; i < RMS_FRAME_LEN; i++) {
				square += powf(data_raw[i], 2);
			}
			mean = (square / (float) RMS_FRAME_LEN);
			square_root = sqrtf(mean);
			data_rms = square_root;

			if(data_rms > RMS_THRES && data_rms < RMS_MAX) //jeżeli wartość RMS przekracza określoną wartość, czujnik prawdopodobnie nie jest założony poprawnie. Klikanie nie odbywa się
			{
				mouse.button = 1;
			}

			adc_flg = 0;
		}

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
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
  /** Enable the SYSCFG APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();
  /** Configures CRS
  */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1000);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
	while (1) {
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin); //w razie wystąpienia błędu mruga dioda LD3 na płytce Nucleo
		HAL_Delay(500);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
