/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Estados da máquina
typedef enum {
	NORMAL, PEDESTRE, NOTURNO
} Estado;
typedef enum {
	E1, E2, E3, E4
} SFSM;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBOUNCE_DELAY_MS 50
// Definição do endereço do dispositivo escravo
#define ENDERECO_ESCRAVO 0x20 // Endereço do Arduino Nano (sem deslocamento)

// Definições dos estados dos LEDs
#define VERMELHO 0x04
#define AMARELO  0x02
#define VERDE    0x01

//Controle do Print
#define LCD 	1
#define SERIAL  2
#define BUFFER_SIZE 20  // Buffer para armazenar a string "HH:MM:SS DD/MM/YYYY"

// Defines para tempos configuráveis
#define TEMPO_VERDE 10     // Tempo em segundos para o verde
#define TEMPO_AMARELO 1    // Tempo em segundos para o amarelo
#define TEMPO_PEDESTRE 5   // Tempo da contagem regressiva
#define TEMPO_PISCAR 10   // Tempo em ms para o piscar do amarelo no modo noturno
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void EnviarComando(I2C_HandleTypeDef *hi2c, uint8_t semaforo, uint8_t estado);
void send_request_for_time(RTC_TimeTypeDef *sTime, RTC_DateTypeDef *sDate);
void update_time(RTC_TimeTypeDef *sTime, RTC_DateTypeDef *sDate);
// Funções auxiliares
void configurar_semaforo(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4);
void mostrar_contagem_regressiva(uint8_t tempo);
uint32_t millis(); // Implementação baseada no timer
uint16_t ler_ldr(); // Leitura analógica do LDR
void configurar_interrupcao_botao();
void configurar_timer();
void FSM(RTC_TimeTypeDef *sTime);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char imprime = LCD; //controle do print
Estado estado_atual = NORMAL;
SFSM estado = E1;
uint32_t leitura;
volatile uint32_t tempo_inicio = 0;  // Armazena o tempo do último evento
uint8_t contador_pedestre = TEMPO_PEDESTRE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
	if (imprime == LCD) {
		if (ch == '\n') {
			//lcd_wrchar('\r'); // Caractere de retorno de carro (opcional)
			//lcd_wrchar('\n'); // Quebra de linha no display
		} else {
			lcd_wrchar(ch);   // Escreve o caractere no display
		}
	} else if (imprime == SERIAL) {
		HAL_UART_Transmit(&huart2, &ch, 1, 10);
	}
	return ch;
}
uint8_t isButtonPressed(void) {
	static uint32_t lastPressTime = 0;

	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == GPIO_PIN_RESET) {
		if (HAL_GetTick() - lastPressTime > DEBOUNCE_DELAY_MS) {
			lastPressTime = HAL_GetTick();
			return 1; // Botão pressionado
		}
	}
	return 0; // Botão não pressionado
}
// Função para envio do comando
void send_request_for_time(RTC_TimeTypeDef *sTime, RTC_DateTypeDef *sDate) {
	char buffer[BUFFER_SIZE] = { 0 };

	imprime = SERIAL;
	printf("Hora:\n");

	// Recebendo a string completa da UART
	if (HAL_UART_Receive(&huart2, (uint8_t*) buffer, BUFFER_SIZE - 1, 2000)
			== HAL_OK) {

	}
	uint8_t hours, minutes, seconds;
	uint8_t day, month, year;

	// Verifica o tamanho e formato da string recebida
	if (strlen(buffer) >= 19 && buffer[2] == ':' && buffer[5] == ':'
			&& buffer[8] == ' ' && buffer[11] == '/' && buffer[14] == '/') {

		// Parsing da hora HH:MM:SS
		hours = ((10 * (buffer[0] - '0')) + (buffer[1] - '0') - 2);
		minutes = (10 * (buffer[3] - '0')) + (buffer[4] - '0');
		seconds = (10 * (buffer[6] - '0')) + (buffer[7] - '0');

		sTime->Hours = hours;
		sTime->Minutes = minutes;
		sTime->Seconds = seconds;

		day = (10 * (buffer[9] - '0')) + buffer[10];
		month = (10 * (buffer[12] - '0')) + buffer[13];
		year = 2024;

		sDate->Year = year;
		sDate->Month = month;
		sDate->Date = day;

		HAL_RTC_SetDate(&hrtc, sDate, RTC_FORMAT_BIN);
		HAL_RTC_SetTime(&hrtc, sTime, RTC_FORMAT_BIN);
		//HAL_UART_Transmit(&huart2, sDate);

	}
}

void update_time(RTC_TimeTypeDef *sTime, RTC_DateTypeDef *sDate) {

	// Obtém os valores atuais do RTC
	if (HAL_RTC_GetTime(&hrtc, sTime, RTC_FORMAT_BIN) == HAL_OK
			&& HAL_RTC_GetDate(&hrtc, sDate, RTC_FORMAT_BIN) == HAL_OK) {

		uint8_t hours = sTime->Hours;
		uint8_t minutes = sTime->Minutes;
		uint8_t seconds = sTime->Seconds;
		imprime = LCD;
		lcd_goto(3, 1);
		printf("Hora:\n");
		lcd_goto(8, 1);
		printf("%02d:%02d:%02d\n", hours, minutes, seconds);
		// Transmite os valores via UART ao Node-RED
		//char buffer[50];
		//sprintf(buffer, "Hora: %02d:%02d:%02d", hours, minutes, seconds);
		//HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

		//HAL_Delay(1000);  // Pausa de 1 segundo entre as leituras
	}
}

void FSM_SEMAFORO(uint32_t tempo_atual) {
	switch (estado) {
	case E1:
		configurar_semaforo(VERMELHO, VERDE, VERMELHO, VERDE);
		if (tempo_inicio == 0)
			tempo_inicio = tempo_atual;
		if (tempo_atual - tempo_inicio >= TEMPO_VERDE - TEMPO_AMARELO) {
			tempo_inicio = 0;
			estado = E2;
		}
		break;
	case E2:
		configurar_semaforo(VERMELHO, AMARELO, VERMELHO, AMARELO);
		if (tempo_inicio == 0)
			tempo_inicio = tempo_atual;
		if (tempo_atual - tempo_inicio >= TEMPO_AMARELO) {
			tempo_inicio = 0;
			estado = E3;
		}
		break;
	case E3:
		configurar_semaforo(VERDE, VERMELHO, VERDE, VERMELHO);
		if (tempo_inicio == 0)
			tempo_inicio = tempo_atual;
		if (tempo_atual - tempo_inicio >= TEMPO_VERDE - TEMPO_AMARELO) {
			tempo_inicio = 0;
			estado = E4;
		}
		break;
	case E4:
		configurar_semaforo(AMARELO, VERMELHO, AMARELO, VERMELHO);
		if (tempo_inicio == 0)
			tempo_inicio = tempo_atual;
		if (tempo_atual - tempo_inicio >= TEMPO_AMARELO) {
			tempo_inicio = 0;
			estado = E1;
		}
		break;
	}
}

void modoNoturno(){
	// leitura ADC pelo meotod polling
			HAL_ADC_Start(&hadc1);
			if (HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK) {
				leitura = HAL_ADC_GetValue(&hadc1);
			}
			HAL_ADC_Stop(&hadc1);
			if (leitura >= 4000) {
				estado_atual = NOTURNO;
			} else {
				estado_atual = NORMAL;
			}
			configurar_semaforo(AMARELO, 0, 0, 0);
			HAL_Delay(125);
			configurar_semaforo(0, AMARELO, 0, 0);
			HAL_Delay(125);
			configurar_semaforo(0, 0, AMARELO, 0);
			HAL_Delay(125);
			configurar_semaforo(0, 0, 0, AMARELO);
			HAL_Delay(125);
}
void FSM(RTC_TimeTypeDef *sTime) {
	uint32_t tempo_atual = sTime->Hours * 3600 + sTime->Minutes * 60
			+ sTime->Seconds;
	if (isButtonPressed()) {
		estado_atual = PEDESTRE;
		tempo_inicio = 0;
	}
	switch (estado_atual) {
	case NORMAL:
		// leitura ADC pelo meotod polling
		HAL_ADC_Start(&hadc1);
		if (HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK) {
			leitura = HAL_ADC_GetValue(&hadc1);
		}
		HAL_ADC_Stop(&hadc1);
		if (leitura >= 4000) {
			estado_atual = NOTURNO;
		} else {
			estado_atual = NORMAL;
		}
		FSM_SEMAFORO(tempo_atual);
		break;

	case PEDESTRE:
		if (tempo_inicio == 0)
			tempo_inicio = tempo_atual;
		if (tempo_atual - tempo_inicio >= TEMPO_PEDESTRE) {
			estado_atual = NORMAL;
		} else {
			configurar_semaforo(VERMELHO, VERMELHO, VERMELHO, VERMELHO);
			estado_atual = PEDESTRE;
		}
		break;

	case NOTURNO:
		modoNoturno();
		break;
	}
}

void configurar_semaforo(uint8_t s1, uint8_t s2, uint8_t s3, uint8_t s4) {
	// Lógica para controlar os semáforos:
	EnviarComando(&hi2c1, 1, s1);  // Semáforo 1
	EnviarComando(&hi2c1, 2, s2);  // Semáforo 2
	EnviarComando(&hi2c1, 3, s3);  // Semáforo 3
	EnviarComando(&hi2c1, 4, s4);  // Semáforo 4
}

void mostrar_contagem_regressiva(uint8_t tempo) {
	lcd_goto(0, 0);
	lcd_printf("Pedestres: %d s", tempo);
}

uint32_t millis() {
	// Implementação para obter o tempo em milissegundos baseado no timer
}

uint16_t ler_ldr() {
	// Implementação para leitura do sensor LDR conectado ao pino analógico
}

void configurar_interrupcao_botao() {
	// Configura interrupção externa para o botão de pedestre
}

void configurar_timer() {
	// Configura o timer para gerar o tempo base (1ms ou similar)
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	MX_RTC_Init();
	MX_SPI1_Init();
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	lcd_init(cursor_off);
	lcd_goto(4, 0);
	printf("SEMAFORO\n");
	send_request_for_time(&sTime, &sDate);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		update_time(&sTime, &sDate);
		FSM(&sTime);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.LowPowerAutoPowerOff = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
	hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
	hadc1.Init.OversamplingMode = DISABLE;
	hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x00503D58;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void) {

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */

	/** Initialize RTC Only
	 */
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
	if (HAL_RTC_Init(&hrtc) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */

	/* USER CODE END RTC_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_7B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_14 | GPIO_PIN_4 | GPIO_PIN_5,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC3 */
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB14 PB4 PB5 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_14 | GPIO_PIN_4 | GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PA8 PA9 */
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PC7 */
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void EnviarComando(I2C_HandleTypeDef *hi2c, uint8_t semaforo, uint8_t estado) {
	uint8_t comando = (semaforo << 4) | (estado << 1); // Monta o comando
	imprime = SERIAL;
	// Envia o comando ao escravo via I2C
	if (HAL_I2C_Master_Transmit(hi2c, ENDERECO_ESCRAVO << 1, &comando, 1,
	HAL_MAX_DELAY) == HAL_OK) {
		// Sucesso na transmissão

		printf("Comando enviado: Semáforo %d, Estado: ", semaforo);
		switch (estado) {
		case VERMELHO:
			printf("Vermelho\n");
			break;
		case AMARELO:
			printf("Amarelo\n");
			break;
		case VERDE:
			printf("Verde\n");
			break;
		default:
			printf("Estado desconhecido\n");
			break;
		}
	} else {
		// Erro na transmissão
		printf("Erro ao enviar comando para o semáforo %d\n", semaforo);
	}
}
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//	if(GPIO_Pin == GPIO_PIN_3)
//	{
//		estado_atual = PEDESTRE;
//		tempo_inicio = 0;
//	}
//}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
