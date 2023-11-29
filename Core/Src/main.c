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
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
uint8_t Vector_tx[7]="hola";
uint8_t Vector_rx[7];
uint8_t Valor_parseo;
uint8_t Vector_recepcion[130];
uint8_t* Vector_recepcion_aux;
uint8_t Direccion[3];
uint8_t* Payload;
uint32_t CRC_calc=0;
int a=0;

int i=0;

struct Encabezado_msg {
    uint8_t nodo_origen;
    uint8_t nodo_destino;
    uint8_t size;
    uint8_t payload[125];			//de 0 a 124 bytes (125 en total).Una lástima por el CRC, se pondrá después
}; //Falta pensar cómo incorporar el payload de tamaño variable, y luego el CRC de 4 bytes fijos.
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ORIGEN 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
uint8_t convertir_ascii(uint8_t*vector_direccion);
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
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  //Necesito declarar la estructura de la forma larga, para que me permita la inicializacion
  struct Encabezado_msg Encabezado = {
	    .nodo_origen = 0,
	    .nodo_destino = 0,
	    .size = 0,
	    .payload = {0}  // Esto inicializa todos los elementos del arreglo a cero
  };

  struct Encabezado_msg* Encabezado_ptr=&Encabezado;

  //Vector_tx[4]='\n';
  //Vector_tx[5]='\r';
  //HAL_UART_Transmit_DMA(&huart2, Vector_tx, sizeof(Vector_tx));
  //Máquina de estados:

  enum states{ESPERANDO_1ER_ENTER, RECIBIENDO_CARACTERES,ENVIANDO_VECTOR};

  enum states state;
  state=ESPERANDO_1ER_ENTER;

  Vector_recepcion_aux=Vector_recepcion;			//El vector aux me queda apuntando a Vector_recepcion

  while(1)					//Bucle de recepcion de enter
							//Conviene hacer máquina de estados: Estado 0: esperando 1er enter, estado 1: anotando vector, pasa al estado 2 al recibir enter, estado 2: enviar vector, ir a estado 1
							//Implementar interrupción para el rx por rs485.
  {
	switch (state){

	case ESPERANDO_1ER_ENTER:

		HAL_UART_Receive(&huart2, (uint8_t *)&Valor_parseo, 1 , HAL_MAX_DELAY);		//Aquí se chequea que lo ingresado no sea un enter
		if(Valor_parseo==0xD)
			{Valor_parseo=0;
			Vector_recepcion[0]=0x0D;							//Le pongo el enter al principio de la recepción como pide el enunciado
			Vector_recepcion_aux++;								//Preparo al puntero para seguir desde ahí.
			//HAL_Delay(30);									//Delay para que no vuele al otro estado estado de 1.
			state=RECIBIENDO_CARACTERES;}						//Quedmaos listos para pasar de estado
		break;

	case RECIBIENDO_CARACTERES:

		HAL_UART_Receive(&huart2, (uint8_t *)&Valor_parseo, 1 , HAL_MAX_DELAY);		//Aquí se chequea que lo ingresado no sea un enter
		if(Valor_parseo==0xD)
			{Valor_parseo=0;
			*Vector_recepcion_aux=0x0D;			//Dejamos en el último valor del vector de recepcion, el valor 0x0D. Recordar que no hay que
			//HAL_Delay(30);					//incrementar porque ya fue incrementado previamente.
			state=ENVIANDO_VECTOR;
			break;}
		if(i<127)
			{*Vector_recepcion_aux=Valor_parseo;		//Correccion para evitar doble tecleo de cada tecla
			Vector_recepcion_aux++;
			//HAL_UART_Receive(&huart2, Vector_recepcion_aux++,1, HAL_MAX_DELAY);	//Primero actua, despues incrementa en 1 así sigue cargando a Vector_recepcion
			i++;}														//de forma correcta. Falta limitar a 130 caracteres
		else
			{Vector_recepcion[127]=0x0D;							//Fijamos el último valor en 0x0D (sólo en el caso de llenar todo el vector)
			Vector_recepcion_aux=Vector_recepcion;					//Volvemos a darle la dirección del vector original para la siguiente pasada
			state=ENVIANDO_VECTOR;}									//Pasamos de estado de forma forzosa.
		break;

	case ENVIANDO_VECTOR:
		//Lo primero es conformar el vector direccion y el vector payload
		//En i quedó el tamaño de datos.
		//for(a=1;a<4;a++)
		//{
		//	Direccion[a-1]=Vector_recepcion[a];			//esto quedó al dope porque la funcion convertir_ascii no lo requiere
		//}
		//Direccion quedó formado
		Payload=Vector_recepcion+4;		//El inicio del Payload está en Vector_recepcion +1(enter)+3(bytes de direccion destino).
		//(i-3) tendrá el valor de hasta donde se extiende "Payload".

		//Comienzo a armar la estructura del mensaje


		Encabezado.nodo_origen=ORIGEN;
		Encabezado.nodo_destino= convertir_ascii(Vector_recepcion);	//Se toman los bytes correctos de "Vector_recepcion" en la conversión
		memcpy(Encabezado.payload, Payload, (i-3) );				//Le paso el puntero previamente creado. Luego movemos la cantidad de datos a transmitir. No se puede hacer de otra forma. Tiene Destino, Origen, size de bytes. Todo punteros.
		Encabezado.size=(i-3);

		//Antes de enviar, calculamos CRC
		//Parece que esta función requiere que la cantidad de bytes a hacerle CRC sea divisible por 4 sí o sí.
		//Por ende, hay que rellenar con ceros el vector total hasta lograrlo.
		//Sin embargo, gracias a cómo fue inicializado Encabezado, el resto de payload ya es cero. Asi que no pasa nada
		//size = (i%4>0) ? ((i/4)+1) : (i/4);			//Chequea si el tamaño del vector total es divisible por 4 exacto, o si no lo es, hay que pasar i/4 +1, con ese resto de bytes extra rellenados con cero.
		//Terminamos decidiendo en clase que hacemos CRC de todo, de los 125 bytes con todo relleno con cero.
		HAL_CRC_Init(&hcrc);			//Para que estados pasados de CRC no afecten
		CRC_calc=HAL_CRC_Calculate(&hcrc, (uint32_t*)Encabezado_ptr, (sizeof(Encabezado) / 4));
		//Procedo a transmitir uno tras otro
		HAL_UART_Transmit(&huart6, (uint8_t*)Encabezado_ptr, i, HAL_MAX_DELAY);	//La UART de recepcion se hace sobre la 6!!
		HAL_UART_Transmit(&huart6, (uint8_t*) &CRC_calc, 4, HAL_MAX_DELAY);		//La de impresion por consola, por la 2 (la de la PC)
		//Listorti la parte de envio
		HAL_Delay(300);							//Delay porque si
		state=RECIBIENDO_CARACTERES;			//Ojo que alguna variable quiza quedo en un estado raro y haya que volverlo a su inicial.

		//En algún momento de este estado se manda el vector via RS485, y se pasa al estado 1 de nuevo (con un delay si se quiere)
		//La manera mas facik es usar varios UART TRANSMIT de cada campo, uno tras otro, con el size correcto pasandole el puntero, y fue.
		break;
	//Todo: implementar callback de interrupcion UART para recepcion por rs485 (UART 6).

		//Aunque se puede hacer con DMA y queda bonito, nunca se puede saber por interrupción cuando se termino de llenar el vector
		//Me parece mejor hacerlo por interrupción y parsear en la callback.
	}
  }
  /* USER CODE END 2 */

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

uint8_t convertir_ascii(uint8_t* vector_direccion)
{
	uint8_t destino=0;
	//Recordar que vector_direccion[0] tendria el CR
	destino=(vector_direccion[1]-48)*100 + (vector_direccion[2]-48)*10 + (vector_direccion[3]-48);
	return destino;
}
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
