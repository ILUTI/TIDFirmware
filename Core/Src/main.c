/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "tacometro.h"
#include "calibracion_flash.h"
#include "rak3172.h"
#include "servo.h"
#include "rtc_reloj.h"
#include "gps.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define INTERVALO_ENVIO_RPM_MS   30000U   // mandar RPM cada 30 segundos
/* ⚠️ SUPUESTO / placeholder: por ahora hardcodeado a 1 (equivale a
 * "DSL-0001"). Cuando exista mas de un nodo motor, resolver esto desde
 * CalibFlash_GetNodeId() (ID 18, NODE_ID) en vez de una constante. */
#define MOTOR_ID_NUMERIC   1U

/* Codigos de estado -- DEBEN coincidir exactamente con ESTADO_NOMBRES
 * de decoder.py del lado AWS. */
#define ESTADO_ACTIVO     1U  /* motor encendido Y con presion de salida -- no alcanzable aun, falta presion.c */
#define ESTADO_APAGADO    2U
#define ESTADO_ENCENDIDO  3U  /* motor girando, sin lectura de presion real todavia (placeholder) */

/* Coordenadas fijas del sitio -- ultimo respaldo si el GPS nunca ha
 * conseguido fix (ni en este arranque ni en ninguno anterior, ver
 * CalibFlash_GetUltimaLatitudConocida()). Con el GPS ya instalado,
 * este valor solo se usaria en el primerisimo arranque sin vista al
 * cielo. */
#define LATITUD_FIJA    14.27387764641955f
#define LONGITUD_FIJA  -91.09260397779028f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_RTC_Init(void);
static void MX_LPUART1_UART_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_RTC_Init();
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */

  CalibFlash_Init();
  Reloj_Init(&hrtc);

  /* Estimacion inicial del reloj, mientras no llegue la resincronizacion
     * real con la red -- mejor que el default de MX_RTC_Init() (2000). */
  uint32_t horaConocidaFlash = CalibFlash_GetUltimaHoraUtcConocida();
  if (horaConocidaFlash > 0U) {
	Reloj_CargarHoraAproximada(horaConocidaFlash);
  }

  Tacometro_Init(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);

  /* RAK3172 en USART1 (PA9/PA10) */
  RAK3172_Init(&huart1);

  /* GPS (SIM7600X) en USART2 (PB3/PB4) -- ver gps.h */
  GPS_Init(&huart2);

  HAL_Delay(500);  // dar tiempo al SIM7600X a terminar su propio arranque

  /* Enciende el motor GNSS. Confirmado en campo (2026-08-18) via
   * USB-TTL directo al modulo: contesta "OK" (o "ERROR" si ya estaba
   * encendido de una sesion anterior -- inofensivo, ver nota de abajo). */
  GPS_EnviarComandoAT("AT+CGPS=1");

  /* Pausa necesaria entre comandos -- confirmada en campo (2026-08-18):
   * sin esta espera, tras un "ERROR" a AT+CGPS=1 (GPS ya encendido de
   * una sesion previa) el modulo dejaba de contestar CUALQUIER cosa,
   * ni siquiera el eco de AT+CGPSINFO=10 mandado justo despues -- el
   * modulo necesita este instante para terminar de procesar/
   * estabilizarse antes de aceptar el siguiente comando. */
  HAL_Delay(1500);

  /* Habilita el auto-reporte periodico de posicion cada 10 segundos --
   * a partir de aca el modulo manda "+CGPSINFO: ..." por su cuenta
   * (URC), sin que el host tenga que volver a pedirlo. Confirmado que
   * este modulo NO transmite NMEA crudo espontaneamente por esta UART;
   * "AT+CGPSINFO=<1-255>" es el mecanismo real de auto-reporte (ver
   * gps.c). Ajustar el intervalo (10) si se necesita una posicion mas
   * fresca o se quiere reducir el trafico en la UART. */
  GPS_EnviarComandoAT("AT+CGPSINFO=10");
  printf("GPS: inicializado en USART2, auto-reporte cada 10s habilitado\r\n");

    HAL_Delay(500);  // dar tiempo al RAK3172 a terminar su propio arranque

    /* AT+MASK=0002 (sub-banda 2): fix real y necesario para el bug
     * documentado de RUI3 en US915 (con AT+MASK=00FF el join falla con
     * AT_ERROR) -- NO es diagnostico, se queda. Las 8 consultas de
     * solo lectura que corrian aca (AT+NWM=?, AT+NJM=?, etc.) SI eran
     * diagnostico puro para confirmar el provisioning durante el
     * bring-up -- se quitaron (2026-08-18) porque cada una podia
     * esperar hasta 3s, sumando varios segundos al arranque sin
     * aportar nada ahora que el problema de join ya esta resuelto. */
    RAK3172_EnviarComandoAT("AT+MASK=0002");
    {
        uint32_t inicioEspera = HAL_GetTick();
        while (!RAK3172_ComandoListo() && (HAL_GetTick() - inicioEspera) < 3000U) {
            RAK3172_Update();
        }
    }

    RAK3172_Join();

  Servo_Init(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  Servo_SetPulsoUs(1500); /* punto medio, posición segura al arrancar */

  printf("Tacometro STM32G431 - inicio, join LoRaWAN solicitado...\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  Tacometro_Update();
  	  RAK3172_Update();
  	  GPS_Update();

  	  /* Persiste la primera posicion valida del arranque en flash --
  	   * una sola vez, no en cada actualizacion cada 10s (desgastaria
  	   * la flash sin necesidad para un nodo que practicamente no se
  	   * mueve). Sirve de respaldo si el proximo arranque no consigue
  	   * fix a tiempo (ver prioridad de posicion en el uplink LIVE). */
  	  static bool yaPersistioGpsEsteArranque = false;
  	  if (!yaPersistioGpsEsteArranque && GPS_TieneFix()) {
  		  yaPersistioGpsEsteArranque = true;
  		  CalibFlash_SetUltimaPosicionConocida(GPS_GetLatitud(), GPS_GetLongitud());
  		  printf("GPS: primera posicion del arranque persistida en flash (%.6f, %.6f)\r\n",
  		         GPS_GetLatitud(), GPS_GetLongitud());
  	  }

  	  /* Aviso una sola vez cuando el join se confirme */
  	  static bool yaAvisoJoin = false;
  	  if (!yaAvisoJoin && RAK3172_EstaUnido()) {
  		  yaAvisoJoin = true;
  		  printf("RAK3172: unido a la red LoRaWAN (+EVT:JOINED)\r\n");
  	  }

  	  /* ================== Sincronizacion de hora (DeviceTimeReq) ==================
  	   * Secuencia: 1) una vez unido, pedir la hora (AT+TIMEREQ=1); 2) esperar
  	   * a que un uplink la traiga de vuelta (+EVT:TIMEREQ); 3) consultar el
  	   * valor (AT+LTIME=?) y setear el RTC. Se reintenta solo una vez por
  	   * arranque -- si falla, Reloj_EstaSincronizado() sigue en false y el
  	   * uplink LIVE queda pausado (ver mas abajo) hasta el proximo intento
  	   * manual/reset. */
  	  static bool horaSolicitada = false;
  	  static bool horaConsultada = false;

  	  if (RAK3172_EstaUnido() && !horaSolicitada && RAK3172_ComandoListo()) {
  		  horaSolicitada = RAK3172_SolicitarHoraRed();
  		  if (horaSolicitada) {
  			  printf("RAK3172: hora de red solicitada (AT+TIMEREQ=1), esperando proximo uplink...\r\n");
  		  }
  	  }

  	  if (horaSolicitada && !horaConsultada && RAK3172_HoraDeRedDisponible() && RAK3172_ComandoListo()) {
  		  horaConsultada = RAK3172_ConsultarHoraRed();
  	  }

  	  if (horaConsultada && !Reloj_EstaSincronizado() && RAK3172_ComandoListo()) {
		  char respuesta[RAK3172_RX_BUFFER_SIZE];
		  if (RAK3172_GetUltimaRespuesta(respuesta, sizeof(respuesta))) {
			  /* Formato REAL confirmado en campo (2026-08-17):
			   * "AT+LTIME=15h08m55s on 08/17/2026" (MM/DD/YYYY) --
			   * NO es un epoch plano como se había asumido sin
			   * ejemplo. Se parsea directo a campos de calendario. */
			  unsigned int hora, minuto, segundo, mes, dia, anio;
			  int camposLeidos = sscanf(respuesta, "AT+LTIME=%2uh%2um%2us on %2u/%2u/%4u",
										 &hora, &minuto, &segundo, &mes, &dia, &anio);
			  if (camposLeidos == 6) {
				  Reloj_SetHoraUtc((uint16_t)anio, (uint8_t)mes, (uint8_t)dia,
								   (uint8_t)hora, (uint8_t)minuto, (uint8_t)segundo);
				  CalibFlash_SetUltimaHoraUtcConocida(Reloj_GetUnixTimeUtc());
				  printf("Reloj sincronizado con la red: %02u:%02u:%02u UTC, %02u/%02u/%04u\r\n",
						 hora, minuto, segundo, mes, dia, anio);
			  } else {
				  printf("RAK3172: respuesta de AT+LTIME=? no reconocida: '%s'\r\n", respuesta);
			  }
		  }
	  }

  	  /* ================== Estado del motor + uplink LIVE extendido ==================
  	   * ⚠️ SUPUESTO / placeholder: sin sensor de presion de motor todavia
  	   * (ver README, seccion 5 -- "disenado, no construido"), el estado
  	   * ACTIVO (requiere presion) NO es alcanzable aun. Se deriva
  	   * ENCENDIDO/APAGADO solo del RPM. Cuando exista presion.c, agregar
  	   * la condicion real de ACTIVO aqui. */
  	  static uint8_t estadoAnterior = ESTADO_APAGADO;
	  static uint32_t inicioEstadoLocal = 0;
	  static bool estadoInicializado = false;
	  static bool relojSincronizadoAnterior = false;
	  static uint32_t fechaHoraLocalIterAnterior = 0;

	  float rpmActual = Tacometro_GetRPMFiltrada();
	  uint8_t estadoActual = (rpmActual > 0.0f) ? ESTADO_ENCENDIDO : ESTADO_APAGADO;

	  uint32_t fechaHoraLocalIterActual = Reloj_GetUnixTimeLocal();
	  bool relojRecienSincronizado = (!relojSincronizadoAnterior && Reloj_EstaSincronizado());
	  relojSincronizadoAnterior = Reloj_EstaSincronizado();

	  if (!estadoInicializado || estadoActual != estadoAnterior) {
		  estadoAnterior = estadoActual;
		  inicioEstadoLocal = fechaHoraLocalIterActual;
		  estadoInicializado = true;
	  } else if (relojRecienSincronizado) {
		  /* El reloj acaba de saltar de un valor aproximado/placeholder
		   * al real -- si el estado NO cambio, desplazar el ancla por
		   * el mismo salto en vez de reiniciarla a "ahora", para no
		   * perder el tiempo ya transcurrido en el estado actual (ej.
		   * el motor llevaba apagado un rato desde el arranque, y la
		   * sincronizacion no deberia reiniciar ese conteo a cero). */
		  inicioEstadoLocal += (fechaHoraLocalIterActual - fechaHoraLocalIterAnterior);
	  }
	  fechaHoraLocalIterAnterior = fechaHoraLocalIterActual;
  	  /* Envío periódico del uplink LIVE, solo una vez unido a la red Y con
  	   * el reloj ya sincronizado (antes de eso, fecha_hora/inicio_operacion
  	   * no tendrían ningún valor confiable que mandar). */
	  static uint32_t ultimoEnvioRPM = 0;
	  bool tocaEnviarPorIntervalo = (HAL_GetTick() - ultimoEnvioRPM >= INTERVALO_ENVIO_RPM_MS);
	  bool tocaEnviarPorForzado = CalibFlash_HayReporteForzado();

	  if (RAK3172_EstaUnido() && (tocaEnviarPorIntervalo || tocaEnviarPorForzado) && RAK3172_ComandoListo()) {
  		  ultimoEnvioRPM = HAL_GetTick();

  		  uint32_t fechaHoraLocal = fechaHoraLocalIterActual;
  		  uint32_t segundosTranscurridos = fechaHoraLocal - inicioEstadoLocal;

  		  /* Prioridad de posicion: GPS con fix vivo > ultima posicion
  		   * conocida persistida en flash (ver CalibFlash_SetUltimaPosicionConocida()
  		   * mas abajo) > coordenada fija de respaldo, para cuando el
  		   * modulo GPS nunca ha conseguido fix (recien instalado, sin
  		   * vista al cielo, etc.). */
  		  float latitudActual = LATITUD_FIJA;
  		  float longitudActual = LONGITUD_FIJA;
  		  if (GPS_TieneFix()) {
  			  latitudActual = GPS_GetLatitud();
  			  longitudActual = GPS_GetLongitud();
  		  } else if (CalibFlash_GetUltimaLatitudConocida() != 0.0f) {
  			  latitudActual = CalibFlash_GetUltimaLatitudConocida();
  			  longitudActual = CalibFlash_GetUltimaLongitudConocida();
  		  }

  		  bool encolado = RAK3172_EnviarUplinkLive(
  			  MOTOR_ID_NUMERIC,
  			  rpmActual,
  			  0.0f,  /* presion: placeholder hasta que exista presion.c */
  			  estadoActual,
  			  fechaHoraLocal,
  			  inicioEstadoLocal,
  			  segundosTranscurridos,
  			  latitudActual,
  			  longitudActual
  		  );

  		  (void)encolado;
  		  if (tocaEnviarPorForzado) {
  			  CalibFlash_LimpiarReporteForzado();
  		  }

	  }
  	  /* NOTA DE SEGURIDAD: RESET_REMOTO (CalibFlash_HayResetPendiente())
  	   * deliberadamente NO se conecta todavía. Antes de ejecutar un
  	   * reinicio remoto hay que definir qué hace el servo del
  	   * acelerador durante el reboot (mantener su última posición
  	   * física vs. quedar sin control por unos segundos) -- ver
  	   * discusión de diseño del gobernador de motor. Hasta entonces,
  	   * el comando se recibe y se valida (requiere byte de
  	   * confirmación 0xA5), pero no dispara ninguna acción. */

  	  /* Reporte de resultado del último comando AT (join o send) */
  	  static RAK3172_Resultado_t ultimoResultadoMostrado = (RAK3172_Resultado_t)-1;
  	  RAK3172_Resultado_t resultadoActual = RAK3172_GetUltimoResultado();
  	  if (RAK3172_ComandoListo() && resultadoActual != ultimoResultadoMostrado) {
  		  ultimoResultadoMostrado = resultadoActual;
  		  printf("RAK3172: resultado=%d (0=OK,1=ERROR,2=TIMEOUT,3=BUSY)\r\n", resultadoActual);
  	  }

  	  static uint32_t ultimoReporte = 0;
  	  if (HAL_GetTick() - ultimoReporte >= 1000) {
  		  ultimoReporte = HAL_GetTick();
  		  printf("Frecuencia: %.1f Hz | RPM instant: %.1f | RPM filtrada: %.1f | Detenido: %d | RuidoFiltrado: %lu | RelojSync: %d\r\n",
  			  Tacometro_GetFrecuenciaHz(),
  			  Tacometro_GetRPMInstantanea(),
  			  Tacometro_GetRPMFiltrada(),
  			  Tacometro_EstaDetenido(),
  			  Tacometro_GetContadorRuidoFiltrado(),
  			  Reloj_EstaSincronizado());
  	  }


  	  /* PRUEBA DE BANCO -- barrido lento del servo entre sus límites
  	   * configurados, solo para confirmar visualmente el movimiento
  	   * antes de conectarlo a la varilla real. QUITAR/comentar este
  	   * bloque una vez que el PID tome el control real del servo. */
  	  static uint32_t ultimoPasoServo = 0;
  	  static int16_t direccionServo = 25; /* µs por paso */
  	  if (HAL_GetTick() - ultimoPasoServo >= 20) { /* un paso cada 20ms */
  		  ultimoPasoServo = HAL_GetTick();
  		  uint16_t actual = Servo_GetPulsoActualUs();
  		  uint16_t minimo = CalibFlash_GetServoPulsoMinUs();
  		  uint16_t maximo = CalibFlash_GetServoPulsoMaxUs();

  		  int32_t siguiente = (int32_t)actual + direccionServo;
  		  if (siguiente >= maximo) {
  			  siguiente = maximo;
  			  direccionServo = -direccionServo;
  		  } else if (siguiente <= minimo) {
  			  siguiente = minimo;
  			  direccionServo = -direccionServo;
  		  }
  		  Servo_SetPulsoUs((uint16_t)siguiente);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  hlpuart1.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  hlpuart1.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart1.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  huart1.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart2.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  huart2.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 99;
  hrtc.Init.SynchPrediv = 319;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.SubSeconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 169;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 8;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 169;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    Tacometro_CaptureCallback(htim);
}

#ifdef __GNUC__
int __io_putchar(int ch)
{

    HAL_UART_Transmit(&hlpuart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
#endif

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
     /* RAK3172 ahora en USART1 */
    if (huart->Instance == USART1) {
        RAK3172_RxEventCallback(huart, Size);
    }
    /* GPS (SIM7600X) en USART2 -- ver gps.h */
    else if (huart->Instance == USART2) {
        GPS_RxEventCallback(huart, Size);
    }
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
#ifdef USE_FULL_ASSERT
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
