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
#include "pid.h"
#include "rtc_reloj.h"
#include "gps.h"
#include "comando_serial.h"
#include "presion.h"
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define INTERVALO_ENVIO_RPM_MS   30000U   // mandar RPM cada 30 segundos

/* Auto-calibracion de ralenti (CALIBRAR_RALENTI, ver calibracion_flash.c) --
 * ventana total larga porque un motor frio puede arrancar bien por debajo
 * de su ralenti real y tardar en subir (ej. ~500 -> ~800 observado en
 * campo); de esa ventana total solo se promedian los ultimos segundos
 * (RALENTI_CAL_VENTANA_PROMEDIO_MS), descartando la rampa de calentamiento
 * de los primeros ~90s. RALENTI_CAL_MARGEN_RPM se suma al promedio antes de
 * aplicarlo como RPM_MIN, para que el ruido normal de medicion (+-10-15 RPM,
 * ver hallazgos de PID) no cruce el umbral por accidente. */
#define RALENTI_CAL_DURACION_TOTAL_MS     120000U
#define RALENTI_CAL_VENTANA_PROMEDIO_MS    30000U
#define RALENTI_CAL_MARGEN_RPM                30.0f

/* Auto-calibracion de SERVO_PULSO_MIN (CONTROL_HABILITADO=4) -- barre el
 * pulso hacia arriba en pasos chicos desde el SERVO_PULSO_MIN actual,
 * buscando donde el motor empieza a acelerar de verdad (la "zona muerta"
 * mecanica del acelerador, observada en campo: unos grados de recorrido
 * sin ningun efecto antes de que el motor reaccione). Parametros pensados
 * para que el barrido sea autolimitado y de bajo riesgo -- ver README
 * seccion 10 para el analisis de riesgo completo. */
#define SERVOMIN_CAL_PASO_US                     4U    /* incremento por paso -- BAJADO de 8 a 4 (2026-09-07) al pasar a montaje directo del servo sobre el eje de la palanca: el recorrido util completo (relenti a fondo) paso a ser ~30 grados de servo en vez de los ~60 grados de la biela-manivela, es decir la MISMA variacion de RPM ahora ocurre en la mitad del rango de pulso -- la ganancia promedio por microsegundo se duplica como minimo, y probablemente iguala o supera el punto mas empinado que tenia la curva no lineal vieja. Revalidar con el primer barrido real: si "subida"/"salto" siguen saliendo grandes con este paso, bajarlo mas. */
#define SERVOMIN_CAL_TECHO_US                   180U    /* tope maximo sobre el SERVO_PULSO_MIN de entrada -- si se llega sin detectar nada, se aborta con error en vez de seguir subiendo a ciegas */
#define SERVOMIN_CAL_DWELL_MS                  2500U    /* espera de asentamiento antes de leer la RPM en cada paso */
#define SERVOMIN_CAL_BASELINE_MS               12000U    /* ventana para medir una linea base fresca de RPM antes de barrer */
#define SERVOMIN_CAL_UMBRAL_DETECCION_RPM        30.0f  /* cuanto debe subir la RPM sobre la linea base para contar como "empezo a acelerar" -- mayor al ruido normal de medicion (+-10-15 RPM) */
#define SERVOMIN_CAL_SALTO_ANORMAL_RPM  (SERVOMIN_CAL_UMBRAL_DETECCION_RPM * 4.0f) /* si un solo paso sube esto o mas, es un salto fuera de lo esperado -- abortar ya, no seguir */
#define SERVOMIN_CAL_CONFIRMACIONES_NECESARIAS    2U    /* lecturas seguidas por encima del umbral, en el mismo pulso, antes de dar por confirmada la deteccion */
#define SERVOMIN_CAL_MARGEN_PASOS                 6U    /* pasos de colchon hacia atras al aplicar el resultado, para no dejar SERVO_PULSO_MIN pegado justo al borde. SUBIDO de 3 a 6 (2026-09-03) tras confirmar en campo que la transicion NO siempre es un escalon limpio -- una corrida real mostro subida sostenida (14.8/7.3/26.8/26.1/27.0 RPM) durante ~6 pasos antes de confirmar deteccion, y 3 pasos de margen dejo el SERVO_PULSO_MIN resultante a mitad de esa rampa (RPM de reposo termino ~20-30 RPM por encima del relenti real). 6 pasos cubre esa rampa observada sin perjudicar el caso de transicion abrupta (el barrido original de esta misma zona muerta vino en puro ruido hasta justo antes del punto de deteccion). */

/* Auto-calibracion de ALPHA -- CONTROL_HABILITADO=5 (ver
 * calibracion_flash.c) -- mide la dispersion real de RPM_instantanea
 * (sin filtrar) durante una ventana corta con el motor ya estable, y
 * calcula el ALPHA necesario para que la RPM ya filtrada quede dentro
 * de un objetivo de ruido fijo (ALPHA_CAL_OBJETIVO_DESVIACION_RPM),
 * usando la relacion de atenuacion de un filtro EMA:
 * alpha = 2r^2/(1+r^2), con r = objetivo/desviacion_cruda. No mueve el
 * servo (mismo motivo que el modo 3 -- pidActivoAhora nunca es true en
 * este modo, cae solo en la rama segura de abajo). Ver README seccion 12.
 *
 * CASO BORDE encontrado en campo 2026-09-03: si el ruido crudo medido
 * ya sale por debajo/cerca del objetivo, "r" se acerca a 1 y el alpha
 * calculado se va casi a 1.0 (practicamente sin filtro) -- una ventana
 * de 8s puede simplemente no alcanzar a capturar alguna de las rafagas
 * de perturbacion periodicas (~15-25s) ya conocidas en este motor, asi
 * que "se ve limpio en la ventana" no es garantia de que no haga falta
 * nada de suavizado. ALPHA_CAL_TECHO_ALPHA pone un techo -- nunca se
 * aplica un alpha mas agresivo (mas cercano a 1) que ese, sin importar
 * que tan bajo salga el ruido medido. */
#define ALPHA_CAL_VENTANA_MS                    8000U   /* duracion total de la ventana de muestreo */
#define ALPHA_CAL_INTERVALO_MUESTREO_MS          200U   /* cada cuanto se toma una muestra dentro de la ventana -- mismo periodo que el log PID_TEST */
#define ALPHA_CAL_OBJETIVO_DESVIACION_RPM        10.0f  /* objetivo de ruido en la RPM YA FILTRADA -- ver discusion README seccion 12 */
#define ALPHA_CAL_TECHO_ALPHA                     0.5f  /* nunca aplicar mas filtrado "agresivo" que esto (ver caso borde arriba) -- algo mas permisivo que el 0.35 ya validado, pero lejos de "sin filtro" */

/* Mapeo de curva de ganancia -- CONTROL_HABILITADO=6 (ver
 * calibracion_flash.c). Barre el pulso hacia arriba en pasos chicos
 * desde SERVO_PULSO_MIN, en lazo ABIERTO (sin PID), registrando el par
 * (pulso, RPM real) en cada paso -- a diferencia de CONTROL_HABILITADO=4
 * (que solo busca el borde de la zona muerta), esto mapea la forma
 * completa de la curva de ganancia en el rango de operacion real, para
 * diagnostico/diseno de una futura correccion de gain-scheduling en el
 * PID. Por ahora SOLO mide y loguea -- todavia no aplica ninguna
 * correccion (ver README seccion 12 y discusion sobre la geometria del
 * mecanismo brazo-varilla).
 *
 * Mismo paso de 8us que la zona muerta (no uno mas grande) a proposito:
 * el usuario pidio explicitamente no arriesgarse a pasarse de largo del
 * techo de RPM de un salto grande en la zona de mas ganancia -- 8us
 * tarda mas (~3 min peor caso) pero acota mejor el salto por paso. */
#define GANANCIA_CAL_PASO_US                       4U    /* mismo tamano que SERVOMIN_CAL, a proposito -- ver nota arriba. BAJADO de 8 a 4 junto con SERVOMIN_CAL_PASO_US (2026-09-07) por el mismo motivo: montaje directo, recorrido util a la mitad de grados que la biela-manivela -> ganancia por microsegundo al menos el doble. */
#define GANANCIA_CAL_DWELL_MS                    2500U   /* espera de asentamiento antes de leer la RPM en cada paso */
#define GANANCIA_CAL_RPM_TECHO                  1500.0f  /* nunca pasar de esta RPM durante el barrido -- elegido por el usuario como el limite seguro de esta prueba, no el maximo del motor */
#define GANANCIA_CAL_SALTO_ANORMAL_RPM            150.0f  /* si un solo paso sube la RPM esto o mas, frenar ya -- protege contra pasarse del techo de un salto Y detecta lecturas anormales */


/* El RTC de este nodo corre del LSI interno (~32kHz nominal, sin cristal
 * externo -- ver hallazgos de hardware), que no esta calibrado ni
 * compensado por temperatura. Deriva medida en campo: ~0.8% (equivale a
 * ~12 min/dia sin correccion). Por eso se resincroniza periodicamente
 * en vez de una sola vez al arrancar -- ver bloque de sincronizacion en
 * el superloop.
 *
 * Fuente PRIMARIA: hora UTC del propio GPS (satelital, mas precisa, y
 * no compite por el canal AT del RAK3172 ni gasta duty cycle) -- se
 * intenta cada INTERVALO_RESYNC_GPS_MS si hay fix vivo. Ese intervalo
 * se dejo igual al de envio de RPM (30s) porque intentarlo no cuesta
 * nada -- el modulo SIM7600X igual solo refresca su propio reporte
 * cada 10s (AT+CGPSINFO=10), asi que preguntar mas seguido no trae
 * dato mas fresco.
 * Fuente de RESPALDO: DeviceTimeReq por LoRaWAN -- compite libremente
 * con el GPS para el primer sync (el que llegue primero gana). Una vez
 * que YA hubo un sync exitoso (de cualquier fuente), el respaldo por
 * LoRa se limita a activarse solo si pasan INTERVALO_FALLBACK_LORA_MS
 * sin ningun resync exitoso nuevo (ej. GPS sin vista al cielo por un
 * rato largo) -- para no competir por el canal AT del RAK3172 sin
 * necesidad mientras el GPS este sincronizando bien. */
#define INTERVALO_RESYNC_GPS_MS      INTERVALO_ENVIO_RPM_MS   // intentar resync por GPS cada vez que se manda RPM (30s)
#define INTERVALO_FALLBACK_LORA_MS   (30UL * 60UL * 1000UL)   // forzar resync por LoRaWAN si no hay exito en 30 min

/* El propio RAK3172 ya reintenta el join 8 veces cada 10s dentro de UN
 * solo comando (ver AT+JOIN=1:0:10:8 en RAK3172_Join(), ~80s en total),
 * pero si esas 8 fallan (o el modulo se queda sin cobertura un rato)
 * nada volvia a pedir el join de nuevo -- el nodo quedaba sin unirse
 * para siempre. 2 min da margen sobre esos ~80s antes de reintentar. */
#define INTERVALO_REINTENTO_JOIN_MS  (120UL * 1000UL)
/* ⚠️ SUPUESTO / placeholder: por ahora hardcodeado a 1 (equivale a
 * "DSL-0001"). Cuando exista mas de un nodo motor, resolver esto desde
 * CalibFlash_GetNodeId() (ID 18, NODE_ID) en vez de una constante. */
#define MOTOR_ID_NUMERIC   1U

/* Codigos de estado -- DEBEN coincidir exactamente con ESTADO_NOMBRES
 * de decoder.py del lado AWS. */
#define ESTADO_ACTIVO     1U  /* motor operando Y con presion de salida real (ver presion.c) */
#define ESTADO_APAGADO    2U
#define ESTADO_ENCENDIDO  3U  /* motor girando, sin presion de salida suficiente aun (arranque/sin carga) */

/* Umbral de presion (PSI) que distingue ACTIVO de solo ENCENDIDO.
 * ⚠️ SUPUESTO / placeholder sin dato de campo todavia -- ajustar una
 * vez que se tenga una sesion real con el sistema hidraulico cargado
 * (ver README seccion 8). */
#define PRESION_UMBRAL_ACTIVO_PSI  5.0f

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
ADC_HandleTypeDef hadc1;

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
static void MX_ADC1_Init(void);
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
  MX_ADC1_Init();
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

  Presion_Init(&hadc1);

  /* RAK3172 en USART1 (PA9/PA10). LoRa es prioridad ante todo lo demas
   * en el arranque -- se inicializa y se pide el join ANTES del GPS,
   * para que el join salga lo antes posible y no compita con los
   * HAL_Delay() bloqueantes del bring-up del GPS (~2.5s). */
  RAK3172_Init(&huart1);

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
  /* Impreso aparte (con etiqueta) porque el reporte generico de
   * "resultado=" en el loop principal no distingue de que comando fue
   * -- para ese momento ya se sobreescribe con el resultado del JOIN
   * de abajo, y un AT+MASK que fallo (ERROR/TIMEOUT) quedaba invisible
   * en el log. */
  printf("RAK3172: AT+MASK=0002 -> resultado=%d (0=OK,1=ERROR,2=TIMEOUT,3=BUSY)\r\n",
         RAK3172_GetUltimoResultado());

  RAK3172_Join();

  /* GPS (SIM7600X) en USART2 (PB3/PB4) -- ver gps.h. Va DESPUES del
   * join de LoRa a proposito (ver comentario arriba). */
  GPS_Init(&huart2);

  HAL_Delay(500);  // dar tiempo al SIM7600X a terminar su propio arranque

  /* AT+CGPSCOLD se probo (2026-08-25) para forzar un cold start
   * explicito -- devolvio ERROR, descartado (no soportado en esta
   * version de firmware del modulo). Vuelto a AT+CGPS=1. Contesta
   * "OK" (arranque en frio) o "ERROR" (ya estaba encendido de una
   * sesion previa) -- inofensivo. */
  GPS_EnviarComandoAT("AT+CGPS=1");

  /* Pausa necesaria entre comandos -- sin ella, el modulo deja de
   * contestar cualquier cosa, ni siquiera el eco de AT+CGPSINFO=10
   * mandado justo despues. NO es la causa del bug de "nunca da fix
   * tras un corte real" que se investigo extensamente (2026-08-25) --
   * esa causa resulto ser el cable USB del Nucleo a la PC metiendo
   * ruido al plano de tierra mientras el GNSS intenta enganchar
   * satelites (ver README seccion 2.6) -- no timing de comandos. */
  HAL_Delay(1500);

  /* Habilita el auto-reporte periodico de posicion cada 10 segundos --
   * a partir de aca el modulo manda "+CGPSINFO: ..." por su cuenta
   * (URC), sin que el host tenga que volver a pedirla. */
  GPS_EnviarComandoAT("AT+CGPSINFO=10");
  printf("GPS: inicializado en USART2, auto-reporte cada 10s habilitado\r\n");

  Servo_Init(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  /* SERVO_PULSO_MIN = posición segura (sin aceleración/ralentí) --
   * confirmado en el montaje físico real, no es un valor arbitrario.
   * Se manda de una vez al arrancar (no se conoce la posición previa
   * del servo, quedó como estuviera al perder alimentación). */
  Servo_SetPulsoUs(CalibFlash_GetServoPulsoMinUs());

  /* Mando manual TEMPORAL por el mismo puerto de debug (LPUART1),
   * mientras no hay red LoRa en campo para probar downlinks reales --
   * ver comando_serial.h. Quitar cuando ya no se necesite. */
  ComandoSerial_Init(&hlpuart1);

  printf("Tacometro STM32G431 - inicio, join LoRaWAN solicitado...\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  Tacometro_Update();
	  Presion_Update();

  	  /* MEDIDA TEMPORAL DE DIAGNOSTICO (2026-09-03): en CONTROL_HABILITADO=7
  	   * (sintonizacion de PID) se saltea RAK3172_Update()/GPS_Update() por
  	   * completo -- se sospecha que los reintentos de join del RAK3172
  	   * (o el modulo GPS) estan generando ruido electrico/caidas de
  	   * voltaje que contaminan la medicion de RPM durante las pruebas de
  	   * escalon, justo cuando se vio al RAK3172 reiniciandose solo
  	   * (banner de arranque completo repetido) en mitad de una prueba
  	   * con oscilacion severa. Aisla la variable para las pruebas de PID:
  	   * si la oscilacion desaparece con esto, confirma que el problema
  	   * era electrico, no del PID. QUITAR esta condicion (dejar
  	   * RAK3172_Update()/GPS_Update() corriendo siempre) en cuanto se
  	   * resuelva la causa real de los reinicios del RAK3172 -- no es una
  	   * solucion definitiva, el nodo no puede operar en campo sin LoRa. */
  	  if (CalibFlash_GetControlHabilitado() != 7U) {
  		  RAK3172_Update();
  		  GPS_Update();
  	  }
  	  ComandoSerial_Update();

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

  	  /* Aviso cuando el join se confirme -- por FLANCO (no unido ->
  	   * unido), no una sola vez para siempre: si el modulo se
  	   * reinicia solo y pierde el join a mitad de sesion (ver
  	   * "AT_NO_NETWORK_JOINED" en rak3172.c, que apaga
  	   * RAK3172_EstaUnido()), este aviso debe poder repetirse cuando
  	   * se vuelva a unir. */
  	  static bool estabaUnidoAntes = false;
  	  bool estaUnidoAhora = RAK3172_EstaUnido();
  	  if (!estabaUnidoAntes && estaUnidoAhora) {
  		  printf("RAK3172: unido a la red LoRaWAN (+EVT:JOINED)\r\n");
  	  }
  	  estabaUnidoAntes = estaUnidoAhora;

  	  /* Reintento de join: sin esto, si los 8 intentos internos del
  	   * modulo (AT+JOIN=1:0:10:8) fallan o nunca llego un +EVT:JOINED
  	   * (ej. sin cobertura un rato en el arranque), el nodo se quedaba
  	   * sin unir para siempre -- nada volvia a pedir el join de nuevo.
  	   *
  	   * Se reafirma AT+MASK=0002 justo antes de cada reintento (no solo
  	   * la vez del arranque): si ese comando fallo silenciosamente en
  	   * el arranque (visto en campo: "resultado=1/ERROR" ahi), el join
  	   * se quedaria fallando para siempre por el mismo bug de sub-banda
  	   * ya documentado (ver hallazgos de hardware), sin que este
  	   * reintento lo pudiera arreglar. Maquina de 2 pasos no bloqueante
  	   * (MASK, despues JOIN) para no frenar el resto del loop. */
  	  static uint32_t ultimoIntentoJoinTick = 0;
  	  static bool reenviandoMaskAntesDeJoin = false;
  	  if (reenviandoMaskAntesDeJoin && RAK3172_ComandoListo()) {
  		  reenviandoMaskAntesDeJoin = false;
  		  printf("RAK3172: aun no unido a la red -- reintentando join (AT+JOIN)\r\n");
  		  RAK3172_Join();
  	  } else if (!RAK3172_EstaUnido() && !reenviandoMaskAntesDeJoin && RAK3172_ComandoListo() &&
  			  (HAL_GetTick() - ultimoIntentoJoinTick >= INTERVALO_REINTENTO_JOIN_MS)) {
  		  ultimoIntentoJoinTick = HAL_GetTick();
  		  printf("RAK3172: aun no unido a la red -- reafirmando AT+MASK=0002 antes de reintentar join\r\n");
  		  RAK3172_EnviarComandoAT("AT+MASK=0002");
  		  reenviandoMaskAntesDeJoin = true;
  	  }

  	  /* ================== Sincronizacion de hora: GPS (primaria) + LoRaWAN (respaldo) ==================
  	   * Ver definicion de INTERVALO_RESYNC_GPS_MS / INTERVALO_FALLBACK_LORA_MS
  	   * arriba para el porque de la redundancia. */
  	  static uint32_t ultimoIntentoGpsTick = 0;
  	  static uint32_t ultimoResyncExitosoTick = 0;
  	  static bool huboResyncAlgunaVez = false;
  	  bool relojFueCorregidoEsteCiclo = false;

  	  /* --- Fuente primaria: GPS. Antes del primer sync (de cualquier
  	   * fuente) se intenta en CADA vuelta del loop, no cada 10 min, para
  	   * no perder tiempo esperando si el fix ya esta disponible. */
  	  bool tocaIntentarGps = (HAL_GetTick() - ultimoIntentoGpsTick >= INTERVALO_RESYNC_GPS_MS) ||
  			  !Reloj_EstaSincronizado();

  	  if (tocaIntentarGps) {
  		  ultimoIntentoGpsTick = HAL_GetTick();

  		  uint16_t anioGps; uint8_t mesGps, diaGps, horaGps, minutoGps, segundoGps;
  		  if (GPS_GetFechaHoraUtc(&anioGps, &mesGps, &diaGps, &horaGps, &minutoGps, &segundoGps)) {
  			  Reloj_SetHoraUtc(anioGps, mesGps, diaGps, horaGps, minutoGps, segundoGps);
  			  CalibFlash_SetUltimaHoraUtcConocida(Reloj_GetUnixTimeUtc());
  			  relojFueCorregidoEsteCiclo = true;
  			  ultimoResyncExitosoTick = HAL_GetTick();
  			  huboResyncAlgunaVez = true;
  			  printf("GPS: reloj sincronizado con hora satelital: %02u:%02u:%02u UTC, %02u/%02u/%04u\r\n",
  					 horaGps, minutoGps, segundoGps, mesGps, diaGps, anioGps);
  		  }
  	  }

  	  /* --- Fuente de respaldo: DeviceTimeReq por LoRaWAN. Antes del
  	   * primer sync (de cualquier fuente) compite libremente con el GPS
  	   * -- el que llegue primero gana. Despues del primer sync, se
  	   * limita a activarse solo si ya paso INTERVALO_FALLBACK_LORA_MS
  	   * desde el ultimo resync exitoso sin que el GPS lo haya logrado --
  	   * ej. antena sin vista al cielo por un rato largo. */
  	  bool necesitaFallbackLora = !huboResyncAlgunaVez ||
  			  (HAL_GetTick() - ultimoResyncExitosoTick >= INTERVALO_FALLBACK_LORA_MS);

  	  static bool horaSolicitada = false;
  	  static bool horaConsultada = false;

  	  if (RAK3172_EstaUnido() && necesitaFallbackLora && !horaSolicitada && RAK3172_ComandoListo()) {
  		  horaSolicitada = RAK3172_SolicitarHoraRed();
  		  if (horaSolicitada) {
  			  if (huboResyncAlgunaVez) {
  				  printf("RAK3172: sin resync por GPS en %lu min -- solicitando hora de red como respaldo (AT+TIMEREQ=1)\r\n",
  						 (unsigned long)(INTERVALO_FALLBACK_LORA_MS / 60000UL));
  			  } else {
  				  printf("RAK3172: hora de red solicitada (AT+TIMEREQ=1), compitiendo con el GPS por el primer sync...\r\n");
  			  }
  		  }
  	  }

  	  if (horaSolicitada && !horaConsultada && RAK3172_HoraDeRedDisponible() && RAK3172_ComandoListo()) {
  		  horaConsultada = RAK3172_ConsultarHoraRed();
  	  }

  	  if (horaConsultada && RAK3172_ComandoListo()) {
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
				  relojFueCorregidoEsteCiclo = true;
				  ultimoResyncExitosoTick = HAL_GetTick();
				  huboResyncAlgunaVez = true;
				  printf("RAK3172: reloj sincronizado con la red (respaldo): %02u:%02u:%02u UTC, %02u/%02u/%04u\r\n",
						 hora, minuto, segundo, mes, dia, anio);
			  } else {
				  printf("RAK3172: respuesta de AT+LTIME=? no reconocida: '%s'\r\n", respuesta);
			  }
		  }
		  /* Listo para el proximo ciclo (exito o fallo -- si fallo,
		   * necesitaFallbackLora sigue en true y se reintenta en la
		   * proxima vuelta del loop en vez de quedar trabado). */
		  horaSolicitada = false;
		  horaConsultada = false;
	  }

  	  /* ================== Estado del motor + uplink LIVE extendido ==================
  	   * ACTIVO requiere motor operando Y presion de salida real de la
  	   * motobomba (no solo RPM > 0) -- ver README seccion 4.2/5.
  	   * PRESION_UMBRAL_ACTIVO_PSI es un primer valor conservador sin
  	   * dato de campo todavia: distingue "presion real de bombeo" de
  	   * ruido/offset residual con la bomba sin carga. Ajustar con datos
  	   * reales del sistema hidraulico cuando esten disponibles. */
	  static uint8_t estadoAnterior = ESTADO_APAGADO;
	  static uint32_t inicioEstadoLocal = 0;
	  static bool estadoInicializado = false;
	  static uint32_t fechaHoraLocalIterAnterior = 0;

	  float rpmActual = Tacometro_GetRPMFiltrada();
	  float presionActual = Presion_GetPresionPsi();
	  uint8_t estadoActual;
	  if (rpmActual <= 0.0f) {
		  estadoActual = ESTADO_APAGADO;
	  } else if (Presion_SensorValido() && presionActual > PRESION_UMBRAL_ACTIVO_PSI) {
		  estadoActual = ESTADO_ACTIVO;
	  } else {
		  estadoActual = ESTADO_ENCENDIDO;
	  }

	  uint32_t fechaHoraLocalIterActual = Reloj_GetUnixTimeLocal();

	  if (!estadoInicializado || estadoActual != estadoAnterior) {
		  estadoAnterior = estadoActual;
		  inicioEstadoLocal = fechaHoraLocalIterActual;
		  estadoInicializado = true;
	  } else if (relojFueCorregidoEsteCiclo) {
		  /* El reloj se acaba de corregir (primer sync desde un valor
		   * aproximado/placeholder, o resync periodico contra la deriva
		   * del LSI) -- si el estado NO cambio, desplazar el ancla por
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
  			  presionActual,
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

  	  /* Reporte de resultado del último comando AT (join o send).
  	   * ⚠️ Detecta el FLANCO de "comando recien terminado" (en_curso ->
  	   * listo), NO un cambio de VALOR -- con el chequeo viejo
  	   * (imprimir solo si resultadoActual != el ultimo mostrado), dos
  	   * comandos consecutivos con el MISMO resultado (ej. TIMEOUT tras
  	   * TIMEOUT si el modulo deja de responder) se volvian invisibles
  	   * despues del primero: el log parecia "dejo de intentar" cuando
  	   * en realidad seguia intentando y fallando cada vez igual. */
  	  static bool comandoEnCursoAnterior = true;
  	  bool comandoEnCursoAhora = !RAK3172_ComandoListo();
  	  if (comandoEnCursoAnterior && !comandoEnCursoAhora) {
  		  printf("RAK3172: resultado=%d (0=OK,1=ERROR,2=TIMEOUT,3=BUSY)\r\n", RAK3172_GetUltimoResultado());
  	  }
  	  comandoEnCursoAnterior = comandoEnCursoAhora;

  	  static uint32_t ultimoReporte = 0;
  	  if (HAL_GetTick() - ultimoReporte >= 1000) {
  		  ultimoReporte = HAL_GetTick();
  		  printf("Frecuencia: %.1f Hz | RPM instant: %.1f | RPM filtrada: %.1f | Detenido: %d | RuidoFiltrado: %lu | RelojSync: %d | Presion: %.2f PSI (%.2fmA%s)\r\n",
  			  Tacometro_GetFrecuenciaHz(),
  			  Tacometro_GetRPMInstantanea(),
  			  Tacometro_GetRPMFiltrada(),
  			  Tacometro_EstaDetenido(),
  			  Tacometro_GetContadorRuidoFiltrado(),
  			  Reloj_EstaSincronizado(),
  			  Presion_GetPresionPsi(),
  			  Presion_GetCorrienteMa(),
  			  Presion_SensorValido() ? "" : ",FALLA_SENSOR");
  	  }

  	  /* Log de alta frecuencia para identificar el modelo de la planta
  	   * (motor+servo) y probar escalones de SET_RPM -- ver README
  	   * sección 9. Prefijo "PID_TEST," fijo para poder filtrar estas
  	   * líneas con un script (ignorando el resto del log, que sigue
  	   * intercalado) sin parsear el resto del texto.
  	   * Usa HAL_GetTick() (ms desde el arranque) y no la hora del RTC a
  	   * propósito: el RTC se resincroniza cada ~30s por GPS/LoRaWAN (ver
  	   * sección 2.2) y esos saltos de corrección contaminarían el
  	   * tiempo relativo de un escalón en curso -- HAL_GetTick() es
  	   * monótono y no depende de si el reloj ya sincronizó.
  	   *
  	   * Gateado por CONTROL_HABILITADO==7 (modo sintonización PID, ver
  	   * README sección 4.4/9): fuera de ese modo esto no imprime nada
  	   * (el monitor se ve igual que antes de que existiera este log).
  	   * Se apaga solo en cuanto se sale del modo 7 (a diferencia de una
  	   * bandera de una sola vía, sigue el mismo estado que ya gatea si
  	   * PID_KP/KI/KD se pueden tocar -- una sola fuente de verdad). */
  	  static uint32_t ultimoLogPrueba = 0;
  	  if (CalibFlash_GetControlHabilitado() == 7U && HAL_GetTick() - ultimoLogPrueba >= 200U) {
  		  ultimoLogPrueba = HAL_GetTick();
  		  printf("PID_TEST,%lu,%.1f,%.1f,%u\r\n",
  			  (unsigned long)HAL_GetTick(),
  			  CalibFlash_GetSetRpm(),
  			  Tacometro_GetRPMFiltrada(),
  			  Servo_GetPulsoActualUs());
  	  }

  	  /* Imprime las ganancias vigentes del PID (Kp/Ki/Kd) apenas se
  	   * entra a CONTROL_HABILITADO=7, y cada vez que alguna cambia
  	   * mientras se sigue ahi -- cubre tanto el mando manual de serial
  	   * como un downlink LoRa (los dos pasan por
  	   * CalibFlash_ProcesarParametroConEstado, esto solo lee el
  	   * resultado ya aplicado). Agregado 2026-09-08 tras un incidente
  	   * de campo donde un reset de la calibracion en flash (magic
  	   * distinto por un cambio de estructura) dejo Kp/Ki en los
  	   * valores de fabrica sin que fuera obvio en el monitor serial --
  	   * ver README seccion 9. */
  	  static uint8_t  modoAnteriorParaGanancias = 0U;
  	  static float    pidKpAnteriorImpreso = 0.0f;
  	  static float    pidKiAnteriorImpreso = 0.0f;
  	  static float    pidKdAnteriorImpreso = 0.0f;
  	  uint8_t modoActualParaGanancias = CalibFlash_GetControlHabilitado();
  	  if (modoActualParaGanancias == 7U) {
  		  float pidKpActual = CalibFlash_GetPidKp();
  		  float pidKiActual = CalibFlash_GetPidKi();
  		  float pidKdActual = CalibFlash_GetPidKd();
  		  if (modoAnteriorParaGanancias != 7U ||
  		      pidKpActual != pidKpAnteriorImpreso ||
  		      pidKiActual != pidKiAnteriorImpreso ||
  		      pidKdActual != pidKdAnteriorImpreso) {
  			  printf("PID_GANANCIAS,Kp=%.4f,Ki=%.4f,Kd=%.4f\r\n",
  				  pidKpActual, pidKiActual, pidKdActual);
  			  pidKpAnteriorImpreso = pidKpActual;
  			  pidKiAnteriorImpreso = pidKiActual;
  			  pidKdAnteriorImpreso = pidKdActual;
  		  }
  	  }
  	  modoAnteriorParaGanancias = modoActualParaGanancias;

  	  /* Log de calibración del servo -- gateado a CONTROL_HABILITADO=1
  	   * (barrido) o =2 (manual), ver README sección 2.4/4.4. Antes de
  	   * esto, la única forma de saber los límites vigentes en banco era
  	   * el ACK de cada downlink de SERVO_PULSO_MIN/MAX ("valor
  	   * vigente=X") -- esto muestra en vivo, mientras se observa el
  	   * servo moverse, dónde están los límites configurados y qué pulso
  	   * se le está aplicando en ese instante. Cadencia de 1s (a
  	   * diferencia de PID_TEST, esto es solo para lectura humana, no se
  	   * analiza después con un script). */
  	  static uint32_t ultimoLogServoCal = 0;
  	  uint8_t modoParaLogServo = CalibFlash_GetControlHabilitado();
  	  if ((modoParaLogServo == 1U || modoParaLogServo == 2U) &&
  	      HAL_GetTick() - ultimoLogServoCal >= 1000U) {
  		  ultimoLogServoCal = HAL_GetTick();
  		  printf("SERVO_CAL,min=%uus,max=%uus,pulso_logico=%uus,pulso_fisico=%uus\r\n",
  			  CalibFlash_GetServoPulsoMinUs(),
  			  CalibFlash_GetServoPulsoMaxUs(),
  			  Servo_GetPulsoActualUs(),
  			  Servo_GetPulsoFisicoActualUs());
  	  }


  	  /* Apagado local de seguridad: si el motor arranca mientras se
  	   * está en modo de calibración del SERVO (CONTROL_HABILITADO=1 o
  	   * =2), se sale de inmediato -- sin esperar ningún downlink -- para
  	   * no seguir moviendo el servo (en barrido o manual) con el motor
  	   * operando. Mismo criterio que el resto del firmware
  	   * (Tacometro_EstaDetenido()), nunca un switch remoto para algo de
  	   * seguridad física.
  	   * NO incluye el modo 7 (sintonización de PID) a propósito: ese
  	   * modo necesita que el motor siga operando durante toda la
  	   * sesión (README sección 9) -- el servo en modo 7 lo maneja el
  	   * PID normal exactamente igual que en modo 0, así que no hay
  	   * barrido/posición manual que "se quede corriendo" sin control. */
  	  uint8_t modoCalibracionServo = CalibFlash_GetControlHabilitado();
  	  if (!Tacometro_EstaDetenido() && (modoCalibracionServo == 1U || modoCalibracionServo == 2U)) {
  		  CalibFlash_SetControlHabilitado(0U);
  		  CalibFlash_LimpiarObjetivoManualServo(); /* invalida cualquier
  		                                             * objetivo manual pendiente,
  		                                             * no arrastrarlo a una
  		                                             * futura sesion de calibracion */
  		  CalibFlash_SetSetRpm(0.0f); /* mismo criterio que al ENTRAR a 1/2/3
  		                                 * (calibracion_flash.c) -- si habia
  		                                 * quedado un SET_RPM viejo mandado
  		                                 * mientras se calibraba el servo, no
  		                                 * debe activar el PID solo al volver
  		                                 * a modo 0 por este apagado de
  		                                 * seguridad. */
  	  }

  	  /* PRUEBA DE BANCO -- servo en modo calibración mientras
  	   * CONTROL_HABILITADO != 0 (ver protocolo de downlinks): =1 barrido
  	   * automático MIN<->MAX, =2 manual (se queda quieto salvo un
  	   * downlink nuevo de SERVO_PULSO_MIN/MAX), =3 sintonización de PID
  	   * (el servo NO cambia de comportamiento respecto a =0 -- sigue
  	   * siendo pid.c quien lo maneja -- lo único que cambia con =3 es
  	   * que se desbloquean PID_KP/KI/KD y se activa el log PID_TEST, ver
  	   * calibracion_flash.c). QUITAR/comentar este bloque si algún día
  	   * se retira también la calibración remota. */
  	  bool motorOperandoAhora = !Tacometro_EstaDetenido();
  	  float rpmMin = CalibFlash_GetRpmMin();
  	  float rpmMax = CalibFlash_GetRpmMax();

  	  /* ================== MODO: de donde sale el setpoint de RPM ==================
  	   * Agregado 2026-09-07 -- antes MODO se recibia/persistia/ACKeaba
  	   * pero nadie lo leia aca, asi que TODO motor se comportaba siempre
  	   * como MODO_LOCAL sin importar su valor real. Ahora si rama:
  	   *
  	   *   RALENTI (0): sin control activo, sin importar SET_RPM/PRESION
  	   *     -- setpoint forzado a "sin comandar" (0.0f, mismo valor que
  	   *     el default de SET_RPM). Cae en la rama de "sin control" de
  	   *     mas abajo (servo en SERVO_PULSO_MIN), igual que siempre.
  	   *   LOCAL (1): setpoint = SET_RPM (downlink directo) -- el
  	   *     comportamiento historico, de facto el unico que existia
  	   *     antes de este cambio.
  	   *   REMOTO (2): setpoint = PRESION_OFFSET_RPM + PRESION_GANANCIA_RPM
  	   *     * PRESION -- formula lineal simple. NO existe (todavia) una
  	   *     relacion presion->RPM conocida de antemano (depende del
  	   *     sistema hidraulico motor+aspersores real) -- por eso, igual
  	   *     que PID_KP/KI/KD, esta ganancia/offset se calibran EN CAMPO
  	   *     (solo se aceptan con CONTROL_HABILITADO=7, ver
  	   *     calibracion_flash.c) en vez de asumir una formula fija.
  	   *
  	   * ⚠️ MIGRACION -- unidades ya desplegadas: como MODO nunca se leia,
  	   * cualquier nodo ya operando con SET_RPM (sin haber tocado MODO
  	   * jamas) va a arrancar en RALENTI tras este cambio (0 sigue siendo
  	   * el default) y va a DEJAR de responder a SET_RPM hasta que se
  	   * mande "MODO 1" explicito. Mandar ese downlink a cada nodo activo
  	   * antes/justo despues de actualizar este firmware. */
  	  CalibFlash_Modo_t modoMotor = CalibFlash_GetModo();
  	  float setpointRpmCrudo = 0.0f;

  	  /* Watchdog de TIMEOUT_SIN_COMANDO_S, solo aplica en REMOTO: si no
  	   * llega una PRESION valida en mas de ese tiempo (cuenta desde el
  	   * boot si nunca llego ninguna), se fuerza el setpoint a "sin
  	   * comandar" -- el motor cae a ralenti natural en vez de sostener
  	   * indefinidamente el ultimo valor de presion, ya viejo. */
  	  static bool presionExpiradaAntes = false;
  	  bool presionExpiradaAhora = false;

  	  if (modoMotor == CALIB_MODO_LOCAL) {
  		  setpointRpmCrudo = CalibFlash_GetSetRpm();
  	  } else if (modoMotor == CALIB_MODO_REMOTO) {
  		  uint32_t timeoutMs = CalibFlash_GetTimeoutSinComandoS() * 1000UL;
  		  presionExpiradaAhora = (HAL_GetTick() - CalibFlash_GetPresionUltimoTickMs()) >= timeoutMs;
  		  if (presionExpiradaAhora) {
  			  setpointRpmCrudo = 0.0f; /* sin comandar -- cae a ralenti natural mas abajo */
  		  } else {
  			  setpointRpmCrudo = CalibFlash_GetPresionOffsetRpm()
  					  + CalibFlash_GetPresionGananciaRpm() * CalibFlash_GetPresion();
  		  }

  		  if (presionExpiradaAhora != presionExpiradaAntes) {
  			  if (presionExpiradaAhora) {
  				  printf("MODO_REMOTO: sin PRESION valida hace mas de %lus -- forzando ralenti (TIMEOUT_SIN_COMANDO_S)\r\n",
  						 (unsigned long)CalibFlash_GetTimeoutSinComandoS());
  			  } else {
  				  printf("MODO_REMOTO: PRESION valida de nuevo -- reanudando control por presion\r\n");
  			  }
  		  }
  	  } else {
  		  presionExpiradaAhora = false; /* fuera de REMOTO, no aplica -- que la proxima entrada a REMOTO loguee fresco */
  	  }
  	  presionExpiradaAntes = presionExpiradaAhora;
  	  /* RALENTI (0): setpointRpmCrudo se queda en 0.0f -- sin control, ver
  	   * comentario del bloque de arriba. */

  	  /* Modo 0 (ralentí) = sin control activo: el motor sube solo de
  	   * 0Hz a su ralentí natural (distinto en cada unidad, por eso
  	   * RPM_MIN es "límite duro / ralentí" y no una constante) sin que
  	   * el PID intervenga -- el servo se queda quieto en
  	   * SERVO_PULSO_MIN. El lazo solo se activa si el setpoint elegido
  	   * arriba (según MODO) supera ese ralentí configurado; 0.0f (sin
  	   * comandar, o MODO_RALENTI) o apenas unos pocos RPM nunca debe
  	   * forzar el servo. */
  	  bool controlSolicitado = setpointRpmCrudo > rpmMin;

  	  uint8_t modoControl = CalibFlash_GetControlHabilitado(); /* 0=desactivado, 1=barrido, 2=manual, 3=auto ralenti, 4=auto zona muerta, 5=auto ALPHA, 6=mapeo ganancia, 7=sintonizacion PID */

  	  /* Auto-calibracion de ralenti -- CONTROL_HABILITADO=3 (ver
  	   * calibracion_flash.c: solo se puede pedir viniendo de modo 0, con
  	   * o sin el motor operando). Mientras se esta en modo 3:
  	   *   - si el motor no esta operando, se espera -- el servo ya
  	   *     queda en SERVO_PULSO_MIN automaticamente (pidActivoAhora es
  	   *     siempre false en modo 3, cae en la rama de abajo).
  	   *   - apenas el motor arranca (o si ya estaba andando al entrar a
  	   *     este modo), arranca la ventana de medicion.
  	   *   - si el motor se detiene a mitad de la ventana, se aborta la
  	   *     medicion en curso y se vuelve a esperar (sin salir de modo
  	   *     4) -- no tiene sentido promediar con el motor parado, pero
  	   *     tampoco hace falta reenviar el downlink si se puede
  	   *     reintentar solo arrancando el motor de nuevo.
  	   *   - al completar los 2 minutos, se aplica RPM_MIN y se vuelve
  	   *     sola a modo 0 (mismo criterio que el apagado de seguridad de
  	   *     1/2 en este mismo loop, que tambien fuerza CONTROL_HABILITADO
  	   *     localmente sin esperar un downlink). */
  	  {
  		  static bool     ralentiCalEsperandoMotor = false;
  		  static bool     ralentiCalMidiendo = false;
  		  static uint32_t ralentiCalInicioMs = 0;
  		  static uint32_t ralentiCalUltimoLogMs = 0;
  		  static float    ralentiCalSuma = 0.0f;
  		  static uint32_t ralentiCalMuestras = 0;

  		  if (modoControl == 3U) {
  			  if (!ralentiCalEsperandoMotor && !ralentiCalMidiendo) {
  				  ralentiCalEsperandoMotor = true;
  				  printf("RALENTI_CAL,INICIO,esperando_motor=%d\r\n", (int)!motorOperandoAhora);
  			  }

  			  if (ralentiCalEsperandoMotor && motorOperandoAhora) {
  				  ralentiCalEsperandoMotor = false;
  				  ralentiCalMidiendo = true;
  				  ralentiCalInicioMs = HAL_GetTick();
  				  ralentiCalUltimoLogMs = ralentiCalInicioMs;
  				  ralentiCalSuma = 0.0f;
  				  ralentiCalMuestras = 0;
  				  printf("RALENTI_CAL,MIDIENDO,duracion_total_s=%lu,ventana_promedio_s=%lu\r\n",
  					  (unsigned long)(RALENTI_CAL_DURACION_TOTAL_MS / 1000U),
  					  (unsigned long)(RALENTI_CAL_VENTANA_PROMEDIO_MS / 1000U));
  			  }

  			  if (ralentiCalMidiendo) {
  				  if (!motorOperandoAhora) {
  					  printf("RALENTI_CAL,ABORTADO,motivo=motor_se_detuvo\r\n");
  					  ralentiCalMidiendo = false;
  					  ralentiCalEsperandoMotor = true;
  				  } else {
  					  uint32_t transcurridoMs = HAL_GetTick() - ralentiCalInicioMs;

  					  if (transcurridoMs >= (RALENTI_CAL_DURACION_TOTAL_MS - RALENTI_CAL_VENTANA_PROMEDIO_MS)) {
  						  ralentiCalSuma += Tacometro_GetRPMFiltrada();
  						  ralentiCalMuestras++;
  					  }

  					  if (HAL_GetTick() - ralentiCalUltimoLogMs >= 5000U) {
  						  ralentiCalUltimoLogMs = HAL_GetTick();
  						  printf("RALENTI_CAL,PROGRESO,transcurrido_s=%lu,rpm_actual=%.1f\r\n",
  							  (unsigned long)(transcurridoMs / 1000U), Tacometro_GetRPMFiltrada());
  					  }

  					  if (transcurridoMs >= RALENTI_CAL_DURACION_TOTAL_MS) {
  						  if (ralentiCalMuestras > 0U) {
  							  float promedio = ralentiCalSuma / (float)ralentiCalMuestras;
  							  float nuevoRpmMin = promedio + RALENTI_CAL_MARGEN_RPM;
  							  bool ok = CalibFlash_SetRpmMin(nuevoRpmMin);
  							  printf("RALENTI_CAL,COMPLETO,promedio=%.1f,RPM_MIN_nuevo=%.1f,muestras=%lu,ok=%d\r\n",
  								  promedio, nuevoRpmMin, (unsigned long)ralentiCalMuestras, (int)ok);
  							  CalibFlash_ForzarReporte(); /* "ACK" por LoRa -- uplink
  							                                 * inmediato con el reporte
  							                                 * estandar, en vez de
  							                                 * esperar al proximo
  							                                 * intervalo periodico */
  						  } else {
  							  printf("RALENTI_CAL,ERROR,sin_muestras\r\n");
  						  }
  						  ralentiCalMidiendo = false;
  						  CalibFlash_SetControlHabilitado(0U); /* vuelve sola a operacion normal */
  					  }
  				  }
  			  }
  		  } else {
  			  ralentiCalEsperandoMotor = false;
  			  ralentiCalMidiendo = false;
  		  }
  	  }

  	  /* Auto-calibracion de ALPHA -- CONTROL_HABILITADO=5 (ver
  	   * calibracion_flash.c: solo se puede pedir viniendo de modo 0, con
  	   * o sin el motor operando). Mismo patron que el bloque de modo 3
  	   * arriba -- espera el motor, mide, aplica, vuelve sola a modo 0. */
  	  {
  		  static bool     alphaCalEsperandoMotor = false;
  		  static bool     alphaCalMidiendo = false;
  		  static uint32_t alphaCalInicioMs = 0;
  		  static uint32_t alphaCalUltimoMuestreoMs = 0;
  		  static uint32_t alphaCalMuestras = 0;
  		  static float    alphaCalMedia = 0.0f;
  		  static float    alphaCalM2 = 0.0f; /* Welford -- evita cancelacion catastrofica al restar RPM~cientos/miles al cuadrado */

  		  if (modoControl == 5U) {
  			  if (!alphaCalEsperandoMotor && !alphaCalMidiendo) {
  				  alphaCalEsperandoMotor = true;
  				  printf("ALPHA_CAL,INICIO,esperando_motor=%d\r\n", (int)!motorOperandoAhora);
  			  }

  			  if (alphaCalEsperandoMotor && motorOperandoAhora) {
  				  alphaCalEsperandoMotor = false;
  				  alphaCalMidiendo = true;
  				  alphaCalInicioMs = HAL_GetTick();
  				  alphaCalUltimoMuestreoMs = alphaCalInicioMs;
  				  alphaCalMuestras = 0U;
  				  alphaCalMedia = 0.0f;
  				  alphaCalM2 = 0.0f;
  				  printf("ALPHA_CAL,MIDIENDO,duracion_s=%lu\r\n", (unsigned long)(ALPHA_CAL_VENTANA_MS / 1000U));
  			  }

  			  if (alphaCalMidiendo) {
  				  if (!motorOperandoAhora) {
  					  printf("ALPHA_CAL,ABORTADO,motivo=motor_se_detuvo\r\n");
  					  alphaCalMidiendo = false;
  					  alphaCalEsperandoMotor = true;
  				  } else {
  					  uint32_t transcurridoMs = HAL_GetTick() - alphaCalInicioMs;

  					  if (HAL_GetTick() - alphaCalUltimoMuestreoMs >= ALPHA_CAL_INTERVALO_MUESTREO_MS) {
  						  alphaCalUltimoMuestreoMs = HAL_GetTick();
  						  float muestra = Tacometro_GetRPMInstantanea();
  						  alphaCalMuestras++;
  						  float delta = muestra - alphaCalMedia;
  						  alphaCalMedia += delta / (float)alphaCalMuestras;
  						  float delta2 = muestra - alphaCalMedia;
  						  alphaCalM2 += delta * delta2;
  					  }

  					  if (transcurridoMs >= ALPHA_CAL_VENTANA_MS) {
  						  if (alphaCalMuestras >= 2U) {
  							  float varianza = alphaCalM2 / (float)(alphaCalMuestras - 1U);
  							  float desviacionCruda = sqrtf(varianza);
  							  float r = (desviacionCruda > ALPHA_CAL_OBJETIVO_DESVIACION_RPM)
  										  ? (ALPHA_CAL_OBJETIVO_DESVIACION_RPM / desviacionCruda)
  										  : 1.0f; /* ruido crudo ya en/por debajo del objetivo -- el techo de abajo evita que esto se traduzca en "sin filtro" */
  							  float alphaCalculado = (2.0f * r * r) / (1.0f + r * r);
  							  if (alphaCalculado > ALPHA_CAL_TECHO_ALPHA) {
  								  alphaCalculado = ALPHA_CAL_TECHO_ALPHA;
  							  }
  							  bool ok = CalibFlash_SetAlphaFiltro(alphaCalculado);
  							  printf("ALPHA_CAL,COMPLETO,desviacion_cruda=%.2f,alpha_nuevo=%.4f,muestras=%lu,ok=%d\r\n",
  								  desviacionCruda, CalibFlash_GetAlphaFiltro(), (unsigned long)alphaCalMuestras, (int)ok);
  							  CalibFlash_ForzarReporte(); /* "ACK" por LoRa, mismo criterio que ralenti/servo-min */
  						  } else {
  							  printf("ALPHA_CAL,ERROR,sin_muestras_suficientes\r\n");
  						  }
  						  alphaCalMidiendo = false;
  						  CalibFlash_SetControlHabilitado(0U); /* vuelve sola a operacion normal */
  					  }
  				  }
  			  }
  		  } else {
  			  alphaCalEsperandoMotor = false;
  			  alphaCalMidiendo = false;
  		  }
  	  }

  	  static bool pidActivoAntes = false;
  	  /* Modo 7 (sintonizacion PID) usa el MISMO camino de control que el
  	   * modo 0 -- el servo lo maneja pid.c igual en ambos casos, la unica
  	   * diferencia es que en modo 7 ademas se permite tocar PID_KP/KI/KD
  	   * (ver calibracion_flash.c) y se activa el log PID_TEST de mas
  	   * arriba. No hace falta un branch aparte para 7 en el if/else de
  	   * abajo -- como no es 1 ni 2, cae solo en esta rama. */
  	  bool pidActivoAhora = (modoControl == 0U || modoControl == 7U) && motorOperandoAhora && controlSolicitado;
  	  if (pidActivoAhora && !pidActivoAntes) {
  		  PID_Init(); /* flanco de entrada -- evita un dt inflado por tiempo inactivo */
  	  }
  	  pidActivoAntes = pidActivoAhora;

  	  static uint8_t modoControlAnterior = 0U;

  	  if (modoControl == 1U) {
  		  /* Pausa en cada extremo (SERVO_BARRIDO_PAUSA_EXTREMOS_MS,
  		   * servo.h) antes de invertir direccion -- el pulso PWM llega
  		   * exacto al limite en cuanto Servo_MoverHacia() lo reporta,
  		   * pero el servo fisico tiene su propio tiempo de asentamiento;
  		   * sin esta espera, el firmware invertia la marcha en la misma
  		   * vuelta del loop en que el pulso llegaba al extremo, y el
  		   * brazo real nunca alcanzaba a terminar de llegar. */
  		  static bool haciaMaximoServo = true;
  		  static bool esperandoEnExtremo = false;
  		  static uint32_t inicioEsperaExtremoMs = 0;
  		  uint16_t minimo = CalibFlash_GetServoPulsoMinUs();
  		  uint16_t maximo = CalibFlash_GetServoPulsoMaxUs();
  		  uint16_t destinoServo = haciaMaximoServo ? maximo : minimo;
  		  uint16_t aplicadoServo = Servo_MoverHacia(destinoServo);
  		  if (!esperandoEnExtremo) {
  			  if (aplicadoServo == destinoServo) {
  				  esperandoEnExtremo = true;
  				  inicioEsperaExtremoMs = HAL_GetTick();
  			  }
  		  } else if (HAL_GetTick() - inicioEsperaExtremoMs >= SERVO_BARRIDO_PAUSA_EXTREMOS_MS) {
  			  haciaMaximoServo = !haciaMaximoServo;
  			  esperandoEnExtremo = false;
  		  }
  	  } else if (modoControl == 2U) {
  		  /* Modo manual de calibración (CONTROL_HABILITADO=2): a
  		   * diferencia del barrido (=1), el servo se queda quieto en su
  		   * posición -- solo se mueve cuando llega un downlink de
  		   * SERVO_PULSO_MIN o SERVO_PULSO_MAX, yendo directo a ese valor
  		   * (para posicionar el servo en un punto exacto durante el
  		   * ajuste en banco, sin esperar una vuelta completa del
  		   * barrido). El "llegó un downlink nuevo" se consume como
  		   * bandera (CalibFlash_HayObjetivoManualServo(), fijada en
  		   * calibracion_flash.c en el momento del downlink) -- NO se
  		   * detecta comparando contra el valor anterior, porque el
  		   * valor pedido puede coincidir con el que ya estaba (ej. los
  		   * defaults de fábrica, 1000/2000) y un downlink real igual
  		   * debe mover el servo. */
  		  static uint16_t objetivoManualServoUs;
  		  if (modoControlAnterior != 2U) {
  			  /* Flanco de entrada a este modo -- se queda donde está,
  			   * no salta a MIN/MAX hasta que llegue un downlink real. */
  			  objetivoManualServoUs = Servo_GetPulsoActualUs();
  		  }
  		  if (CalibFlash_HayObjetivoManualServo()) {
  			  objetivoManualServoUs = CalibFlash_GetObjetivoManualServoUs();
  			  CalibFlash_LimpiarObjetivoManualServo();
  		  }
  		  Servo_MoverHacia(objetivoManualServoUs);
  	  } else if (modoControl == 4U) {
  		  /* Calibración automática de SERVO_PULSO_MIN (umbral de
  		   * aceleración) -- a diferencia de 1/2, SÍ mira la RPM en cada
  		   * paso, así que puede entrar sin el motor operando y
  		   * simplemente esperar. Estados: 1=esperando motor,
  		   * 2=midiendo línea base fresca en el SERVO_PULSO_MIN actual,
  		   * 3=barriendo hacia arriba en pasos chicos. Ver README
  		   * sección 10 para el diseño completo y el análisis de riesgo. */
  		  static uint8_t  servoMinCalEstado = 1U;
  		  static uint32_t servoMinCalInicioEtapaMs = 0;
  		  static float    servoMinCalSuma = 0.0f;
  		  static uint32_t servoMinCalMuestras = 0;
  		  static float    servoMinCalBaselineRpm = 0.0f;
  		  static uint16_t servoMinCalPulsoBase = 0;
  		  static uint16_t servoMinCalPulsoPaso = 0;
  		  static uint8_t  servoMinCalConfirmaciones = 0;

  		  if (modoControlAnterior != 4U) {
  			  /* Flanco de entrada -- siempre arranca esperando el motor,
  			   * aunque ya esté operando (el siguiente bloque lo detecta
  			   * en la misma vuelta del loop si ya está corriendo). */
  			  servoMinCalEstado = 1U;
  			  printf("SERVOMIN_CAL,INICIO,esperando_motor=%d\r\n", (int)!motorOperandoAhora);
  		  }

  		  if (servoMinCalEstado == 1U && motorOperandoAhora) {
  			  servoMinCalPulsoBase = CalibFlash_GetServoPulsoMinUs();
  			  servoMinCalEstado = 2U;
  			  servoMinCalInicioEtapaMs = HAL_GetTick();
  			  servoMinCalSuma = 0.0f;
  			  servoMinCalMuestras = 0U;
  			  printf("SERVOMIN_CAL,BASELINE,pulso_base=%u,duracion_s=%lu\r\n",
  				  servoMinCalPulsoBase, (unsigned long)(SERVOMIN_CAL_BASELINE_MS / 1000U));
  		  }

  		  if (servoMinCalEstado == 2U && !motorOperandoAhora) {
  			  printf("SERVOMIN_CAL,ABORTADO,motivo=motor_se_detuvo\r\n");
  			  servoMinCalEstado = 1U;
  		  } else if (servoMinCalEstado == 2U) {
  			  servoMinCalSuma += Tacometro_GetRPMFiltrada();
  			  servoMinCalMuestras++;
  			  if (HAL_GetTick() - servoMinCalInicioEtapaMs >= SERVOMIN_CAL_BASELINE_MS) {
  				  servoMinCalBaselineRpm = servoMinCalSuma / (float)servoMinCalMuestras;
  				  servoMinCalPulsoPaso = servoMinCalPulsoBase;
  				  servoMinCalConfirmaciones = 0U;
  				  servoMinCalEstado = 3U;
  				  servoMinCalInicioEtapaMs = HAL_GetTick();
  				  printf("SERVOMIN_CAL,BARRIENDO,baseline_rpm=%.1f,pulso_inicial=%u\r\n",
  					  servoMinCalBaselineRpm, servoMinCalPulsoPaso);
  			  }
  		  }

  		  if (servoMinCalEstado == 3U && !motorOperandoAhora) {
  			  printf("SERVOMIN_CAL,ABORTADO,motivo=motor_se_detuvo\r\n");
  			  servoMinCalEstado = 1U;
  		  } else if (servoMinCalEstado == 3U &&
  		             HAL_GetTick() - servoMinCalInicioEtapaMs >= SERVOMIN_CAL_DWELL_MS) {
  			  float rpmActual = Tacometro_GetRPMFiltrada();
  			  float subidaRpm = rpmActual - servoMinCalBaselineRpm;
  			  printf("SERVOMIN_CAL,PASO,pulso=%u,rpm=%.1f,subida=%.1f,confirmaciones=%u\r\n",
  				  servoMinCalPulsoPaso, rpmActual, subidaRpm, servoMinCalConfirmaciones);

  			  if (subidaRpm >= SERVOMIN_CAL_SALTO_ANORMAL_RPM) {
  				  /* Salto mucho mas grande de lo esperado en un solo
  				   * paso -- frenar ya, no seguir empujando ni aplicar
  				   * ningun resultado automatico. */
  				  printf("SERVOMIN_CAL,ABORTADO,motivo=salto_rpm_anormal\r\n");
  				  servoMinCalEstado = 1U;
  				  CalibFlash_SetControlHabilitado(0U);
  			  } else if (subidaRpm >= SERVOMIN_CAL_UMBRAL_DETECCION_RPM) {
  				  servoMinCalConfirmaciones++;
  				  if (servoMinCalConfirmaciones >= SERVOMIN_CAL_CONFIRMACIONES_NECESARIAS) {
  					  uint16_t margenUs = (uint16_t)(SERVOMIN_CAL_MARGEN_PASOS * SERVOMIN_CAL_PASO_US);
  					  uint16_t nuevoServoMin = servoMinCalPulsoBase;
  					  if (servoMinCalPulsoPaso > (uint16_t)(servoMinCalPulsoBase + margenUs)) {
  						  nuevoServoMin = servoMinCalPulsoPaso - margenUs;
  					  }
  					  bool ok = CalibFlash_SetServoPulsoMinUs(nuevoServoMin);
  					  printf("SERVOMIN_CAL,COMPLETO,pulso_umbral=%u,SERVO_PULSO_MIN_nuevo=%u,ok=%d\r\n",
  						  servoMinCalPulsoPaso, nuevoServoMin, (int)ok);
  					  CalibFlash_ForzarReporte();
  					  servoMinCalEstado = 1U;
  					  CalibFlash_SetControlHabilitado(0U);
  				  } else {
  					  /* Sostener el mismo pulso un paso mas para
  					   * confirmar antes de dar la deteccion por buena. */
  					  servoMinCalInicioEtapaMs = HAL_GetTick();
  				  }
  			  } else {
  				  servoMinCalConfirmaciones = 0U;
  				  uint16_t siguientePulso = (uint16_t)(servoMinCalPulsoPaso + SERVOMIN_CAL_PASO_US);
  				  if (siguientePulso > (uint16_t)(servoMinCalPulsoBase + SERVOMIN_CAL_TECHO_US)) {
  					  printf("SERVOMIN_CAL,ERROR,motivo=techo_alcanzado_sin_deteccion\r\n");
  					  servoMinCalEstado = 1U;
  					  CalibFlash_SetControlHabilitado(0U);
  				  } else {
  					  servoMinCalPulsoPaso = siguientePulso;
  					  servoMinCalInicioEtapaMs = HAL_GetTick();
  				  }
  			  }
  		  }

  		  /* El pulso real aplicado sigue siempre el estado actual: en
  		   * SERVO_PULSO_MIN mientras se espera el motor o se mide la
  		   * línea base, y en el pulso del paso en curso mientras se
  		   * barre. Servo_MoverHacia() respeta SERVO_VELOCIDAD_MAX_US_S,
  		   * así que cada paso llega rampeado, no de un salto. */
  		  uint16_t destinoServoMinCal = (servoMinCalEstado == 3U) ? servoMinCalPulsoPaso : CalibFlash_GetServoPulsoMinUs();
  		  Servo_MoverHacia(destinoServoMinCal);
  	  } else if (modoControl == 6U) {
  		  /* Mapeo de curva de ganancia -- ver definicion de las
  		   * constantes GANANCIA_CAL_* mas arriba. Barre en lazo ABIERTO
  		   * (sin PID) desde SERVO_PULSO_MIN, logueando (pulso, RPM) en
  		   * cada paso, hasta GANANCIA_CAL_RPM_TECHO o SERVO_PULSO_MAX.
  		   * Solo mide/loguea -- no aplica ninguna correccion todavia.
  		   * Estados: 1=esperando motor, 2=barriendo. */
  		  static uint8_t  ganCalEstado = 1U;
  		  static uint32_t ganCalInicioEtapaMs = 0;
  		  static uint16_t ganCalPulsoPaso = 0;
  		  static float    ganCalRpmAnterior = 0.0f;

  		  if (modoControlAnterior != 6U) {
  			  ganCalEstado = 1U;
  			  printf("GANANCIA_CAL,INICIO,esperando_motor=%d\r\n", (int)!motorOperandoAhora);
  		  }

  		  if (ganCalEstado == 1U && motorOperandoAhora) {
  			  ganCalPulsoPaso = CalibFlash_GetServoPulsoMinUs();
  			  ganCalRpmAnterior = Tacometro_GetRPMFiltrada();
  			  ganCalEstado = 2U;
  			  ganCalInicioEtapaMs = HAL_GetTick();
  			  printf("GANANCIA_CAL,BARRIENDO,pulso_inicial=%u,rpm_techo=%.0f\r\n",
  				  ganCalPulsoPaso, GANANCIA_CAL_RPM_TECHO);
  		  }

  		  if (ganCalEstado == 2U && !motorOperandoAhora) {
  			  printf("GANANCIA_CAL,ABORTADO,motivo=motor_se_detuvo\r\n");
  			  ganCalEstado = 1U;
  		  } else if (ganCalEstado == 2U &&
  		             HAL_GetTick() - ganCalInicioEtapaMs >= GANANCIA_CAL_DWELL_MS) {
  			  float rpmActual = Tacometro_GetRPMFiltrada();
  			  float saltoRpm = rpmActual - ganCalRpmAnterior;
  			  printf("GANANCIA_CAL,PASO,pulso=%u,rpm=%.1f,salto=%.1f\r\n",
  				  ganCalPulsoPaso, rpmActual, saltoRpm);
  			  ganCalRpmAnterior = rpmActual;

  			  if (saltoRpm >= GANANCIA_CAL_SALTO_ANORMAL_RPM) {
  				  /* Protege contra pasarse del techo de RPM de un salto
  				   * grande en la zona de mas ganancia, y detecta
  				   * lecturas fuera de lo esperado -- frenar ya. */
  				  printf("GANANCIA_CAL,ABORTADO,motivo=salto_rpm_anormal\r\n");
  				  ganCalEstado = 1U;
  				  CalibFlash_SetControlHabilitado(0U);
  			  } else if (rpmActual >= GANANCIA_CAL_RPM_TECHO) {
  				  printf("GANANCIA_CAL,COMPLETO,motivo=techo_rpm_alcanzado,pulso_final=%u,rpm_final=%.1f\r\n",
  					  ganCalPulsoPaso, rpmActual);
  				  ganCalEstado = 1U;
  				  CalibFlash_SetControlHabilitado(0U);
  			  } else {
  				  uint16_t siguientePulso = (uint16_t)(ganCalPulsoPaso + GANANCIA_CAL_PASO_US);
  				  uint16_t maximoServo = CalibFlash_GetServoPulsoMaxUs();
  				  if (siguientePulso > maximoServo) {
  					  printf("GANANCIA_CAL,ERROR,motivo=servo_pulso_max_alcanzado_sin_llegar_al_techo\r\n");
  					  ganCalEstado = 1U;
  					  CalibFlash_SetControlHabilitado(0U);
  				  } else {
  					  ganCalPulsoPaso = siguientePulso;
  					  ganCalInicioEtapaMs = HAL_GetTick();
  				  }
  			  }
  		  }

  		  uint16_t destinoGanCal = (ganCalEstado == 2U) ? ganCalPulsoPaso : CalibFlash_GetServoPulsoMinUs();
  		  Servo_MoverHacia(destinoGanCal);
  	  } else if (pidActivoAhora) {
  		  /* Motor operando (en modo 0 normal, o en modo 7 sintonizando
  		   * el PID -- ver arriba, mismo comportamiento del servo en
  		   * ambos), y con un SET_RPM por encima del ralentí -- lazo de
  		   * control real. Setpoint = SET_RPM (downlink directo, uso de
  		   * prueba hasta que exista la maquina Modo 0/1/2, ver README
  		   * seccion 4.3), recortado solo contra el techo RPM_MAX (el
  		   * piso ya lo garantiza controlSolicitado). */
  		  float setpointRpm = setpointRpmCrudo;
  		  if (setpointRpm > rpmMax) setpointRpm = rpmMax;

  		  uint16_t salidaPidUs = PID_CalcularSalidaUs(setpointRpm, Tacometro_GetRPMFiltrada());
  		  Servo_MoverHacia(salidaPidUs);
  	  } else {
  		  /* Sin control activo: motor detenido, o motor en su ralentí
  		   * natural sin SET_RPM comandado por encima de RPM_MIN (Modo
  		   * 0). El servo siempre va (o se queda) en SERVO_PULSO_MIN, la
  		   * posición segura sin aceleración. Cubre también la salida
  		   * del modo calibración por el apagado de seguridad de arriba
  		   * -- en ese caso regresa gradualmente (respetando
  		   * SERVO_VELOCIDAD_MAX_US_S), no de un salto, sin importar en
  		   * qué punto del barrido se haya quedado. */
  		  Servo_MoverHacia(CalibFlash_GetServoPulsoMinUs());
  	  }
  	  modoControlAnterior = modoControl;

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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

/* Sin este callback, un solo error de UART (overrun/framing/ruido --
 * ej. el USB-a-PC metiendo ruido al plano de tierra, ver hallazgos de
 * hardware) deja la recepcion por DMA de esa UART muerta para el
 * resto de la sesion: HAL la aborta internamente al entrar en error y
 * HAL_UARTEx_RxEventCallback() nunca vuelve a dispararse para ella,
 * aunque el modulo del otro lado (RAK3172 o GPS) siga funcionando
 * bien -- todo comando futuro por esa UART solo expira por TIMEOUT sin
 * que el host jamas vea la respuesta real. Coincide con el sintoma de
 * campo "dejo de responder y ya no volvio a intentar". */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        RAK3172_ErrorCallback(huart);
    } else if (huart->Instance == USART2) {
        GPS_ErrorCallback(huart);
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
