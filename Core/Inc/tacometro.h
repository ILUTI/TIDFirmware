/**
 * @file    tacometro.h
 * @brief   Módulo de medición de RPM vía Input Capture (TIM2, canal 1).
 *
 * Mide el período entre flancos de bajada de la señal proveniente del
 * H11AA1 (terminal W del alternador, ya acondicionada), y calcula
 * frecuencia de pulsos y RPM con:
 *   - Filtrado de ruido en dos capas (filtro de hardware del timer +
 *     guarda mínima de período por software).
 *   - Detección de motor detenido (timeout sin capturas nuevas).
 *   - Filtro de suavizado (media móvil exponencial) para no entregarle
 *     al lazo de control una lectura con jitter.
 *
 * Uso típico en main.c:
 *   CalibFlash_Init();                          // ver calibracion_flash.h
 *   Tacometro_Init(&htim2, TIM_CHANNEL_1);
 *   HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
 *   ...
 *   while (1) {
 *       Tacometro_Update();
 *       float rpm = Tacometro_GetRPMFiltrada();
 *   }
 *
 * Y en el callback global de HAL (en main.c, USER CODE):
 *   void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
 *       Tacometro_CaptureCallback(htim);
 *   }
 *
 * Calibración en tiempo de ejecución:
 *   TACOMETRO_PULSOS_POR_REVOLUCION y TACOMETRO_ALPHA_FILTRO siguen
 *   existiendo como VALORES POR DEFECTO (se usan la primera vez que
 *   corre el firmware, antes de que exista una calibración guardada
 *   en flash). Una vez que se llama a Tacometro_SetPulsosPorRevolucion()
 *   o Tacometro_SetAlphaFiltro() (por ejemplo, desde un downlink
 *   LoRaWAN vía rak3172.c), el nuevo valor se guarda en flash
 *   (calibracion_flash.c) y se usa en todos los cálculos posteriores,
 *   incluso después de un reset.
 */

#ifndef TACOMETRO_H
#define TACOMETRO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ==================== CONFIGURACIÓN AJUSTABLE (valores por defecto) ==================== */

/* Pulsos generados por el H11AA1 por cada revolución del alternador.
 * PENDIENTE DE CONFIRMAR EN CAMPO (ver reporte de validación, secc. 8):
 *   - Con señal simétrica (generador de señales): factor 2 pulsos/ciclo AC.
 *   - Con alternador real (banco, taladro): factor 1 pulso/ciclo AC confirmado
 *     con 22 muestras (Gate del osciloscopio), pendiente de reconfirmar en
 *     motor diésel real antes de aplicar el cambio aquí.
 *
 * Valor confirmado en campo (ver calibración con tacómetro digital):
 *   ~17.5 pulsos/vuelta equivalentes, incluyendo la relación de
 *   poleas alternador:motor.
 *
 * NOTA: este valor es solo el DEFAULT usado la primera vez que corre
 * el firmware. Después de la primera calibración remota, el valor
 * real en uso vive en flash (ver calibracion_flash.h) y se consulta
 * con Tacometro_GetPulsosPorRevolucion().
 */
#define TACOMETRO_PULSOS_POR_REVOLUCION   17.5f

/* Guarda mínima de período por software (µs). Descarta capturas que
 * lleguen más rápido que este intervalo (ruido/rebote), como segunda
 * capa además del filtro de hardware del timer (Input Capture Filter).
 * Calculado con margen sobre el techo real medido en banco (~1800-1850
 * pulsos/seg con el clamp instalado, ver reporte secc. 3.7/7.6).
 */
#define TACOMETRO_PERIODO_MINIMO_US       450U

/* Timeout sin capturas nuevas para declarar el motor detenido (ms).
 * A la RPM mínima real de interés (cranking, ~150 RPM), el período entre
 * pulsos es de decenas de ms — 500ms da margen amplio sin retrasar
 * demasiado la detección de "motor parado".
 */
#define TACOMETRO_TIMEOUT_DETENIDO_MS     500U

/* Coeficiente del filtro de media móvil exponencial (0.0 - 1.0).
 * Más alto = responde más rápido pero más ruidoso.
 * Más bajo = más suave pero más lento para seguir cambios reales de RPM.
 * Ajustar experimentalmente según qué tan suave necesite verse la RPM
 * para el lazo de control de presión.
 *
 * NOTA: este valor es solo el DEFAULT -- ver Tacometro_GetAlphaFiltro().
 */
#define TACOMETRO_ALPHA_FILTRO            0.35f

/* Resolución del timer: 1 tick = 1µs (ajustar si se cambia el Prescaler
 * en el .ioc; debe coincidir exactamente con esa configuración). */
#define TACOMETRO_TICKS_POR_SEGUNDO       1000000UL

/* Rango de duty cycle aceptado como "semiciclo real" del H11AA1 (~50%
 * esperado). Un glitch de disparo simétrico del clamp Zener (dos LEDs
 * antiparalelos conduciendo por igual) produce picos angostos muy por
 * debajo de este rango — ver validación de banco, disparo simétrico
 * ±8.2V duplicando 300Hz -> 600Hz con duty << 30%.
 */
#define TACOMETRO_DUTY_MINIMO   0.30f
#define TACOMETRO_DUTY_MAXIMO   0.70f

/* ==================== API PÚBLICA ==================== */

/**
 * Inicializa el módulo. Debe llamarse una vez en main(), después de
 * CalibFlash_Init() (para tener disponible la calibración guardada,
 * si existe) y después de que el timer ya fue inicializado por
 * MX_TIM2_Init() (generado por CubeMX), pero antes de arrancar la
 * captura con HAL_TIM_IC_Start_IT().
 */
void Tacometro_Init(TIM_HandleTypeDef *htim, uint32_t canal);

/**
 * Debe llamarse desde el callback global HAL_TIM_IC_CaptureCallback()
 * en main.c, pasando el mismo puntero de timer que recibe ese callback.
 * Es seguro llamarlo aunque el callback dispare por otros timers/canales:
 * la función verifica internamente que sea el timer/canal correcto.
 */
void Tacometro_CaptureCallback(TIM_HandleTypeDef *htim);

/**
 * Debe llamarse periódicamente desde el loop principal (cada iteración,
 * o al menos cada pocos ms). Actualiza la detección de timeout y el
 * filtro de suavizado. No hace nada pesado — es seguro llamarla seguido.
 */
void Tacometro_Update(void);

/** RPM instantánea, sin suavizar (última medición cruda válida). */
float Tacometro_GetRPMInstantanea(void);

/** RPM suavizada con el filtro de media móvil — usar esta para control. */
float Tacometro_GetRPMFiltrada(void);

/** Frecuencia de pulsos en Hz (para diagnóstico/depuración). */
float Tacometro_GetFrecuenciaHz(void);

/** true si el motor se considera detenido (timeout sin pulsos nuevos). */
bool Tacometro_EstaDetenido(void);

/** Contador de capturas descartadas por la guarda de período mínimo
 * (ruido filtrado). Útil para diagnóstico en campo. */
uint32_t Tacometro_GetContadorRuidoFiltrado(void);

/**
 * Cambia en tiempo de ejecución el factor de pulsos por revolución.
 * Internamente delega en calibracion_flash.h para persistir el valor
 * (sobrevive resets/cortes de energía). Valida rango (0.1 - 200)
 * antes de aplicar.
 *
 * @return true si el valor era válido y se aplicó y guardó en flash.
 */
bool Tacometro_SetPulsosPorRevolucion(float nuevoValor);

/** Valor actual de pulsos por revolución en uso (flash, o el default
 * de TACOMETRO_PULSOS_POR_REVOLUCION si nunca se ha calibrado). */
float Tacometro_GetPulsosPorRevolucion(void);

/**
 * Cambia en tiempo de ejecución el coeficiente del filtro de media
 * móvil exponencial. Valida rango (0.0 exclusivo - 1.0 inclusivo)
 * antes de aplicar y persistir en flash.
 */
bool Tacometro_SetAlphaFiltro(float nuevoValor);

/** Valor actual del coeficiente del filtro en uso (flash, o el
 * default de TACOMETRO_ALPHA_FILTRO si nunca se ha calibrado). */
float Tacometro_GetAlphaFiltro(void);

#ifdef __cplusplus
}
#endif

#endif /* TACOMETRO_H */
