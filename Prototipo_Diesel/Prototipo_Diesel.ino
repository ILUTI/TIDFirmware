// ============================================================
// Tacómetro basado en terminal W del alternador (vía H11AA1)
// Prueba en protoboard - Arduino Nano
// ============================================================

// --- Definición de pines ---
const byte pinTacometro = 2; // Pin D2 (Interrupción 0)

// --- Variables para el cálculo (volátiles: se modifican en la interrupción) ---
volatile unsigned long contadorPulsos = 0;
volatile unsigned long ultimoPulsoMicros = 0; // para el debounce por software

unsigned long tiempoAnterior = 0;

// --- Configuración del debounce ---
// Ajustar según la RPM máxima física esperada del motor.
// A mayor RPM máxima esperada, menor debe ser este valor (para no descartar pulsos reales).
const unsigned long minIntervaloMicros = 500; // 500us => permite hasta ~2000 pulsos/seg

// --- Configuración del alternador (ajustar según tu motor) ---
// NOTA: El H11AA1 genera 2 pulsos por cada ciclo de la terminal W.
// Si tu alternador genera, por ejemplo, 6 ciclos por cada vuelta del motor,
// recibirás 12 pulsos por revolución (RPM).
const float pulsosPorRevolucion = 12.0;

void setup() {
  Serial.begin(9600);

  // Como ya tienes R2 (pull-up externo), basta con INPUT (sin pull-up interno)
  pinMode(pinTacometro, INPUT);

  // Interrupción en flanco de bajada (el fototransistor conduce y jala el nodo a LOW)
  attachInterrupt(digitalPinToInterrupt(pinTacometro), contarPulso, FALLING);

  tiempoAnterior = millis();

  Serial.println("Pulsos/seg (Hz) | RPM estimadas");
}

void loop() {
  unsigned long tiempoActual = millis();

  // Procesar y mostrar cada 1 segundo (1000 ms)
  if (tiempoActual - tiempoAnterior >= 1000) {

    // Copiar y reiniciar el contador de forma segura (sin interrupciones activas)
    noInterrupts();
    unsigned long pulsosCopiados = contadorPulsos;
    contadorPulsos = 0;
    interrupts();

    // --- Frecuencia de pulsos (Hz) ---
    // Al medir en una ventana de 1 segundo, pulsos/seg = frecuencia en Hz directamente.
    float frecuenciaHz = pulsosCopiados; // Hz

    // --- RPM estimadas ---
    float rpm = (pulsosCopiados * 60.0) / pulsosPorRevolucion;

    // --- Mostrar en el monitor serie ---
    Serial.print("Frecuencia: ");
    Serial.print(frecuenciaHz, 1);
    Serial.print(" Hz | Pulsos/seg: ");
    Serial.print(pulsosCopiados);
    Serial.print(" | RPM estimadas: ");
    Serial.println(rpm, 1);

    tiempoAnterior = tiempoActual;
  }
}

// --- Función de interrupción con debounce por software ---
// Ignora pulsos que llegan demasiado rápido después del anterior (ruido/rebote).
void contarPulso() {
  unsigned long ahora = micros();
  if (ahora - ultimoPulsoMicros > minIntervaloMicros) {
    contadorPulsos++;
    ultimoPulsoMicros = ahora;
  }
}
