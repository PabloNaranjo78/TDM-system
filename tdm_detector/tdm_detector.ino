#include "arduinoFFT.h"

#define SAMPLES 128
#define SAMPLING_FREQUENCY 3000
#define MIN_MAGNITUDE 10

int sensorPin = A0;
int ledPins[] = {4, 2, 3};  // Pines de los LEDs

arduinoFFT FFT = arduinoFFT();

unsigned int samplingPeriod;
unsigned long microSeconds;

double vReal[SAMPLES];
double vImag[SAMPLES];

// Rango central y su tolerancia para cada LED
struct FrequencyRange {
  int center1, tolerance1, center2, tolerance2;
};

// Rango de frecuencias para cada LED
FrequencyRange ledRanges[] = {
  {450, 75, 1200, 75},   // LED 1 (pin 4)
  {600, 75, 1350, 75},  // LED 2 (pin 2)
  {700, 75, 1200, 75}    // LED 3 (pin 3)
};

void setup() {
  Serial.begin(115200);
  samplingPeriod = 1000000 / SAMPLING_FREQUENCY;
  for (int i = 0; i < 3; i++) {
    pinMode(ledPins[i], OUTPUT);  // Configurar los pines de los LEDs como salida
  }
}

void loop() {
  // Captura de muestras
  for (int i = 0; i < SAMPLES; i++) {
    microSeconds = micros();
    vReal[i] = analogRead(sensorPin) - 512; // Centrar los datos en torno a 0
    vImag[i] = 0;  // Inicializar el componente imaginario
    while (micros() < (microSeconds + samplingPeriod)) {}
  }

  // Aplicar FFT
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

  // Obtener las dos frecuencias pico
  double peakFrequency = findPeakFrequency();
  double peakFrequency_2 = findPeakFrequency();

  // Encender o apagar LEDs según las frecuencias pico
  for (int i = 0; i < 3; i++) {
    if (shouldTurnOnLED(peakFrequency, peakFrequency_2, ledRanges[i])) {
      digitalWrite(ledPins[i], HIGH);  // Encender el LED
    } else {
      digitalWrite(ledPins[i], LOW);   // Apagar el LED
    }
  }

  // Imprimir frecuencias pico
  Serial.print("Peak Frequency: ");
  Serial.println(peakFrequency);
  Serial.print("Peak Frequency 2: ");
  Serial.println(peakFrequency_2);

  delay(10); // Pausa para evitar saturar la salida del puerto serie
}

// Función para encontrar la frecuencia pico
double findPeakFrequency() {
  double peakFrequency = 0;
  double maxMagnitude = 0;
  int peakFrequency_index = 0;

  for (int i = 0; i < (SAMPLES / 2); i++) {
    double frequency = (i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES;
    double magnitude = vReal[i];

    if (frequency >= 200 && magnitude > maxMagnitude && magnitude > MIN_MAGNITUDE) {
      maxMagnitude = magnitude;
      peakFrequency = frequency;
      peakFrequency_index = i;
    }
  }

  // Borrar frecuencias cercanas a la frecuencia pico encontrada
  for (int i = 0; i < 10; i++) {
    vReal[peakFrequency_index + i] = 0;
    vReal[peakFrequency_index - i] = 0;
  }

  return peakFrequency;
}

// Función para determinar si un LED debe encenderse
bool shouldTurnOnLED(double peakFrequency, double peakFrequency_2, FrequencyRange range) {
  return ((peakFrequency >= range.center1 - range.tolerance1 && peakFrequency <= range.center1 + range.tolerance1 &&
           peakFrequency_2 >= range.center2 - range.tolerance2 && peakFrequency_2 <= range.center2 + range.tolerance2) ||
          (peakFrequency_2 >= range.center1 - range.tolerance1 && peakFrequency_2 <= range.center1 + range.tolerance1 &&
           peakFrequency >= range.center2 - range.tolerance2 && peakFrequency <= range.center2 + range.tolerance2));
}
