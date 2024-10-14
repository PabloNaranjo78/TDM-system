#include "arduinoFFT.h"

#define SAMPLES 128
#define SAMPLING_FREQUENCY 3000

int sensorPin = A0;

arduinoFFT FFT = arduinoFFT();

unsigned int samplingPeriod;
unsigned long microSeconds;

double vReal[SAMPLES];
double vImag[SAMPLES];

void setup() {
  Serial.begin(115200);
  samplingPeriod = 1000000 / SAMPLING_FREQUENCY;
}

void loop() {
  // Captura de muestras
  for (int i = 0; i < SAMPLES; i++) {
    microSeconds = micros();

    vReal[i] = analogRead(sensorPin) - 512; // Centrar los datos en torno a 0
    vImag[i] = 0;  // Inicializar el componente imaginario

    while (micros() < (microSeconds + samplingPeriod)) {}
  }

  // Aplicar ventana de Hamming y realizar la FFT
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

  double peakFrequency = 0;
  double maxMagnitude = 0;

  // Recorrer las frecuencias para encontrar la frecuencia más alta
  for (int i = 0; i < (SAMPLES / 2); i++) {
    
    double frequency = (i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES;  // Calcular frecuencia correspondiente
    double magnitude = vReal[i];

    Serial.print(frequency);
    Serial.print(":");
    Serial.println(magnitude);

    // Verificar si la frecuencia es mayor o igual a 200 Hz
    if (frequency >= 200 && magnitude > maxMagnitude) {
      maxMagnitude = magnitude;  // Actualizar la magnitud máxima
      peakFrequency = frequency;  // Actualizar la frecuencia del pico
      vReal = 0;
    }
  }



  // Si no se encuentra ninguna frecuencia válida, establecer 0
  if (peakFrequency < 230) {
    peakFrequency = 0;  // Asignar 0 si la frecuencia máxima detectada es menor que 200 Hz
  }

  // Imprimir la frecuencia pico detectada
  Serial.print("Peak Frequency: ");
  Serial.println(peakFrequency);

  delay(10); // Pausa para evitar saturar la salida del puerto serie
}
