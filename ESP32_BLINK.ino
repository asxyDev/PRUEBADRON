#include <Adafruit_NeoPixel.h>

#define LED_PIN    45
#define NUM_LEDS   1

Adafruit_NeoPixel pixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(115200);
  delay(1000); // Una pausa para que el monitor serial se estabilice

  Serial.println("Inicializando el Monitor Serial...");

  pixel.begin(); // Intentamos inicializar la librería del LED

  Serial.println("Libreria NeoPixel inicializada.");
}

void loop() {
  Serial.println("El programa esta corriendo en el loop.");
  delay(2000);
}