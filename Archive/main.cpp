#include <Arduino.h>

// On the Ebyte board, the status LED is connected to PC13
#define LED_1 PB13
#define LED_2 PB12

void setup() {
    // Initialize the physical status LED pin
    pinMode(LED_1, OUTPUT);
    pinMode(LED_2, OUTPUT);
    
    // Initialize the hardware Serial interface (PA9=TX, PA10=RX)
    Serial.begin(115200);
    while(!Serial) {
        ; // Wait for serial port connection
    }
    
    Serial.println("--- E80-900MBL-02 Board Ready ---");
}

void loop() {
    // Blink LED to indicate loop activity
    digitalWrite(LED_1, LOW);   // Turns the onboard active-low LED ON
    digitalWrite(LED_2, HIGH);   // Turns the onboard active-low LED ON
    Serial.println("LED Status: ON");
    delay(1000);
    
    digitalWrite(LED_1, HIGH);  // Turns the onboard active-low LED OFF
    digitalWrite(LED_2, LOW);  // Turns the onboard active-low LED OFF
    Serial.println("LED Status: OFF");
    delay(1000);
}