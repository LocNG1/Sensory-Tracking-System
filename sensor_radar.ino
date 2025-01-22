#include <IRremote.h>

// Define the pin for the IR receiver
const int RECV_PIN = 3;

// Define the pin for the LED
const int LED_PIN = 2;

// Create an IR receiver object
IRrecv irrecv(RECV_PIN);
decode_results results;

// State of the LED
bool isLedOn = false;

void setup() {
  // Initialize serial monitor
  Serial.begin(9600);

  // Enable the IR receiver
  irrecv.enableIRIn();

  // Set the LED pin as output
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Start with the LED off
}

void loop() {
  // Check if an IR signal is received
  if (irrecv.decode(&results)) {
    // Print the received value to the serial monitor
    Serial.println(results.value, HEX);

    // Check the value and toggle the LED
    if (results.value == 0xFFA25D) { // Replace 0xFFA25D with your remote's button code
      isLedOn = !isLedOn; // Toggle the LED state
      digitalWrite(LED_PIN, isLedOn ? HIGH : LOW);
    }

    // Resume receiving the next value
    irrecv.resume();
  }
}
