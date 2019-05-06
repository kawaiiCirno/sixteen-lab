/* Set the serial connection speed */
#define BAUD_RATE       9600

/* Set which bits correspond to which pins */
#define BIT             P3_2

void setup()
{
  // Start a serial connection to send data to the computer
  Serial.begin(BAUD_RATE);

  // Set pins to output mode
  pinMode(BIT, OUTPUT);

  reset_blinker();
}

int bit_value = 0;

void loop()
{
  if (bit_value) {
    digitalWrite(BIT, HIGH);
    bit_value = 0;
  } else {
    digitalWrite(BIT, LOW);
    bit_value = 1;
  }

  // Change this value to change the frequency of the sound wave
  // Delay of 160 corresponds to about 600 Hz
  delayMicroseconds(160);
}

// Reset success indicator
void reset_blinker() {
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(RED_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(GREEN_LED, LOW);
}
