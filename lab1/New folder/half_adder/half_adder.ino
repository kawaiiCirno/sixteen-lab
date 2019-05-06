/* Alias pins for use on MSP430 */
#define A          P1_4                           
#define B          P1_5                          
#define SUM        P2_4
#define C_OUT      P2_5

/* Gets a bit value from the user's serial monitor */
int get_bit() {
  int incoming_bit = -1;
  /* Incoming bit must be either 1 or 0 */
  while (1) {
    incoming_bit = Serial.read();
    if (incoming_bit == 1 || incoming_bit == 0) {
      return incoming_bit;
    } else {
      Serial.println("Please enter a valid bit value (1 or 0)");
    }
  }
}

/* Writes a, b to A, B output pins */
void half_add(int a, int b) {
    
    // Write bits (1 => 3.3 V or 0 => 0 V) to pins
    digitalWrite(A, a);
    digitalWrite(B, b);
    
    /* Uncomment for debugging */
    /*
    Serial.println("INPUTS:");
    Serial.print("a=");
    Serial.println(a, DEC);
    Serial.print("b=");

    Serial.println("");
    */
        
    /* Wait for changes to propogate through circuit */
    delay(100);

    /* Print expected outputs. Uncomment for debugging. */
    /*
    int expected_sum = a ^ b;
    int expected_c_out = a & b;

    Serial.println("EXPECTED OUTPUTS:");
    Serial.print("sum=");
    Serial.println(expected_sum, DEC);
    Serial.print("c_out=");
    Serial.println(expected_c_out, DEC);

    Serial.println("");
    */
    
    /* Reads in bits from circuit. Uncomment for debugging. */
    /*
    int sum = digitalRead(SUM);
    int c_out = digitalRead(C_OUT);

    Serial.println("ACTUAL OUTPUTS:");
    Serial.print("sum=");
    Serial.println(sum, DEC);
    Serial.print("c_out=");
    Serial.println(c_out, DEC);
    */
}

void setup() {
    /* Code here is run once at the beginning of the program
    * Usually used to set up pins on the MSP */
    Serial.begin(38400);

    pinMode(A, OUTPUT); 
    pinMode(B, OUTPUT);
    pinMode(SUM, INPUT);
    pinMode(C_OUT, INPUT);

    Serial.println("Begin circuit adder");
}

void loop() {
    /* Usually contains the main logic of a program.
     * which runs repeatedly in a loop */

    /* Sets the added bits `a` and `b`
     * You can also hardcode these */
    Serial.println("a=?");
    int a = get_bit();
    Serial.println("b=?");
    int b = get_bit();

    Serial.println("");

    half_add(a, b);

    /* UNCOMMENT THIS CODE FOR CHECKOFF */
    /*
    Serial.println("Its time - to add");
    Serial.println("A = True, B = False");
    half_add(1, 0);

    delay(3000);

    Serial.println("A = True, B = True");
    half_add(1, 1);

    delay(3000);

    Serial.println("A = False, B = False");

    half_add(0, 0);

    delay(3000);
    */

    Serial.println("Press any key to choose new bit values");
    Serial.read();
    Serial.println("");
}
