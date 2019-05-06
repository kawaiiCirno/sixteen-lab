/* USE THIS FILE IF `half_adder.ino` DOES NOT WORK */

//Pick appropriate pins on the MSP430 board
#define A          P1_5                           
#define B          P1_4                          


/*sends a,b to A,B output pins
reads back sum and carry from half adder circuit and sends value to 
sum and carry LEDS*/
void half_add(int a, int b) {
    
    //write in bits
    digitalWrite(A, a);
    digitalWrite(B, b);
    
    /* For your debugging pleasure
    Serial.println("INPUT" );
    Serial.println("BIT A: " + str(a));
    Serial.println("BIT B: " + str(b));
    */
        
    delay(5000); //wait for circuit to do its magic
    
    //read in bits
    
    /*For your debugging pleasure
    Serial.println("SUM: " + str(digitalRead(SUM));
    Serial.println("C_OUT: " + str(digitalRead(C_OUT));
    
    */
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);

  pinMode(A, OUTPUT); 
  pinMode(B, OUTPUT);
    
  Serial.println("Begin circuit adder");
  
}

void loop() {
  // put your main code here, to run repeatedly: 
  //FOR TESTING

  //CODE BLOCK TO MODIFY
  //============================
  int a = 1 ;//Set first bit here;
  int b = 1 ;//Set second bit here;
  //=============================

  half_add(a,b);
  
  //UNCOMMENT THIS CODE FOR CHECKOFF
  Serial.println("Its time - to add");
  Serial.println("A = True, B = False");
  half_add(1, 0);

  Serial.println("A = True, B = True");
  half_add(1, 1);

  Serial.println("A = False, B = False");

  half_add(0,0);
}
