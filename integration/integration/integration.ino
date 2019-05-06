/*
 * integration.ino
 * Final sketch for SIXT33N Speech version
 *
 * EE16B Fall 2016
 * Emily Naviasky & Nathaniel Mailoa
 *
 * EE 16B Fall 2017
 * Andrew Blatner
 *
 */

/********************************************************/
/********************************************************/
/***                                                  ***/
/*** Constants and global variables from turning.ino. ***/
/***                                                  ***/
/********************************************************/
/********************************************************/

#define LEFT_MOTOR                  P2_0
#define LEFT_ENCODER                P6_1
#define RIGHT_MOTOR                 P1_5
#define RIGHT_ENCODER               P6_2

#define SAMPLING_INTERVAL           100
int sample_lens[4] = {0};

// Operation modes
#define MODE_LISTEN                 0
#define MODE_DRIVE                  1
int timer_mode = MODE_LISTEN;

#define DRIVE_FAR                   0
#define DRIVE_LEFT                  1
#define DRIVE_CLOSE                 2
#define DRIVE_RIGHT                 3

#define JOLT_STEPS                  2

boolean loop_mode = MODE_DRIVE;
int drive_mode = 0;

int step_num = 0;
volatile boolean do_loop = 0; // timer signal to increment timestep

typedef struct encoder {
  int pin;
  int pos;
  bool level;
  int avg;
} encoder_t;

encoder_t left_encoder = {LEFT_ENCODER, 0, LOW, 0};
encoder_t right_encoder = {RIGHT_ENCODER, 0, LOW, 0};

/*---------------------------*/
/*      CODE BLOCK CON1      */
/*      From turning.ino     */
/*---------------------------*/

float theta_left = 0.3359;
float theta_right = 0.2948;
float beta_left = -16.68;
float beta_right = -12.25;
float v_star = 54.8;

// PWM inputs to jolt the car straight
int left_jolt = 155;
int right_jolt= 190;

// Control gains
float k_left = 0.5;
float k_right = 0.5;

/*---------------------------*/
/*      CODE BLOCK CON2      */
/*      From turning.ino     */
/*---------------------------*/

float driveStraight_left(float delta) {
  return (v_star + beta_left)/theta_left - k_left * delta / theta_left;
}

float driveStraight_right(float delta) {
  return (v_star + beta_right)/theta_right + k_right * delta / theta_right;
}

/*---------------------------*/
/*      CODE BLOCK CON3      */
/*      From turning.ino     */
/*---------------------------*/

float delta_ss = -2;

/*---------------------------*/
/*      CODE BLOCK CON4      */
/*      From turning.ino     */
/*---------------------------*/

#define CAR_WIDTH                   15.0 // in cm
#define TURN_RADIUS                 91 // in cm - 6 feet diameter = 3 tiles in 125 Cory
// #define TURN_RADIUS                 60 // in cm - 4 feet diameter = 2 tiles in 125 Cory

int run_times[4] = {7000, 5000, 2500, 5000};

float delta_reference(int k) {
  // YOUR CODE HERE
  if (drive_mode == DRIVE_RIGHT) {
    return v_star*CAR_WIDTH*k/TURN_RADIUS/5;
  }
  else if (drive_mode == DRIVE_LEFT) {
    return -v_star*CAR_WIDTH*k/TURN_RADIUS/5;
  }
  else { // DRIVE_FAR, DRIVE_CLOSE
    return 0;
  }
}


/*---------------------------*/
/*      CODE BLOCK CON5      */
/*      From turning.ino     */
/*---------------------------*/
#define INFINITY                    (3.4e+38)
#define STRAIGHT_RADIUS             INFINITY

float straight_correction(int k) {
  // YOUR CODE HERE
  return 0;
}

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

/*********************************************************/
/*********************************************************/
/***                                                   ***/
/*** Constants and glboal variables from classify.ino. ***/
/***                                                   ***/
/*********************************************************/
/*********************************************************/

#define MIC_INPUT                   P6_0

#define SIZE                        2752
#define SIZE_AFTER_FILTER           172
#define ADC_TIMER_MS                0.35

/*---------------------------*/
/*      CODE BLOCK PCA1      */
/*     From classify.ino     */
/*---------------------------*/

// Enveloping and K-means constants
#define SNIPPET_SIZE                40
#define PRELENGTH                   5
#define THRESHOLD                   0.5


#define KMEANS_THRESHOLD            0.05
#define LOUDNESS_THRESHOLD          700

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/


/*---------------------------*/
/*      CODE BLOCK PCA2      */
/*     From classify.ino     */
/*---------------------------*/

// DRIVE_FAR
// DRIVE_LEFT
// DRIVE_CLOSE
// DRIVE_RIGHT
float pca_vec1[SNIPPET_SIZE] = {0.0089822464129, 0.0213434503151, 0.047115484625, 0.0753743390816, 0.0879450318211, 0.122275842609, 0.182157704242, 0.234179477278, 0.239041944724, 0.26832098943, 0.282195065235, 0.266375686756, 0.244788471689, 0.249456174445, 0.209403975565, 0.0827282608796, 0.01916237624, -0.0320445520257, -0.0733518891919, -0.118887388727, -0.136743907882, -0.150322123362, -0.170821975663, -0.188259512985, -0.202349283258, -0.195226591573, -0.185104326351, -0.191185029422, -0.189909425748, -0.179386340286, -0.165997608363, -0.150269609472, -0.12572271038, -0.10197263561, -0.0684102239852, -0.0423561523195, -0.00834018173064, 0.00859200495284, 0.0159012840626, 0.0113216579703};
float pca_vec2[SNIPPET_SIZE] = {0.00114657367344, -0.0146435069462, 0.0025701109607, 0.0369264948776, 0.01389011426, 0.148071380876, 0.0343055468499, 0.0723269742769, 0.0439787297709, 0.0215980437279, 0.00502688304002, -0.0702978107997, -0.0493728477851, -0.102297079175, -0.181262289949, -0.270435334956, -0.32370176239, -0.294747075969, -0.260497479574, -0.218122155212, -0.192997018035, -0.155931677853, -0.149673921367, -0.117296179673, -0.0638222253559, -0.0136076630797, -0.00895362881382, 0.0260180922718, 0.0620546875841, 0.0805124517601, 0.0951385595301, 0.124088995596, 0.136180305794, 0.132595250524, 0.170035896602, 0.217406022651, 0.236172045316, 0.245404812049, 0.26640378289, 0.315807902051};
float mean_vec[SNIPPET_SIZE] = {0.00676600939921, 0.00805964496229, 0.0102605789791, 0.0151379694452, 0.0225326415959, 0.0371823894237, 0.0415881810002, 0.0456547163242, 0.0473954235438, 0.0486145293898, 0.0491308118685, 0.0481371134332, 0.0457142830774, 0.0439848724734, 0.0409404939979, 0.0341706054615, 0.0298673300109, 0.026742867197, 0.0242246491424, 0.0229350573971, 0.0223886730878, 0.0222197505831, 0.0226867196527, 0.0227847465321, 0.0223167021076, 0.0209035811073, 0.0200154373339, 0.0184729885548, 0.0177479630964, 0.0168133444643, 0.016319177077, 0.0155962978935, 0.0145343395888, 0.0134186578532, 0.013380982036, 0.0136746394789, 0.0134859882038, 0.0140722596073, 0.0146917812441, 0.0154358023745};
float centroid1[2] = {-0.04204102, -0.04025712};
float centroid2[2] = { 0.07167931,  0.03029722};
float centroid3[2] = { 0.04590677, -0.02432553};
float centroid4[2] = {-0.07554506,  0.03428543};
float* centroids[4] = {
  (float *) &centroid1, (float *) &centroid2,
  (float *) &centroid3, (float *) &centroid4
};

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

float result[SNIPPET_SIZE] = {0};
float proj1 = 0;
float proj2 = 0;

// Data array and index pointer
int re[SIZE] = {0};
volatile int re_pointer = 0;

/*---------------------------------------------------*/
/*---------------------------------------------------*/
/*---------------------------------------------------*/

/*---------------------------*/
/*       Norm functions      */
/*---------------------------*/

// Compute the L2 norm of (dim1, dim2) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        centroid: size-2 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm(float dim1, float dim2, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2));
}

// Compute the L2 norm of (dim1, dim2, dim3) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        dim3: 3rd dimension coordinate
//        centroid: size-3 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm3(float dim1, float dim2, float dim3, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2) + pow(dim3-centroid[2],2));
}

void setup(void) {
  Serial.begin(38400);

  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_ENCODER, INPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_ENCODER, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(MIC_INPUT, INPUT);

  for (int i = 0; i <= 4; i++) {
    sample_lens[i] = run_times[i]/SAMPLING_INTERVAL;
  }

  write_pwm(0, 0);
  delay(2000); // Wait 2 seconds to put down car
  reset_blinker();
  start_listen_mode();
}

void loop(void) {
  check_encoders();
  if (timer_mode == MODE_LISTEN && re_pointer == SIZE){
    // Stop motor
    write_pwm(0, 0);
    digitalWrite(GREEN_LED, LOW);

    // if enveloped data is above some preset value
    if (envelope(re, result)) {

      // Reset projection result variables declared above
      proj1 = 0;
      proj2 = 0;

      /*---------------------------*/
      /*      CODE BLOCK PCA3      */
      /*     From classify.ino     */
      /*     with more changes     */
      /*---------------------------*/

      // Project 'result' on the principal components
      // YOUR CODE HERE
      for (int i = 0; i < SNIPPET_SIZE; i++) {
          float demeaned_entry = result[i] - mean_vec[i];
          proj1 += pca_vec1[i] * demeaned_entry;
          proj2 += pca_vec2[i] * demeaned_entry;
      }

      // Classification
      // Use the function l2_norm defined above
      // ith centroid: centroids[i]
      
      // YOUR CODE HERE
      float best_dist = 999999;
      int best_index = -1;
      float dist;
      for (int i = 0; i < 4; i++) {
        dist = l2_norm(proj1, proj2, centroids[i]);
        if (dist < best_dist) {
          best_dist = dist;
          best_index = i;
        }
      }
      


      // Check against KMEANS_THRESHOLD and print result over serial
      // YOUR CODE HERE
      if (best_dist < KMEANS_THRESHOLD) {
        drive_mode = best_index; // from 0-3, inclusive
        start_drive_mode();
      }

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }

    delay(2000);
    re_pointer = 0; // start recording from beginning if we don't start driving
  }

  else if (loop_mode == MODE_DRIVE && do_loop) {
    if (step_num < JOLT_STEPS) {
      write_pwm(left_jolt, right_jolt);
    }
    else {

      // Save positions because _left_position and _right_position
      // can change in the middle of one loop.
      int left_position = left_encoder.pos;
      int right_position = right_encoder.pos;

      /*---------------------------*/
      /*      CODE BLOCK CON0      */
      /*---------------------------*/

      float delta = left_position - right_position + delta_ss;
      delta = delta - delta_reference(step_num) - straight_correction(step_num);

      // Drive straight using feedback
      // Compute the needed pwm values for each wheel using delta and v_star
      int left_cur_pwm = driveStraight_left(delta);
      int right_cur_pwm = driveStraight_right(delta);
      write_pwm(left_cur_pwm, right_cur_pwm);

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }

    // Counter for how many times loop is executed since entering DRIVE MODE
    step_num++;

    if (step_num == sample_lens[drive_mode]) {
      // Completely stop and go back to listen MODE after 3 seconds
      start_listen_mode();
    }

    do_loop = 0;
  }
}

// Enveloping function with thresholding and normalizing,
// returns snippet of interest (containing speech)
bool envelope(int* data, float* data_out) {
  int32_t avg = 0;
  float maximum = 0;
  int32_t total = 0;
  int block;

  // Apply enveloping filter while finding maximum value
  for (block = 0; block < SIZE_AFTER_FILTER; block++) {
    avg = 0;
    for (int i = 0; i < 16; i++) {
      avg += data[i+block*16];
    }
    avg = avg >> 4;
    data[block] = abs(data[block*16] - avg);
    for (int i = 1; i < 16; i++) {
      data[block] += abs(data[i+block*16] - avg);
    }
    if (data[block] > maximum) {
      maximum = data[block];
    }
  }

  // If not loud enough, return false
  if (maximum < LOUDNESS_THRESHOLD) {
    return false;
  }

  // Determine threshold
  float thres = THRESHOLD * maximum;

  // Figure out when interesting snippet starts and write to data_out
  block = PRELENGTH;
  while (data[block++] < thres);
  if (block > SIZE_AFTER_FILTER - SNIPPET_SIZE) {
    block = SIZE_AFTER_FILTER - SNIPPET_SIZE;
  }
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data[block-PRELENGTH+i];
    total += data_out[i];
  }

  // Normalize data_out
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data_out[i] / total;
  }

  return true;
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void write_pwm(int pwm_left, int pwm_right) {
  analogWrite(LEFT_MOTOR, (int) min(max(0, pwm_left), 255));
  analogWrite(RIGHT_MOTOR, (int) min(max(0, pwm_right), 255));
}

void reset_blinker(void) {
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
  digitalWrite(GREEN_LED, LOW);
}

void start_listen_mode(void) {
  re_pointer = 0;
  write_pwm(0, 0);
  delay(3000); // 3 seconds buffer for mic cap settling
  timer_mode = MODE_LISTEN;
  setTimer(MODE_LISTEN);
}

void start_drive_mode(void) {
  timer_mode = MODE_DRIVE;
  step_num = 0;
  left_encoder.pos = 0;
  right_encoder.pos = 0;
  setTimer(MODE_DRIVE);
}

/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/

#define AVG_DECAY_RATE              0.3
#define LOW_THRESH                  ((int) (0.2*4096))
#define HIGH_THRESH                 ((int) (0.8*4096))

void check_encoder(encoder_t* enc) {
  int new_val = analogRead(enc->pin);
  enc->avg = (int) (AVG_DECAY_RATE*enc->avg + (1 - AVG_DECAY_RATE)*new_val);
  if ((enc->level == LOW && HIGH_THRESH < enc->avg) ||
      (enc->level == HIGH && enc->avg < LOW_THRESH)) {
    enc->pos++;
    enc->level = !enc->level;
  }
}

void check_encoders(void) {
  check_encoder(&left_encoder);
  check_encoder(&right_encoder);
}

// Set timer for timestep; use A2 since A0 & A1 are used by PWM
void setTimer(boolean mode) {
  if (mode == MODE_LISTEN) {
    // Set the timer based on 25MHz clock
    TA2CCR0 = (unsigned int) (25000*ADC_TIMER_MS);
    TA2CCTL0 = CCIE;
    __bis_SR_register(GIE);
    TA2CTL = TASSEL_2 + MC_1 + TACLR + ID_0;
  }
  else if (mode == MODE_DRIVE) {
    TA2CCR0 = (unsigned int) (32.768*SAMPLING_INTERVAL); // set the timer based on 32kHz clock
    TA2CCTL0 = CCIE; // enable interrupts for Timer A
    __bis_SR_register(GIE);
    TA2CTL = TASSEL_1 + MC_1 + TACLR + ID_0;
  }
  timer_mode = mode;
}

// ISR for timestep
#pragma vector=TIMER2_A0_VECTOR    // Timer A ISR
__interrupt void Timer2_A0_ISR(void) {
  if (timer_mode == MODE_LISTEN) {
    if (re_pointer < SIZE) {
      digitalWrite(GREEN_LED, HIGH);
      re[re_pointer] = (analogRead(MIC_INPUT) >> 4) - 128;
      re_pointer += 1;
    }
  }
  else if (timer_mode == MODE_DRIVE) {
    do_loop = 1;
  }
}
