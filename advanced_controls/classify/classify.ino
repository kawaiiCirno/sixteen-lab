/*
 * classify.ino
 *
 * EE16B Spring 2016
 * Emily Naviasky & Nathaniel Mailoa
 *
 * EE 16B Fall 2017
 * Andrew Blatner
 *
 */

#define MIC_INPUT                   P6_0

#define SIZE                        2752
#define SIZE_AFTER_FILTER           172
#define ADC_TIMER_MS                0.35

/*---------------------------*/
/*      CODE BLOCK PCA1      */
/*---------------------------*/

// Enveloping and K-means constants
#define SNIPPET_SIZE                40
#define PRELENGTH                   5
#define THRESHOLD                   0.5

#define KMEANS_THRESHOLD            0.05
#define LOUDNESS_THRESHOLD          700

/*---------------------------*/
/*      CODE BLOCK PCA2      */
/*---------------------------*/

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

  pinMode(MIC_INPUT, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  re_pointer = 0;
  reset_blinker();
  setTimer();
}

void loop(void) {
  if (re_pointer == SIZE) {
    digitalWrite(GREEN_LED, LOW);

    // Apply enveloping function and get snippet with speech.
    // Do classification only if loud enough.
    if (envelope(re, result)) {

      // Reset projection result variables declared above
      proj1 = 0;
      proj2 = 0;

      /*---------------------------*/
      /*      CODE BLOCK PCA3      */
      /*---------------------------*/

      // Demean 'result' and project it on the principal components
      // Hint: 'result' is an array
      // Hint: do this entire operation in 1 loop by replacing the '...'
      // YOUR CODE HERE
      for (int i = 0; i < SNIPPET_SIZE; i++) {
          float demeaned_entry = result[i] - mean_vec[i];
          proj1 += pca_vec1[i] * demeaned_entry;
          proj2 += pca_vec2[i] * demeaned_entry;
      }

      // Classification
      // Use the function 'l2_norm' defined above
      // ith centroid: 'centroids[i]'
      float best_dist = 999999;
      int best_index = -1;
      // YOUR CODE HERE
      float dist;
      for (int i = 0; i < 4; i++) {
        dist = l2_norm(proj1, proj2, centroids[i]);
        if (dist < best_dist) {
          best_dist = dist;
          best_index = i;
        }
      }
      // Compare 'best_dist' against the 'KMEANS_THRESHOLD' and print the result
      // If 'best_dist' is less than the 'KMEANS_THRESHOLD', the recording is a word
      // Otherwise, the recording is noise
      // YOUR CODE HERE
      if (best_dist > KMEANS_THRESHOLD) {
        Serial.println("NOISE");
      }
      else if (best_dist < KMEANS_THRESHOLD) {
        Serial.println("Centroid: ");
        Serial.println(best_index);
      }
      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }
    else {
      Serial.println("Below LOUDNESS_THRESHOLD.");
    }


    delay(2000);
    re_pointer = 0;
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

void reset_blinker(void) {
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

/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/

// ISR for timestep
#pragma vector=TIMER2_A0_VECTOR    // Timer A ISR
__interrupt void Timer2_A0_ISR(void) {
  if (re_pointer < SIZE) {
    digitalWrite(GREEN_LED, HIGH);
    re[re_pointer] = (analogRead(MIC_INPUT) >> 4) - 128;
    re_pointer += 1;
  }
}

// Set timer for timestep; use A2 since A0 & A1 are used by PWM
void setTimer(void) {
  // Set the timer based on 25MHz clock
  TA2CCR0 = (unsigned int) (25000*ADC_TIMER_MS);
  TA2CCTL0 = CCIE;
  __bis_SR_register(GIE);
  TA2CTL = TASSEL_2 + MC_1 + TACLR + ID_0;
}
