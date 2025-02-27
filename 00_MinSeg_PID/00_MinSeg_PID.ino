/***********************************************
 * LIBRARIES
 ***********************************************/


#include <Wire.h>
#include <MPU6050.h>

// IMU
MPU6050 mpu;


/***********************************************
 * PIN ASSIGNMENTS
***********************************************/


#define ENCODER_A1 3   // Motor 1 Channel A
#define ENCODER_B1 2   // Motor 1 Channel B
#define ENCODER_A2 19  // Motor 2 Channel A
#define ENCODER_B2 18  // Motor 2 Channel B

#define MOTOR_RIGHT_FORWARD 6   // Right motor forward (PWM)
#define MOTOR_RIGHT_BACKWARD 7  // Right motor backward (PWM)
#define MOTOR_LEFT_FORWARD 9    // Left motor forward (PWM)
#define MOTOR_LEFT_BACKWARD 10  // Left motor backward (PWM)


/***********************************************
 * CONSTANTS, GLOBAL VARIABLES, INSTANCES
 ***********************************************/


// sampling time
#define t_s 0.005

// states
float phi1 = 0.0, phi2 = 0.0;       // Motor positions in radians
float dphi1 = 0.0, dphi2 = 0.0;     // Angular velocities in rad/s
float theta = 0.0, dtheta = 0.0;    // Tilt angle and angular velocity

/********** INITIALIZATION ENCODER VARIABLES **********/
#define COUNTS_PER_REV 1400 // Encoder pulses per wheel revolution

volatile long encoder_counts1 = 0; // Motor 1 encoder counter
volatile long encoder_counts2 = 0; // Motor 2 encoder counter

// Previous encoder states
byte lastEncA1 = 0, lastEncB1 = 0;
byte lastEncA2 = 0, lastEncB2 = 0;

/********** INITIALIZATION OBSERVER SYSTEM **********/
typedef float MatrixA[3][3]; // System matrix
typedef float MatrixC[2][3]; // Measurement matrix
typedef float MatrixP[3][3]; // Initial covariance matrix
typedef float MatrixQ[3][3]; // Process noise covariance
typedef float MatrixR[2][2]; // Measurement noise covariance
typedef float VectorX[3][1]; // State vector
typedef float VectorY[2][1]; // Measurement vector
typedef float MatrixL[3][2]; // Kalman gain matrix

// arrays needed for kalmanFilter
float x_pred[3][1];
float P_pred[3][3];
float At[3][3];
float L[3][2];
float Ct[3][2];
float S[2][3];
float T[2][2];
float Cx[2][1];
float yCx[2][1];
float L_y_Cx[3][1];
float LC[3][3];
float LCP[3][3];

VectorX x = {{0}, {0}, {0}}; // Initial state: [dtheta; theta; bias]

VectorY y = {{theta}, {dtheta}};

// Initial covariance matrix
#define pw 10
#define pb 10

MatrixP P = {{t_s * pw, 0.5 * t_s * t_s * pw, 0},
             {0.5 * t_s * t_s * pw, (1.0 / 3.0) * t_s * t_s * t_s * pw, 0},
             {0,  0,   t_s * pb}};

// Observer system matrix A
MatrixA A = {{1, 0, 0},
             {t_s, 1, 0},
             {0, 0, 1}};

// Observer measurement matrix C
MatrixC C = {{0, 1, 0},
             {1, 0, 1}};

// Process noise covariance Q
#define qw 0.3
#define qb 0.0001

MatrixQ Q = {{t_s * qw, 0.5 * t_s * t_s * qw, 0},
             {0.5 * t_s * t_s * qw, (1.0 / 3.0) * t_s * t_s * t_s * qw, 0},
             {0, 0, t_s * qb}};

// Measurement noise covariance R
#define r_theta 5e-5
#define r_dtheta 1e-4

MatrixR R = {{r_theta / t_s, 0},
             {0, r_dtheta / t_s}};


/********** PID CONSTANTS **********/
#define KP -26   // Proportional gain
#define KI -14   // Integral gain
#define KD 0  // Derivative gain

float error = 0.0;
float prev_error = 0.0;
float integral_error = 0.0;
float derivative_error = 0.0;
float u = 0.0;

/********** MOTOR CONTROL CONSTANTS **********/
#define theta_max 0.333  // Maximum allowed tilt angle
#define baselinePWM 0
int pwm;


/***********************************************
 * TIMERS AND ISR
 ***********************************************/


// Timer Flag
volatile bool timerFlag = false;

// Timer1 setup
void setupTimer1() {
  TCCR1A = 0;                // Normal mode
  TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); // CTC mode, Prescaler 64
  OCR1A = 1249;              // Compare value for 5 ms (16 MHz / (64 * 200 Hz) - 1)
  TIMSK1 = (1 << OCIE1A);    // Enable Timer1 Compare Match A interrupt
}

// Timer1 ISR
ISR(TIMER1_COMPA_vect) {
  timerFlag = true;
}

// Encoder 1 ISR
void encoder1ISR() {
  byte encA = digitalRead(ENCODER_A1);
  byte encB = digitalRead(ENCODER_B1);

  if (encA != lastEncA1) { // Channel A changed
    encoder_counts1 += (encA != encB) ? 1 : -1;
  } else if (encB != lastEncB1) { // Channel B changed
    encoder_counts1 += (encB == encA) ? 1 : -1;
  }

  lastEncA1 = encA;
  lastEncB1 = encB;
}

// Encoder 2 ISR
void encoder2ISR() {
  byte encA = digitalRead(ENCODER_A2);
  byte encB = digitalRead(ENCODER_B2);

  if (encA != lastEncA2) { // Channel A changed
    encoder_counts2 += (encA != encB) ? 1 : -1;
  } else if (encB != lastEncB2) { // Channel B changed
    encoder_counts2 += (encB == encA) ? 1 : -1;
  }

  lastEncA2 = encA;
  lastEncB2 = encB;
}


/***********************************************
 * SETUP
 ***********************************************/


void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize MPU6050
  mpu.initialize();

  // Initialize Timer1
  setupTimer1();

  // Enable global interrupts
  sei();

  // Initialize encoders pins
  pinMode(ENCODER_A1, INPUT_PULLUP);
  pinMode(ENCODER_B1, INPUT_PULLUP);
  pinMode(ENCODER_A2, INPUT_PULLUP);
  pinMode(ENCODER_B2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A1), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A2), encoder2ISR, CHANGE);

  // Set motor control pins as OUTPUT
  pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);
  pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
  pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
}


/***********************************************
 * LOOP
 ***********************************************/


void loop() {
  if (timerFlag) {         // Check if timer flag is set
    timerFlag = false;     // Reset flag

    readSensor();

    kalmanFilter();

    pidController();

    motorControl();
  }
}


/***********************************************
 * FUNCTIONS
 ***********************************************/


/********** READ SENSOR DATA FUNCTION **********/

void readSensor() {
  static unsigned long last_time = micros();
  unsigned long current_time = micros();
  float dt = (current_time - last_time) / 1000000.0; // Calculate elapsed time in seconds
  last_time = current_time;

  static float phi1_old = 0.0;
  static float phi2_old = 0.0;
  
  // Process Motor 1 encoder
  phi1 = 2 * PI * encoder_counts1 / COUNTS_PER_REV;
  dphi1 = (phi1 - phi1_old) / dt;

  // Process Motor 2 encoder
  phi2 = 2 * PI * encoder_counts2 / COUNTS_PER_REV;
  dphi2 = (phi2 - phi2_old) / dt;

  phi1_old = phi1;
  phi2_old = phi2;

  // Read MPU6050 data
  int16_t ay, az, gx;
  mpu.getMotion6(nullptr, &ay, &az, &gx, nullptr, nullptr);

  // Calculate tilt angle (theta) and angular velocity (dtheta)
  theta = radians(90) - atan2(-ay / 16384.0, az / 16384.0);
  dtheta = radians(gx / 131.0);
}


/********** KALMAN FILTER **********/

void kalmanFilter() {

  // PREDICTION

  multiplyMatrix3x3_3x1(A, x, x_pred); // A * x
  
  // P_pred = A * P * ~A + Q
  transposeMatrix3x3(A, At); // transpose A
  multiplyMatrix3x3_3x3(A, P, P_pred);
  multiplyMatrix3x3_3x3(P_pred, At, P_pred);
  addMatrix3x3(P_pred, Q, P_pred); // final P_pred

  // UPDATE

  // L = P_pred * ~C * Inverse(C * P_pred * ~C + R)
  transposeMatrix2x3(C, Ct); // transpose C
  multiplyMatrix2x3_3x3(C, P_pred, S); // S = C * P_pred
  multiplyMatrix2x3_3x2(S, Ct, T); // T = S * Ct = C * P_pred * Ct
  addMatrix2x2(T, R, T); // T = C * P_pred * ~C + R
  inverseMatrix2x2(T, T);
  multiplyMatrix3x3_3x2(P_pred, Ct, L); // L = P_pred * Ct
  multiplyMatrix3x2_2x2(L, T, L); // final L

  // x = x_pred + L * (y - C * x_pred)
  multiplyMatrix2x3_3x1(C, x_pred, Cx); // Cx = C * x_pred
  subtractMatrix2x1(y, Cx, yCx); // yCx = y - C * x_pred
  multiplyMatrix3x2_2x1(L, yCx, L_y_Cx); // L = L * (y - C * x_pred)
  addMatrix3x1(x_pred, L_y_Cx, x); // final x_pred

  // P = P_pred - L * C * P_pred
  multiplyMatrix3x2_2x3(L, C, LC); // LC = L * C
  multiplyMatrix3x3_3x3(LC, P_pred, LCP); // LCP = L * C * P_pred
  subtractMatrix3x3(P_pred, LCP, P); // final P_pred
}


/********** CONTROLER FUNCTION **********/

void pidController() {
  // theta_ref = 0
  error = -theta;

  integral_error += error * t_s;
  integral_error = constrain(integral_error, -0.05, 0.05);

  derivative_error = (error - prev_error) / t_s;
  
  u = KP * error + KI * integral_error + KD * derivative_error;
  
  // Store previous error for next iteration
  prev_error = error;
}


/********** MOTOR CONTROL FUNCTION **********/

void motorControl() {
  // Stop motors if the angle exceeds the maximum allowed value
  if (fabs(theta) > theta_max) {

    pwm = 0;

    analogWrite(MOTOR_RIGHT_FORWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
    analogWrite(MOTOR_LEFT_FORWARD, 0);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    return;
  }

  if (u > 0) {
    // 'u' is multiplied by 255/7.2 so that 7.2 maps to 255.
    pwm = baselinePWM + (int)(fabs(u) * 255.0/7.2);
    pwm = constrain(pwm, baselinePWM, 255);

    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
    analogWrite(MOTOR_LEFT_FORWARD, pwm);
    analogWrite(MOTOR_RIGHT_FORWARD, pwm);

  } else {
    pwm = (int)(fabs(u) * 255.0/7.2);
    pwm = constrain(pwm, 0, 255);  

    // Move backward
    analogWrite(MOTOR_LEFT_FORWARD, 0);
    analogWrite(MOTOR_RIGHT_FORWARD, 0);
    analogWrite(MOTOR_LEFT_BACKWARD, pwm);
    analogWrite(MOTOR_RIGHT_BACKWARD, pwm);
  }
}


/********** MATRIX FUNCTIONS **********/

void multiplyMatrix2x3_3x3(float A[2][3], float B[3][3], float result[2][3]) {
    result[0][0] = A[0][0] * B[0][0] + A[0][1] * B[1][0] + A[0][2] * B[2][0];
    result[0][1] = A[0][0] * B[0][1] + A[0][1] * B[1][1] + A[0][2] * B[2][1];
    result[0][2] = A[0][0] * B[0][2] + A[0][1] * B[1][2] + A[0][2] * B[2][2];
    
    result[1][0] = A[1][0] * B[0][0] + A[1][1] * B[1][0] + A[1][2] * B[2][0];
    result[1][1] = A[1][0] * B[0][1] + A[1][1] * B[1][1] + A[1][2] * B[2][1];
    result[1][2] = A[1][0] * B[0][2] + A[1][1] * B[1][2] + A[1][2] * B[2][2];
}


void multiplyMatrix3x2_2x3(float A[3][2], float B[2][3], float result[3][3]) {
    result[0][0] = A[0][0] * B[0][0] + A[0][1] * B[1][0];
    result[0][1] = A[0][0] * B[0][1] + A[0][1] * B[1][1];
    result[0][2] = A[0][0] * B[0][2] + A[0][1] * B[1][2];
    
    result[1][0] = A[1][0] * B[0][0] + A[1][1] * B[1][0];
    result[1][1] = A[1][0] * B[0][1] + A[1][1] * B[1][1];
    result[1][2] = A[1][0] * B[0][2] + A[1][1] * B[1][2];
    
    result[2][0] = A[2][0] * B[0][0] + A[2][1] * B[1][0];
    result[2][1] = A[2][0] * B[0][1] + A[2][1] * B[1][1];
    result[2][2] = A[2][0] * B[0][2] + A[2][1] * B[1][2];
}


void multiplyMatrix3x2_2x2(float A[3][2], float B[2][2], float result[3][2]) {
    result[0][0] = A[0][0] * B[0][0] + A[0][1] * B[1][0];
    result[0][1] = A[0][0] * B[0][1] + A[0][1] * B[1][1];

    result[1][0] = A[1][0] * B[0][0] + A[1][1] * B[1][0];
    result[1][1] = A[1][0] * B[0][1] + A[1][1] * B[1][1];
    
    result[2][0] = A[2][0] * B[0][0] + A[2][1] * B[1][0];
    result[2][1] = A[2][0] * B[0][1] + A[2][1] * B[1][1];
}

void multiplyMatrix3x2_2x1(float A[3][2], float B[2][1], float result[3][1]) {
    result[0][0] = A[0][0] * B[0][0] + A[0][1] * B[1][0];
    
    result[1][0] = A[1][0] * B[0][0] + A[1][1] * B[1][0];
    
    result[2][0] = A[2][0] * B[0][0] + A[2][1] * B[1][0];
}


void addMatrix3x1(float A[3][1], float B[3][1], float result[3][1]) {
    result[0][0] = A[0][0] + B[0][0];
    result[1][0] = A[1][0] + B[1][0];
    result[2][0] = A[2][0] + B[2][0];
}


void multiplyMatrix3x3_3x1(float A[3][3], float B[3][1], float result[3][1]) {
    result[0][0] = A[0][0] * B[0][0] + A[0][1] * B[1][0] + A[0][2] * B[2][0];
    
    result[1][0] = A[1][0] * B[0][0] + A[1][1] * B[1][0] + A[1][2] * B[2][0];
    
    result[2][0] = A[2][0] * B[0][0] + A[2][1] * B[1][0] + A[2][2] * B[2][0];
}


void multiplyMatrix3x3_3x3(float A[3][3], float B[3][3], float result[3][3]) {
    result[0][0] = A[0][0] * B[0][0] + A[0][1] * B[1][0] + A[0][2] * B[2][0];
    result[0][1] = A[0][0] * B[0][1] + A[0][1] * B[1][1] + A[0][2] * B[2][1];
    result[0][2] = A[0][0] * B[0][2] + A[0][1] * B[1][2] + A[0][2] * B[2][2];
    
    result[1][0] = A[1][0] * B[0][0] + A[1][1] * B[1][0] + A[1][2] * B[2][0];
    result[1][1] = A[1][0] * B[0][1] + A[1][1] * B[1][1] + A[1][2] * B[2][1];
    result[1][2] = A[1][0] * B[0][2] + A[1][1] * B[1][2] + A[1][2] * B[2][2];
    
    result[2][0] = A[2][0] * B[0][0] + A[2][1] * B[1][0] + A[2][2] * B[2][0];
    result[2][1] = A[2][0] * B[0][1] + A[2][1] * B[1][1] + A[2][2] * B[2][1];
    result[2][2] = A[2][0] * B[0][2] + A[2][1] * B[1][2] + A[2][2] * B[2][2];
}


void addMatrix3x3(float A[3][3], float B[3][3], float result[3][3]) {
    result[0][0] = A[0][0] + B[0][0];
    result[0][1] = A[0][1] + B[0][1];
    result[0][2] = A[0][2] + B[0][2];
    
    result[1][0] = A[1][0] + B[1][0];
    result[1][1] = A[1][1] + B[1][1];
    result[1][2] = A[1][2] + B[1][2];
    
    result[2][0] = A[2][0] + B[2][0];
    result[2][1] = A[2][1] + B[2][1];
    result[2][2] = A[2][2] + B[2][2];
}


void subtractMatrix3x3(float A[3][3], float B[3][3], float result[3][3]) {
    result[0][0] = A[0][0] - B[0][0];
    result[0][1] = A[0][1] - B[0][1];
    result[0][2] = A[0][2] - B[0][2];
    
    result[1][0] = A[1][0] - B[1][0];
    result[1][1] = A[1][1] - B[1][1];
    result[1][2] = A[1][2] - B[1][2];
    
    result[2][0] = A[2][0] - B[2][0];
    result[2][1] = A[2][1] - B[2][1];
    result[2][2] = A[2][2] - B[2][2];
}


void transposeMatrix3x3(float A[3][3], float At[3][3]) {
    At[0][0] = A[0][0];
    At[0][1] = A[1][0];
    At[0][2] = A[2][0];
    
    At[1][0] = A[0][1];
    At[1][1] = A[1][1];
    At[1][2] = A[2][1];
    
    At[2][0] = A[0][2];
    At[2][1] = A[1][2];
    At[2][2] = A[2][2];
}


void inverseMatrix2x2(float A[2][2], float inverse[2][2]) {
    float det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
    float invDet = 1.0 / det;

    inverse[0][0] =  A[1][1] * invDet;
    inverse[0][1] = -A[0][1] * invDet;
    inverse[1][0] = -A[1][0] * invDet;
    inverse[1][1] =  A[0][0] * invDet;
}


void transposeMatrix2x3(float A[2][3], float result[3][2]) {
    result[0][0] = A[0][0];
    result[0][1] = A[1][0];
    
    result[1][0] = A[0][1];
    result[1][1] = A[1][1];
    
    result[2][0] = A[0][2];
    result[2][1] = A[1][2];
}


void multiplyMatrix2x3_3x2(float A[2][3], float B[3][2], float result[2][2]) {
    result[0][0] = A[0][0] * B[0][0] + A[0][1] * B[1][0] + A[0][2] * B[2][0];
    result[0][1] = A[0][0] * B[0][1] + A[0][1] * B[1][1] + A[0][2] * B[2][1];
    
    result[1][0] = A[1][0] * B[0][0] + A[1][1] * B[1][0] + A[1][2] * B[2][0];
    result[1][1] = A[1][0] * B[0][1] + A[1][1] * B[1][1] + A[1][2] * B[2][1];
}


void addMatrix2x2(float A[2][2], float B[2][2], float result[2][2]) {
    result[0][0] = A[0][0] + B[0][0];
    result[0][1] = A[0][1] + B[0][1];
    
    result[1][0] = A[1][0] + B[1][0];
    result[1][1] = A[1][1] + B[1][1];
}


void multiplyMatrix3x3_3x2(float A[3][3], float B[3][2], float result[3][2]) {
    result[0][0] = A[0][0] * B[0][0] + A[0][1] * B[1][0] + A[0][2] * B[2][0];
    result[0][1] = A[0][0] * B[0][1] + A[0][1] * B[1][1] + A[0][2] * B[2][1];
    
    result[1][0] = A[1][0] * B[0][0] + A[1][1] * B[1][0] + A[1][2] * B[2][0];
    result[1][1] = A[1][0] * B[0][1] + A[1][1] * B[1][1] + A[1][2] * B[2][1];
    
    result[2][0] = A[2][0] * B[0][0] + A[2][1] * B[1][0] + A[2][2] * B[2][0];
    result[2][1] = A[2][0] * B[0][1] + A[2][1] * B[1][1] + A[2][2] * B[2][1];
}


void multiplyMatrix2x3_3x1(float A[2][3], float B[3][1], float result[2][1]) {
    result[0][0] = A[0][0] * B[0][0] + A[0][1] * B[1][0] + A[0][2] * B[2][0];
    
    result[1][0] = A[1][0] * B[0][0] + A[1][1] * B[1][0] + A[1][2] * B[2][0];
}


void subtractMatrix2x1(float A[2][1], float B[2][1], float result[2][1]) {
    result[0][0] = A[0][0] - B[0][0];
    result[1][0] = A[1][0] - B[1][0];
}