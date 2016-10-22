#include <CurieIMU.h>
#include <MadgwickAHRS.h>
#include <AnalogSmooth.h>
Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;
//Serial.println(microsecondsToClockCycles(1)); // gives a result of 16 clock cycles per microsecond
boolean mode = false;
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;

// define pins
//#define ADJ_PIN 1 // Adjustment pin is analog 0
//#define PWM_PIN 8 // PWM output pin is digital 8

// setup PWM values
#define PWM_FREQ 500 // PWM Hz, must be greater than 60Hz to avoid delayMicroseconds issues
#define MAX_V 5.00 // the maximum voltage we can output

long cycle_length;
float v_out;
float duty_cycle;
int on_time;
int off_time;

AnalogSmooth as = AnalogSmooth();
AnalogSmooth as100 = AnalogSmooth(100);

int calibrateOffsets = 1;

void setup()
{
  // start up serial for debugging
  Serial.begin(9600);
  // start the IMU and filter
  CurieIMU.begin();
  CurieIMU.attachInterrupt(eventCallback);
  if (calibrateOffsets == 1) {
    Serial.println("Internal sensor offsets BEFORE calibration...");
    Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS));
    Serial.print("\t"); // -76
    Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS));
    Serial.print("\t"); // -235
    Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS));
    Serial.print("\t"); // 168
    Serial.print(CurieIMU.getGyroOffset(X_AXIS));
    Serial.print("\t"); // 0
    Serial.print(CurieIMU.getGyroOffset(Y_AXIS));
    Serial.print("\t"); // 0
    Serial.println(CurieIMU.getGyroOffset(Z_AXIS));

    // To manually configure offset compensation values,
    // use the following methods instead of the autoCalibrate...() methods below
    //CurieIMU.setAccelerometerOffset(X_AXIS,495.3);
    //CurieIMU.setAccelerometerOffset(Y_AXIS,-15.6);
    //CurieIMU.setAccelerometerOffset(Z_AXIS,491.4);
    //CurieIMU.setGyroOffset(X_AXIS,7.869);
    //CurieIMU.setGyroOffset(Y_AXIS,-0.061);
    //CurieIMU.setGyroOffset(Z_AXIS,15.494);

    Serial.println("About to calibrate. Make sure your board is stable and upright");
    delay(5000);

    // The board must be resting in a horizontal position for
    // the following calibration procedure to work correctly!
    Serial.print("Starting Gyroscope calibration and enabling offset compensation...");
    CurieIMU.autoCalibrateGyroOffset();
    Serial.println(" Done");

    Serial.print("Starting Acceleration calibration and enabling offset compensation...");
    CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
    CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
    Serial.println(" Done");

    Serial.println("Internal sensor offsets AFTER calibration...");
    Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS));
    Serial.print("\t"); // -76
    Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS));
    Serial.print("\t"); // -2359
    Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS));
    Serial.print("\t"); // 1688
    Serial.print(CurieIMU.getGyroOffset(X_AXIS));
    Serial.print("\t"); // 0
    Serial.print(CurieIMU.getGyroOffset(Y_AXIS));
    Serial.print("\t"); // 0
    Serial.println(CurieIMU.getGyroOffset(Z_AXIS));
  }

  CurieIMU.setGyroRate(25);
  CurieIMU.setAccelerometerRate(25);
  filter.begin(25);

  // Set the accelerometer range to 2G
  CurieIMU.setAccelerometerRange(2);
  
  // Set the gyroscope range to 250 degrees/second
  CurieIMU.setGyroRange(250);

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();
  
  // set pin states
  for (int x = 13 ; x > 4 ; x--){
  pinMode(x, OUTPUT);
  digitalWrite(x, LOW);
  }
  // calculate the cycle length
  cycle_length = 1000000/PWM_FREQ; // the length of a single cycle of the PWM signal

  /* Enable Zero Motion Detection */
  CurieIMU.setDetectionThreshold(CURIE_IMU_ZERO_MOTION, 10);  // 50mg
  CurieIMU.setDetectionDuration(CURIE_IMU_ZERO_MOTION, 10);    // 2s
  CurieIMU.interrupts(CURIE_IMU_ZERO_MOTION);

  /* Enable Motion Detection */
  CurieIMU.setDetectionThreshold(CURIE_IMU_MOTION, 5);      // 20mg
  CurieIMU.setDetectionDuration(CURIE_IMU_MOTION, 100);       // trigger times of consecutive slope data points
  CurieIMU.interrupts(CURIE_IMU_MOTION);
  mode = 1;
  Serial.println("IMU initialisation complete, waiting for events...");
}

void loop()
{

  //stabAnimation();
  if (mode)
  flashAnimation();
  //flowAnimation();
  else
  bringPitch();
  
}

static void eventCallback(void){
  if (CurieIMU.getInterruptStatus(CURIE_IMU_ZERO_MOTION)) {
    mode = true; 
  }
  if (CurieIMU.getInterruptStatus(CURIE_IMU_MOTION)) {
    mode = false;
  }   
}

void bringPitch(){
  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, heading;
  unsigned long microsNow;
  int constraint;
  // check if it's time to read data and update the filter
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {

    // read raw data from CurieIMU
    CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

    // convert from raw data to gravity and degrees/second units
    ax = convertRawAcceleration(aix);
    ay = convertRawAcceleration(aiy);
    az = convertRawAcceleration(aiz);
    gx = convertRawGyro(gix);
    gy = convertRawGyro(giy);
    gz = convertRawGyro(giz);

    // update the filter, which computes orientation
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // print the heading, pitch and roll
    pitch = filter.getPitch();
    //Serial.print("analog");
    //Serial.print(pitch);
    //Serial.print(" ");
    float analog = pitch;    
    // Smoothing with window size 10
    float analogSmooth = as.smooth(analog);
    //Serial.print("Smooth (10): "); 
    Serial.print(analogSmooth);
    Serial.print(" ");
    int mapped = map(analogSmooth , -128 ,128 , 123 , -105);
    Serial.print(mapped);
    Serial.print(" ");
    constraint = constrain(mapped , 5 , 13);
    Serial.print(constraint);
    Serial.print(" ");
    Serial.print(mode);
    Serial.println(" ");
    // Smoothing with window size 100
    float analogSmooth100 = as100.smooth(analog);
    //Serial.print("Smooth (100): ");  
    //Serial.print(analogSmooth100);
    //Serial.println(" ");
    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
    for(int x = 13 ; x > 4 ; x--)
    digitalWrite(x, LOW);

  digitalWrite(constraint, HIGH);
  //delay(35);
  }
  }
  
void flashAnimation(){
  
  const long interval = 5000;
  int ledState = LOW;
  currentMillis = millis();
  Serial.print(currentMillis);
  Serial.print(" ");
  Serial.println(previousMillis);
  
  for (int x = 13 ; x > 4 ; x--){
      digitalWrite(x, LOW);
      }
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    digitalWrite(9, HIGH);
    delay(50);
    for (int x = 13 ; x > 4 ; x--){
      digitalWrite(x, LOW);
      }

    digitalWrite(8,HIGH);
    digitalWrite(10,HIGH);
    delay(50);
    for (int x = 13 ; x > 4 ; x--){
      digitalWrite(x, LOW);
      }
      
    digitalWrite(7,HIGH);
    digitalWrite(11,HIGH);
    delay(50);
    for (int x = 13 ; x > 4 ; x--){
      digitalWrite(x, LOW);
      }

    digitalWrite(6,HIGH);
    digitalWrite(12,HIGH);
    delay(50);
    for (int x = 13 ; x > 4 ; x--){
      digitalWrite(x, LOW);
      }
    digitalWrite(5, HIGH);
    digitalWrite(13, HIGH);
    delay(50);
  }
}

void flowAnimation(){
  for (int x = 13 ; x > 4 ; x--){
    digitalWrite(x , HIGH);
    delay(35);
    digitalWrite(x , LOW);
    }
  for (int x = 5 ; x < 14 ; x++){
    digitalWrite(x , HIGH);
    delay(35);
    digitalWrite(x , LOW);
    }
  }

void stabAnimation(){
   // read in the potentiometer value
for(int j = 0 ; j < 1025 ; j++){
  int val = j;

  // map the pot value to the PWM value - 0-5V, to two decimal places
  v_out = map(val, 0,1024, 0, 500);
  duty_cycle = (v_out/100) / MAX_V; // work out what percentage of the PWM cycle we should set high
  on_time = duty_cycle * cycle_length;
  off_time = cycle_length - on_time;

  // now set high, then delay for the duty_cycle percentage * cycle_length
  if(on_time > 0)
  {
    digitalWrite(13, HIGH);
    digitalWrite(5, HIGH);
    delayMicroseconds(on_time);
  }
  digitalWrite(13, LOW);
  digitalWrite(5, LOW);
  delayMicroseconds(off_time);
}

for(int j = 1025 ; j > 0 ; j--){
  int val = j;

  // map the pot value to the PWM value - 0-5V, to two decimal places
  v_out = map(val, 0,1024, 0, 500);
  duty_cycle = (v_out/100) / MAX_V; // work out what percentage of the PWM cycle we should set high
  on_time = duty_cycle * cycle_length;
  off_time = cycle_length - on_time;

  // now set high, then delay for the duty_cycle percentage * cycle_length
  if(on_time > 0)
  {
    digitalWrite(13, HIGH);
    digitalWrite(5, HIGH);
    delayMicroseconds(on_time);
  }
  digitalWrite(13, LOW);
  digitalWrite(5, LOW);
  delayMicroseconds(off_time);
}
  }

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}
