#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <Servo.h>

LSM9DS1 imu;
Servo myservo; // create a servo object

//  I2C Setup

// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW
#define PRINT_SPEED 1// 250 ms between prints

int pos = 90;  
float tau = 0.075;
float a = 0.0;

// complimentary1:
float x_angleC = 0;

// complimentary2:
float x_angle2C = 0;
float x1 = 0;
float y1 = 0;
float x2 = 0;

// KasBot V1 - Kalman filter module
float x_angleK = 0;
float Q_angle  =  0.01; //0.001
float Q_gyro   =  0.0003;  //0.003
float R_angle  =  0.01;  //0.03
float x_bias = 0;
float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
float  y, S;
float K_0, K_1;

void setup() 
{
  Serial.begin(115200);
  myservo.attach(10);
  
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;

  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1)
      ;
  }
}

void loop()
{
  imu.readGyro(); // get readings from gyroscope (gx, gy, gz)
  imu.readAccel(); // get readings from accelerometer (ax, ay, az)
  imu.readMag(); // get readings from magnetometer (mx, my, mz)

  float tiltAngle = getPitchAcc(imu.ax, imu.ay, imu.az);
  // Determining tile angle using Complementary filter of 1st order
  float AngleComplementary1 = Complementary1(tiltAngle, imu.calcGyro(imu.gy), 10);
  // Determining tile angle using Complementary filter of 2nd order
  float AngleComplementary2 = Complementary2(tiltAngle, imu.calcGyro(imu.gy), 10);
  // Determining tile angle using Kalman filter
  float AngleKalman = kalmanCalculate(tiltAngle, imu.calcGyro(imu.gy), 13);

  Serial.print(tiltAngle);
  Serial.print(",");
  Serial.print(AngleComplementary1);
  Serial.print(",");
  Serial.print(AngleComplementary2);
  Serial.print(",");
  Serial.print(AngleKalman);
  Serial.println();

  // For the final code Kalman filter was used
  myservo.write(95 - int(AngleKalman));
  delay(0);
}

// Method that calculates tilt angle using accelerometer only
float getPitchAcc(
float ax, float ay, float az)
{
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  pitch = pitch * 180.0 / PI;
  return pitch;
}

// Method that calculates tilt angle using complementary filter of 1st order
float Complementary1(
float newAngle, float newRate, float looptime)
{
  float dtC = float(looptime) / 1000.0;
  a = tau/(tau+dtC);
  x_angleC = a* (x_angleC + newRate*dtC) + (1-a) * (newAngle);
  //Serial.print("Complementary1: ");
  //Serial.println(x_angleC);
  //Serial.println("------------------------------");
  return x_angleC;
}

// Method that calculates tilt angle using complementary filter of 2nd order
float Complementary2(float newAngle, float newRate,int looptime) {
float k=10;
float dtc2=float(looptime)/1000.0;

x1 = (newAngle -   x_angle2C)*k*k;
y1 = dtc2*x1 + y1;
x2 = y1 + (newAngle -   x_angle2C)*2*k + newRate;
x_angle2C = dtc2*x2 + x_angle2C;
return x_angle2C;
}

// Method that calculates tilt angle using Kalman filter
float kalmanCalculate(float newAngle, float newRate,int looptime)
{
float dt = float(looptime)/1000;
x_angleK += dt * (newRate - x_bias);
P_00 +=  - dt * (P_10 + P_01) + Q_angle * dt;
P_01 +=  - dt * P_11;
P_10 +=  - dt * P_11;
P_11 +=  + Q_gyro * dt;

y = newAngle - x_angleK;
S = P_00 + R_angle;
K_0 = P_00 / S;
K_1 = P_10 / S;

x_angleK +=  K_0 * y;
x_bias  +=  K_1 * y;
P_00 -= K_0 * P_00;
P_01 -= K_0 * P_01;
P_10 -= K_1 * P_00;
P_11 -= K_1 * P_01;

return x_angleK;
}
