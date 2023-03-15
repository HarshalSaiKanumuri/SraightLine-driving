#include <PID_v1.h>
#include <Wire.h>
#include <math.h>
#define I2C_SLAVE_ADDR 0x04
#define MPU_6050_ADDR 0x68

#include <MPU6050_tockn.h>

MPU6050 mpu6050(Wire);
double Input;
double Output;


double Setpoint ; // will be the desired value  ( <---- copy this ) 
//PID parameters
double Kp=2.0, Ki=1.25, Kd=0; 

void ChangeDirection(int leftMotor, int rightMotor, int SteeringAngle)
{
  Wire.beginTransmission(I2C_SLAVE_ADDR); // transmit to device #4
  
  //send leftMotorSpeed
  Wire.write((byte)((leftMotor & 0x0000FF00) >> 8));    // first byte of x, containing bits 16 to 9
  Wire.write((byte)(leftMotor & 0x000000FF));           // second byte of x, containing the 8 LSB - bits 8 to 1
  //Send rightMotorSpeed
  Wire.write((byte)((rightMotor & 0x0000FF00) >> 8));    // first byte of y, containing bits 16 to 9
  Wire.write((byte)(rightMotor & 0x000000FF));           // second byte of y, containing the 8 LSB - bits 8 to 1
  //send steering angle to arduino
  Wire.write((byte)((SteeringAngle & 0x0000FF00) >> 8));    
  Wire.write((byte)(SteeringAngle & 0x000000FF));
 
  Wire.endTransmission();   // stop transmitting
}

int findAngle()
{
  mpu6050.update();//get data from mpu
  int ZAngle = mpu6050.getAngleZ();  //Find Angle Z
  return(ZAngle);
}

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {

  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  // put your setup code here, to run once:
  Setpoint = 0;
  //myPID.SetOutputLimits(-255,255);
  myPID.SetMode(AUTOMATIC);
  myPID.SetTunings(Kp,Ki,Kd);

}

void loop() {
  // put your main code here, to run repeatedly:
  //read the value from the MPU-6050 kalman filter output

  // MPU yaw angle  = Angle
  int Angle = findAngle();
  Serial.println(Angle);

  Angle = map(Angle,30,-30,120,180);
  //myPID.Compute();

  ChangeDirection(150,150,Angle);

}
