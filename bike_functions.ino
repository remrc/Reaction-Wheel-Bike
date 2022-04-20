#define MPU6050      0x68         // Device address
#define ACCEL_CONFIG 0x1C         // Accelerometer configuration address
#define GYRO_CONFIG  0x1B         // Gyro configuration address

//Registers: Accelerometer, Temp, Gyroscope
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H   0x41
#define TEMP_OUT_L   0x42
#define GYRO_XOUT_H  0x43
#define GYRO_XOUT_L  0x44
#define GYRO_YOUT_H  0x45
#define GYRO_YOUT_L  0x46
#define GYRO_ZOUT_H  0x47
#define GYRO_ZOUT_L  0x48

#define PWR_MGMT_1   0x6B
#define PWR_MGMT_2   0x6C

#define Gyro_amount 0.996   // percent of gyro in complementary filter

void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

void angle_setup() {
  Wire.begin();
  delay(100);
  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3);   // Specifying output scaling of accelerometer
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3);   // Specifying output scaling of gyroscope
  delay(100);

  // calc X gyro offset by averaging 1024 values
  for (int i = 0; i < 1024; i++) {
    angle_calc();
    GyX_offset_sum += GyX;
    delay(5);
  }
  GyX_offset = GyX_offset_sum >> 10;
  Serial.print("GyX offset: ");  Serial.println(GyX_offset);
}

void angle_calc() {

  // read raw accel/gyro measurements from device
  Wire.beginTransmission(MPU6050);
  Wire.write(ACCEL_YOUT_H);                       
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);   
  AcY = Wire.read() << 8 | Wire.read(); 
  
  Wire.beginTransmission(MPU6050);
  Wire.write(ACCEL_ZOUT_H);                  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);  
  AcZ = Wire.read() << 8 | Wire.read(); 

  Wire.beginTransmission(MPU6050);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true); 
  GyX = Wire.read() << 8 | Wire.read();

  // add mpu6050 offset values
  AcY -= AcY_offset;
  AcZ -= AcZ_offset;
  GyX -= GyX_offset;

  //use complementary filter to calculate robot angle
  robot_angle += -GyX * 6.07968E-5;                      // ? this is not correct
  Acc_angle = atan2(AcY, -AcZ) * 57.2958;                // angle from acc. values * 57.2958 (deg/rad)
  robot_angle = robot_angle * Gyro_amount + Acc_angle * (1.0 - Gyro_amount);

  //Serial.println(robot_angle);
  
  if (abs(robot_angle) > 10) vertical = false;
  if (abs(robot_angle) < 0.3) vertical = true;

}

int sign(int x) {
  if (x > 0) return 1;
  if (x < 0) return -1;
  if (x = 0) return 0;
}

// function constraining the angle in between -pi and pi, in degrees -180 and 180
float constrainAngle(float x){
    x = fmod(x + M_PI, _2PI);
    if (x < 0)
        x += _2PI;
    return x - M_PI;
}

float controlleR(float p_angle, float p_vel, float m_vel, float a_vel) {
  float u =  K1Gain * p_angle + K2Gain * p_vel + K3Gain * m_vel + K4Gain * a_vel;
  if (abs(u) > 12 * 0.8) 
    u = sign(u) * 12 * 0.8;
  return u;
}
