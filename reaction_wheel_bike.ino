#include <SimpleFOC.h>  // ver. 2.0.1
#include <Servo.h>

Servo steering_servo, wheel_servo;  

BLDCMotor motor = BLDCMotor(7);    
BLDCDriver3PWM driver = BLDCDriver3PWM(3, 5, 6);  // BLDC motor init 9, 10, 11 or 3, 5, 6 (gimbal controller)
MagneticSensorSPI sensor = MagneticSensorSPI(14, 14, 0x3FFF);

//Sensor output scaling
#define accSens   0               // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens  1               // 0 = 250rad/s, 1 = 500rad/s, 2 1000rad/s, 3 = 2000rad/s

#define VBAT                 A3
#define STEERING_MAX         250
#define STEERING_CENTER      1450
#define MAX_SERVO_ST         5
#define MAX_SERVO_SPEED      5

int16_t  AcY, AcZ, GyX, gyroX;

//IMU offset values
int16_t  AcY_offset = 3340; 
int16_t  AcZ_offset = -872;
int16_t  GyX_offset = 0;
int32_t  GyX_offset_sum = 0;

float robot_angle;
float robot_angle_offset;
float Acc_angle;            

float alpha = 0.10;         // low pass from 0.00 to 1.00 lower value - higher filtering
float gyroXfilt;

float K1Gain = 0.84;  
float K2Gain = 0.16;  
float K3Gain = 0.14;  
float K4Gain = 0.003; 
int lt = 8; 

int loop_count = 0;
int loop2_count = 0;
int steering_value = STEERING_CENTER;
int speed_value = 1500;
int steering_remote = 0; 
int speed_remote = 0; 

float m_speed = 0;

bool vertical = false;

uint32_t timer;

void setup() {

  Serial.begin(115200);
  delay(100);
  steering_servo.attach(15);                                    // attaches the servo on pin
  wheel_servo.attach(16);                                       // attaches the servo on pin
  steering_servo.writeMicroseconds(steering_value);
  wheel_servo.writeMicroseconds(1500);
  
  delay(1000);
  sensor.init();                                                // initialise magnetic sensor hardware
  motor.linkSensor(&sensor);                                    // link the motor to the sensor

  Serial.println("Calibrating gyro...");
  angle_setup();

  driver.voltage_power_supply = 12;                             // power supply voltage [V]
  driver.init();
  motor.linkDriver(&driver);                                    // link the motor and the driver
  motor.controller = ControlType::voltage;                      // set control loop type to be used
  motor.voltage_sensor_align = 4;                               // aligning voltage 
  motor.init();                                                 // initialize motor
  motor.initFOC();                                              // align encoder and start FOC
                                      
}

void loop() {
  // ~1ms 
  motor.loopFOC();

  angle_calc();
  gyroX = -GyX / 131.0; // Convert to deg/s
  gyroXfilt = alpha * gyroX + (1 - alpha) * gyroXfilt;
  
  if (loop_count++ > lt) {  // control loop 
    //Tuning();
    readControlParameters();
    float target_voltage = 0;
    if (vertical) {      
      target_voltage = controlleR(robot_angle - robot_angle_offset, gyroXfilt, motor.shaftVelocity(), m_speed);
      m_speed += motor.shaftVelocity() / 2; 

      motor.move(target_voltage);

      if ((steering_value - STEERING_CENTER - steering_remote) > MAX_SERVO_ST)
        steering_value -= MAX_SERVO_ST;
      else if ((steering_value - STEERING_CENTER - steering_remote) < -MAX_SERVO_ST)
        steering_value += MAX_SERVO_ST;
      else
        steering_value = STEERING_CENTER + steering_remote;

       robot_angle_offset = (float) (steering_value - STEERING_CENTER) / 50;
      
      if ((speed_value - 1500 - speed_remote) > MAX_SERVO_SPEED)
        speed_value -= MAX_SERVO_SPEED;
      else if ((speed_value - 1500 - speed_remote) < -MAX_SERVO_SPEED)
        speed_value += MAX_SERVO_SPEED;
      else
        speed_value = 1500 + speed_remote;  
      
      steering_servo.writeMicroseconds(steering_value);
      wheel_servo.writeMicroseconds(speed_value);
    } else {
      motor.move(target_voltage);
      steering_value = STEERING_CENTER;
      steering_remote = 0;
      speed_remote = 0;
      speed_value = 1500;
      steering_servo.writeMicroseconds(STEERING_CENTER);
      wheel_servo.writeMicroseconds(1500);
      m_speed = 0;
    }
    loop_count = 0;
  }
  if (loop2_count++ > 200) {
    if (analogRead(VBAT) / 51.5 < 9.4) vertical = false;  // "51.5" the value depends on the resistor divider
    loop2_count = 0;
  } 
}

