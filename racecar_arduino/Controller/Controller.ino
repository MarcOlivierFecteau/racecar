/*
Description: Arduino controller for the Slash platform (UdeS Racecar)
Authors: SherbyRobotics, Marc-Olivier Fecteau, Justine Landry, Loïc Legault, Félix Tremblay
Project Start: 2024-08-28
Last Updated: 2024-09-25
*/

//==========================================================================//
// DEPENDENCIES
//==========================================================================//

#include <Arduino.h>
#include <assert.h>
#include <SPI.h>
#include <Servo.h>
#include <HardwareSerial.h>
#include <USBAPI.h>
#include "PBUtils.h"
#include "floatarray.pb.h"
#include <stddef.h>

#define USB_USBCON
#define IMU // User-specified define

#ifdef IMU
#include "MPU9250.h"
MPU9250 imu(Wire, 0x68);
#endif

//==========================================================================//
// USER-SPECIFIED VARIABLES
//==========================================================================//

static const uint8_t SLAVE_SELECT_ENCODER_PIN = 45;
static const uint8_t SERVO_PWM_PIN = 9;
static const uint8_t DRIVE_PWM_PIN = 6; // H-bridge drive PWM
static const uint8_t DRIVE_DIRECTION_PIN = 42;

// static const float FILTER_RC = 0.1f;
static const float VELOCITY_KP = 9.0f;
static const float VELOCITY_KI = 24.0f;
static const float VELOCITY_KD = 0.0f;
static const float POSITION_KP = 7.0f;
static const float POSITION_KI = 0.0f;
static const float POSITION_KD = 1.3f;

static const float TIME_PERIOD_LOW = 2.0f;          // Internal PID loop cycle (ms)
static const unsigned long TIME_PERIOD_HIGH = 20ul; // ROS 2 communication cycle (ms)
static const unsigned long MAX_COM_DELAY = 1000ul;  // Maximum communication delay (ms)

static const uint8_t DRIVE_WAKEUP_TIME = 20u; // µs

//==========================================================================//
// CONTROLLER CONSTANTS
//==========================================================================//

static const int PWM_MIN_SERVO = 30;
static const int PWM_ZERO_SERVO = 90;
static const int PWM_MAX_SERVO = 150;

static const int PWM_MIN_DRIVE = -511;
static const int PWM_ZERO_DRIVE = 0;
static const int PWM_MAX_DRIVE = 511;

static const float MAX_BATTERY_VOLTAGE = 8.0f;              // V
static const float MAX_STEERIG_ANGLE = 40.0f * PI / 180.0f; // rad
static const float RAD2PWM = (float)(PWM_ZERO_SERVO - PWM_MIN_SERVO) / MAX_STEERIG_ANGLE;
static const float VOLT2PWM = (float)(PWM_ZERO_DRIVE - PWM_MIN_DRIVE) / MAX_BATTERY_VOLTAGE;
static const float TICK2METER = 0.000002752; // TODO: adjust value

//==========================================================================//
// CONTROLLER VARIABLES
//==========================================================================//

Servo steeringServo;

static long timer_debug = 0;
static long time_micros = 0;
static long time_micros_last = 0;

static float servo_reference = 0.0f; // rad
static float drive_reference = 0.0f; // V
static int8_t control_mode = 0;

static int servo_pwm = 0;
static int drive_pwm = 0;
static float drive_cmd = 0.0f;

static long encoder_now = 0;
static long encoder_old = 0;

static float position_now = 0.0f; // m
static float velocity_now = 0.0f; // m/s
static float velocity_old = 0.0f; // m/s

static unsigned long velocity_time_now = 0; // ms
static unsigned long velocity_time_old = 0; // ms
static unsigned long position_time_now = 0; // ms
static unsigned long position_time_old = 0; // ms

static float velocity_error_integral = 0.0f;
static float velocity_error_differential = 0.0f;
static float position_error_integral = 0.0f;
static float position_error_differential = 0.0f;

static unsigned long time_last_com = 0; // COM watchdog

static long encoder_last_high = 0;

//==========================================================================//
// (CUSTOM) MOVING AVERAGE FOR CONTROL LOOP
//==========================================================================//

static const uint8_t MOVING_AVERAGE_SIZE = 20;

typedef struct MovingAverage {
  uint8_t cursor = 0;
  float arr[MOVING_AVERAGE_SIZE] = {0};
  float total = 0.0;
} MovingAverage;

MovingAverage moving_average;

void mavg_init(MovingAverage* mavg) {
  for(uint8_t i = 0; i < MOVING_AVERAGE_SIZE; ++i) {
    mavg->arr[i] = 0.0f;
  }
}

void mavg_add(MovingAverage* mavg, float value) {
  mavg->total -= mavg->arr[mavg->cursor];
  mavg->arr[mavg->cursor++] = value;
  mavg->total += value;
  mavg->cursor = (mavg->cursor == MOVING_AVERAGE_SIZE ? 0 : mavg->cursor); // Circular buffer
}

float mavg_get_avg(MovingAverage* mavg) {
  return mavg->total / (float)MOVING_AVERAGE_SIZE;
}

//==========================================================================//
// TOPICS
//==========================================================================//

FloatArray sensors_msg = FloatArray_init_zero;
FloatArray cmd_msg = FloatArray_init_zero;

const Topic topics[2] = {
  {SENSORS, FloatArray_fields, &sensors_msg},
  {CMD, FloatArray_fields, &cmd_msg}
};

//==========================================================================//
// SERIAL COMMUNICATION
//==========================================================================//

PBUtils pbUtils(topics);

static char input_cmd[MAX_MSG_LEN] = "\0";
static bool input_cmd_complete = false;
static int input_cmd_type = -1;
static int new_msgs_count = 0;
static bool msg_discarded_length = false;

static const unsigned long BAUD_RATE = 250000ul;

//==========================================================================//
// FUNCTIONS
//==========================================================================//

void encoder_init() {
  pinMode(SLAVE_SELECT_ENCODER_PIN, OUTPUT);
  digitalWrite(SLAVE_SELECT_ENCODER_PIN, HIGH);
  SPI.begin();
  /*
    Initialize encoder
      Clock division factor: 0
      Negative index input
      Free-running count mode
      x4 quadrature count mode (four counts / quadrature cycle)
    NOTE: for more information on commands, see datasheet
  */
  digitalWrite(SLAVE_SELECT_ENCODER_PIN, LOW);  // Begin SPI communication
  SPI.transfer(0x88);                           // Write to MDR0
  SPI.transfer(0x03);                           // Configure to 4-byte mode
  digitalWrite(SLAVE_SELECT_ENCODER_PIN, HIGH); // Terminate SPI communication
}

long encoder_read() {
  // Initialize temporary variables for SPI read
  static unsigned int count_1, count_2, count_3, count_4 = 0u;
  static long count_value = 0l;

  // Read encoder
  digitalWrite(SLAVE_SELECT_ENCODER_PIN, LOW);  // Begin SPI communication
  SPI.transfer(0x60);                           // Request count
  count_1 = SPI.transfer(0x00);                 // Read highest order byte
  count_2 = SPI.transfer(0x00);
  count_3 = SPI.transfer(0x00);
  count_4 = SPI.transfer(0x00);                 // Read lowest order byte
  digitalWrite(SLAVE_SELECT_ENCODER_PIN, HIGH); // Terminate SPI communication

  // Calculate encoder count
  count_value = (count_1 << 8) + count_2;
  count_value = (count_value << 8) + count_3;
  count_value = (count_value << 8) + count_4;

  return count_value;
}

void encoder_clear_count() {
  /* Set encoder1's data register to 0 */
  digitalWrite(SLAVE_SELECT_ENCODER_PIN, LOW);  // Begin SPI communication
  SPI.transfer(0x98);                           // Write to DTR
  SPI.transfer(0x00);                           // Highest order byte
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);                           // lowest order byte
  digitalWrite(SLAVE_SELECT_ENCODER_PIN, HIGH); // Terminate SPI communication

  delayMicroseconds(100); // Provides some breathing room between SPI communications

  // Set encoder1's current data register to center
  digitalWrite(SLAVE_SELECT_ENCODER_PIN, LOW);  // Begin SPI communication
  SPI.transfer(0xE0);
  digitalWrite(SLAVE_SELECT_ENCODER_PIN, HIGH); // Terminate SPI communication
}

#define clamp(x, min, max) ((x)<(min)?(min):((x)>(max)?(max):(x)))

int servo_angle_to_pwm(float cmd) {
  static float pwm_d = cmd * RAD2PWM + (float)PWM_ZERO_SERVO;  // Scale and offset
  static int pwm = (int)(pwm_d + 0.5f);  // Rounding and conversion

  return clamp(pwm, PWM_MIN_SERVO, PWM_MAX_SERVO);
}

int voltage_cmd_to_pwm(float cmd) {
  static int pwm = (int)(cmd / MAX_BATTERY_VOLTAGE * (float)PWM_MAX_DRIVE + 0.5f); // Scale and offset

  return clamp(pwm, PWM_MIN_DRIVE, PWM_MAX_DRIVE);
}

void set_pwm(int pwm) {
  static bool drive_standby = false;

  if(pwm == 0) {
    digitalWrite(DRIVE_PWM_PIN, LOW);
    drive_standby = true;
  } else {
    if(drive_standby) {
      digitalWrite(DRIVE_PWM_PIN, HIGH);
      delayMicroseconds(DRIVE_WAKEUP_TIME);
      drive_standby = false;
    }

    if(pwm < 0) {
      digitalWrite(DRIVE_DIRECTION_PIN, HIGH);
    } else {
      digitalWrite(DRIVE_DIRECTION_PIN, LOW);
    }

    /* === Registery-based pwm duty cycle adjustement === */

    // Fast PWM, 9-bit, prescaler divider = 1
    TCCR4A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM41);
    TCCR4B = _BV(CS20) | _BV(WGM42);

    OCR4A = abs(pwm) - 1; // Set the Duty-Cycle of pin 6
  }
}

void controller() {
  static float velocity_error_last = 0.0f;
  static float position_error_last = 0.0f;
  static long velocity_time = 0; // s

  servo_pwm = servo_angle_to_pwm(servo_reference);
  steeringServo.write(servo_pwm);

  encoder_now = encoder_read();
  position_now = (float)encoder_now * TICK2METER;

  // Velocity computation
  velocity_time_now = millis();
  velocity_time = velocity_time_now - velocity_time_old;

  /* === Dérivée filtrée === */
  // float vel_raw = (enc_now - enc_old) * tick2m / dt_low * 1000;
  float vel_raw = (float)(encoder_now - encoder_old) * TICK2METER / (float)velocity_time * 1000;
  mavg_add(&moving_average, vel_raw);
  float velocity_filtered = mavg_get_avg(&moving_average);
  velocity_time_old = velocity_time_now;

  /*
  Control modes:
    - 0: Zero output
    - 1: Full open-loop
    - 2: Low-level velocity control
    - 3: Low-level position control
    - 4: Reset encoder count
  */

  if(control_mode == 0) {
    drive_pwm = PWM_ZERO_DRIVE;
    
    velocity_error_integral = 0.0f;
    position_error_integral = 0.0f;
  } else if (control_mode == 1) {
    // Commands received in Volts directly
    drive_cmd = drive_reference;
    drive_pwm = voltage_cmd_to_pwm(drive_cmd);

    velocity_error_integral = 0.0f;
    position_error_integral = 0.0f;
  } else if (control_mode == 2) {
    // Commands received in [m/s] as setpoints
    static float velocity_reference, velocity_error = 0.0f;

    velocity_reference = drive_reference;
    velocity_error = velocity_reference - velocity_filtered;
    velocity_error_integral += velocity_error;
    velocity_error_differential = velocity_error_last - velocity_error;

    float velocity_proportional = VELOCITY_KP * velocity_error;
    float velocity_integral = VELOCITY_KI * velocity_error_integral * (TIME_PERIOD_LOW / 1000.0f);
    float velocity_differential = VELOCITY_KD * velocity_error_differential / (TIME_PERIOD_LOW/1000.0f);

    drive_cmd = velocity_proportional + velocity_integral + velocity_differential;
    drive_pwm = voltage_cmd_to_pwm(drive_cmd);

    velocity_error_last = velocity_error;
  } else if(control_mode == 3) {
    // Commands received in [m] as setpoints
    static const float POSITION_ERROR_INTEGRAL_SATURATION = 100.0f;
    static float position_reference, position_error = 0.0f;
    static float position_time = 0; // s

    position_reference = drive_reference;
    position_error = position_reference - position_now;
    position_error_integral += position_error;
    position_error_differential = position_error_last - position_error;

    // Anti wind-up
    if(position_error_integral > POSITION_ERROR_INTEGRAL_SATURATION) {
      position_error_integral = POSITION_ERROR_INTEGRAL_SATURATION;
    }

    position_time_now = millis();
    position_time = (float)(position_time_now - position_time_old) / 1000.0f;

    float position_proportional = POSITION_KP * position_error;
    float position_integral = POSITION_KI * position_error_integral * position_time;
    float position_differential = POSITION_KD * position_error_differential / position_time;

    drive_pwm = voltage_cmd_to_pwm(drive_cmd);

    position_error_last = position_error;
  } else if(control_mode == 4) {
    encoder_clear_count();
    drive_pwm = PWM_ZERO_DRIVE;
    
    velocity_error_integral = 0.0f;
    position_error_integral = 0.0f;
  } else {
    drive_pwm = PWM_ZERO_DRIVE;
    
    velocity_error_integral = 0;
    position_error_integral = 0;
  }
  
  set_pwm(drive_pwm); // Update H-bridge PWM

  encoder_old = encoder_now;
  velocity_old = velocity_filtered;
}

//==========================================================================//
// CALLBACKS
//==========================================================================//

void cmd_callback() {
  servo_reference = -cmd_msg.data[0]; // [rad]
  drive_reference = cmd_msg.data[1];  // [V] | [m/s] | [m]
  control_mode = (uint8_t)cmd_msg.data[2];

  time_last_com = millis();
}

void sensors_callback(unsigned long dt) {
  sensors_msg.data_count = 19;
  sensors_msg.data[0] = position_now; // wheel position (m)
  sensors_msg.data[1] = velocity_old; // wheel velocity (m/s)

  // For DEBUG
  sensors_msg.data[2] = drive_reference;                                       // Set point received by Arduino
  sensors_msg.data[3] = drive_cmd;                                             // Drive set point (V)
  sensors_msg.data[4] = (float)drive_pwm;                                      // Drive set point (PWM)
  sensors_msg.data[5] = (float)encoder_now;                                    // Raw encoder counts
  sensors_msg.data[6] = (float)servo_reference;                                // Steering angle (GRO830)
  sensors_msg.data[7] = (float)control_mode;                                   // For COM debug
  sensors_msg.data[8] = (float)dt;                                             // Time elapsed since last publish (GRO830)
  sensors_msg.data[9] = (float)(encoder_now - encoder_last_high) * TICK2METER; // distance travelled since last publish (GRO830)

// Read IMU (GRO830)
#ifdef IMU
  imu.readSensor();
  sensors_msg.data[10] = imu.getAccelX_mss();
  sensors_msg.data[11] = imu.getAccelY_mss();
  sensors_msg.data[12] = imu.getAccelZ_mss();
  sensors_msg.data[13] = imu.getGyroX_rads();
  sensors_msg.data[14] = imu.getGyroY_rads();
  sensors_msg.data[15] = imu.getGyroZ_rads();
  sensors_msg.data[16] = imu.getMagX_uT();
  sensors_msg.data[17] = imu.getMagY_uT();
  sensors_msg.data[18] = imu.getMagZ_uT();
#endif

  pbUtils.pbSend(1, SENSORS);
  Serial.flush();
}

//==========================================================================//
// ENTRY POINTS (MAIN)
//==========================================================================//

void setup() {
  assert(Serial.begin(BAUD_RATE));
  delay(10);

  steeringServo.attach(SERVO_PWM_PIN);
  pinMode(DRIVE_DIRECTION_PIN, OUTPUT);
  pinMode(DRIVE_PWM_PIN, OUTPUT);

  encoder_init();
  encoder_clear_count();

  mavg_init(&moving_average);

  steeringServo.write(PWM_MAX_SERVO);
  set_pwm(0);

#ifdef IMU
  assert(imu.begin() == 1);
  assert(imu.setAccelRange(MPU9250::ACCEL_RANGE_2G) == 1);
  assert(imu.setGyroRange(MPU9250::GYRO_RANGE_250DPS) == 1);
  assert(imu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ) == 1);
  assert(imu.setSrd(9) == 1); //100 Hz update rate
#endif

  delay(3000);
  steeringServo.write(PWM_ZERO_SERVO);
}

void loop() {
  static unsigned long time_now, time_last_low, time_last_high = 0ul;
  static int new_msgs_IDs[MAX_NBS_MSG];

  time_now = millis();

  if((time_now - time_last_com) > MAX_COM_DELAY) {
    drive_reference = 0.0f;
    control_mode = 0;
  }

  if((time_now - time_last_low) > TIME_PERIOD_LOW) {
    controller();
    
    time_last_low = time_now;
    timer_debug = time_micros - time_micros_last;
    time_micros_last = time_micros;
  }

  if(input_cmd_complete) {
    input_cmd_complete = false;
    bool status = pbUtils.decodePb(input_cmd, new_msgs_IDs, new_msgs_count);

    if(status && new_msgs_count > 0) {
      for(uint8_t i = 0; i < new_msgs_count; ++i) {
        if(new_msgs_IDs[i] == CMD) cmd_callback();
        else break;
      }
    } else {
      input_cmd_type = -1;
    }
  }

  unsigned long dt = time_now - time_last_high;
  if (dt > TIME_PERIOD_HIGH) {
    sensors_callback(dt);

    time_last_high = time_now;
    encoder_last_high = encoder_now;
  }
}

//==========================================================================//
// SERIAL (UNUSED)
//==========================================================================//

void serial_event() {
  static const String START_DELIMITER = "<{";
  static const String END_DELIMITER = ">}";
  static bool reception_in_progress = false;
  static uint8_t input_cmd_index = 0;


  while(Serial.available() > 0) {
    char input_character = Serial.read();
    int start_delimiter_index = START_DELIMITER.indexOf(input_character);

    if(reception_in_progress) {
      int end_delimiter_index = END_DELIMITER.indexOf(input_character); // If start and end index is diff, will always return end
      if(end_delimiter_index == -1) {
        input_cmd[input_cmd_index++] = input_character;

        if(input_cmd_index >= MAX_MSG_LEN - 1) {  // If the last char isn't the end delimiter the message is NOT going to fit
          msg_discarded_length = true;
          input_cmd[input_cmd_index] = '\0';
          reception_in_progress = false;
          input_cmd_index = 0;
        }
      } else {
        input_cmd[input_cmd_index] = '\0';
        reception_in_progress = false;
        input_cmd_index = 0;
        input_cmd_type = end_delimiter_index;
        input_cmd_complete = true;
      }
    } else if(start_delimiter_index != -1) {
      reception_in_progress = true;
    }
  }
}
