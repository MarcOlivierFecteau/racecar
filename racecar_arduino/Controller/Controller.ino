#include <Arduino.h>
#include <assert.h>
#include <Servo.h>
#include <SPI.h>
#include "floatarray.pb.h"
#include "PBUtils.h"
#define USB_USBCON
// #define DEBUG // Toggle comment for enabling/disabling debug features
#define IMU // Toggle comment for enabling/disabling IMU (GRO830)

#ifdef IMU
#include "MPU9250.h"
MPU9250 imu(Wire, 0x68);
#endif

//============================================================================
// Global Constants
//============================================================================

const unsigned long BAUD_RATE = 250000ul;

const int ENCODER_SLAVE_SELECT_PIN = 45;
const int SERVO_PIN = 9;
const int DRIVE_PWM_PIN = 6;  // H-bridge drive PWM
const int DRIVE_DIRECTION_PIN = 42;

#ifdef DEBUG
unsigned long timer_debug = 0ul;
unsigned long time_micros = 0ul;
unsigned long time_micros_last = 0ul;
#endif

const float FILTER_RC = 0.1f;
const float VELOCITY_KP = 9.0f;
const float VELOCITY_KI = 24.0f;
const float VELOCITY_KD = 0.0f;
const float POSITION_KP = 7.0f;
const float POSITION_KI = 0.0f;
const float POSITION_KD = 1.3f;
const float POSITION_ERROR_INTEGRAL_SATURATION = 100.0f;

const unsigned int TIME_PERIOD_LOW = 5u;   // Internal PID loop cycle time (ms)
const unsigned int TIME_PERIOD_HIGH = 20u; // ROS communication cycle time (ms) --- WARNING: heavy on CPU load
const unsigned int MAX_COM_DELAY = 1000u;  // Maximum communcation delay (ms)

const int PWM_MIN_SERVO = 30;
const int PWM_ZERO_SERVO = 90;
const int PWM_MAX_SERVO = 150;
const int PWM_MIN_DRIVE = -511;
const int PWM_ZERO_DRIVE = 0;
const int PWM_MAX_DRIVE = 511;

const unsigned int DRIVE_WAKEUP_TIME = 20u; // Drive wakeup time (us)

const float U_BAT = 8.4f;                             // Nominal battery voltage (V)
const float MAX_STEERING_ANGLE = 40.0f * PI / 180.0f; // Maximum steering angle (rad)
const float RAD2PWM = (float)(PWM_ZERO_SERVO - PWM_MIN_SERVO) / MAX_STEERING_ANGLE;
const float VOLT2PWM = (float)(PWM_ZERO_DRIVE - PWM_MIN_DRIVE) / U_BAT;
const float TICK2METER = 0.000002752f; // TODO: Confirm conversion factor

const String START_DELIMITER = "<{";
const String END_DELIMITER = ">}";

//============================================================================
// Global Variables
//============================================================================

Servo steeringServo;

FloatArray sensors_msg = FloatArray_init_zero;
FloatArray cmd_msg = FloatArray_init_zero;

const Topic _TOPICS[2] = {
    {SENSORS, FloatArray_fields, &sensors_msg},
    {CMD, FloatArray_fields, &cmd_msg},
};

PBUtils pbUtils(_TOPICS);

bool reception_in_progress = false;
int input_cmd_index = 0;
char input_cmd[MAX_MSG_LEN] = {"\0"};
bool input_cmd_complete = false;
int input_cmd_type = -1;
int new_msgs_count = 0;
int new_msgs_IDs[MAX_NBS_MSG] = {0};
bool msg_discarded_length = false;

float servo_ref = 0.0f; // (rad)
float drive_ref = 0.0f; // (V)
int ctl_mode = 0;
bool drive_standby = false;
int servo_pwm = 0;
int drive_pwm = 0;
float drive_cmd = 0.0f;
long encoder_now = 0l;
long encoder_old = 0l;
long encoder_last_high = 0l;

float velocity_now = 0.0f;
float velocity_old = 0.0f;
unsigned long velocity_time_now_millis = 0ul;
unsigned long velocity_time_old_millis = 0ul;
float velocity_dt = 0.0f;
float velocity_error_integral = 0.0f;
float velocity_error_differential = 0.0f;

float position_now = 0.0f;
unsigned long position_time_now = 0ul;
unsigned long position_time_old = 0ul;
float position_dt = 0.0f;
float position_error_integral = 0.0f;
float position_error_differential = 0.0f;

unsigned long time_now = 0ul;
unsigned long time_last_low = 0ul;
unsigned long time_last_high = 0ul;
unsigned long time_last_com = 0ul; // COM watchdog

//============================================================================
// (Custom) Moving Average for PID control loop (filter on differential)
//============================================================================

const uint8_t MOVING_AVERAGE_SIZE = 20u;

struct MovingAverage {
  uint8_t cursor = 0u;
  float arr[MOVING_AVERAGE_SIZE] = {0.0f};
  float total = 0.0f;
};

MovingAverage moving_average;

void mavg_init(MovingAverage* mavg) {
  for (size_t i = 0; i < MOVING_AVERAGE_SIZE; ++i) {
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

//============================================================================
// Entry Points (main)
//============================================================================

void setup() {
  Serial.begin(BAUD_RATE);
  delay(10);

  steeringServo.attach(SERVO_PIN);
  pinMode(DRIVE_DIRECTION_PIN, OUTPUT);
  pinMode(DRIVE_PWM_PIN, OUTPUT);

  init_encoder();
  clear_encoder_count();

  steeringServo.write(PWM_MAX_SERVO);
  set_pwm(0);

#ifdef IMU
  assert(imu.begin() == 1);
  assert(imu.setAccelRange(MPU9250::ACCEL_RANGE_2G) == 1);
  assert(imu.setGyroRange(MPU9250::GYRO_RANGE_250DPS) == 1);
  assert(imu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ) == 1);
  assert(imu.setSrd(9) == 1);
#endif

  delay(3000);
  steeringServo.write(PWM_ZERO_SERVO);
}

void loop() {
  time_now = millis();
  if((time_now - time_last_com) > MAX_COM_DELAY) {
    drive_ref = 0.0f;
    ctl_mode = 2;
  }
  if((time_now - time_last_low) > TIME_PERIOD_LOW) {
    controller(time_now - time_last_low);
    time_last_low = time_now;
#ifdef DEBUG
    time_debug = time_micros - time_micros_last;
    time_micros_last = time_micros;
#endif
  }

  if(input_cmd_complete) {
    input_cmd_complete = false;
    bool status = pbUtils.decodePb(input_cmd, new_msgs_IDs, new_msgs_count);
    if(status && new_msgs_count > 0) {
      for(size_t i = 0; i < new_msgs_count; ++i) {
        if(new_msgs_IDs[i] == TOPICS::CMD) {
          cmd_callback();
        } else {
          break;
        }
      }
    } else {
      input_cmd_type = -1;
    }
  }

  unsigned long dt = time_now - time_last_high;
  if(dt > TIME_PERIOD_HIGH) {
    sensors_callback(dt);
    time_last_high = time_now;
    encoder_last_high = encoder_now;
  }
}

//============================================================================
// Functions
//============================================================================

void init_encoder() {
  pinMode(ENCODER_SLAVE_SELECT_PIN, OUTPUT);
  // Communication begins when you drop the select signal
  digitalWrite(ENCODER_SLAVE_SELECT_PIN, HIGH);
  SPI.begin();
  /*
    Initialize encoder
      - Clock division factor: 0
      - Negative index input
      - Free-running count mode
      - x4 quadrature count mode (four counts / quadrature cycle)
    NOTE: For more information on commands, see datasheet
  */
  digitalWrite(ENCODER_SLAVE_SELECT_PIN, LOW);  // Begin SPI conversation
  SPI.transfer(0x88);                           // Write to MDR0
  SPI.transfer(0x03);                           // Configure to 4-byte mode
  digitalWrite(ENCODER_SLAVE_SELECT_PIN, HIGH); // Terminate SPI conversation
}

long read_encoder() {
  unsigned int count_1, count_2, count_3, count_4 = 0u;
  long count_value = 0l;

  digitalWrite(ENCODER_SLAVE_SELECT_PIN, LOW); // Begin SPI conversation
  SPI.transfer(0x60);                          // Request count
  count_1 = SPI.transfer(0x00);                // Read highest order byte
  count_2 = SPI.transfer(0x00);
  count_3 = SPI.transfer(0x00);
  count_4 = SPI.transfer(0x00);                 // Read lowest order byte
  digitalWrite(ENCODER_SLAVE_SELECT_PIN, HIGH); // Terminate SPI conversation

  // Calculate encoder count
  count_value = (count_1 << 8) + count_2;
  count_value = (count_value << 8) + count_3;
  count_value = (count_value << 8) + count_4;

  return count_value;
}

void clear_encoder_count() {
  // Set encoder's data register to 0
  digitalWrite(ENCODER_SLAVE_SELECT_PIN, LOW); // Begin SPI conversation
  // Write to DTR
  SPI.transfer(0x98);
  // Load data
  SPI.transfer(0x00); // Highest order byte
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);                           // Lowest order byte
  digitalWrite(ENCODER_SLAVE_SELECT_PIN, HIGH); // Terminate SPI conversation

  delayMicroseconds(100); // Provides some breathing room between SPI conversations

  // Set encoder's current data register to center
  digitalWrite(ENCODER_SLAVE_SELECT_PIN, LOW); // Begin SPI conversation
  SPI.transfer(0xE0);
  digitalWrite(ENCODER_SLAVE_SELECT_PIN, HIGH); // Terminate SPI conversation
}

int servo2pwm(float cmd) {
  float pwm_d = cmd * RAD2PWM + (float)PWM_ZERO_SERVO; // Scale and offset
  int pwm = (int)(ceil((double)pwm_d));

  return constrain(pwm, PWM_MIN_SERVO, PWM_MAX_SERVO);
}

int cmd2pwm(float cmd) {
  int pwm = (int)(cmd / U_BAT * PWM_MAX_DRIVE + 0.5f);

  return constrain(pwm, PWM_MIN_DRIVE, PWM_MAX_DRIVE);
}

void set_pwm(int pwm) {
  if(pwm == 0) {
    digitalWrite(DRIVE_PWM_PIN, LOW);
    drive_standby = true;
  } else {
    if(drive_standby == 1) {
      digitalWrite(DRIVE_PWM_PIN, HIGH);
      delayMicroseconds(DRIVE_WAKEUP_TIME);
      drive_standby = false;
    }
    if(pwm < 0) {
      digitalWrite(DRIVE_DIRECTION_PIN, HIGH);
    } else {
      digitalWrite(DRIVE_DIRECTION_PIN, LOW);
    }
    // Registery-based PWM Duty-Cycle adjustement
    // Fast PWM, 9-bit, prescaler divider = 1
    TCCR4A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM41);
    TCCR4B = _BV(CS20) | _BV(WGM42);

    OCR4A = abs(pwm) - 1; // Set the Duty-Cycle of pin 6
  }
}

void controller(int dt_low) {
  static float velocity_filtered_last = 0.0f;
  static float velocity_error_last = 0.0f;

  servo_pwm = servo2pwm(servo_ref);
  steeringServo.write(servo_pwm);

  // Compute position
  encoder_now = read_encoder();
  position_now = (float)encoder_now * TICK2METER;

  // Compute velocity time
  velocity_time_now_millis = millis();
  velocity_dt = (float)(velocity_time_now_millis - velocity_time_old_millis) / 1000.0f;

  float velocity_raw = (float)(encoder_now - encoder_old) * TICK2METER / velocity_dt;
  float velocity_filtered = FILTER_RC * velocity_raw + (1 - FILTER_RC) * velocity_filtered_last;
  velocity_time_old_millis = velocity_time_now_millis;
  velocity_filtered_last = velocity_filtered;

  if(ctl_mode == 0) {
    // Zero output
    drive_pwm = PWM_ZERO_DRIVE;
    velocity_error_integral = 0.0f;
    position_error_integral = 0.0f;
  } else if(ctl_mode == 1) {
    // Fully open-loop. Commands received directly (V).
    drive_cmd = drive_ref;
    drive_pwm = cmd2pwm(drive_cmd);
    velocity_error_integral = 0.0f;
    position_error_integral = 0.0f;
  } else if(ctl_mode == 2) {
    // Low-level velocity control. Commands received in setpoints (m/s).

    float velocity_reference = drive_ref;
    float velocity_error = velocity_reference - velocity_filtered;
    velocity_error_integral += velocity_error;
    velocity_error_differential = velocity_error - velocity_error_last;

    drive_cmd =
        VELOCITY_KP * velocity_error +
        VELOCITY_KI * velocity_error_integral * (float)(TIME_PERIOD_LOW / 1000u) +
        VELOCITY_KD * velocity_error_differential / (float)(TIME_PERIOD_LOW) * 1000.0f;

    drive_pwm = cmd2pwm(drive_cmd);
    velocity_error_last = velocity_error;
  } else if(ctl_mode == 3) {
    // Low-level position control. Commands received in setpoints (m).

    static float position_error_last = 0.0f;

    float position_reference = drive_ref;
    float position_error = position_reference - position_now;
    position_error_integral += position_error;
    position_error_differential = position_error - position_error_last;

    if(position_error_integral > POSITION_ERROR_INTEGRAL_SATURATION) {
      position_error_integral = POSITION_ERROR_INTEGRAL_SATURATION;
    }

    // Compute position time
    position_time_now = millis();
    position_dt = (float)(position_time_now - position_time_old) / 1000.0f;
    position_time_old = position_time_now;

    drive_cmd =
        POSITION_KP * position_error +
        POSITION_KI * position_error_integral * position_dt +
        POSITION_KD * position_error_differential / position_dt;

    drive_pwm = cmd2pwm(drive_cmd);
    position_error_last = position_error;
  } else if(ctl_mode == 4) {
    // Reset encoder counts
    clear_encoder_count();
    velocity_error_integral = 0.0f;
    position_error_integral = 0.0f;

    drive_pwm = PWM_ZERO_DRIVE;
  } else {
    drive_pwm = PWM_ZERO_DRIVE;
    velocity_error_integral = 0.0f;
    position_error_integral = 0.0f;
  }
  set_pwm(drive_pwm); // H-bridge pwm update
  encoder_old = encoder_now;
  velocity_old = velocity_filtered;
}

/*
  | ctl_mode | drive_ref |
  | -------- | --------- |
  | 1        | [V]       |
  | 2        | [m/s]     |
  | 3        | [m]       |

  NOTE: This is a bad practice. drive_ref SHOULD be replaced by:
    - `voltage_reference`;
    - `velocity_reference`;
    - `position_reference`;
*/
void cmd_callback() {
  servo_ref = -cmd_msg.data[0];
  drive_ref = cmd_msg.data[1];
  ctl_mode = cmd_msg.data[2];
  time_last_com = millis();
}

void sensors_callback(unsigned long dt) {
  sensors_msg.data_count = 2;
  sensors_msg.data[0] = position_now; // Wheel position (m)
  sensors_msg.data[1] = velocity_old; // Wheel velocity (m/s)
#ifdef DEBUG
  sensors_msg.data_count += 8;
  sensors_msg.data[2] = drive_ref;                                             // Setpoint received by Arduino
  sensors_msg.data[3] = drive_cmd;                                             // Drive setpoint (V)
  sensors_msg.data[4] = (float)(drive_pwm);                                    // Drive setpoint (PWM)
  sensors_msg.data[5] = (float)(encoder_now);                                  // Raw encoder counts
  sensors_msg.data[6] = servo_ref;                                             // Steering angle (rad) (don't remove/change, used for GRO830)
  sensors_msg.data[7] = (float)(ctl_mode);                                     // For COM debug
  sensors_msg.data[8] = (float)(dt);                                           // Time elapsed since last publish (don't remove/change, used for GRO830)
  sensors_msg.data[9] = (float)(encoder_now - encoder_last_high) * TICK2METER; // Distance travelled since last publish (don't remove/change, used for GRO830)
#endif
#ifdef IMU
  sensors_msg.data_count += 9;
  assert(imu.readSensor() == 1);
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
  Serial.flush(); // Why would we need that?
}

/// @brief Unused function. Can we remove it?
void serialEvent() {
  while(Serial.available() > 0) {
    char input_char = Serial.read();
    int start_delimiter_index = START_DELIMITER.indexOf(input_char);
    if(reception_in_progress) {
      int end_delimiter_index = END_DELIMITER.indexOf(input_char); // If start and end index is diff, will always return end
      if(end_delimiter_index == -1) {
        input_cmd[input_cmd_index++] = input_char;
        // If the last char isn't the end delimiter the message is not going to fit
        if(input_cmd_index >= MAX_MSG_LEN - 1) {
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
    } else if (start_delimiter_index != -1) {
      reception_in_progress = true;
    }
  }
}
