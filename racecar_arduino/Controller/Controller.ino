#include <Arduino.h>
#include <assert.h>
// #include <HardwareSerial.h>
#include <Servo.h>
#include <SPI.h>

#define IMU

#ifdef IMU
#include "MPU9250.h"
#endif

//==========================================================================//
// USER-SPECIFIED CONTROLLER CONSTANTS
//==========================================================================//

static const float FILTER_RC = 0.1f;  // Velocity RC filter factor base value
static const float VEL_KP = 1.0f;
static const float VEL_KI = 0.0f;
static const float VEL_KD = 0.0f;
static const float POS_KP = 0.0f;
static const float POS_KI = 0.0f;
static const float POS_KD = 0.0f;
static const float POS_ERROR_INTEGRAL_SAT = 100.0f;

static const uint32_t LOW_LEVEL_CYCLE_MS = (uint32_t)2;    // Internal PID loop cycle (ms)
static const uint32_t SERIAL_COM_CYCLE_MS = (uint32_t)50;  // ROS 2 communication cycle (ms)
static const uint32_t MAX_COM_DELAY_MS = (uint32_t)1000;   // Maximum communication delay (i.e. watchdog) (ms)

static const uint32_t DRIVE_WAKEUP_TIME_US = (uint32_t)20;  // µs

//==========================================================================//
// CONTROLLER CONSTANTS
//==========================================================================//

static const uint8_t SLAVE_SELECT_ENCODER_PIN = (uint8_t)45;
static const uint8_t SERVO_PWM_PIN = (uint8_t)9;
static const uint8_t DRIVE_PWM_PIN = (uint8_t)6;  // H-Bridge drive PWM
static const uint8_t DRIVE_DIR_PIN = (uint8_t)42;

// Protocol definitions
#define SERIAL_OK 0
#define SERIAL_SYNC_ERROR -1
#define SERIAL_IO_ERROR -2
#define SERIAL_TIMEOUT_ERROR -3
static const uint16_t TIMEOUT_MS = (uint16_t)1000;
static const uint8_t INIT_BYTE = 0x11;
static const uint8_t INIT_ACK_BYTE = 0x1A;
static const uint8_t INIT_END_BYTE = 0x1E;
static const uint8_t CMD_MSG_START_BYTE = 0xCA;
static const uint8_t CMD_MSG_END_BYTE = 0xCF;
static const size_t CMD_MSG_PACKET_SIZE = sizeof(int8_t) + sizeof(float) * 2;
static const uint8_t ODOMETRY_START_BYTE = 0x0A;
static const uint8_t ODOMETRY_END_BYTE = 0x0F;
static const size_t ODOMETRY_SIZE = sizeof(float) * 15 + sizeof(int32_t) * 2 + sizeof(uint32_t) + sizeof(int8_t);

static const uint32_t BAUD_RATE = (uint32_t)9600;

static const int32_t PWM_MIN_SERVO = (int32_t)30;
static const int32_t PWM_ZERO_SERVO = (int32_t)90;
static const int32_t PWM_MAX_SERVO = (int32_t)150;

static const int32_t PWM_MIN_DRIVE = (int32_t)-511;
static const int32_t PWM_ZERO_DRIVE = (int32_t)0;
static const int32_t PWM_MAX_DRIVE = (int32_t)511;

static const float MAX_BATTERY_VOLTAGE = 8.0f;                // V
static const float MAX_STEERING_ANGLE = 40.0f * PI / 180.0f;  // rad
static const float RAD2PWM = (float)(PWM_ZERO_SERVO - PWM_MIN_SERVO) / MAX_STEERING_ANGLE;
static const float VOLT2PWM = (float)(PWM_ZERO_DRIVE - PWM_MIN_DRIVE) / MAX_BATTERY_VOLTAGE;
static const float TICK2METER = 0.000002752f;  // TODO: Ajuster la valeur en fonction des mesures

// I/O Data Structures

typedef struct {
  int8_t control_mode;
  float drive_ref;
  float servo_ref;
} CmdMsg;

typedef struct {
  float pos;
  float vel;
  float drive_ref;
  float drive_cmd;
  int32_t drive_pwm;
  int32_t encoder;
  float servo_ref;
  int8_t control_mode;
  uint32_t dt_ms;
  float delta_distance_m;
  float accel_x_mss;
  float accel_y_mss;
  float accel_z_mss;
  float gyro_x_rads;
  float gyro_y_rads;
  float gyro_z_rads;
  float mag_x_uT;
  float mag_y_uT;
  float mag_z_uT;
} Sensors;

//==========================================================================//
// CONTROLLER VARIABLES (GLOBAL)
//==========================================================================//

#ifdef IMU
MPU9250 imu(Wire, 0x68);
#endif

Servo steeringServo;

CmdMsg cmd_msg_ = {
  .control_mode = (int8_t)0,
  .drive_ref = 0.0f,
  .servo_ref = 0.0f;
};

Sensors payload_ = {
  .pos = 0.0f,
  .vel = 0.0f,
  .drive_ref = 0.0f,
  .drive_cmd = 0.0f,
  .drive_pwm = PWM_ZERO_DRIVE,
  .encoder = (uint32_t)0,
  .servo_ref = 0.0f,
  .control_mode = (int8_t)-1,
  .dt_ms = (uint32_t)0,
  .delta_distance_m = 0.0f,
  .accel_x_mss = 0.0f,
  .accel_y_mss = 0.0f,
  .accel_z_mss = 0.0f,
  .gyro_x_rads = 0.0f,
  .gyro_y_rads = 0.0f,
  .gyro_z_rads = 0.0f,
  .mag_x_uT = 0.0f,
  .mag_y_uT = 0.0f,
  .mag_z_uT = 0.0f,
};

// === Memory Variables ===

// Inputs
float servo_ref = 0.0f;  // rad
float drive_ref = 0.0f;  // V
int8_t control_mode = (int8_t)0;
bool drive_standby = false;

// Outputs
int32_t servo_pwm = (int32_t)0;
int32_t drive_pwm = (int32_t)0;
float drive_cmd = 0.0f;

// Timing variables
uint32_t time_last_com = millis();

// Odometry
int32_t encoder = (int32_t)0;
int32_t encoder_last = (int32_t)0;
float pos, pos_error_integral, pos_error_differential = 0.0f;
float vel_filtered, vel_error_integral = 0.0f;

//==========================================================================//
// FUNCTION PROTOTYPES
//==========================================================================//

void encoder_init();
int32_t encoder_read();
void encoder_clear_count();
int8_t cmd_callback(CmdMsg* cmd_msg);
void sensors_callback(uint32_t dt, Sensors* sensors);
int32_t servo_pwm_from_cmd(float cmd);
int32_t drive_pwm_from_cmd(float cmd);
void set_drive_pwm(int32_t pwm);
void reset_integral_actions();
void controller(uint32_t dt_ms);
int8_t handshake();

//==========================================================================//
// ENTRY POINTS (MAIN)
//==========================================================================//

void setup() {
  uint32_t start_time = millis();

  Serial.begin(BAUD_RATE);
  delay(10);

  steeringServo.attach(SERVO_PWM_PIN);
  pinMode(DRIVE_DIR_PIN, OUTPUT);
  pinMode(DRIVE_PWM_PIN, OUTPUT);

  encoder_init();
  encoder_clear_count();

  steeringServo.write(PWM_MAX_SERVO);
  set_drive_pwm(0);

#ifdef IMU
  assert(imu.begin() == 1);
  assert(imu.setAccelRange(MPU9250::ACCEL_RANGE_2G) == 1);
  assert(imu.setGyroRange(MPU9250::GYRO_RANGE_250DPS) == 1);
  assert(imu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ) == 1);
  assert(imu.setSrd(9) == 1);  //100 Hz update rate
#endif

  while (millis() - start_time < 3000) { ;; }
  steeringServo.write(PWM_ZERO_SERVO);

  handshake();
  sensors_callback(1, &payload_);
}

void loop() {
  static uint32_t time_last_high, time_last_low = (uint32_t)0;

  static uint32_t start_time = millis();

  if (start_time - time_last_com > MAX_COM_DELAY_MS) {
    drive_ref = 0.0f;
    control_mode = -1;
  }

  uint32_t dt = start_time - time_last_low;
  if (dt > LOW_LEVEL_CYCLE_MS) {
    controller(dt);

    time_last_low = start_time;
  }

  if (dt > SERIAL_COM_CYCLE_MS) {
    assert(cmd_callback(&cmd_msg_) == SERIAL_OK);
    sensors_callback(millis() - time_last_high, &payload_);

    encoder_last = encoder;
    time_last_high = start_time;
  }
}

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
  digitalWrite(SLAVE_SELECT_ENCODER_PIN, LOW);   // Begin SPI communication
  SPI.transfer(0x88);                            // Write to MDR0
  SPI.transfer(0x03);                            // Configure to 4-byte mode
  digitalWrite(SLAVE_SELECT_ENCODER_PIN, HIGH);  // Terminate SPI communication
}

int32_t encoder_read() {
  // Initialize temporary variables for SPI read
  uint32_t count_1, count_2, count_3, count_4 = (uint32_t)0;
  uint32_t count = (uint32_t)0;

  // Read encoder
  digitalWrite(SLAVE_SELECT_ENCODER_PIN, LOW);  // Begin SPI conversation
  SPI.transfer(0x60);                           // Request count
  count_1 = SPI.transfer(0x00);                 // Read highest order byte
  count_2 = SPI.transfer(0x00);
  count_3 = SPI.transfer(0x00);
  count_4 = SPI.transfer(0x00);                  // Read lowest order byte
  digitalWrite(SLAVE_SELECT_ENCODER_PIN, HIGH);  // Terminate SPI conversationt

  // Calculate encoder count
  count = (count_1 << 8) + count_2;
  count = (count << 8) + count_3;
  count = (count << 8) + count_4;

  // Equivalent: count = (count_1 << 24) + (count_2 << 16) + (count_3 << 8) + count_4;

  return count;
}

void encoder_clear_count() {
  // Set encoder's data register to 0
  digitalWrite(SLAVE_SELECT_ENCODER_PIN, LOW);  // Begin SPI communication
  SPI.transfer(0x98);                           // Write to DTR
  SPI.transfer(0x00);                           // Highest order byte
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);                            // lowest order byte
  digitalWrite(SLAVE_SELECT_ENCODER_PIN, HIGH);  // Terminate SPI communication

  delayMicroseconds(100);  // Provides some breathing room between SPI communications

  // Set encoder's current data register to center
  digitalWrite(SLAVE_SELECT_ENCODER_PIN, LOW);  // Begin SPI communication
  SPI.transfer(0xE0);
  digitalWrite(SLAVE_SELECT_ENCODER_PIN, HIGH);  // Terminate SPI communication
}

int8_t cmd_callback(CmdMsg* cmd_msg) {
  uint32_t start_time = millis();

  // Await content on serial bus
  while (Serial.available() < 1) {
    if (millis() - start_time > TIMEOUT_MS) {
      return SERIAL_TIMEOUT_ERROR;
    }
  }

  // Read until start byte (discard incomplete and odometry payloads)
  while (Serial.read() != CMD_MSG_START_BYTE) {
    if (millis() - start_time > TIMEOUT_MS) {
      return SERIAL_TIMEOUT_ERROR;
    }
  }

  // Read the payload
  uint8_t buffer[CMD_MSG_PACKET_SIZE];
  size_t bytes_read = 0;

  while (bytes_read < CMD_MSG_PACKET_SIZE) {
    if (Serial.available()) {
      buffer[bytes_read] = Serial.read();
      ++bytes_read;
    }

    if (millis() - start_time > TIMEOUT_MS) {
      return SERIAL_TIMEOUT_ERROR;
    }
  }

  // Read end byte
  while (Serial.available() < 1) {
    if (millis() - start_time > TIMEOUT_MS) {
      return SERIAL_TIMEOUT_ERROR;
    }
    if (Serial.read() != CMD_MSG_END_BYTE) {
      return SERIAL_IO_ERROR;
    }
  }

  // Parse the message
  memcpy(&cmd_msg->control_mode, &buffer[0], sizeof(int8_t));
  memcpy(&cmd_msg->drive_ref, &buffer[1], sizeof(float));
  memcpy(&cmd_msg->servo_ref, &buffer[5], sizeof(float));

  // Send the acknowledgment
  // Serial.write(ACK_BYTE);

  time_last_com = millis();

  // Update controller variables
  drive_ref = cmd_msg->drive_ref;
  servo_ref = cmd_msg->servo_ref;
  control_mode = cmd_msg->control_mode;

  return SERIAL_OK;
}

void sensors_callback(uint32_t dt_ms, Sensors* sensors) {
  // Update I/O payload
  sensors->pos = pos;
  sensors->vel = vel_filtered;
  sensors->drive_ref = drive_ref;
  sensors->drive_cmd = drive_cmd;
  sensors->drive_pwm = drive_pwm;
  sensors->encoder = encoder;
  sensors->servo_ref = servo_ref;
  sensors->control_mode = control_mode;
  sensors->dt_ms = dt_ms;
  sensors->delta_distance_m = (float)(encoder - encoder_last) * TICK2METER;

#ifdef IMU
  imu.readSensor();
  sensors->accel_x_mss = imu.getAccelX_mss();
  sensors->accel_y_mss = imu.getAccelY_mss();
  sensors->accel_z_mss = imu.getAccelZ_mss();
  sensors->gyro_x_rads = imu.getGyroX_rads();
  sensors->gyro_y_rads = imu.getGyroY_rads();
  sensors->gyro_z_rads = imu.getGyroZ_rads();
  sensors->mag_x_uT = imu.getMagX_uT();
  sensors->mag_y_uT = imu.getMagY_uT();
  sensors->mag_z_uT = imu.getMagZ_uT();
#endif

  const size_t data_size = sizeof(Sensors);

  Serial.write(ODOMETRY_START_BYTE);

  // Send the payload
  const uint8_t* byte_ptr = (const uint8_t*)sensors;
  for (size_t i = 0; i < data_size; ++i) {
    Serial.write(byte_ptr[i]);
  }

  Serial.write(ODOMETRY_END_BYTE);

  // Ensure all data is transmitted
  Serial.flush();

  // Wait for acknowledgment with timeout
  // uint32_t ack_start_time = millis();
  // while (Serial.available() < 1) {
  //   if (millis() - ack_start_time > TIMEOUT_MS) {
  //     return SERIAL_TIMEOUT_ERROR;
  //   }
  // }

  // if (Serial.read() != ACK_BYTE) {
  //   return SERIAL_IO_ERROR;
  // }

  time_last_com = millis();

  // return SERIAL_OK;
}

#define clamp(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

int32_t servo_pwm_from_cmd(float cmd) {
  static float pwm_d = cmd * RAD2PWM + (float)(PWM_ZERO_SERVO);  // Scale and offset
  static int32_t pwm = (int32_t)ceilf(pwm_d);                    // Round and cast

  return clamp(pwm, PWM_MIN_SERVO, PWM_MAX_SERVO);
}

int32_t drive_pwm_from_cmd(float cmd) {
  static int32_t pwm = (int32_t)(cmd / MAX_BATTERY_VOLTAGE * ceilf((float)PWM_MAX_DRIVE));

  return clamp(pwm, PWM_MIN_DRIVE, PWM_MAX_DRIVE);
}

void set_drive_pwm(int32_t pwm) {
  if (pwm == 0) {
    digitalWrite(DRIVE_PWM_PIN, LOW);
    drive_standby = true;
  } else {
    if (drive_standby) {
      digitalWrite(DRIVE_PWM_PIN, HIGH);
      delayMicroseconds(DRIVE_WAKEUP_TIME_US);
      drive_standby = false;
    }

    // PWM direction
    if (pwm < 0) {
      digitalWrite(DRIVE_DIR_PIN, HIGH);
    } else {
      digitalWrite(DRIVE_DIR_PIN, LOW);
    }

    // Registery-based PWM Duty-Cycle adjustement

    //Fast PWM, 9-bit, prescaler divider = 1
    TCCR4A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM41);
    TCCR4B = _BV(CS20) | _BV(WGM42);

    OCR4A = abs(pwm) - 1;  //set the duty cycle of pin 6
  }
}

void reset_integral_actions() {
  pos_error_integral = 0.0f;
  vel_error_integral = 0.0f;
}

void controller(uint32_t dt_ms) {
  // Memory variables
  static float vel_last = 0.0f;

  // Servo open-loop
  servo_pwm = servo_pwm_from_cmd(servo_ref);
  steeringServo.write(servo_pwm);

  // Retrieve current encoder counter
  encoder = encoder_read();

  // Position computation
  pos = (float)encoder * TICK2METER;

  // Velocity computation
  // TODO: COMPLÉTER LA DÉRIVÉE FILTRÉE
  float vel_raw = (encoder - encoder_last) * TICK2METER / dt_ms * 1000;
  static const float alpha = 0.0f;  // TODO: TROUVER LE FACTEUR DU FILTRE
  vel_filtered = vel_raw;           // TODO: FILTRE

  // Propulsion controllers
  /*
   * === Low-Level Control Modes ===
   * | Value |     Description      | Units |
   * |-------|----------------------|-------|
   * |  -1   | Full Stop (Lost COM) | NULL  |
   * |   0   | Full Stop (Disabled) | NULL  |
   * |   1   | Full Open-Loop       | V     |
   * |   2   | Closed-Loop Velocity | m/s   |
   * |   3   | Closed-Loop Position | m     |
   * |   4   | Reset Encoder        | NULL  |
   */
  if (control_mode == 0 || control_mode == -1) {
    drive_pwm = PWM_ZERO_DRIVE;

    reset_integral_actions();
  } else if (control_mode == 1) {
    drive_cmd = drive_ref;
    drive_pwm = drive_pwm_from_cmd(drive_cmd);

    reset_integral_actions();
  } else if (control_mode == 2) {
    // TODO: COMPLÉTER LE CONTRÔLEUR SUIVANT
    float vel_ref = drive_ref;
    float vel_error = vel_ref - vel_filtered;
    vel_error_integral = 0.0f;       // TODO
    drive_cmd = VEL_KP * vel_error;  // TODO: Ajouter composante intégrale et/ou différentielle

    drive_pwm = drive_pwm_from_cmd(drive_cmd);
  } else if (control_mode == 3) {
    // TODO: COMPLÉTER LE CONTRÔLEUR SUIVANT
    float pos_ref = drive_ref;
    float pos_error = pos - pos_ref;
    float pos_error_differential = 0.0f;  // TODO
    pos_error_integral = 0.0f;            // TODO

    // Anti wind-up (Saturation)
    if (pos_error_integral > POS_ERROR_INTEGRAL_SAT) {
      pos_error_integral = POS_ERROR_INTEGRAL_SAT;
    }

    drive_cmd = POS_KP * pos_error;  // TODO
    drive_pwm = drive_pwm_from_cmd(drive_cmd);
  } else if (control_mode == 4) {
    encoder_clear_count();
    reset_integral_actions();
  } else {
    return;  // (Racecar staff) TODO: Add 'asserts' / error handling for controller
  }

  // H-Bridge PWM update
  set_drive_pwm(drive_pwm);

  // Update memory variables
  encoder_last = encoder;
  vel_last = vel_filtered;
}

// Serial handshake between Raspberry Pi and Arduino before entering main loop
int8_t handshake() {
  uint32_t start_time = millis();
  uint8_t count_ = 0;

  while (Serial.available() < 1) {
    uint32_t elapsed_ms = millis() - start_time;
    if (elapsed_ms > 250) {
      // Do a "dance" with the steering while waiting
      if (count_ == 0) { steeringServo.write(PWM_ZERO_SERVO); }
      else if (count_ == 1) { steeringServo.write(PWM_MIN_SERVO); }
      else if (count_ == 2) { steeringServo.write(PWM_ZERO_SERVO); }
      else if (count_ == 3) {
        steeringServo.write(PWM_MAX_SERVO);
        count_ = 0;
      }
      ++count_;
      start_time = millis();
    }
  }

  // Stop the dance
  steeringServo.write(PWM_ZERO_SERVO);

  if (Serial.read() != INIT_BYTE) {
    return SERIAL_SYNC_ERROR;
  }

  Serial.write(INIT_ACK_BYTE);

  while (Serial.available() < 1) {
    if (millis() - start_time > TIMEOUT_MS) {
      return SERIAL_TIMEOUT_ERROR;
    }
  }

  if (Serial.read != INIT_END_BYTE) {
    return SERIAL_IO_ERROR;
  }
}