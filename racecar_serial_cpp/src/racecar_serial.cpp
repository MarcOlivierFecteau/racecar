/*
 * Racecar - Serial Communication
 */

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/select.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "racecar_custom_interfaces/msg/sensors.hpp"
using std::placeholders::_1;

//==========================================================================//
// CONTROLLER CONSTANTS
//==========================================================================//

// I/O Data Structures
typedef struct
{
  std::int8_t control_mode;
  float drive_ref;
  float servo_ref;
} CmdMsg;

typedef struct
{
  float pos;
  float vel;
  float drive_ref;
  float drive_cmd;
  std::int32_t drive_pwm;
  std::int32_t encoder;
  float servo_ref;
  std::int8_t control_mode;
  std::uint32_t dt_ms;
  float delta_distance_m;
  float accelX_mss;
  float accelY_mss;
  float accelZ_mss;
  float gyroX_rads;
  float gyroY_rads;
  float gyroZ_rads;
  float magX_uT;
  float magY_uT;
  float magZ_uT;
} Sensors;

// Protocol Definitions
static const std::uint16_t TIMEOUT_MS = (std::uint16_t)1000;
static const std::uint8_t INIT_BYTE = 0x11;
static const std::uint8_t INIT_ACK_BYTE = 0x1A;
static const std::uint8_t INIT_END_BYTE = 0x1E;
static const std::uint8_t CMD_MSG_START_BYTE = 0xCA;
static const std::uint8_t CMD_MSG_END_BYTE = 0xCF;
static const ssize_t CMD_MSG_PACKET_SIZE = 2 + sizeof(std::int8_t) + sizeof(float) * 2;
static const std::uint8_t ODOMETRY_START_BYTE = 0x0A;
static const std::uint8_t ODOMETRY_END_BYTE = 0x0F;
static const ssize_t ODOMETRY_SIZE = sizeof(float) * 15 + sizeof(std::int32_t) * 2 + sizeof(std::uint32_t) + sizeof(std::int8_t);

// Error Codes
#define SERIAL_OK 0
#define SERIAL_SYNC_ERROR -1
#define SERIAL_IO_ERROR -2
#define SERIAL_TIMEOUT_ERROR -3

static const std::uint32_t BAUD_RATE = (std::uint32_t)9600;

//==========================================================================//
// FUNCTION PROTOTYPES
//==========================================================================//

int configureSerialPort(int fd);
void handshake();

//==========================================================================//
// ROS 2
//==========================================================================//

using namespace std::chrono_literals;

class ArduinoCommunicationNode : public rclcpp::Node
{
public:
  ArduinoCommunicationNode(int fd) : Node("arduino_communication")
  {
    this->fd_ = fd;
    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("prop_cmd", 10, std::bind(&ArduinoCommunicationNode::updateCmdMsg, this, _1));

    sensors_pub_ = this->create_publisher<racecar_custom_interfaces::msg::Sensors>("prop_sensors", 10);

    serial_com_timer_ = this->create_wall_timer(50ms, std::bind(&ArduinoCommunicationNode::serial_com_callback, this));
  }

  void updateCmdMsg(const geometry_msgs::msg::Twist::SharedPtr twist)
  {
    this->cmd_msg_.control_mode = (int8_t)(twist->linear.z);
    this->cmd_msg_.drive_ref = (float)(twist->linear.x);
    this->cmd_msg_.servo_ref = (float)(twist->angular.z);
  }

  std::int8_t sendCmdMsg()
  {
    std::uint8_t buffer[CMD_MSG_PACKET_SIZE] = {};
    ssize_t bytes_written = 0;

    // Prepare the packet
    buffer[0] = CMD_MSG_START_BYTE;
    memcpy(&buffer[1], &this->cmd_msg_.control_mode, sizeof(std::int8_t));
    memcpy(&buffer[1 + sizeof(std::int8_t)], &this->cmd_msg_.drive_ref, sizeof(float));
    memcpy(&buffer[1 + sizeof(std::int8_t) + sizeof(float)], &this->cmd_msg_.servo_ref, sizeof(float));
    buffer[CMD_MSG_PACKET_SIZE - 1] = CMD_MSG_END_BYTE;

    // Send the packet
    bytes_written = write(this->fd_, buffer, CMD_MSG_PACKET_SIZE);
    if (bytes_written != CMD_MSG_PACKET_SIZE)
    {
      return SERIAL_IO_ERROR;
    }

    return SERIAL_OK;
  }

  int readSerial(void *buffer, size_t length, std::uint16_t timeout_ms)
  {
    size_t bytes_read = 0;
    unsigned char *buf = (unsigned char *)buffer;

    struct timespec start_time, current_time;
    clock_gettime(CLOCK_MONOTONIC, &start_time);

    while (bytes_read < length)
    {
      ssize_t result = read(this->fd_, buf + bytes_read, length - bytes_read);
      if (result < 0)
      {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
          clock_gettime(CLOCK_MONOTONIC, &current_time);
          std::uint32_t elasped_ms = (current_time.tv_sec - start_time.tv_sec) * 1000;
          if (elasped_ms >= timeout_ms)
          {
            return SERIAL_TIMEOUT_ERROR;
          }
          continue;
        }
        return SERIAL_IO_ERROR;
      }

      bytes_read += result;
    }

    return bytes_read;
  }

  int receiveSensorsData(Sensors *sensors)
  {
    std::uint8_t _byte = 0;
    wchar_t sensors_buf[sizeof(float) * 15 +
                        sizeof(std::int32_t) * 2 +
                        sizeof(std::uint32_t) +
                        sizeof(std::int8_t)] = {};

    // Wait for start byte
    do
    {
      if (this->readSerial(&_byte, 1, TIMEOUT_MS) != 1)
      {
        return SERIAL_TIMEOUT_ERROR;
      }
    } while (_byte != ODOMETRY_START_BYTE);

    // Read payload
    if (this->readSerial(&sensors_buf, ODOMETRY_SIZE, TIMEOUT_MS) != ODOMETRY_SIZE)
    {
      return SERIAL_IO_ERROR;
    }

    // Read end byte
    if (this->readSerial(&_byte, 1, 10) != 1 || _byte != ODOMETRY_END_BYTE)
    {
      return SERIAL_IO_ERROR;
    }

    // Update data structure
    std::memcpy(&sensors->pos, &sensors_buf[0], sizeof(Sensors::pos));
    std::memcpy(&sensors->vel, &sensors_buf[4], sizeof(Sensors::vel));
    std::memcpy(&sensors->drive_ref, &sensors_buf[8], sizeof(Sensors::drive_ref));
    std::memcpy(&sensors->drive_cmd, &sensors_buf[12], sizeof(Sensors::drive_cmd));
    std::memcpy(&sensors->drive_pwm, &sensors_buf[16], sizeof(Sensors::drive_pwm));
    std::memcpy(&sensors->encoder, &sensors_buf[20], sizeof(Sensors::encoder));
    std::memcpy(&sensors->servo_ref, &sensors_buf[24], sizeof(Sensors::servo_ref));
    std::memcpy(&sensors->control_mode, &sensors_buf[28], sizeof(Sensors::control_mode));
    std::memcpy(&sensors->dt_ms, &sensors_buf[29], sizeof(Sensors::dt_ms));
    std::memcpy(&sensors->delta_distance_m, &sensors_buf[33], sizeof(Sensors::delta_distance_m));
    std::memcpy(&sensors->accelX_mss, &sensors_buf[37], sizeof(Sensors::accelX_mss));
    std::memcpy(&sensors->accelY_mss, &sensors_buf[41], sizeof(Sensors::accelY_mss));
    std::memcpy(&sensors->accelZ_mss, &sensors_buf[45], sizeof(Sensors::accelZ_mss));
    std::memcpy(&sensors->gyroX_rads, &sensors_buf[49], sizeof(Sensors::gyroX_rads));
    std::memcpy(&sensors->gyroY_rads, &sensors_buf[53], sizeof(Sensors::gyroY_rads));
    std::memcpy(&sensors->gyroZ_rads, &sensors_buf[57], sizeof(Sensors::gyroZ_rads));
    std::memcpy(&sensors->magX_uT, &sensors_buf[61], sizeof(Sensors::magX_uT));
    std::memcpy(&sensors->magY_uT, &sensors_buf[65], sizeof(Sensors::magY_uT));
    std::memcpy(&sensors->magZ_uT, &sensors_buf[69], sizeof(Sensors::magZ_uT));

    return SERIAL_OK;
  }

  void serial_com_callback()
  {
    // Clear the input and output buffers
    if (tcflush(this->fd_, TCIOFLUSH) == -1)
    {
      std::cerr << "Error clearing serial buffer: " << strerror(errno) << std::endl;
    }

    assert(this->sendCmdMsg() == SERIAL_OK);
    assert(this->receiveSensorsData(&this->sensors_) == SERIAL_OK);

    // Directly publishing private member doesn't compile
    auto message = racecar_custom_interfaces::msg::Sensors();
    message.pos = this->sensors_.pos;
    message.vel = this->sensors_.vel;
    message.drive_ref = this->sensors_.drive_ref;
    message.drive_cmd = this->sensors_.drive_cmd;
    message.drive_pwm = this->sensors_.drive_pwm;
    message.encoder = this->sensors_.encoder;
    message.servo_ref = this->sensors_.servo_ref;
    message.control_mode = this->sensors_.control_mode;
    message.dt_ms = this->sensors_.dt_ms;
    message.delta_distance_m = this->sensors_.delta_distance_m;
    message.accel_x_mss = this->sensors_.accelX_mss;
    message.accel_y_mss = this->sensors_.accelY_mss;
    message.accel_z_mss = this->sensors_.accelZ_mss;
    message.gyro_x_rads = this->sensors_.gyroX_rads;
    message.gyro_y_rads = this->sensors_.gyroY_rads;
    message.gyro_z_rads = this->sensors_.gyroZ_rads;
    message.mag_x_ut = this->sensors_.magX_uT;
    message.mag_y_ut = this->sensors_.magY_uT;
    message.mag_z_ut = this->sensors_.magZ_uT;
    this->sensors_pub_->publish(message);
  }

private:
  int fd_ = 0;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  CmdMsg cmd_msg_ = {.control_mode = (int8_t)0, .drive_ref = 0.0f, .servo_ref = 0.0f};
  rclcpp::TimerBase::SharedPtr serial_com_timer_;
  rclcpp::Publisher<racecar_custom_interfaces::msg::Sensors>::SharedPtr sensors_pub_;
  Sensors sensors_ =
  {
    .pos = 0.0f,
    .vel = 0.0f,
    .drive_ref = 0.0f,
    .drive_cmd = 0.0f,
    .drive_pwm = (std::int32_t)0,
    .encoder = (std::int32_t)0,
    .servo_ref = 0.0f,
    .control_mode = (std::int8_t)0,
    .dt_ms = (std::uint32_t)0,
    .delta_distance_m = 0.0f,
    .accelX_mss = 0.0f,
    .accelY_mss = 0.0f,
    .accelZ_mss = 0.0f,
    .gyroX_rads = 0.0f,
    .gyroY_rads = 0.0f,
    .gyroZ_rads = 0.0f,
    .magX_uT = 0.0f,
    .magY_uT = 0.0f,
    .magZ_uT = 0.0f
  };
};

//==========================================================================//
// ENTRY POINT (MAIN)
//==========================================================================//

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  int serial_fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd < 0)
  {
    perror("Error opening serial port\n");
    return 1;
  }

  // Configure serial port
  if (configureSerialPort(serial_fd) != 0)
  {
    perror("Error configuring serial port\n");
    close(serial_fd);
    return 1;
  }

  rclcpp::spin(std::make_shared<ArduinoCommunicationNode>(serial_fd));
  rclcpp::shutdown();

  close(serial_fd);

  return 0;
}

//==========================================================================//
// FUNCTIONS
//==========================================================================//

int configureSerialPort(int fd)
{
  struct termios tty;

  if (tcgetattr(fd, &tty) != 0)
  {
    return -1;
  }

  // Set baud rate
  cfsetispeed(&tty, BAUD_RATE);
  cfsetospeed(&tty, BAUD_RATE);

  // 8-bit characters
  tty.c_cflag |= CS8;
  // Disable parity
  tty.c_cflag &= ~PARENB;
  // One stop bit
  tty.c_cflag &= ~CSTOPB;
  // No hardware flow control
  tty.c_cflag &= ~CRTSCTS;
  // Enable receiver
  tty.c_cflag |= CREAD | CLOCAL;

  // Raw input mode
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  // Raw output mode
  tty.c_oflag &= ~OPOST;
  // No input processing
  tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);

  // Set read timeout
  tty.c_cc[VTIME] = TIMEOUT_MS / 10; // deciseconds
  tty.c_cc[VMIN] = 0;

  if (tcsetattr(fd, TCSANOW, &tty) != 0)
  {
    return -1;
  }

  return 0;
}