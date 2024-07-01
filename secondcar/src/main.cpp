#include <asm-generic/errno-base.h>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <libevdev-1.0/libevdev/libevdev.h>
#include <linux/i2c-dev.h>
#include <linux/input.h>
#include <pigpio.h>
#include <pthread.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <vector>

const char *I2C_DEV = "/dev/i2c-1";
const int ADDR = 0x18;
const int MOTOR_PWM1 = 4;
const int MOTOR_PWM2 = 5;
const int MOTOR_DIR1 = 6;
const int MOTOR_DIR2 = 7;

class MovementUnit {
public:
  int leftX;     // 0-255
  int direction; // 0 (forward), 1 (reverse)
  int speed;     // 0-255

  MovementUnit(int _leftX, int _leftY, int _direction, int _speed)
      : leftX(_leftX), direction(_direction), speed(_speed) {}
};

// Writes to an i2c register.
int writeReg(int fd, int cmd, int value) {

  unsigned char buffer[3];
  buffer[0] = cmd;
  buffer[1] = value >> 8;
  buffer[2] = value & 0xFF;

  if (write(fd, buffer, 3) != 3) {
    printf("Failed to write.\n");
    return -1;
  }

  return 0;
}

// Kills all motor, resets their direction to forward.
void killMotors(int fd) {
  writeReg(fd, MOTOR_PWM1, 0);
  writeReg(fd, MOTOR_PWM2, 0);
  writeReg(fd, MOTOR_DIR1, 0);
  writeReg(fd, MOTOR_DIR2, 0);
}

// Scales input value against PWM max (1000)
int scalePWM(int pwmVal) { return (pwmVal * 1000) / 255; }

// Drives the car.
void driveCar(int fd, MovementUnit *mvUnit) {
  writeReg(fd, MOTOR_PWM1, scalePWM(mvUnit->speed));
  writeReg(fd, MOTOR_PWM2, scalePWM(mvUnit->speed));
  writeReg(fd, MOTOR_DIR1, mvUnit->direction);
  writeReg(fd, MOTOR_DIR2, mvUnit->direction);
}

// *****
// For interacting with a ps5 controller
void handleControllerInput(int i2cFd, MovementUnit *mvUnit) {
  int fd;
  int rc;
  struct libevdev *dev;

  fd = open("/dev/input/event4", O_RDONLY | O_NONBLOCK);
  if (fd < 0) {
    printf("Error getting controller event.");
  }

  rc = libevdev_new_from_fd(fd, &dev);
  if (fd < 0) {
    fprintf(stderr, "error: %d %s\n", -rc, strerror(-rc));
  }

  struct input_event ev;

  while (true) {
    while (libevdev_next_event(dev, LIBEVDEV_READ_FLAG_NORMAL, &ev) ==
           LIBEVDEV_READ_STATUS_SUCCESS) {

      std::string type = libevdev_event_type_get_name(ev.type);
      std::string code = libevdev_event_code_get_name(ev.type, ev.code);

      if (type != "EV_SYN") {

        if (code == "ABS_RZ") {
          mvUnit->direction = 0;
          mvUnit->speed = ev.value;
        }

        else if (code == "ABS_Z") {
          mvUnit->direction = 1;
          mvUnit->speed = ev.value;
        }

        else if (code == "ABS_X") {
        }

        else if (code == "ABS_Y") {
        }

        driveCar(i2cFd, mvUnit);
      }
    }
  }

  libevdev_free(dev);
  close(fd);
}

// Main.
int main() {
  int fd = open(I2C_DEV, O_RDWR);
  if (fd < 0) {
    printf("Could not open I2C device.\n");
    return -1;
  }

  if (ioctl(fd, I2C_SLAVE, ADDR) < 0) {
    printf("Could not set I2C device address\n");
    return -1;
  }
  MovementUnit mvUnit(255 / 2, 255 / 2, 0, 0);

  handleControllerInput(fd, &mvUnit);

  return 0;
}
