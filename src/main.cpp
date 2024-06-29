#include <asm-generic/errno-base.h>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <libevdev-1.0/libevdev/libevdev.h>
#include <linux/input.h>
#include <pigpio.h>
#include <pthread.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <vector>

using namespace std;

// For each motor, [0] = IN pin (forward), [1] = OUT pin (backward).
vector<int> motor1 = {24, 23};
vector<int> motor2 = {25, 18};
vector<int> motor3 = {4, 17};
vector<int> motor4 = {27, 22};

// Store them all in a vector so we can easily iterate.
vector<vector<int>> motors = {motor1, motor2, motor3, motor4};

// *****
// Sets each motor to out mode, and confirms this by printing each pin's mode.
int setup() {
  if (gpioInitialise() < 0) {
    cerr << "pigpio initialization failed.";
    return -1;
  }

  for (auto &motor : motors) {
    gpioSetMode(motor[0], PI_OUTPUT);
    gpioSetMode(motor[1], PI_OUTPUT);
  }

  return 0;
}

// ******
// Test a single motor
// Parameters: Both of the motors 'pins', the desired 'speed' (0-255), the time
// to it for in 'milliseconds', the 'direction' 1 = fwd, 0 = bckwd
void testMotor(vector<int> pins, int speed, uint milliseconds, int direction) {

  if (direction == 0) {
    direction = pins[1];
  } else if (direction == 1) {
    direction = pins[0];
  } else {
    cerr << "Direction is 0 for forward or 1 for backward.";
  }

  int microseconds = (milliseconds * 1000.0);

  if (speed > 255) {
    speed = 255;
  }

  gpioPWM(direction, speed);

  usleep(microseconds);

  gpioPWM(direction, 0);

  return;
}

// ******
// Kill all motors.
void kill() {
  for (auto &motor : motors) {
    gpioPWM(motor[0], 0);
    gpioPWM(motor[1], 0);
  }
}

// *****
// Returns pin 0 for 'dir' == true, pin 1 for 'dir' == false.
// (This is slightly confusing because it's essentially reversing the bool, but
// I think it's more intuitive for 'dir' == true to mean forward, and 'dir'
// false mean backward.
auto pinFromDirection = [](bool dir) -> int { return dir ? 0 : 1; };

// *****
// Drive forward (true) or backward (false) depending on 'direction' param.
void drive(bool direction, int speed) {

  int pin = pinFromDirection(direction);

  for (auto &motor : motors) {
    gpioPWM(motor[pin], speed);
  }

  return;
}

// *****
int CURRENT_LJ_Y;
// *****
int GetLeftJoystickYVal() { return CURRENT_LJ_Y; }
void SetLeftJoystickYVal(int yVal) { CURRENT_LJ_Y = yVal; }

// *****
// Activates the drive function with the given directyion and speed (to be used
// with the RT and LT on the PS5 controller.)
auto triggerDrive = [](bool direction, int speed) -> void {
  speed > 0 ? drive(direction, speed) : kill();
};

// *****
// Drives either forward or backward depending on the trigger down press
// value/direction.
void driveCar(string code, int value) {
  // Right Trigger
  if (code == "ABS_RZ")
    triggerDrive(true, value);

  // Left Trigger
  if (code == "ABS_Z")
    triggerDrive(false, value);

  if (code == "ABS_Z" && code == "ABS_RZ") {
    cout << "cndasjkcvndsakjlvnfdskljvnfdjkslvnfdskjlvnfdskjlvnfdjkslvnfdskljvn"
            "fdsjkvnfdskjlvn";
  }
}

// TODO
void turnCar(int xVal) { int yVal = GetLeftJoystickYVal(); }

// *****
// For interacting with ps5 controller
int fd;
int rc;
struct libevdev *dev;
// *****
void ps5() {
  fd = open("/dev/input/event4", O_RDONLY | O_NONBLOCK);
  if (fd < 0) {
    fprintf(stderr, "error: %d %s\n", errno, strerror(errno));
  }

  rc = libevdev_new_from_fd(fd, &dev);
  if (fd < 0) {
    fprintf(stderr, "error: %d %s\n", -rc, strerror(-rc));
  }

  printf("Device: %s\n", libevdev_get_name(dev));

  struct input_event ev;

  while (true) {
    while (libevdev_next_event(dev, LIBEVDEV_READ_FLAG_NORMAL, &ev) ==
           LIBEVDEV_READ_STATUS_SUCCESS) {
      string type = libevdev_event_type_get_name(ev.type);
      string code = libevdev_event_code_get_name(ev.type, ev.code);
      int value = ev.value;

      if (type != "EV_SYN") {

        std::cout << "Type " << type << ", Code " << code << ", Value " << value
                  << std::endl;

        if (code == "ABS_Y") {
          SetLeftJoystickYVal(value);
        }
        if (code == "ABS_X") {
          turnCar(value);
        }
        driveCar(code, value);
      }
    }
  }

  libevdev_free(dev);
  close(fd);
}

// *****
// Main.
int main() {

  if (setup() < 0) {
    cout << "Failed at setup.";
    return 0;
  }

  kill();

  ps5();

  kill();

  gpioTerminate();
  return 0;
}
