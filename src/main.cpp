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

// ** Basic Helpers **
bool isInRange(int val, int low, int high) { return low < val && val < high; }

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
/*
 Parameters: Both of the motors 'pins', the desired 'speed' (0-255), the time
 to it for in 'milliseconds', the 'direction' 1 = fwd, 0 = bckwd
*/
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

// Data structure for a unit of trigger press (either LT or RT).
struct TriggerUnit {
  bool &direction; // true = forward, false = backward
  int &speed;      // 0-255
};

// Data structure for a unit of movement (includes left joystick movement + the
// current TriggerUnit).
struct MovementUnit {
  int &leftX; // 0-255
  TriggerUnit *triggerUnit;
};

// *****
// Drive forward (true) or backward (false) depending on 'direction' param.
void basicDrive(bool direction, int speed) {

  for (auto &motor : motors) {
    gpioPWM(motor[direction], speed);
  }

  return;
}

// *****
// Used to steer the car.
void complexDrive(bool direction, int speed, int leftX) {

  if (leftX < 255 / 2) {
    gpioPWM(motor1[direction], 0);
    gpioPWM(motor1[direction], speed);
    gpioPWM(motor2[direction], speed);
    gpioPWM(motor2[direction], 0);
    gpioPWM(motor3[direction], speed);
    gpioPWM(motor3[direction], 0);
    gpioPWM(motor4[direction], 0);
    gpioPWM(motor4[direction], speed);
  } else {
    gpioPWM(motor1[direction], speed);
    gpioPWM(motor1[direction], 0);
    gpioPWM(motor2[direction], 0);
    gpioPWM(motor2[direction], speed);
    gpioPWM(motor3[direction], 0);
    gpioPWM(motor3[direction], speed);
    gpioPWM(motor4[direction], speed);
    gpioPWM(motor4[direction], 0);
  }
}

// ******
// The main function for directing the handling of the car based on
// controller input.
void handleMovement(MovementUnit *&unit) {
  basicDrive(unit->triggerUnit->direction, unit->triggerUnit->speed);
  // complexDrive(direction, speed, leftX);
}

// *****
// For interacting with a ps5 controller
int fd;
int rc;
struct libevdev *dev;
// *****
void handleControllerInput() {
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

  // Initialize drive values;
  MovementUnit *movementUnit;
  movementUnit->leftX = 255 / 2;
  movementUnit->triggerUnit->direction = true;
  movementUnit->triggerUnit->speed = 0;
  cout << "here";

  while (true) {
    while (libevdev_next_event(dev, LIBEVDEV_READ_FLAG_NORMAL, &ev) ==
           LIBEVDEV_READ_STATUS_SUCCESS) {

      string type = libevdev_event_type_get_name(ev.type);
      string code = libevdev_event_code_get_name(ev.type, ev.code);

      if (type != "EV_SYN") {

        if (code == "ABS_RZ") {
          movementUnit->triggerUnit->direction = true;
          movementUnit->triggerUnit->speed = ev.value;
        }

        else if (code == "ABS_Z") {
          movementUnit->triggerUnit->direction = false;
          movementUnit->triggerUnit->speed = ev.value;
        }

        else if (code == "ABS_X") {
          movementUnit->leftX = ev.value;
        }

        handleMovement(movementUnit);
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

  handleControllerInput();

  kill();

  gpioTerminate();
  return 0;
}
