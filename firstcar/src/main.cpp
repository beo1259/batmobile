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

const int RIGHT_FRONT_FORWARD = 23;
const int RIGHT_FRONT_REVERSE = 24;
const int LEFT_FRONT_FORWARD = 18;
const int LEFT_FRONT_REVERSE = 25;
const int RIGHT_BACK_FORWARD = 17;
const int RIGHT_BACK_REVERSE = 4;
const int LEFT_BACK_FORWARD = 22;
const int LEFT_BACK_REVERSE = 27;

// The 2d array of each motor and it's two GPIO pins. Made a struct for
// readability.
struct MotorGroup {
  int motors[4][2];
};

// Data structure for a unit of trigger press (either LT or RT).
class TriggerUnit {
public:
  bool direction; // true = forward, false = reverse
  int speed;      // 0-255

  TriggerUnit(bool direction, int speed)
      : direction(direction), speed(speed) {};
};

// Data structure for a unit of movement (includes left joystick movement + the
// current TriggerUnit).
class MovementUnit {
public:
  int leftX; // 0-255
  int leftY; // 0-255
  TriggerUnit *triggerUnit;

  MovementUnit(int leftX, int leftY, TriggerUnit *triggerUnit)
      : leftX(leftX), leftY(leftY), triggerUnit(triggerUnit) {};
};

// *****
// Helper #1
bool bothInRange(int val1, int val2, int low, int high) {
  return ((low <= val1) && (low <= val2)) && ((val1 <= high) && (val2 <= high));
}

// *****
// Sets each motor to output mode.
int setup(MotorGroup *motorGroup) {
  if (gpioInitialise() < 0) {
    cerr << "pigpio initialization failed.";
    return -1;
  }

  for (auto &motor : motorGroup->motors) {
    gpioSetMode(motor[0], PI_OUTPUT);
    gpioSetMode(motor[1], PI_OUTPUT);
  }

  return 0;
}

// ******
// Kill all motors.
void kill(MotorGroup *motorGroup) {
  for (auto &motor : motorGroup->motors) {
    gpioPWM(motor[0], 0);
    gpioPWM(motor[1], 0);
  }
}

// ******
// Kill a single motor's pin.
void killPin(int pin) { gpioPWM(pin, 0); }

// *****
// Tests a single motor for 2 seconds.
void testMotor(int pin) {
  gpioPWM(pin, 255);

  usleep(2000 * 1000);

  killPin(pin);
}

// *****
// Drive forward (true) or reverse (false) depending on 'direction' param.
void driveStraight(MotorGroup *motorGroup, TriggerUnit *triggerUnit) {
  for (auto &motor : motorGroup->motors) {
    gpioPWM(motor[!triggerUnit->direction], triggerUnit->speed);
  }
}

// *****
// Used to steer the car.
void complexDrive(MotorGroup *motorGroup, MovementUnit *unit) {
  int &speed = unit->triggerUnit->speed;
  int &X = unit->leftX;

  if (X < 255 / 2) { // Left turn.
    gpioPWM(RIGHT_BACK_FORWARD, speed);
    gpioPWM(RIGHT_BACK_REVERSE, 0);
    gpioPWM(RIGHT_FRONT_FORWARD, speed);
    gpioPWM(RIGHT_FRONT_REVERSE, 0);
    gpioPWM(LEFT_BACK_REVERSE, speed);
    gpioPWM(LEFT_BACK_FORWARD, 0);
    gpioPWM(LEFT_FRONT_REVERSE, speed);
    gpioPWM(LEFT_FRONT_FORWARD, 0);
  } else {
    gpioPWM(RIGHT_BACK_FORWARD, 0);
    gpioPWM(RIGHT_BACK_REVERSE, speed);
    gpioPWM(RIGHT_FRONT_FORWARD, 0);
    gpioPWM(RIGHT_FRONT_REVERSE, speed);
    gpioPWM(LEFT_BACK_REVERSE, 0);
    gpioPWM(LEFT_BACK_FORWARD, speed);
    gpioPWM(LEFT_FRONT_REVERSE, 0);
    gpioPWM(LEFT_FRONT_FORWARD, speed);
  }
}

// ******
// The function for directing the handling of the car based on
// controller input.
void handleMovement(MotorGroup *motorGroup, MovementUnit *unit) {
  if (unit->triggerUnit->speed == 0) {
    kill(motorGroup);
    return;
  }

  if (bothInRange(unit->leftX, unit->leftY, 70, 200)) {
    kill(motorGroup);
    driveStraight(motorGroup, unit->triggerUnit);
  } else {
    complexDrive(motorGroup, unit);
  }
}

// *****
// For interacting with a ps5 controller
// *****
void handleControllerInput(MotorGroup *motorGroup) {
  int fd;
  int rc;
  struct libevdev *dev;

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
  TriggerUnit triggerUnit(true, 0);
  MovementUnit movementUnit(255 / 2, 255 / 2, &triggerUnit);

  while (true) {
    while (libevdev_next_event(dev, LIBEVDEV_READ_FLAG_NORMAL, &ev) ==
           LIBEVDEV_READ_STATUS_SUCCESS) {

      string type = libevdev_event_type_get_name(ev.type);
      string code = libevdev_event_code_get_name(ev.type, ev.code);

      if (type != "EV_SYN") {

        if (code == "ABS_RZ") {
          movementUnit.triggerUnit->direction = true;
          movementUnit.triggerUnit->speed = ev.value;
        }

        else if (code == "ABS_Z") {
          movementUnit.triggerUnit->direction = false;
          movementUnit.triggerUnit->speed = ev.value;
        }

        else if (code == "ABS_X") {
          movementUnit.leftX = ev.value;
        }

        else if (code == "ABS_Y") {
          movementUnit.leftY = ev.value;
        }

        handleMovement(motorGroup, &movementUnit);
      }
    }
  }

  libevdev_free(dev);
  close(fd);
}

// *****
// Main.
int main() {

  // Each inner array element are the two GPIO pins that control each motor.
  MotorGroup motors = {{{RIGHT_FRONT_FORWARD, RIGHT_FRONT_REVERSE},
                        {LEFT_FRONT_FORWARD, LEFT_FRONT_REVERSE},
                        {RIGHT_BACK_FORWARD, RIGHT_BACK_REVERSE},
                        {LEFT_BACK_FORWARD, LEFT_BACK_REVERSE}}};

  if (setup(&motors) < 0) {
    cout << "Failed at setup.";
    return 0;
  }

  // testMotor(RIGHT_FRONT_FORWARD);
  // return 0;

  kill(&motors);

  handleControllerInput(&motors);

  kill(&motors);

  gpioTerminate();
  return 0;
}
