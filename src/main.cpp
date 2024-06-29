#include <iostream>
#include <pigpio.h>
#include <pthread.h>
#include <stdio.h>
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
void staticDrive(bool direction) {

  int pin = pinFromDirection(direction);

  for (auto &motor : motors) {
    gpioPWM(motor[pin], 255);
  }

  return;
}

// *****
// Turn right forward (true) or backward (false) depending on 'direction' param.
// Activates on w + d
void staticRightTurn(
    bool direction) { // direction = true for forward, false for backward.

  int pin = pinFromDirection(direction);
}

// *****
// Main.
int main() {

  std::cout << "Using pigpio version " << gpioVersion() << std::endl;
  std::cout << "Running on " << gpioHardwareRevision() << std::endl;

  if (setup() < 0) {
    cout << "Failed at setup.";
    return 0;
  }

  // testMotor(motor1, 255, 1000, 0);
  // staticDrive(true);

  kill();
  gpioTerminate();

  return 0;
}
