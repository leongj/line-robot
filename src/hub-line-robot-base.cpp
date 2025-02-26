
#include <tuple>
#include <memory>
#include <string>
#include <stdexcept>
#include <stdio.h>

#include "../arduino_secrets.h"
#include <Arduino.h>

#include "hub-line-robot-base.h"

#include "libs/SeeedOLED.h"
#include "libs/picohttpparser.h"
#include "libs/ACROBOTIC_SSD1306.h"
#include "hub-line-robot-routes.h"


/***********************************
 * RobotState Class Implementation *
 ***********************************/
 
void noop() { /* */ }
void noopLong(long) { /* */ }

RobotState::RobotState() {
    this->drive_state = 0;
    this->btn_state = 1;
    this->ir1 = 0;
    this->ir2 = 0;
    this->ir3 = 0;
    this->ir4 = 0;
    this->ir1_hard_threshold = 1200;
    this->ir2_hard_threshold = 1200;
    this->ir3_hard_threshold = 1200;
    this->ir4_hard_threshold = 1200;
    this->ir1_soft_threshold = 600;
    this->ir2_soft_threshold = 600;
    this->ir3_soft_threshold = 600;
    this->ir4_soft_threshold = 600;
    this->ir1_raw = 0;
    this->ir2_raw = 0;
    this->ir3_raw = 0;
    this->ir4_raw = 0;
    this->poseX = 0;
    this->poseY = 0;
    this->poseZ = 0;
    this->idleStartFn = noop;
    this->idleEndFn = noop;
    this->primedStartFn = noop;
    this->primedEndFn = noop;
    this->drivingStartFn = noop;
    this->drivingEndFn = noop;
    this->btn_state_processed = false;
    this->btn_time_in_state = 0;
    this->btn_time_in_prev_state = 0;
    this->btn_prev_time_in_down = 0;
    this->btn_prev_time_in_up = 0;
    this->btnPressedFn = noop;
    this->btnLongPressedFn = noopLong;
    this->btnDoublePressedFn = noop;
}

void RobotState::setState(int state) {
    this->handleStateChange(state);
}

void RobotState::handleStateChange(int to_state) {
  // If the state is the same, do nothing
  if (to_state == this->drive_state) {
    return;
  }

  // Trigger end of state function
  if (this->drive_state == 0) {  // IDLE
    this->idleEndFn();        
  } else if (drive_state == 1) {  // PRIMED
    this->primedEndFn();
  } else if (drive_state == 2) {  // DRIVING
    this->drivingEndFn();
  }

  // Trigger start of state function
  if (to_state == 0) {            // IDLE
    this->idleStartFn();
  } else if (to_state == 1) {     // PRIMED
    this->primedStartFn();
  } else if (to_state == 2) {     // DRIVING
    this->drivingStartFn();
  }

  this->drive_state = to_state;
}

bool debouncedBtnState(int btnState) {
  static unsigned long lastDebounceTime = 0;
  static int lastBtnState = 1;
  const unsigned long debounceDelay = 50;
  
  if (btnState != lastBtnState) {
    lastDebounceTime = millis();
  }

  bool btnStateStable = false;
  if ((millis() - lastDebounceTime) > debounceDelay) {
    btnStateStable = true;
  }
  lastBtnState = btnState;
  return btnStateStable;
}
void RobotState::updateBtnState(int btnState) {
  bool btnStateStable = debouncedBtnState(btnState);
  if (!btnStateStable) {
    return;
  }

  bool btnStateChanged = (btnState != this->btn_state) && btnStateStable;
  if (!btnStateChanged && this->btn_state_processed) {
    return;
  }

  if (btnStateChanged) {
    this->btn_state = btnState;
    this->btn_state_processed = false;
    this->btn_time_in_prev_state = millis() - this->btn_time_in_state;
    this->btn_time_in_state = millis();
  }

  if (this->btn_time_in_state == 0) {
    return;
  }

  long state_duration = millis() - this->btn_time_in_state;
  if (btnState == 0) {  // If the button is pressed (We have a Pull up resistor + button is grounded, so LOW is pressed)
    this->btnDown(btnStateChanged, state_duration, this->btn_time_in_prev_state);
  } else {            // If the button is not pressed
    this->btnUp(btnStateChanged, state_duration, this->btn_time_in_prev_state);
  }
}

void RobotState::btnDown(bool enteringState, long duration, long timeInPrevState) {
  this->btn_prev_time_in_up = btn_time_in_prev_state;
  this->btn_state_processed = true;  
}

void RobotState::btnUp(bool enteringState, long duration, long timeInPrevState) {
  this->btn_prev_time_in_down = btn_time_in_prev_state;

  // Button was just released, let's consider whether this is a long press or not
  bool longPress = timeInPrevState > 1000;

  // LOGGER.println("Button Up: D: " + String(duration) + "ms, TPrevS: " + String(timeInPrevState) + "ms, PrevUpTime: " + String(this->btn_prev_time_in_up) + "ms, LP: " + String(longPress) + ", ES: " + String(enteringState));
  // There are three cases to consider: 
  // 1. Button was pressed and released quickly (less than 400ms) - this is a short press
  // 2. Button was pressed and released after 1000ms - this is a long press
  // 3. Button was pressed twice in quick succession (less than 800ms between presses) - this is a double press
  if (longPress) {
    // Button was pressed for a long time, so long press
    this->btnLongPressedFn(timeInPrevState);
    this->btn_state_processed = true;
  } else if (timeInPrevState < 600 && this->btn_prev_time_in_up <= 1000) {
    // Button was pressed twice in quick succession, so double press
    this->btnDoublePressedFn();
    this->btn_state_processed = true;
  } else if (duration > 200 && this->btn_prev_time_in_up > 1000) { // We give it some time to be sure the user is not going to press again (for a double  press)
    // Button was pressed and released quickly, and not pressed again, so single press
    this->btnPressedFn();
    this->btn_state_processed = true;
  }
}

void RobotState::setOnBtnPressed(std::function<void()> callback) { this->btnPressedFn = callback; }
void RobotState::setOnBtnLongPressed(std::function<void(long)> callback) { this->btnLongPressedFn = callback; }
void RobotState::setOnBtnDoublePressed(std::function<void()> callback) { this->btnDoublePressedFn = callback; }


void RobotState::setIdle() { this->handleStateChange(0); }
void RobotState::setPrimed() { this->handleStateChange(1); }
void RobotState::setDriving() { this->handleStateChange(2); }

bool RobotState::isIdle() { return this->drive_state == 0; }
bool RobotState::isPrimed() { return this->drive_state == 1; }
bool RobotState::isDriving() { return this->drive_state == 2; }
int RobotState::driveState() { return this->drive_state; }

void RobotState::setPose(float x, float y, float z) {
    this->poseX = x;
    this->poseY = y;
    this->poseZ = z;
}

void RobotState::setOnIdleStart(std::function<void()> callback) { this->idleStartFn = callback; }
void RobotState::setOnIdleEnd(std::function<void()> callback) { this->idleEndFn = callback; }
void RobotState::setOnPrimedStart(std::function<void()> callback) { this->primedStartFn = callback; }
void RobotState::setOnPrimedEnd(std::function<void()> callback) { this->primedEndFn = callback; }
void RobotState::setOnDrivingStart(std::function<void()> callback) { this->drivingStartFn = callback; }
void RobotState::setOnDrivingEnd(std::function<void()> callback) { this->drivingEndFn = callback; }



/***************************************
 * END RobotState Class Implementation *
 ***************************************/
 


/************************************
 * Robot Board Class Implementation *
 ************************************/

RobotBoard::~RobotBoard() {
    delete motor1;
    delete motor2;
    delete imu;
    delete adc;
}

void RobotBoard::setSpeedOnBothMotors(int speed) {
  if (this->motor1 == nullptr || this->motor2 == nullptr) {
    LOGGER.println("Error: Cannot set speed on the motors - one or both motors have not been initialized");
      return;
  }

  if (speed == 0) {
      this->motor1->off();
      this->motor2->off();
      this->motor1_speed = 0;
      this->motor2_speed = 0;                
  } else {
      speed = constrain(speed, -255, 255);
      this->motor1->on(speed);
      this->motor2->on(speed);
      this->motor1_speed = speed;
      this->motor2_speed = speed;                
  }
}

void RobotBoard::setMotorSpeedL(int speed) {
    if (this->motor1 == nullptr) {
      LOGGER.println("Error: Cannot set speed on motor1 (Left Motor) - it has not been initialized");
        return;
    }

    if (speed == 0) {
        this->motor1->off();
        this->motor1_speed = 0;
    } else {
        speed = constrain(speed, -255, 255);
        this->motor1->on(speed);
        this->motor1_speed = speed;
    }
}

void RobotBoard::setMotorSpeedR(int speed) {
  if (this->motor2 == nullptr) {
      LOGGER.println("Error: Cannot set speed on motor2 (Right Motor) - it has not been initialized");
      return;
  }

  if (speed == 0) {
      this-> motor2->off();
      this->motor2_speed = 0;
  } else {
      speed = constrain(speed, -255, 255);
      this->motor2->on(speed);
      this->motor2_speed = speed;
  }
}

int RobotBoard::motor1Speed() { return this->motor1_speed; }
int RobotBoard::motor2Speed() { return this->motor2_speed; }
bool RobotBoard::isButtonPressed() { return digitalRead(this->BOARD_BUTTON) == LOW; }
bool RobotBoard::isOLEDConnected() { return this->oled_type != OLED_TYPE_NONE; }

void RobotBoard::init_imu() {
  LOGGER.println("IMU: Initialising...");
  imu->settings.adcEnabled = 0;           // Nothing is connected to the ADC pins
  imu->settings.tempEnabled = 0;          // Temp likely not needed
  imu->settings.accelSampleRate = 50;     // Hz.  Can be: 0,1,10,25,50,100,200,400,1600,5000 Hz
  imu->settings.accelRange = 2;           // Max G force readable.  Can be: 2, 4, 8, 16
  imu->settings.xAccelEnabled = 1;
  imu->settings.yAccelEnabled = 1;
  imu->settings.zAccelEnabled = 1;
  imu->begin();
  LOGGER.println("IMU: Initialised + Ready!");
}

void RobotBoard::printOLEDHeader() {
  if (this->isOLEDConnected()) {
    if (this->oled_type == OLED_TYPE_GROVE) {
      SeeedOled.setTextXY(0, 0);              // Set the cursor to Xth Page, Yth Column  
      SeeedOled.putString(" Innovation Hub ");
      SeeedOled.setTextXY(1, 0);              // Set the cursor to Xth Page, Yth Column  
      SeeedOled.putString("   Line Robot   ");
    } else if (this->oled_type == OLED_TYPE_SSD1306) {
      oled.setTextXY(0,0);                    // Set cursor position, start of line 0
      oled.putString("     Innovation Hub   ");
      oled.setTextXY(1,0);  
      oled.putString("       Line Robot     ");
    }

    // If the wifi is connected, print the IP address
    if (WiFi.status() == WL_CONNECTED) {
      this->setOLEDLine(-1, wifi_address());
    }
  }
}

int RobotBoard::init_oled() {
  LOGGER.println("OLED: Initialising");
  if (this->oled_type == OLED_TYPE_GROVE) {
    SeeedOled.init();                             // Initialze SEEED OLED display
    SeeedOled.clearDisplay();                     // Clear the screen and set start position to top left corner
    SeeedOled.setNormalDisplay();                 // Set display to normal mode (i.e non-inverse mode)
    SeeedOled.setPageMode();                      // Set addressing mode to Page Mode
    this->clearOLED();
    this->printOLEDHeader();
    LOGGER.println("OLED: Initialised + Ready!");
    return 3;   // The line number below the end of the header region
  } else if (this->oled_type == OLED_TYPE_SSD1306) {
    // 64x128
    oled.init();                      // Initialze SSD1306 OLED display
    oled.setNormalDisplay();
    oled.setPageMode();
    oled.clearDisplay();              // Clear screen
    oled.setFont(font5x7);            // Set font type (default 8x8)
    oled.setTextXY(0,0);              // Set cursor position, start of line 0
    this->clearOLED();
    this->printOLEDHeader();
    LOGGER.println("OLED: Initialised + Ready!");
    return 3;   // The line number below the end of the header region
  } else {
    LOGGER.println("Unsupported OLED Type: " + String(this->oled_type));
    return -1;
  }
}


String padLeft(String str, const size_t num, const char padChar = ' ') {
    if (num > str.length()) {
        str = String(std::string(num - str.length(), padChar).c_str()) + str;
    }
    if (str.length() > num) {
        return str.substring(0, num);
    }
    return str;
}


void RobotBoard::readAccelerometerData() {
  if (imu == nullptr) {
    LOGGER.println("Error: Cannot read accelerometer data - IMU has not been initialized");
    return;
  }
  
  // // Read state of the IMU
  float accelX = imu->readFloatAccelX();
  float accelY = imu->readFloatAccelY();
  float accelZ = imu->readFloatAccelZ();
  this->state->setPose(accelX, accelY, accelZ);

  if (this->state->isIdle()) {
    String x = padLeft(String((int)(accelX * 100)), 3, ' ');
    String y = padLeft(String((int)(accelY * 100)), 3, ' ');
    String z = padLeft(String((int)(accelZ * 100)), 3, ' ');
    this->setOLEDLine(2, "Pose: " + x + ' ' + y + ' ' + z, false);
  }
}

float smoothingFilter(float x, float y, float alpha) {
  return alpha * x + (1 - alpha) * y;
}

void RobotBoard::setIRLamps(bool on) {
  if (on) {
    this->setLED(8, true);
    this->setLED(9, true);   
    this->setLED(10, true);
    this->setLED(11, true);
  } else {
    this->setLED(8, false);
    this->setLED(9, false);
    this->setLED(10, false);
    this->setLED(11, false);
  }
}

void RobotBoard::readIRValues() {
  if (this->adc == nullptr) {
    LOGGER.println("Error: Cannot read IR values - ADC has not been initialized");
    return;
  }

  // Read the IR Values
  int count = this->adc->updateAll();
  if (count < 4) {
    LOGGER.println("Error reading ADC Channels: " + String(count));
    return;
  }
  int ir1Val = this->ir1Channel->value();
  int ir2Val = this->ir2Channel->value();
  int ir3Val = this->ir3Channel->value();
  int ir4Val = this->ir4Channel->value();

  // Compare the IR value to the previous read and if it's wildly different, smooth it out
  int ir1Diff = abs(ir1Val - this->state->ir1_raw);
  int ir2Diff = abs(ir2Val - this->state->ir2_raw);
  int ir3Diff = abs(ir3Val - this->state->ir3_raw);
  int ir4Diff = abs(ir4Val - this->state->ir4_raw);
  if (ir1Diff > 4000 && this->state->ir1_raw > 1) {
    ir1Val = smoothingFilter(ir1Val, this->state->ir1_raw, 0.5);
  }
  if (ir2Diff > 4000 && this->state->ir2_raw > 1) {
    ir2Val = smoothingFilter(ir2Val, this->state->ir2_raw, 0.5);
  }
  if (ir3Diff > 4000 && this->state->ir3_raw > 1) {
    ir3Val = smoothingFilter(ir3Val, this->state->ir3_raw, 0.5);
  }
  if (ir4Diff > 4000 && this->state->ir4_raw > 1) {
    ir4Val = smoothingFilter(ir4Val, this->state->ir4_raw, 0.5);
  }

  // Save the IR Values
  this->state->ir1_raw = ir1Val;
  this->state->ir2_raw = ir2Val;
  this->state->ir3_raw = ir3Val;
  this->state->ir4_raw = ir4Val;

  // Remove the Baseline from the IR Values
  ir1Val -= this->ir1_baseline;
  ir2Val -= this->ir2_baseline;
  ir3Val -= this->ir3_baseline;
  ir4Val -= this->ir4_baseline;

  // Save the State
  this->state->ir1 = ir1Val;
  this->state->ir2 = ir2Val;
  this->state->ir3 = ir3Val;
  this->state->ir4 = ir4Val;

  // Display the IR Values
  if (this->state->isIdle()) {
    String ir1 = padLeft(String(ir1Val), 4, ' ');
    String ir2 = padLeft(String(ir2Val), 4, ' ');
    String ir3 = padLeft(String(ir3Val), 4, ' ');
    String ir4 = padLeft(String(ir4Val), 4, ' ');
    String irVals = "IR: " + ir1 + " " + ir2 + " " + ir3 + " " + ir4;
    this->setOLEDLine(1, irVals, false);
  }
}

void RobotBoard::startBaselining() {
  if (!this->state->isIdle()) { // Only allow baselining if we're in the IDLE state
    return;
  }

  this->state->setState(20); // Move into Baselining Part 1
  this->setLED(5, false);
  LOGGER.println("Starting Baselining...");
  this->setIRLamps(true); // Ensure the IR lamps are on

  this->clearOLED();
  delay(200);

  this->setOLEDLine(0, "BASELINE IR", true);  
  this->setOLEDLine(1, "Place car", true);
  this->setOLEDLine(2, "off line", true);
  this->setOLEDLine(3, "Press button", true);
  this->setOLEDLine(4, "to start", true);
}

void RobotBoard::continueBaselining() {
  this->state->setState(21); // Move into Baselining Part 2
  LOGGER.println("Continuing Baselining...");
  this->clearOLED();
  delay(200);
  this->setOLEDLine(-2, "BASELINING", true);
  this->setOLEDLine(1, "Please wait", true);
  
  long start = millis();
  int ir1_sum = 0;
  int ir2_sum = 0;
  int ir3_sum = 0;
  int ir4_sum = 0;
  int sum_count = 0;
  while (millis() - start < 2000) {
    int count = this->adc->updateAll();
    if (count < 4) {
      LOGGER.println("Error reading ADC Channels: " + String(count));
      delay(200);
      continue;
    }

    int ir1Val = this->ir1Channel->value();
    int ir2Val = this->ir2Channel->value();
    int ir3Val = this->ir3Channel->value();
    int ir4Val = this->ir4Channel->value();

    ir1_sum += ir1Val;
    ir2_sum += ir2Val;
    ir3_sum += ir3Val;
    ir4_sum += ir4Val;
    sum_count++;

    int perc = (millis() - start) / 5000;
    size_t num_dots = floor(perc * 16);
    this->setOLEDLine(3, String(std::string(num_dots, '.').c_str()), false);
    delay(100);
  }

  if (sum_count > 0) {
    this->ir1_baseline = ir1_sum / sum_count;
    this->ir2_baseline = ir2_sum / sum_count;
    this->ir3_baseline = ir3_sum / sum_count;
    this->ir4_baseline = ir4_sum / sum_count;
  } else {
    this->ir1_baseline = 0;
    this->ir2_baseline = 0;
    this->ir3_baseline = 0;
    this->ir4_baseline = 0;
  }
    
  this->setOLEDLine(0, "BASELINE DONE", true);
  this->setOLEDLine(1, "IR1: " + String(this->ir1_baseline), true);
  this->setOLEDLine(2, "IR2: " + String(this->ir2_baseline), true);
  this->setOLEDLine(3, "IR3: " + String(this->ir3_baseline), true);
  this->setOLEDLine(4, "IR4: " + String(this->ir4_baseline), true);
  LOGGER.println("Baselining Done!");
  LOGGER.println("BASELINE IR1: " + String(this->ir1_baseline) + ", IR2: " + String(this->ir2_baseline) + ", IR3: " + String(this->ir3_baseline) + ", IR4: " + String(this->ir4_baseline));
  delay(2000);
  
  this->state->ir1 = 0;
  this->state->ir2 = 0;
  this->state->ir3 = 0;
  this->state->ir4 = 0;
  this->state->setState(0); // Move back to IDLE
}

void RobotBoard::initBoardV1_0(RobotState* state, int accelerometer_address, int adc_address, bool connect_wifi, OledDisplayType oled_type) {
// Setup the board pin mappings
    this->MOTOR_1_F = 4;
    this->MOTOR_1_R = 8;
    this->MOTOR_1_EN = 2;
    this->MOTOR_2_F = 5;
    this->MOTOR_2_R = 9;
    this->MOTOR_2_EN = 3;
    this->BOARD_BUTTON = A6;

    this->motor1 = new DCMotor(this->MOTOR_1_F, this->MOTOR_1_R, this->MOTOR_1_EN);
    this->motor2 = new DCMotor(this->MOTOR_2_F, this->MOTOR_2_R, this->MOTOR_2_EN);
    this->adc_address = adc_address;
    this->imu = new LIS3DH(I2C_MODE, accelerometer_address);   // 0x19 Or 0x18 (depending on the switch on the board)
    this->oled_type = oled_type;
    this->inner_init(state, connect_wifi);
}

void RobotBoard::initBoardV1_1(RobotState* state, int accelerometer_address, int adc_address, bool connect_wifi, OledDisplayType oled_type) {
    // Setup the board pin mappings
    this->MOTOR_1_F = 4;
    this->MOTOR_1_R = 8;
    this->MOTOR_1_EN = 2;
    this->MOTOR_2_F = 5;
    this->MOTOR_2_R = 9;
    this->MOTOR_2_EN = 3;
    this->BOARD_BUTTON = 21;

    this->motor1 = new DCMotor(this->MOTOR_1_F, this->MOTOR_1_R, this->MOTOR_1_EN);
    this->motor2 = new DCMotor(this->MOTOR_2_F, this->MOTOR_2_R, this->MOTOR_2_EN);
    this->adc_address = adc_address;
    this->imu = new LIS3DH(I2C_MODE, accelerometer_address);   // 0x19 Or 0x18 (depending on the switch on the board)
    this->oled_type = oled_type;
    this->inner_init(state, connect_wifi);
}



void RobotBoard::inner_init(RobotState* state, bool connect_wifi) {
    this->state = state;
    this->state->setOnBtnPressed([this](){ this->onBtnPressed(); });
    this->state->setOnBtnLongPressed([this](long duration){ this->onBtnLongPressed(duration); });
    this->state->setOnBtnDoublePressed([this](){ this->onBtnDoublePressed(); });
    
    // Set button pinmode to INPUT (With PullUp)
    pinMode(this->BOARD_BUTTON, INPUT_PULLUP);   
    // Can't use the attachInterrupt method because the button is attached to A6 which is not an interrupt pin on this arduino :(
    // attachInterrupt(digitalPinToInterrupt(this->BOARD_BUTTON), [this]() {
    //   this->state->updateBtnState(digitalRead(this->BOARD_BUTTON));
    // }, CHANGE);

    // Configure the RGB Led associated with the Wifi Chip
    WiFiDrv::pinMode(25, OUTPUT); // define GREEN LED
    WiFiDrv::pinMode(26, OUTPUT); // define RED LED
    WiFiDrv::pinMode(27, OUTPUT); // define BLUE LED

    Wire.begin();   // Start using the I2C Bus
    delay(32);
    
    if (this->isOLEDConnected()) {
        // Initialise the OLED Display
        this->oled_start_line = this->init_oled();
    } else {
        LOGGER.println("NO OLED");
    }

    // Initialise the Shift Register
    // We'll reset the register, switch evrything on for 500ms, then switch everything off
    this->sr.reset(); // Clear all outputs on startup
    for (int i = 0; i < 16; i++) { // Roll the leds on and off
      this->sr.setPin(i, ON);
      delay(125); // 2s / 16 LEDs = 125ms per LED
      this->sr.setPin(i, OFF);
    }

    // Initialise the IMU (Accelerometer)
    this->init_imu();

    // Init ADC Board
    this->adc = new ADS7828(0, SINGLE_ENDED | REFERENCE_OFF | ADC_ON, 0b11111111);    
    this->ir1Channel = this->adc->channel(0);
    this->ir1Channel->minScale = 0;
    this->ir1Channel->maxScale = 0x0FFF;
    this->ir2Channel = this->adc->channel(1);
    this->ir2Channel->minScale = 0;
    this->ir2Channel->maxScale = 0x0FFF;
    this->ir3Channel = this->adc->channel(2);
    this->ir3Channel->minScale = 0;
    this->ir3Channel->maxScale = 0x0FFF;
    this->ir4Channel = this->adc->channel(3);
    this->ir4Channel->minScale = 0;
    this->ir4Channel->maxScale = 0x0FFF;
    this->ir1_baseline = 0;
    this->ir2_baseline = 0;
    this->ir3_baseline = 0;
    this->ir4_baseline = 0;
    this->setIRLamps(true);

    // Connect to the network
    if (connect_wifi) {
      setArduinoRGB(255, 255, 0); // Set the RGB LED to YELLOW
      int result = wifi_connect(2000);
      if (result == WL_CONNECTED) {
        setArduinoRGB(0, 200, 0); // Set the RGB LED to GREEN
        server.begin();                           // start the web server
        this->add_default_http_routes();
        LOGGER.println("HTTP server started");
      } else {
        setArduinoRGB(255, 0, 0); // Set the RGB LED to RED
        LOGGER.println("Error connecting to WiFi: " + String(result));
      }

      if (this->isOLEDConnected()) {
        if (result != WL_CONNECTED) {
          this->setOLEDLine(-1, "<NOTCONNECTED>");
        } else {
          this->setOLEDLine(-1, wifi_address());
        }
      }
    }
}

void RobotBoard::setTrafficLight(bool red, bool yellow, bool green) {
    this->setLED(0, red);
    this->setLED(1, yellow);
    this->setLED(2, green);
}

void RobotBoard::setLED(int led, bool on) {
    if (led < 0 || led > 15) {
        return;
    }

    if (on) {
        this->sr.setPin(led, ON);
    } else {
        this->sr.setPin(led, OFF);
    }
}

void RobotBoard::setRobotState(RobotState* state) { this->state = state; }
RobotState* RobotBoard::getRobotState() { return this->state; }

void RobotBoard::setOLEDLine(int line, const String& msg, bool centered) {
  if (this->isOLEDConnected()) {
    // Center the msg on the screen (width dependent on the screen)
    int screenWidth = 16;
    int screenHeight = 8;
    if (this->oled_type == OLED_TYPE_SSD1306) {
      screenWidth = 24;
    }

    int row = constrain(this->oled_start_line + line, 0, screenHeight - 1);
    int padding = screenWidth - msg.length();
    bool addExtra = false;
    String print_line;

    if (centered) {
      padding = (screenWidth - msg.length()) / 2;
      addExtra = (screenWidth - msg.length()) % 2 == 1;
      print_line.reserve(screenWidth);
      for (int i = 0; i < padding; ++i) {
        print_line += ' ';
      }
    }

    print_line += msg;

    // Clear the rest of the line
    for (int i = 0; i < padding; ++i) {
      print_line += ' ';
    }
    if (centered && addExtra) {
      print_line += ' ';
    }

    if (this->oled_type == OLED_TYPE_GROVE) {
      SeeedOled.setTextXY(row, 0);
      SeeedOled.putString(print_line.c_str());
    } else if (this->oled_type == OLED_TYPE_SSD1306) {
      oled.setTextXY(row, 0);
      oled.putString(print_line);
    }
  }
}

void RobotBoard::clearOLED() {
  if (this->isOLEDConnected()) {
    if (this->oled_type == OLED_TYPE_GROVE) {
      SeeedOled.clearDisplay();
    } else if (this->oled_type == OLED_TYPE_SSD1306) {
      for (int i = 0; i < 8; i++) {
        oled.setTextXY(i, 0);
        oled.putString("                        ");
      }
      // oled.clearDisplay(); // Doesn't work because the library we're using only clears the first 16 chars of each lie :[
    }
  }
}


void RobotBoard::onBtnPressed() {
  if (this->state->isPrimed()) {         // If we're in the PRIMED state, then we should move to DRIVING
    this->state->setDriving();
  } else if (this->state->isDriving()) { // If we're in the DRIVING state, then we should move to IDLE
    this->state->setIdle();
  } else if (this->state->driveState() == 20) { // State == 20 means we're in the Baseline Part 1
    this->continueBaselining();
  } else if (this->state->isIdle()) { // If we're in the IDLE state, then we should move to PRIMED
    if (state->poseX > 0.70) {  // If the car is held mostly vertically - then stop driving immediately
      this->startBaselining();
    } else {
      this->state->setPrimed();
    }
  }
}

void RobotBoard::onBtnLongPressed(long duration) {
  // LOGGER.println("Button Long Pressed: " + String(duration));
  if (this->state->isIdle()) {              // If we're in the IDLE state, then we should move to PRIMED
    this->state->setPrimed();
  } else if (this->state->isPrimed()) {     // If we're in the PRIMED state, then we should revert back to IDLE
    this->state->setIdle();
  }
}

void RobotBoard::onBtnDoublePressed() {
  if (this->state->isIdle()) {          // If we're in the IDLE state, then we should move to DRIVING
      // Enter Baseline Mode
      this->startBaselining();
    } else if (this->state->driveState() == 20) { // State == 20 means we're in the Baseline Part 1
      this->continueBaselining();
    }
}


long RobotBoard::getLastTickDuration() { return this->last_tick_duration; }

long RobotBoard::getTickCounter() { return this->tick_counter; }

void RobotBoard::tick() {
    long start = millis();
    this->tick_counter++;
    
    if (this->tick_counter & 1 == 1) { // Only check the button every 2 ticks
      this->state->updateBtnState(digitalRead(this->BOARD_BUTTON));
    }
    
    if (this->tick_counter % 20 == 0) { // Only handle HTTP server connections every 20 ticks
      this->handleServerConnections();
    }
    
    if (this->state->driveState() > 10) { // States above 10 do not do any sensor updates
      return;
    }
    
    this->updateSensorData();

    long end = millis();
    this->last_tick_duration = end - start;
}

void RobotBoard::updateSensorData() {
  this->readIRValues();

  if (this->tick_counter % 5 == 0) {  // Don't update the accelerometer every tick
    this->readAccelerometerData();
  }
}

void RobotBoard::setArduinoRGB(int red, int green, int blue) {
  int r = constrain(red, 0, 255);
  int g = constrain(green, 0, 255);
  int b = constrain(blue, 0, 255);
  WiFiDrv::analogWrite(25, r);  // RED
  WiFiDrv::analogWrite(26, g);  // GREEN
  WiFiDrv::analogWrite(27, b);  // BLUE
}

int wifi_connect(int timeout_millis) {
  if (WiFi.status() == WL_NO_MODULE) {
    LOGGER.println("No Wifi module available - cannot connect to Wifi");
    return WL_NO_MODULE;
  }

  // Check if SECRET_SSID and SECRET_PASS are defined
  if (SECRET_SSID == nullptr || SECRET_PASS == nullptr) {
    LOGGER.println("Wifi SSID and/or Password not defined - cannot connect to Wifi - please ensure you have defined them in the arduino_secrets.h file");
    return WL_CONNECT_FAILED;
  }

  char ssid[] = SECRET_SSID;        // Network SSID (name)
  char pass[] = SECRET_PASS;        // Network password
  int status = WL_IDLE_STATUS;      // Wifi radio's status
  long start = millis();
  LOGGER.println("Connecting to Wifi Network: " + String(ssid));
  while (status != WL_CONNECTED && millis() - start < timeout_millis) {
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);
    if (status != WL_CONNECTED) {
      delay(100);
    } else {
      IPAddress ip = WiFi.localIP();
      long rssi = WiFi.RSSI();
      LOGGER.println("WIFI CONNECTED; IP: " + ip.toString() + "; RSSI: " + String(rssi));
    }
  }
  return status;
}

void RobotBoard::addHttpHandler(String path, std::function<HttpResponse(WiFiClient*, HttpRequest)> handler) {
  this->httpHandlers[path] = handler;
}


int utf8ByteLength(const String& str) {
  int utf_8_byte_length = 0;
  for (char c : str) {
    if ((c & 0x80) == 0) utf_8_byte_length += 1;          // 1-byte character
    else if ((c & 0xE0) == 0xC0) utf_8_byte_length += 2;  // 2-byte character
    else if ((c & 0xF0) == 0xE0) utf_8_byte_length += 3;  // 3-byte character
    else if ((c & 0xF8) == 0xF0) utf_8_byte_length += 4;  // 4-byte character
  }
  return utf_8_byte_length;
}

std::vector<std::string> split(const std::string& str, char delimiter) {
  std::vector<std::string> tokens;
  std::string token;
  for (char c : str) {
    if (c == delimiter) {
      if (!token.empty()) {
        tokens.push_back(token);
        token = "";
      }
    } else {
      token += c;
    }
  }
  if (!token.empty()) {
    tokens.push_back(token);
  }
  return tokens;
}

void RobotBoard::handleServerConnections() {
  // Check for any new client - and add it to the client slots if one is available (we're only supporting 8 clients at a time)
  // LOGGER.println("Will check for new clients...");

  WiFiClient newClient = server.accept();
  if (newClient) {
    // LOGGER.println("New client found...");
    bool found = false;
    for (byte i=0; i < MAX_SERVER_CLIENTS; i++) {
      if (!this->serverClients[i]) {
        // Found a slot for the new client...
        this->serverClients[i] = newClient;
        found = true;
        break;
      }
    }

    if (!found) {
      newClient.println("HTTP/1.1 503 Server Busy");
      newClient.println("Connection: close");
      newClient.println("Content-Length: 0");
      newClient.println();
      newClient.stop();
    }
  }

  // Check connected clients for new data
  for (byte i=0; i < MAX_SERVER_CLIENTS; i++) {
    if (this->serverClients[i] && this->serverClients[i].available() > 0) {
      const char *method, *path;
      std::vector<byte> buf(4096);  
      int parse_result = 0, minor_version;
      struct phr_header headers[100];
      size_t buflen = 0, prevbuflen = 0, method_len, path_len, num_headers;

      while (parse_result <= 0) {
        byte tmpBuf[4096];
        int read_result = this->serverClients[i].read(tmpBuf, 4096);
        if (read_result == -1) {
            // IOError - so print to serial and stop the client
            LOGGER.println("Error reading from HTTP client [" + String(i) + "] - will stop client");
            this->serverClients[i].stop(); 
        } else if (read_result == 0) {
            continue; // No data read - move onto the next client
        }

        buflen = read_result;
        // Check if we need to extend the buffer
        if (buflen + prevbuflen > buf.size()) {
          // Create a new buffer and copy the old buffer into it
          buf.resize(prevbuflen + buflen);
          memcpy(buf.data() + prevbuflen, tmpBuf, buflen);
        } else {
          // Append tmpBuf to buf
          memcpy(buf.data() + prevbuflen, tmpBuf, buflen);
        }
        buflen = buflen + prevbuflen;

        /* parse the request */
        num_headers = sizeof(headers) / sizeof(headers[0]);
        const char* buf_char = reinterpret_cast<const char*>(buf.data());
        parse_result = phr_parse_request(buf_char, buflen, &method, &method_len, &path, &path_len, &minor_version, headers, &num_headers, prevbuflen);
        if (parse_result == -1) {
            // Failed to parse the request, respond with a 400 Bad Request
            LOGGER.println("Failed to parse the request from HTTP client [" + String(i) + "] - will respond with a 400 Bad Request");
            this->serverClients[i].println("HTTP/1.1 400 Bad Request");
            this->serverClients[i].println("Content-Type: text/plain");
            this->serverClients[i].println("Connection: close");
            this->serverClients[i].println("Content-Length: 0");
            this->serverClients[i].println();
            this->serverClients[i].stop();
            break;
        } else if (parse_result == -2) {
            // Request is incomplete, continue the loop
            prevbuflen = buflen;
            continue;
        } else {
            break;
        }
      }

      if (parse_result <= 0) {
        // Failed to parse the request, so move onto the next client
        continue;
      }

      if (buflen == sizeof(buf)) {
          // Request is too long, respond with a 413 Payload Too Large
          LOGGER.println("Request from HTTP client [" + String(i) + "] is too large - will respond with a 413 Payload Too Large");
          this->serverClients[i].println("HTTP/1.1 413 Payload Too Large");
          this->serverClients[i].println("Content-Type: text/plain");
          this->serverClients[i].println("Connection: close");
          this->serverClients[i].println("Content-Length: 0");
          this->serverClients[i].println();
          this->serverClients[i].stop();
          continue;
      }

      // Grab the Request Details
      String body = "";
      // Set body to the remainder of the buffer after the headers
      if (parse_result < buflen) {
        body = String(reinterpret_cast<const char*>(buf.data()) + parse_result);
      }

      HttpRequest req = HttpRequest();
      req.method = String(method, method_len);
      std::string full_path = std::string(path, path_len);
      if (full_path.find('?') != std::string::npos) {
        req.path = String(full_path.substr(0, full_path.find('?')).c_str());
        if (req.path.endsWith("/")) {
          req.path.remove(req.path.length() - 1);
        }
        std::string query_string = full_path.substr(full_path.find('?') + 1);
        std::vector<std::string> query_pairs = split(query_string, '&');
        for (std::string pair : query_pairs) {
          std::vector<std::string> key_value = split(pair, '=');
          if (key_value.size() == 2) {
            req.query[String(key_value[0].c_str())] = String(key_value[1].c_str());
          }
        }
      } else {
        req.path = String(path, path_len);
      }
      req.body = body;
      for (int h = 0; h != num_headers; ++h) {
        req.headers[String(headers[h].name, headers[h].name_len)] = String(headers[h].value, (int)headers[h].value_len);
      }

      // Write some debugging info to the serial port
      if (this->debug) {
        LOGGER.println("New Request: [HTTP Client " + String(i) + "]");
        LOGGER.println("  Method: " + req.method);
        LOGGER.println("  Path: " + req.path);
        LOGGER.println("  Query Params: ");
        for (auto const& [key, val] : req.query) {
          LOGGER.println("    " + key + ": " + val);
        }
        LOGGER.println("  Headers: ");
        for (auto const& [key, val] : req.headers) {
          LOGGER.println("    " + key + ": " + val);
        }
        if (req.body.length() > 0)
          LOGGER.println("  Body: " + req.body);
        LOGGER.println("---");
      }

      // Look for a request handler for the path
      if (this->httpHandlers.find(req.path) != this->httpHandlers.end()) {
        // Pass the request to the handler
        HttpResponse response = this->httpHandlers[req.path](&this->serverClients[i], req);

        // Write the response to the client
        this->serverClients[i].println("HTTP/1.1 " + String(response.status));
        for (auto const& [key, val] : response.headers) {
          this->serverClients[i].println(key + ": " + val);
        }
        
        size_t bytes_written = 0;
        size_t total_bytes = utf8ByteLength(response.body);
        this->serverClients[i].println("Content-Length: " + String(total_bytes));
        this->serverClients[i].println();
        long start = millis();
        size_t write_batch_length = 1024;
        while (bytes_written < total_bytes) {
          size_t chunk_size = min(write_batch_length, total_bytes - bytes_written);
          size_t written = this->serverClients[i].write(response.body.c_str() + bytes_written, chunk_size);
          bytes_written += written;
        
          if (bytes_written == total_bytes) {
            this->serverClients[i].println();
            break;
          }

          if (millis() - start > 1000) {  // Too long - give up - we don't want to waste time here writing to the socket - and 1s is already too long!!
            LOGGER.println("Timeout writing response to HTTP client [" + String(i) + "] - will stop client");
            this->serverClients[i].stop();
            break;
          }
        }
      } else if (req.path.length() == 1 && req.path[0] == '/') {
        // Respond with a default response
        String response_text = "<html><head><title>Innovation Hub - Line Robot</title></head><body><h1>Hello!</h1><h3>You're connected to the Robot!</h3></body></html>";
        this->serverClients[i].println("HTTP/1.1 200 OK");
        this->serverClients[i].println("Content-Type: text/html");
        this->serverClients[i].println("Content-Length: " + String(utf8ByteLength(response_text)));
        this->serverClients[i].println();
        this->serverClients[i].print(response_text.c_str());  // send the response body
      } else {
        // No Handler for this request, respond with a 404 Not Found
        this->serverClients[i].println("HTTP/1.1 404 Not Found");
        this->serverClients[i].println("Content-Type: text/plain");
        this->serverClients[i].println("Content-Length: 0");
        this->serverClients[i].println();
      }
    }
  }

  // Clear any slots of disconnected clients
  for (byte i=0; i < MAX_SERVER_CLIENTS; i++) {
    if (this->serverClients[i] && !this->serverClients[i].connected()) {
      if (this->debug) {
        LOGGER.println("Client [" + String(i) + "] disconnected");
      }
      this->serverClients[i].stop();
    }
  }
}


/**************************************
 * END Robot Board Class Implementation *
 **************************************/




/**************************************
 * Helper Functions Implementation *
 * ************************************/

/**
 * Returns the IP Address of the Wifi Module
 */
String wifi_address() {
  if (WiFi.status() != WL_CONNECTED) {
    return "NOT CONNECTED";
  } else {
    IPAddress ip = WiFi.localIP();
    return ip.toString();
  }
}

/**
 * Returns the Wifi Signal Strength (RSSI) of the Wifi Module
 */
long wifi_strength() {
  if (WiFi.status() != WL_CONNECTED) {
    return 0;
  } else {
    long rssi = WiFi.localIP();
    return rssi;
  }
}


/**
 * Scan for Wifi Networks in the area and print out their SSID and Signal Strength
 */
int wifi_scan() {
  Serial.print("Scanning Wifi Networks...");
    int numSsid = WiFi.scanNetworks();
    if (numSsid == 0) {
      LOGGER.println("None Found");
      return 0;
    } else if (numSsid == -1) {
      LOGGER.println("Failed");
      return 0;
    }

    LOGGER.println(String(numSsid) + " Networks Found");
    for (int i = 0; i < numSsid; i++) {
      Serial.print(i);
      Serial.print(". ");
      Serial.print(WiFi.SSID(i));
      Serial.print("\tSignal: ");
      Serial.print(WiFi.RSSI(i));
      LOGGER.println(" dBm");
    }

  return numSsid;
}

/**
 * Returns the SSID of the Wifi Network that the Wifi Module is connected to
 */
String wifi_ssid(int index) {
  if (WiFi.status() != WL_CONNECTED) {
    return "NOT CONNECTED";
  } else {
    return WiFi.SSID(index);
  }
}

/**
 * Scans the I2C Bus for Devices and prints out the addresses of the devices found
 */
int scan_i2c() {
  LOGGER.println("Scanning I2C Bus for Devices...");
  int nDevices = 0;
  for(int address = 1; address < 127; address++ ) {
    delayMicroseconds(50);
    Wire.beginTransmission(address);
    int error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("- ADDR: 0x");
      if (address < 16)
        Serial.print(F("0"));
      Serial.print(address, HEX);
      Serial.print(" (");
      Serial.print(address);
      LOGGER.println(")");
      nDevices++;
    } else if (error==4) {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print(F("0"));
      LOGGER.println(address,HEX);
    }
  }

  if (nDevices == 0)
    LOGGER.println("No I2C devices found\n");
  
  return nDevices;
}

