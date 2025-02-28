/************************************************************************************************************************************
 *                                                                                                                                  *                     
 *                                            Hub Line Robot Starter Code                                                           *
 *                                                                                                                                  *                
 * This is the starter code for working with the Hub Line Robot Development Board with an attached Arduino MKR 1010.                *
 *                                                                                                                                  *
 * To get started, install the Arduino IDE, and then install the WiFiNINA library (open the Library panel and search for WiFiNINA)  *
 *                                                                                                                                  *
 * The Base Code provides a set of classes and functions to help you get started with the Hub Line Robot Development Board.         *
 *                                                                                                                                  *     
 * The main classes are:                                                                                                            *
 * - RobotState: A class that manages the state of the robot (IDLE, PRIMED, DRIVING)                                                *                    
 * - RobotBoard: A class that manages the hardware of the robot (Motors, Sensors, etc)                                              *
 *                                                                                                                                  *
 *                                                                                                                                  *
 * The robot has three states:                                                                                                      *                       
 * - IDLE      [When the robot is not doing anything]                                                                               *
 * - PRIMED    [When the button is pushed in - hold the button in and it will remain in primed state until you release the button]  *
 * - DRIVING   [When the robot is driving]                                                                                          *
 *                                                                                                                                  *                 
 * You can attach your own code to handle entering and exiting these states, as well as code to run while in each state.            *
 *                                                                                                                                  *                         
 * The base code also provides a simple HTTP server (on port 80) that you can connect to, allowing you to provide the ability to    *
 *  send commands to the robot or obtain information via HTTP requests.                                                             *
 *                                                                                                                                  *               
 * If an OLED display is connected, the default base code will display the IP Address of the device, along with the current state.  *
 *                                                                                                                                  *                 
 ************************************************************************************************************************************/


#include <Arduino.h>
#include "src/hub-line-robot-base.h"
#include <ArduinoHttpClient.h>


/**
Example Custom State for your robot (overriding the default state object)
**/
class MyRobotState : public RobotState {
  public:
    long my_variable = 0;
    
    MyRobotState() : RobotState() { }
};

/**
 * Global Variables
 */
static const long start_millis = millis();
RobotBoard* board = nullptr;
MyRobotState* state = nullptr;

// Create these as global variables at the top of your file with other globals
WiFiClient wifi;
HttpClient httpClient = HttpClient(wifi, "fn-obs-backend01.azurewebsites.net", 80);  // Replace with your server address and port


/**
 * Example function that will be called when the robot enters the IDLE state
 * 
 * Attach your own code here to handle entering the IDLE state.
 */
void onIdleBegin() {
  Serial.println("IDLE Start");
  board->setTrafficLight(true, false, false);
  board->clearOLED();
  board->printOLEDHeader();
  board->setOLEDLine(3, "I D L E", true);
  board->setSpeedOnBothMotors(0);  // Turn off both motors
}

/**
 * Example function that will be called when the robot enters the PRIMED state
 * 
 * Attach your own code here to handle entering the PRIMED state.
 * 
 * The PRIMED state is when the button is pushed in - if you hold the button in it will remain in primed state until you release the button
 */
void onPrimedBegin() {
  Serial.println("Primed Start");
  board->setTrafficLight(false, true, false);
  board->clearOLED();
  board->printOLEDHeader();
  board->setOLEDLine(3, "P R I M E D", true);   
}

/**
 * Example function that will be called when the robot enters the DRIVING state
 * 
 * Attach your own code here to handle entering the DRIVING state.
 */
void onDrivingBegin() {
  Serial.println("Driving Start");
  board->clearOLED();
  board->printOLEDHeader();
  board->setTrafficLight(false, false, true);
  board->setOLEDLine(1, "D R I V I N G", true);
}

/**
 * Example function that will be called when the robot exits the DRIVING state
 * 
 * Attach your own code here to handle exiting the DRIVING state.
 */
void onDrivingEnd() {
  Serial.println("Driving End");
  board->setSpeedOnBothMotors(0);  // Turn off both motors
  delay(200);
}



/**
 * Example function that will be called regularly when the robot is in the IDLE state
 * 
 * Attach your own code here to handle actions during the IDLE state.
 */
void idleActions() {
    // Nothing...
}

/**
 * Example function that will be called regularly when the robot is in the PRIMED state
 * 
 * Attach your own code here to handle actions during the PRIMED state.
 */
void primedActions() {
  // Nothing...
}

/**
 * Example function that will be called regularly when the robot is in the DRIVING state
 * 
 * Attach your own code here to handle actions during the DRIVING state.
 * 
 * This is the place to put your driving code :)
 */
void drivingActions() {
  // If driving for more than 30s, stop driving...
  if (state->poseX > 0.88) {  // If the car is held vertically - then stop driving immediately
    state->setIdle();
    return;
  }

  // constants
  int defaultSpeed = -100;
  float leftWheelOffset = 0.9;
  float turnScale = 0.96;

  int rightWheel = defaultSpeed;
  int leftWheel = defaultSpeed * leftWheelOffset;

  // Get state - IR4 is far left and IR1 is far right
  int ir1_farright = state->ir1;
  int ir2_right = state->ir2;
  int ir3_left = state->ir3;
  int ir4_farleft = state->ir4;

  // Understand where the line is
  // black line is bigger number
  // turning difference will be the difference between right numbers and left numbers
  int turningDifference = (ir1_farright + ir2_right) - (ir3_left + ir4_farleft);

  // turn left or right
  if (turningDifference > 0) {
    // reduce right wheel speed
    rightWheel = rightWheel * turnScale;
    Serial.println(String("-->> ") + "Wheels: L " + leftWheel + " R " + rightWheel +  "  IR: " + ir4_farleft + " | " + ir3_left + " | " + ir2_right + " | " + ir1_farright + "  diff: " + turningDifference);
  } else {
    // reduce left wheel speed
    leftWheel = leftWheel * turnScale;
    Serial.println(String("<<-- ") + "Wheels: L " + leftWheel + " R " + rightWheel +  "  IR: " + ir4_farleft + " | " + ir3_left + " | " + ir2_right + " | " + ir1_farright + "  diff: " + turningDifference);
  }
  
  board->setMotorSpeedL(leftWheel);
  board->setMotorSpeedR(rightWheel);
  

  // drive for a bit
  delay(500);
  
  
  
}


/**
 * This is the setup function that will be called once when the board is powered on (or after a reset)
 * 
 * Attach any setup code you need here.
 */
void setup() {
  
  // We'll initialise the Serial port and give it some time to connect before continuing (Having serial COM is useful for debugging and monitoring when attachd to a computer)
  Serial.begin(9600);
  while (!Serial && millis() - start_millis < 200) {
    delay(10);     // Pause until serial console opens (for a few millis anyway...)
  }


  Serial.println("Innovation Hub - Line Robot - Initialising...");

  // Setup the Robot State object to be the customer state object defined above
  state = new MyRobotState();
  
  // There are three states in the RobotState: 
  //  - IDLE      [When the robot is not doing anything]
  //  - PRIMED    [When the button on the board is pushed in - if you hold the button in it will remain in primed state until you release the button]
  //  - DRIVING   [When the robot is driving]
  // You set a callback function to be invoked whenever you enter or exit one of these states, using the `set[State]Start` and `set[State]End` methods.
  // eg. state->setIdleStart(onIdleBegin);  <-- The provided `onIdleBegin` function will be called whenever the robot enters the IDLE state
  state->setOnIdleStart(onIdleBegin);
  state->setOnPrimedStart(onPrimedBegin);
  state->setOnDrivingStart(onDrivingBegin);
  state->setOnDrivingEnd(onDrivingEnd);

  // Configure + Initialise the Robot board - you will be using version 1.1 of the board, so call the `initBoardV1_1` method
  // There are some switches on the board that you can use to configure the I2C addresses for the Accelerometer and ADC
  // The OLED display is optional, and if you attach one, you must specify the type of OLED display you have connected (OLED_TYPE_GROVE or OLED_TYPE_SSD1306) [These are the two types of display we have]
  board = new RobotBoard();
  board->initBoardV1_1(state, 0x19, 0x48, true, OLED_TYPE_SSD1306);  // 0x19 for the Accelerometer, 0x17 for ADC, true for WiFi connection, OLED_TYPE_SSD1306 for the OLED display

  // The Arduino has a built in wifi, so it can connect to the Devices network (if you don't know the password, just ask any of the Hub team for it).
  //  - You must have the correct SSID and Password in the `arduino_secrets.h` file for the connection to work.
  //  - If the Wifi connection is successful, the LED in the centre of the Arduino will be set to green, and the IP address of the device will be printed to both the serial console and the OLED display (if one is attached)
  //  - If the Wifi connection fails, the LED will be set to red, and an error message will be printed to the serial console.
  //  - If you don't want to connect to the Wifi network, you can set the `connect_wifi` parameter above to false.
  // 
  // Assuming a successful connection to the WIFI, the Board will launch a small HTTP Server, so you can connect to the device and send commands to it via HTTP.
  // The Server will be listening on port 80, and you can add your own handlers to respond to different paths.
  // Here's an example HTTP Handler that will respond on the `/set-traffic-light` path by setting the state of the traffic light LEDs at the front of the board.
  board->addHttpHandler("/set-traffic-light", [](WiFiClient* client, HttpRequest req) {
    std::string msg = "Set Traffic Lights to: \n";
    bool red = false, yellow = false, green = false;
    if (req.query.find("red") != req.query.end()) {
      red = req.query["red"] == "true";
      msg += " -    RED: ";
      msg += (red ? "ON" : "OFF");
      msg += "\n";
    }
    
    if (req.query.find("yellow") != req.query.end()) {
      yellow = req.query["yellow"] == "true";
      msg += " - YELLOW: ";
      msg += (yellow ? "ON" : "OFF");
      msg += "\n";
    } else if (req.query.find("orange") != req.query.end()) {
      yellow = req.query["yellow"] == "true";
      msg += " - ORANGE: ";
      msg += (yellow ? "ON" : "OFF");
      msg += "\n";
    }
    
    if (req.query.find("green") != req.query.end()) {
      green = req.query["green"] == "true";
      msg += " -  GREEN: ";
      msg += (green ? "ON" : "OFF");
      msg += "\n";
    }
    board->setTrafficLight(red, yellow, green);

    HttpResponse resp = HttpResponse();
    resp.status = 200;
    resp.body = String(msg.c_str());
    resp.headers["Content-Type"] = "text/plain";
    return resp;
  });

  // Add any additional HTTP Handlers here...

  // Add any additional setup code here...

  // (optional) Scan for I2C devices, and Print their addresses to the serial port (useful for confirming that you've specified the correct Accelerometer and ADC addresses)
  scan_i2c();

  // We'll run the Idle Begin Callback Once now (as we're technically entering the IDLE state now - we could've done this during init, but but not doing this automatically, it provides more flexibility - perhaps you don't want to start in the IDLE state...)
  onIdleBegin();

  // Take a breather for a bit before we start the main loop
  delay(100);
}

/**
 * This is the main loop function that will be called repeatedly after the setup function has completed
 * 
 * Attach any code that you want to run repeatedly here.
 * 
 */
void loop() {  
  // Step 1: Tick the Board (update sensor data, handle HTTP server connections, etc...)
  board->tick();

  Serial.println("loop start");
  
  // Serial.println("Making HTTP request...");
  // httpClient.get("/api/backendfunction1?name=JV&code=<secret>");
  
  // // Get the status code and response
  // int statusCode = httpClient.responseStatusCode();
  // String response = httpClient.responseBody();
  
  // Serial.print("Status code: ");
  // Serial.println(statusCode);
  // Serial.print("Response: ");
  // Serial.println(response);

  // BUZZ - Replace with new musical sequence
    // Define note frequencies
    const int A3 = 220;    // A3
    const int AS3 = 233;   // A#3
    const int B3 = 247;    // B3
    const int CS4 = 277;   // C#4
    const int DS4 = 311;   // D#4
    const int E4 = 330;    // E4
    const int FS3 = 185;   // F#3
    
    // Define note durations (in milliseconds)
    const int QUARTER = 250;
    const int HALF = 500;
    const int WHOLE = 1000;
    const int TRIPLET = 200;  // QUARTER * 2/3
    
    // Play the sequence
    // F#3 (HALF)
    tone(A4, FS3, HALF);
    delay(HALF + 50);
    
    // E4 (HALF + QUARTER)
    tone(A4, E4, HALF + QUARTER);
    delay(HALF + QUARTER + 50);
    
    // D#4 (QUARTER)
    tone(A4, DS4, QUARTER);
    delay(QUARTER + 50);
    
    // Triplet sequence: C#4, B3, A#3
    tone(A4, CS4, TRIPLET);
    delay(TRIPLET + 20);
    tone(A4, B3, TRIPLET);
    delay(TRIPLET + 20);
    tone(A4, AS3, TRIPLET);
    delay(TRIPLET + 20);
    
    // A3 (WHOLE)
    tone(A4, A3, WHOLE);
    delay(WHOLE + 50);
    
    noTone(A4);  
  delay(3000);


  // // Step 1.5: Update the LEDs based on the IR sensor values (to provide a nice visual indication of the IR sensor values)
  // if (state->driveState() <= 10 && board->getTickCounter() & 0x01 == 1) {
  //   int ir1_state = state->ir1 < state->ir1_soft_threshold;
  //   int ir2_state = state->ir2 < state->ir2_soft_threshold;
  //   int ir3_state = state->ir3 < state->ir3_soft_threshold;
  //   int ir4_state = state->ir4 < state->ir4_soft_threshold;

  //   board->setLED(6, ir1_state);
  //   board->setLED(3, ir2_state);
  //   board->setLED(4, ir3_state);
  //   board->setLED(7, ir4_state);
  // }

  // // Step 2: Perform Action (based on current State)
  // if (state->isIdle()) {                 // IDLE is when the robot is not doing anything
  //   idleActions();
  // } else if (state->isPrimed()) {        // PRIMED is when the button is pushed in - if you hold the button in it will remain in primed state until you release the button
  //   primedActions();
  // } else if (state->isDriving()) {       // DRIVING, the robot is actively driving, so do whatever needs to be done to continue driving for this tick
  //   drivingActions();
  // }

  // Add any additional loop code here...
}


