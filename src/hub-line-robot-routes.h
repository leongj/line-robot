
#ifndef HUB_LINE_ROBOT_ROUTES_H
#define HUB_LINE_ROBOT_ROUTES_H

#include <vector>
#include <string>
#include "hub-line-robot-base.h"

void RobotBoard::add_default_http_routes() {
    // Default Route
    this->addHttpHandler("/", [this](WiFiClient* client, HttpRequest req) {
      HttpResponse resp = HttpResponse();
      resp.status = 200;

      String index_html = R"rawliteral(
      <html>        
      <head>
      <title>Hub Line Robot</title>
      <style>
        body {            
          font-family: Arial, sans-serif;
          background-color: #f4f4f4;
          color: #333;
          margin: 0;
          padding: 20px;
        }
        h1 {
          color: #333;
        }
        .container {
          max-width: 950px;
          margin: 0 auto;
          padding: 20px;
          background-color: #fff;
          border-radius: 5px;
          box-shadow: 0 2px 5px rgba(0, 0, 0, 0.1);
        }
        .button {
          display: inline-block;
          padding: 10px 20px;
          background-color: #007bff;
          color: #fff;
          text-decoration: none;
          border-radius: 5px;
          transition: background-color 0.3s;
          margin-top: 2rem;
          margin-left: 1.5rem;
        }
        .button:hover {
          background-color: #0056b3;
        }
        .button:active {
          background-color: #004085;
        }
        #respDiv {
          font-size: 1.1rem;
          margin-top: 2rem;
          padding: 1rem;
          background-color: #f4f4f4;
          border-radius: 5px;
          box-shadow: 0 2px 5px rgba(0, 0, 0, 0.1);
        }
      </style>
      </head>
      <body>
        <div class="container">
          <h1>Innovation Hub - Line Robot</h1>
          <button onclick="callAction('/btn-press', this)" class="button">Button Press</button>
          <button onclick="callAction('/btn-long-press', this)" class="button">Button Long Press</button>
          <button onclick="callAction('/btn-double-press', this)" class="button">Button Double Press</button>
          <br/>
          <button onclick="callAction('/ir', this)" class="button">Infrared</button>
          <button onclick="callAction('/adc', this)" class="button">ADC</button>
          <button onclick="callAction('/adc', this)" class="button">Pose</button>
          <button onclick="callAction('/state', this)" class="button">State</button>
          <br/>
          <button onclick="callAction('/set-state?state=0', this)" class="button">Set Idle</button>
          <button onclick="callAction('/set-state?state=1', this)" class="button">Set Primed</button>
          <button onclick="callAction('/set-state?state=2', this)" class="button">Set Driving</button>          
          <br />
          <button onclick="callAction('/log', this)" class="button">Get Log</button>
        </div>

        <div id="respDiv"></div>

        <hr />
        <h3>Available Routes:</h3>
        <ul>
          <li>/adc - Reads the ADC and returns the value for each of the 8 channels</li>
          <li>/ir - Returns the current IR values (along with the Raw values, baselines + thresholds)</li>
          <li>/pose - Returns the current pose of the robot, given as x, y, z values between 0-1</li>
          <li>/state - Returns the current state of the robot (eg. 0: IDLE, 1: PRIMED, 2: DRIVING, etc...)</li>
          <li>/btn-press - Simulate a button push</li>
          <li>/btn-long-press - Simulate a long press of the button (>1200ms)</li>
          <li>/btn-double-press - Simulate a double press of the button</li>
          <li>/set-ir-baselines - Sets the IR Baseline values (use ?0=<val>&1=<val>&2=<val>&3=<val>)</li>
          <li>/set-ir-hard-thresholds - Sets the IR Hard Threshold values (use ?0=<val>&1=<val>&2=<val>&3=<val>)</li>
          <li>/set-ir-soft-thresholds - Sets the IR Soft Threshold values (use ?0=<val>&1=<val>&2=<val>&3=<val>)</li>
          <li>/set-led - Sets the state of a LED (use ?<num>=(0|1)), you can also specify ?all=(0|1)</li>
          <li>/set-motor-speed - Sets the speed of the motor(s) (use ?(left|right)=(-255-255)), eg. ?left=200&right=-100</li>
          <li>/start-baseline - Starts the baselining process</li>
          <li>plus any other routes you've added :p</li>
        </ul> 

        <script type="text/javascript">
          function callAction(actionPath, el) {
            let elMsg = '';
            if (el) {
              el.disabled = true;
              elMsg = el.innerText;
              el.innerText = "Processing...";
            }
            fetch(actionPath)
              .then(response => response.text())
              .then(data => {
                console.log(data);
                document.getElementById('respDiv').innerText = data;
              })
              .finally(() => {
                if (el) {                            
                  el.disabled = false;
                  el.innerText = elMsg;
                }
              });
          }
        </script>
      </body>
      </html>
      )rawliteral";
      resp.body = index_html;
      resp.headers["Content-Type"] = "text/html";
      return resp;
    });

    this->addHttpHandler("/log", [this](WiFiClient* client, HttpRequest req) {
        int num_lines = 0;
        if (req.query.find("lines") != req.query.end()) {
            num_lines = req.query["lines"].toInt();
        }
        std::vector<std::string> log = num_lines > 0 ? LOGGER.tail(num_lines) : LOGGER.all();
        String body = "";
        for (std::string line : log) {
            body += String(line.c_str()) + "\n";
        }

        HttpResponse resp = HttpResponse();
        resp.status = 200;
        resp.body = body;
        resp.headers["Content-Type"] = "text/plain";
        return resp;
    });
  
  
    // Retrieve the current "state" and return it
    this->addHttpHandler("/state", [this](WiFiClient* client, HttpRequest req) {
      HttpResponse resp = HttpResponse();
      resp.status = 200;
      int drive_state = this->getRobotState()->driveState();
      if (drive_state == 0) {
        resp.body = String(drive_state) + ": IDLE";
      } else if (drive_state == 1) {
        resp.body = String(drive_state) + ": PRIMED";
      } else if (drive_state == 2) {
        resp.body = String(drive_state) + ": DRIVING";
      } else if (drive_state == 20) {
        resp.body = String(drive_state) + ": SETTING BASELINE";
      } else if (drive_state == 21) {
        resp.body = String(drive_state) + ": BASELINING";
      } else {
        resp.body = String(drive_state) + ": OTHER";
      }
      resp.headers["Content-Type"] = "text/plain";
      return resp;
    });
  
    // Read the ADC and return the values
    this->addHttpHandler("/adc", [this](WiFiClient* client, HttpRequest req) {
      HttpResponse resp = HttpResponse();
      resp.status = 200;
      this->adc->updateAll();
      resp.body = "0: " + String(this->adc->channel(0)->value())
        + "\n1: " + String(this->adc->channel(1)->value())
        + "\n2: " + String(this->adc->channel(2)->value())
        + "\n3: " + String(this->adc->channel(3)->value())
        + "\n4: " + String(this->adc->channel(4)->value())
        + "\n5: " + String(this->adc->channel(5)->value())
        + "\n6: " + String(this->adc->channel(6)->value())
        + "\n7: " + String(this->adc->channel(7)->value());
      resp.headers["Content-Type"] = "text/plain";
      return resp;
    });
  
    // Return the current IR values, along with their raw values, baselines + thresholds
    this->addHttpHandler("/ir", [this](WiFiClient* client, HttpRequest req) {
      HttpResponse resp = HttpResponse();
      resp.status = 200;
      RobotState* state = this->getRobotState();
      resp.body = "IR1: " + String(state->ir1) + "\n"
        + "IR2: " + String(state->ir2) + "\n"
        + "IR3: " + String(state->ir3) + "\n"
        + "IR4: " + String(state->ir4)
        + "\n\nRaw:\n"
        + "IR1: " + String(state->ir1_raw) + "\n"
        + "IR2: " + String(state->ir2_raw) + "\n"
        + "IR3: " + String(state->ir3_raw) + "\n"
        + "IR4: " + String(state->ir4_raw)
        + "\n\nBaselines:\n"
        + "IR1: " + String(this->ir1_baseline) + "\n"
        + "IR2: " + String(this->ir2_baseline) + "\n"
        + "IR3: " + String(this->ir3_baseline) + "\n"
        + "IR4: " + String(this->ir4_baseline)
        + "\n\nSoft Thresholds:\n"
        + "IR1: " + String(state->ir1_soft_threshold) + "\n"
        + "IR2: " + String(state->ir2_soft_threshold) + "\n"
        + "IR3: " + String(state->ir3_soft_threshold) + "\n"
        + "IR4: " + String(state->ir4_soft_threshold)
        + "\n\nHard Thresholds:\n"
        + "IR1: " + String(state->ir1_hard_threshold) + "\n"
        + "IR2: " + String(state->ir2_hard_threshold) + "\n"
        + "IR3: " + String(state->ir3_hard_threshold) + "\n"
        + "IR4: " + String(state->ir4_hard_threshold);
      resp.headers["Content-Type"] = "text/plain";
      return resp;
    });
  
    // Return the current pose
    this->addHttpHandler("/pose", [this](WiFiClient* client, HttpRequest req) {
      HttpResponse resp = HttpResponse();
      resp.status = 200;
      resp.body = "X: " + String(this->getRobotState()->poseX) + "\n"
        + "Y: " + String(this->getRobotState()->poseY) + "\n"
        + "Z: " + String(this->getRobotState()->poseZ);
      resp.headers["Content-Type"] = "text/plain";
      return resp;
    });
    
    
    // Simulate a button press
    this->addHttpHandler("/btn-press", [this](WiFiClient* client, HttpRequest req) {
      this->onBtnPressed();
      HttpResponse resp = HttpResponse();
      resp.status = 200;
      resp.body = "Ok";
      resp.headers["Content-Type"] = "text/plain";
      return resp;
    });
    // Simulate a button long press
    this->addHttpHandler("/btn-long-press", [this](WiFiClient* client, HttpRequest req) {
      this->onBtnLongPressed(2000);
      HttpResponse resp = HttpResponse();
      resp.status = 200;
      resp.body = "Ok";
      resp.headers["Content-Type"] = "text/plain";
      return resp;
    });
    // Simulate a button double press
    this->addHttpHandler("/btn-double-press", [this](WiFiClient* client, HttpRequest req) {
      this->onBtnDoublePressed();
      HttpResponse resp = HttpResponse();
      resp.status = 200;
      resp.body = "Ok";
      resp.headers["Content-Type"] = "text/plain";
      return resp;
    });
  
    // Set the state of the robot
    this->addHttpHandler("/set-state", [this](WiFiClient* client, HttpRequest req) {
      String response_str = "";
      if (req.query.find("state") != req.query.end()) {
        int state = req.query["state"].toInt();
        this->state->setState(state);
        response_str += "Set State to: " + String(state) + "\n";
      } else {
        response_str += "No state provided - please add a state query param: ?state=<val>\n";
      }
      HttpResponse resp = HttpResponse();
      resp.status = 200;
      resp.body = response_str;
      resp.headers["Content-Type"] = "text/plain";
      return resp;
    });
  
    // Set the state of the LEDs
    this->addHttpHandler("/set-led", [this](WiFiClient* client, HttpRequest req) {
      String response_str = "";
  
      if (req.query.find("all") != req.query.end()) {
        String key = String("all");
        bool on = req.query[key] == "true" || req.query[key] == "1";
        for (int i = 0; i <= 15; i++) {
          this->setLED(i, on);
        }
        response_str += "All LEDs: " + String(on ? "ON" : "OFF") + "\n";
      }
  
      for (int i = 0; i <= 15; i++) {
        String key = String(i);
        if (req.query.find(key) != req.query.end()) {
          bool on = req.query[key] == "true" || req.query[key] == "1";
          this->setLED(i, on);
          response_str += "LED " + key + ": " + String(on ? "ON" : "OFF") + "\n";
        }
      }
  
      HttpResponse resp = HttpResponse();
      resp.status = 200;
      resp.body = response_str;
      resp.headers["Content-Type"] = "text/plain";
      return resp;
    });
  
    // Set motor speed
    this->addHttpHandler("/set-motor-speed", [this](WiFiClient* client, HttpRequest req) {
      String response_str = "";
  
      if (req.query.find("left") != req.query.end()) {
        int speed = req.query["left"].toInt();
        this->setMotorSpeedL(speed);
        response_str += "Left Speed: " + String(speed) + "\n";
      }
  
      if (req.query.find("right") != req.query.end()) {
        int speed = req.query["right"].toInt();
        this->setMotorSpeedR(speed);
        response_str += "Right Speed: " + String(speed) + "\n";
      }
  
      HttpResponse resp = HttpResponse();
      resp.status = 200;
      resp.body = response_str;
      resp.headers["Content-Type"] = "text/plain";
      return resp;
    });
  
    // Start the baselining process
    this->addHttpHandler("/start-baseline", [this](WiFiClient* client, HttpRequest req) {
      this->startBaselining();
      HttpResponse resp = HttpResponse();
      resp.status = 200;
      resp.body = "Ok - put the car on the ground and press the button to start baseline process";
      resp.headers["Content-Type"] = "text/plain";
      return resp;
    });
  
    // Set the IR Baseline Values
    this->addHttpHandler("/set-ir-baselines", [this](WiFiClient* client, HttpRequest req) {
      if (req.query.find("1") != req.query.end()) {
        this->ir1_baseline = req.query["1"].toInt();
      }
      if (req.query.find("2") != req.query.end()) {
        this->ir2_baseline = req.query["2"].toInt();
      }
      if (req.query.find("3") != req.query.end()) {
        this->ir3_baseline = req.query["3"].toInt();
      }
      if (req.query.find("4") != req.query.end()) {
        this->ir4_baseline = req.query["4"].toInt();
      }
      
      HttpResponse resp = HttpResponse();  
      resp.status = 200;
      resp.body = "ok";
      resp.headers["Content-Type"] = "text/plain";
      return resp;
    });
  
    // Set the IR HARD Threshold Values
    this->addHttpHandler("/set-ir-hard-thresholds", [this](WiFiClient* client, HttpRequest req) {
      if (req.query.find("1") != req.query.end()) {
        this->state->ir1_hard_threshold = req.query["1"].toInt();
      }
      if (req.query.find("2") != req.query.end()) {
        this->state->ir2_hard_threshold = req.query["2"].toInt();
      }
      if (req.query.find("3") != req.query.end()) {
        this->state->ir3_hard_threshold = req.query["3"].toInt();
      }
      if (req.query.find("4") != req.query.end()) {
        this->state->ir4_hard_threshold = req.query["4"].toInt();
      }
      
      HttpResponse resp = HttpResponse();  
      resp.status = 200;
      resp.body = "ok";
      resp.headers["Content-Type"] = "text/plain";
      return resp;
    });
  
    // Set the IR SOFT Threshold Values
    this->addHttpHandler("/set-ir-soft-thresholds", [this](WiFiClient* client, HttpRequest req) {
      if (req.query.find("1") != req.query.end()) {
        this->state->ir1_soft_threshold = req.query["1"].toInt();
      }
      if (req.query.find("2") != req.query.end()) {
        this->state->ir2_soft_threshold = req.query["2"].toInt();
      }
      if (req.query.find("3") != req.query.end()) {
        this->state->ir3_soft_threshold = req.query["3"].toInt();
      }
      if (req.query.find("4") != req.query.end()) {
        this->state->ir4_soft_threshold = req.query["4"].toInt();
      }
      
      HttpResponse resp = HttpResponse();  
      resp.status = 200;
      resp.body = "ok";
      resp.headers["Content-Type"] = "text/plain";
      return resp;
    });
}

#endif // HUB_LINE_ROBOT_ROUTES_H