#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// Front
// Front-Left
#define PIN_FLD 25
#define PIN_FLM 14
#define PIN_FLT 26
// Front-Left misalignments
#define FLD_MIS 0
#define FLM_MIS 0
#define FLT_MIS 0

// Front-Right
#define PIN_FRD 33
#define PIN_FRM 32
#define PIN_FRT 18
// Front-Right misalignments
#define FRD_MIS -8
#define FRM_MIS 0
#define FRT_MIS 0


// Rear
// Rear-Left
#define PIN_RLD 16
#define PIN_RLM 04
#define PIN_RLT 27
// Rear-Left misalignments
#define RLD_MIS -8
#define RLM_MIS 0
#define RLT_MIS -10

// Rear-Right
#define PIN_RRD 17
#define PIN_RRM 05
#define PIN_RRT 19
// Rear-Right misalignments
#define RRD_MIS 0
#define RRM_MIS 0
#define RRT_MIS -4

// LiDAR servo
#define PIN_DAR 13
#define DAR_MIS 0

// Leg variables (note that both of these values are in mm)
#define DOWNER_JOINT 75.567
#define UPPER_JOINT 54.852


class Point {
private:
  float x;  // X coordinate
  float y;  // Y coordinate

public:
  // Constructor
  Point(float xCoord = 0.0, float yCoord = 0.0) {
    x = xCoord;
    y = yCoord;
  }

  // Copy constructor
  Point(const Point& p) {
    x = p.x;
    y = p.y;
  }

  // Assignment operator
  Point& operator=(const Point& p) {
    if (this == &p) {
      return *this;  // Self-assignment check
    }
    x = p.x;
    y = p.y;
    return *this;
  }

  // Method to set the coordinates
  void setCoordinates(float xCoord, float yCoord) {
    x = xCoord;
    y = yCoord;
  }

  // Method to get the X coordinate
  float getX() {
    return x;
  }

  // Method to get the Y coordinate
  float getY() {
    return y;
  }

  // Method to print the coordinates
  void printCoordinates() {
    Serial.print("X: ");
    Serial.print(x);
    Serial.print(", Y: ");
    Serial.println(y);
  }
};

class Vector2D {
private:
  float x;  // X component
  float y;  // Y component

public:
  // Constructor
  Vector2D(float xComp = 0.0, float yComp = 0.0) {
    x = xComp;
    y = yComp;
  }

  Vector2D(Point point_one, Point point_two) {
    x = point_two.getX() - point_one.getX();
    y = point_two.getY() - point_one.getY();
  }

  // Copy constructor
  Vector2D(const Vector2D& v) {
    x = v.x;
    y = v.y;
  }

  // Assignment operator
  Vector2D& operator=(const Vector2D& v) {
    if (this == &v) {
      return *this;  // Self-assignment check
    }
    x = v.x;
    y = v.y;
    return *this;
  }

  // Method to set the components
  void setComponents(float xComp, float yComp) {
    x = xComp;
    y = yComp;
  }

  // Method to get the X component
  float getX() const {
    return x;
  }

  // Method to get the Y component
  float getY() const {
    return y;
  }

  // Method to add another vector
  Vector2D add(const Vector2D& v) const {
    return Vector2D(x + v.x, y + v.y);
  }

  // Method to subtract another vector
  Vector2D subtract(const Vector2D& v) const {
    return Vector2D(x - v.x, y - v.y);
  }

  // Method to multiply by a scalar
  Vector2D multiply(float scalar) const {
    return Vector2D(x * scalar, y * scalar);
  }

  // Method to calculate the dot product with another vector
  float dot(const Vector2D& v) const {
    return x * v.x + y * v.y;
  }

  // Method to calculate the magnitude (length) of the vector
  float magnitude() const {
    return sqrt(x * x + y * y);
  }

  // Method to print the components
  void printComponents() const {
    Serial.print("X: ");
    Serial.print(x);
    Serial.print(", Y: ");
    Serial.println(y);
  }
};

// Variable constants for lengths of legs (complete, moved legs, not parts)
float front_leg_length = 105;
float back_leg_length = 113;

// Variable constant for step size aka. length of the step (note this value is in mm)
float step_size = 20;

// Front
// Front-Left
Servo fld;
Servo flm;
Servo flt;

// Front-Right
Servo frd;
Servo frm;
Servo frt;

// Rear
// Rear-Left
Servo rld;
Servo rlm;
Servo rlt;

// Rear-Right
Servo rrd;
Servo rrm;
Servo rrt;

// LiDAR
Servo dar;

// Set your access point credentials
const char* ssid = "ESP32-Access-Point";
const char* password = "123456789";

// Create a web server on port 80
WebServer server(80);

// Create an instance of the VL53L0X class
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Right side
float r_point_a_angle = 0;
float r_point_b_angle = 0;
float r_point_c_angle = 0;
float r_point_d_angle = 0;
float r_point_e_angle = 0;
float r_point_f_angle = 0;

float r_a_angle = 0;
float r_b_angle = 0;
float r_c_angle = 0;
float r_d_angle = 0;
float r_x_angle = 0;
float r_y_angle = 0;
float r_z_angle = 0;
float r_p1_angle = 0;
float r_p2_angle = 0;

float r_ratio_back_leg = 0;
float r_ratio_front_leg = 0;

// Left side
float l_point_a_angle = 0;
float l_point_b_angle = 0;
float l_point_c_angle = 0;
float l_point_d_angle = 0;
float l_point_e_angle = 0;
float l_point_f_angle = 0;

float l_a_angle = 0;
float l_b_angle = 0;
float l_c_angle = 0;
float l_d_angle = 0;
float l_x_angle = 0;
float l_y_angle = 0;
float l_z_angle = 0;
float l_p1_angle = 0;
float l_p2_angle = 0;

float l_ratio_back_leg = 0;
float l_ratio_front_leg = 0;

//Overall
float point_a_angle = 0;
float point_b_angle = 0;
float point_c_angle = 0;
float point_d_angle = 0;
float point_e_angle = 0;
float point_f_angle = 0;

float a_angle = 0;
float b_angle = 0;
float c_angle = 0;
float d_angle = 0;
float x_angle = 0;
float y_angle = 0;
float z_angle = 0;
float p1_angle = 0;
float p2_angle = 0;

float ratio_back_leg = 0;
float ratio_front_leg = 0;

// Points
Point point_back;
Point point_front;
Point target_point_back;
Point target_point_front;

// Vector
Vector2D floor_vector;

// LiDAR data stored in the array -> row: 0 = angle, 1 =  distance measured
int dar_sense[2][182];


void setup() {
  Serial.begin(115200);  // Initialize serial communication for debugging

  while (!Serial) {
    delay(1);  // Wait for the serial port to connect.
  }

  fld.setPeriodHertz(50);
  fld.attach(PIN_FLD, 500, 2400);

  flm.setPeriodHertz(50);
  flm.attach(PIN_FLM, 500, 2400);

  flt.setPeriodHertz(50);
  flt.attach(PIN_FLT, 500, 2400);

  frd.setPeriodHertz(50);
  frd.attach(PIN_FRD, 500, 2400);

  frm.setPeriodHertz(50);
  frm.attach(PIN_FRM, 500, 2400);

  frt.setPeriodHertz(50);
  frt.attach(PIN_FRT, 500, 2400);

  rld.setPeriodHertz(50);
  rld.attach(PIN_RLD, 500, 2400);

  rlm.setPeriodHertz(50);
  rlm.attach(PIN_RLM, 500, 2400);

  rlt.setPeriodHertz(50);
  rlt.attach(PIN_RLT, 500, 2400);

  rrd.setPeriodHertz(50);
  rrd.attach(PIN_RRD, 500, 2400);

  rrm.setPeriodHertz(50);
  rrm.attach(PIN_RRM, 500, 2400);

  rrt.setPeriodHertz(50);
  rrt.attach(PIN_RRT, 500, 2400);

  dar.setPeriodHertz(50);
  dar.attach(PIN_DAR, 500, 2400);

  // Set up access point
  WiFi.softAP(ssid, password);

  // Print the IP address
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  // Define the handler for the root path
  server.on("/", []() {
    server.send(200, "text/plain", "ESP32 is running");
  });

  // Define the handler for the command path
  server.on("/command", HTTP_POST, handleCommand);

  // Start the server
  server.begin();
  Serial.println("Server started");



  // Initialize the VL53L0X sensor
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    Serial.println(F("Retrying..."));
    /*while (!lox.begin()) {
      delay(1);
    }*/
  }
  Serial.println(F("VL53L0X ready!"));

  delay(2000);
  setTops();

  restLayingPositionAuto(90);

  delay(1000);

  moveLegDiagonal(front_leg_length, 90 + 20, &fld, &flm, FLD_MIS, FLM_MIS, false);
  moveLegDiagonal(front_leg_length, 90 + -20, &frd, &frm, FRD_MIS, FRM_MIS, true);
  moveLegDiagonal(back_leg_length, 90 + 65, &rld, &rlm, RLD_MIS, RLM_MIS, false);
  moveLegDiagonal(back_leg_length, 90 + -65, &rrd, &rrm, RRD_MIS, RRM_MIS, true);

  rrt.write(100 + RRT_MIS);
  rlt.write(80 + RLT_MIS);
  frt.write(100 + FRT_MIS);
  flt.write(80 + FLT_MIS);

  delay(2000);

  getNextPoint(0);
}

void loop() {
  server.handleClient();
}

// Function to handle incoming commands
void handleCommand() {
  if (server.hasArg("command")) {
    String command = server.arg("command");
    Serial.println("Received command: " + command);

    // Handle different commands
    if (command == "left") {
      Serial.println("Turning left");
      getNextPoint(2);
    } else if (command == "right") {
      Serial.println("Turning right");
      getNextPoint(1);
    } else if (command == "forward") {
      Serial.println("Moving forward");
      getNextPoint(0);
    } else if (command == "backward") {
      Serial.println("Moving backward");
      getNextPoint(3);
    } else if (command == "lay") {
      Serial.println("Laying down");
      restLayingPositionAuto(90);
    } else if (command == "stand") {
      Serial.println("Standing up");
      moveLegDiagonal(front_leg_length, 90 + 20, &fld, &flm, FLD_MIS, FLM_MIS, false);
      moveLegDiagonal(front_leg_length, 90 + -20, &frd, &frm, FRD_MIS, FRM_MIS, true);
      moveLegDiagonal(back_leg_length, 90 + 65, &rld, &rlm, RLD_MIS, RLM_MIS, false);
      moveLegDiagonal(back_leg_length, 90 + -65, &rrd, &rrm, RRD_MIS, RRM_MIS, true);

      rrt.write(100 + RRT_MIS);
      rlt.write(80 + RLT_MIS);
      frt.write(100 + FRT_MIS);
      flt.write(80 + FLT_MIS);
    } else if (command == "sit") {
      Serial.println("Sitting down");
      restSittingPosition(90);
    } else if (command == "dar") {
      String array_return = "";
      Serial.println("DaRing");
      for (int i = 0; i <= 180; i++) {
        VL53L0X_RangingMeasurementData_t measure;

        dar.write(i);
        // Perform a ranging measurement
        lox.rangingTest(&measure, false);

        // Print the measured distance
        if (measure.RangeStatus != 4 && measure.RangeMilliMeter < 6000) {  // 4 means out of range, more than 2m seems unreliable, feel free to experiment though
          Serial.print("Distance (mm): ");
          Serial.println(measure.RangeMilliMeter);
          dar_sense[0][i] = i;
          dar_sense[1][i] = measure.RangeMilliMeter;
        } else {
          Serial.println("Out of range");
        }
      }

      for (int j = 0; j <= 180; j++) {
        Serial.print(dar_sense[0][j]);
        Serial.print(", ");
        Serial.print(dar_sense[1][j]);
        Serial.println("");
        array_return += dar_sense[1][j];
        array_return += ",";
      }

      dar.write(90);
      server.send(200, "text/plain", array_return);
    } else if (command == "get_up_left") {
      Serial.println("Getting up on the left");
      recoverLeft();
    } else if (command == "get_up_right") {
      Serial.println("Getting up on the right");
      recoverRight();
    } else {
      Serial.println("Unknown command");
    }

    server.send(200, "text/plain", "Command received");
  } else {
    server.send(400, "text/plain", "Command not found");
  }
}

void setTops() {
  flt.write(90 + FLT_MIS);
  frt.write(90 + FRT_MIS);
  rlt.write(90 + RLT_MIS);
  rrt.write(90 + RRT_MIS);
}

void getAnglesRight() {
  r_point_a_angle = 180 - rrm.read();
  r_point_b_angle = frm.read();

  r_ratio_back_leg = (110) / (sin((270 - rrd.read() + RRD_MIS) * (PI / 180.0)));
  r_z_angle = asin((DOWNER_JOINT) / (r_ratio_back_leg)) * (180.0 / PI);
  r_x_angle = r_point_a_angle - r_z_angle;
  r_y_angle = 180 - r_z_angle - (270 - rrd.read() + RRD_MIS);

  r_a_angle = r_point_b_angle;
  r_point_c_angle = 270 - frd.read() + FRD_MIS;
  r_b_angle = 360 - r_point_c_angle;
  r_ratio_front_leg = (105) / (sin((270 - frd.read() + FRD_MIS) * (PI / 180.0)));
  r_c_angle = asin((DOWNER_JOINT) / (r_ratio_front_leg)) * (180.0 / PI);
  r_d_angle = r_a_angle + r_c_angle;
}

void getAnglesLeft() {
  l_point_a_angle = rlm.read();
  l_point_b_angle = 180 - flm.read();

  l_ratio_back_leg = (110) / (sin((90 + rld.read() - RLD_MIS) * (PI / 180.0)));
  l_z_angle = asin((DOWNER_JOINT) / (l_ratio_back_leg)) * (180.0 / PI);
  l_x_angle = l_point_a_angle - l_z_angle;
  l_y_angle = 180 - l_z_angle - (90 + rld.read() - RLD_MIS);

  l_a_angle = l_point_b_angle;
  l_point_c_angle = 90 + fld.read() - FLD_MIS;
  l_b_angle = 360 - l_point_c_angle;
  l_ratio_front_leg = (105) / (sin((90 + fld.read() - FLD_MIS) * (PI / 180.0)));
  l_c_angle = asin((DOWNER_JOINT) / (l_ratio_front_leg)) * (180.0 / PI);
  l_d_angle = l_a_angle + l_c_angle;
}

void getAnglesAll() {
  getAnglesRight();
  getAnglesLeft();

  point_a_angle = (r_point_a_angle + l_point_a_angle) / 2;
  point_b_angle = (r_point_b_angle + l_point_b_angle) / 2;
  point_c_angle = (r_point_c_angle + l_point_c_angle) / 2;
  point_d_angle = (r_point_d_angle + l_point_d_angle) / 2;
  point_e_angle = (r_point_e_angle + l_point_e_angle) / 2;
  point_f_angle = (r_point_f_angle + l_point_f_angle) / 2;

  a_angle = (r_a_angle + l_a_angle) / 2;
  b_angle = (r_b_angle + l_b_angle) / 2;
  c_angle = (r_c_angle + l_c_angle) / 2;
  d_angle = (r_d_angle + l_d_angle) / 2;
  x_angle = (r_x_angle + l_x_angle) / 2;
  y_angle = (r_y_angle + l_y_angle) / 2;
  z_angle = (r_z_angle + l_z_angle) / 2;
  p1_angle = (r_p1_angle + l_p1_angle) / 2;
  p2_angle = (r_p2_angle + l_p2_angle) / 2;

  ratio_back_leg = (r_ratio_back_leg + l_ratio_back_leg) / 2;
  ratio_front_leg = (r_ratio_front_leg + l_ratio_front_leg) / 2;
}

/*
* @hypotenuse = input height the leg is set to
* @angle = input full angle between main body and imaginary line connecting middle servo and tip of the bottom part of the leg
* @body_section = input false if front, input true if back
*
* @return = returns coordinates of the point
*/
Point getCoords(float hypotenuse, float angle, bool body_section) {
  float side_a = 0;
  float side_b = 0;

  // the origin is set at the summit of the rear leg's middle servo, all coords are set accordingly (note that length of the body is 117mm)

  if (body_section) {
    if (angle >= 90) {
      angle = angle - 90;
      side_a = sin((90 - angle) * (PI / 180.0)) * hypotenuse;
      side_b = sin(angle * (PI / 180.0)) * hypotenuse;
      Point point_back_more(-side_b, -side_a);
      return point_back_more;
    }
    // since the two lines are parallel and the angle is at the point of intersection with line crossing both parallels, I can reassign the angle later and use it as is
    side_a = sin((angle) * (PI / 180.0)) * hypotenuse;
    angle = 90 - angle;
    side_b = sin(angle * (PI / 180.0)) * hypotenuse;
    Point point_back_less(side_b, -side_a);
    return point_back_less;
  }

  if (angle >= 90) {
    angle = angle - 90;
    side_a = sin((90 - angle) * (PI / 180.0)) * hypotenuse;
    side_b = sin(angle * (PI / 180.0)) * hypotenuse;
    Point point_back_more(117 + side_b, -side_a);
    return point_back_more;
  }
  // since the two lines are parallel and the angle is at the point of intersection with line crossing both parallels, I can reassign the angle later and use it as is
  side_a = sin((angle) * (PI / 180.0)) * hypotenuse;
  angle = 90 - angle;
  side_b = sin(angle * (PI / 180.0)) * hypotenuse;
  Point point_back_less(117 - side_b, -side_a);
  return point_back_less;
}

void getFloorLine() {
  getAnglesAll();

  point_back = getCoords(back_leg_length, x_angle, true);
  point_front = getCoords(front_leg_length, d_angle, false);

  floor_vector = Vector2D(point_back, point_front);
}

/*
* @type = 0 for walking forward, 1 for turning right, 2 for turning left
*/
void getNextPoint(int type) {
  getFloorLine();
  float the_unknown = (step_size) / (sqrt(pow(floor_vector.getX(), 2) + pow(floor_vector.getY(), 2)));

  target_point_back = Point(point_back.getX() + floor_vector.getX() * the_unknown, point_back.getY() + floor_vector.getY() * the_unknown);
  target_point_front = Point(point_front.getX() + floor_vector.getX() * the_unknown, point_front.getY() + floor_vector.getY() * the_unknown);
  Serial.println("Original --------------------");
  point_back.printCoordinates();
  point_front.printCoordinates();
  Serial.println("Target ----------------------");
  target_point_back.printCoordinates();
  target_point_front.printCoordinates();

  float back_target_length = sqrt(pow(target_point_back.getX(), 2) + pow(target_point_back.getY(), 2));
  float front_target_length = sqrt(pow(target_point_front.getX() - 117, 2) + pow(target_point_front.getY(), 2));


  Serial.println(back_target_length);
  Serial.println(front_target_length);

  float back_servo_middler_angle = ((pow(step_size, 2) - pow(back_leg_length, 2) - pow(back_target_length, 2)) / (-2 * back_leg_length * back_target_length));
  float front_servo_middler_angle = ((pow(step_size, 2) - pow(front_leg_length, 2) - pow(front_target_length, 2)) / (-2 * front_leg_length * front_target_length));

  if (back_servo_middler_angle < -1.0) {
    back_servo_middler_angle = back_servo_middler_angle + 1;
  } else {
    if (back_servo_middler_angle > 1.0) {
      back_servo_middler_angle = back_servo_middler_angle - 1;
    }
  }

  if (front_servo_middler_angle < -1.0) {
    front_servo_middler_angle = front_servo_middler_angle + 1;
  } else {
    if (front_servo_middler_angle > 1.0) {
      front_servo_middler_angle = front_servo_middler_angle - 1;
    }
  }

  back_servo_middler_angle = acos(back_servo_middler_angle) * (180.0 / PI);
  front_servo_middler_angle = acos(front_servo_middler_angle) * (180.0 / PI);

  float back_servo_middler_angle_change = rrm.read() - back_servo_middler_angle;
  float front_servo_middler_angle_change = frm.read() - front_servo_middler_angle;

  float back_middle_length = back_target_length - 15;
  float front_middle_length = front_target_length - 15;

  switch (type) {
    case 0:
      moveLegsFwd(back_middle_length, front_middle_length, back_servo_middler_angle_change, front_servo_middler_angle_change, back_target_length, front_target_length);
      break;
    case 1:
      turnRight(back_middle_length, front_middle_length, back_servo_middler_angle_change, front_servo_middler_angle_change, back_target_length, front_target_length);
      break;
    case 2:
      turnLeft(back_middle_length, front_middle_length, back_servo_middler_angle_change, front_servo_middler_angle_change, back_target_length, front_target_length);
      break;
    case 3:
      moveLegsBkwd(back_middle_length, front_middle_length, back_servo_middler_angle_change, front_servo_middler_angle_change, back_target_length, front_target_length);
      break;
  }
}

// decides whether lizarde cannot go forward based on LiDAR measurements
int stuck_decider(int range_low, int range_high) {
  int counter = 0;

  for (int i = range_low; i < range_high; i++) {
    if (dar_sense[1][i] < 100) {
      counter++;
    }
  }

  return counter;
}

void moveLegsFwd(float back_middle_length, float front_middle_length, float back_servo_middler_angle_change, float front_servo_middler_angle_change, float back_target_length, float front_target_length) {
  bool go_left = true;
  bool go_right = true;
  bool go_fwd = true;

  VL53L0X_RangingMeasurementData_t measure_preliminary;

  // Perform a ranging measurement
  lox.rangingTest(&measure_preliminary, false);

  // Print the measured distance
  if (measure_preliminary.RangeStatus != 4 && measure_preliminary.RangeMilliMeter < 100) {  // 4 means out of range, more than 2m seems unreliable, feel free to experiment though
    Serial.print("Distance (mm): ");
    Serial.println(measure_preliminary.RangeMilliMeter);

    for (int i = 0; i <= 180; i++) {
      VL53L0X_RangingMeasurementData_t measure;

      dar.write(i);
      // Perform a ranging measurement
      lox.rangingTest(&measure, false);

      // Print the measured distance
      if (measure.RangeStatus != 4 && measure.RangeMilliMeter < 6000) {  // 4 means out of range, more than 2m seems unreliable, feel free to experiment though
        Serial.print("Distance (mm): ");
        Serial.println(measure.RangeMilliMeter);
        dar_sense[0][i] = i;
        dar_sense[1][i] = measure.RangeMilliMeter;
      } else {
        Serial.println("Out of range");
      }
    }

    int counter_left = stuck_decider(0, 60);
    int counter_right = stuck_decider(60, 120);
    int counter_fwd = stuck_decider(120, 180);

    if (counter_left > 20) {
      go_left = false;
    }

    if (counter_right > 20) {
      go_right = false;
    }

    if (counter_fwd > 20) {
      go_fwd = false;
    }

    String server_response = "";
    server_response += go_left;
    server_response += ",";
    server_response += go_right;
    server_response += ",";
    server_response += go_fwd;
    server_response += ",";

    server.send(200, "text/plain", server_response);

    dar.write(90);
  } else {
    Serial.println("Out of range");
  }

  if (go_fwd) {
    // 0 move to original position
    moveLegDiagonal(front_leg_length, 90 + -20, &frd, &frm, FRD_MIS, FRM_MIS, true);
    moveLegDiagonal(back_leg_length, 90 + 60, &rld, &rlm, RLD_MIS, RLM_MIS, false);

    // 1 move to original position
    moveLegDiagonal(front_leg_length, 90 + 20, &fld, &flm, FLD_MIS, FLM_MIS, false);
    moveLegDiagonal(back_leg_length, 90 + -60, &rrd, &rrm, RRD_MIS, RRM_MIS, true);

    // start of walking

    rrt.write(100 + RRT_MIS);
    rlt.write(80 + RLT_MIS);
    frt.write(100 + FRT_MIS);
    flt.write(80 + FLT_MIS);

    // defining middle positions for travel - note that final position of travel is point_back and point_front respectively
    Point point_middle_back(point_back.getX() + step_size / 2, point_back.getY() + step_size);
    Point point_middle_front(point_front.getX() + step_size / 2, point_front.getY() + step_size);
    // calculating distance to middle position from leg origin
    float length_middle_back = sqrt(pow(point_middle_back.getX(), 2) + pow(point_middle_back.getY(), 2));
    float length_middle_front = sqrt(pow(point_middle_front.getX() - 117, 2) + pow(point_middle_front.getY(), 2));

    // future position already defined by target lengths

    // defining return positions for travel - note that final position of travel is point_back and point_front respectively
    Point point_return_back(point_back.getX() + step_size / 2, point_back.getY() - step_size);
    Point point_return_front(point_front.getX() + step_size / 2, point_front.getY() - step_size);
    // calculating distance to middle position from leg origin
    float length_return_back = sqrt(pow(point_return_back.getX(), 2) + pow(point_return_back.getY(), 2));
    float length_return_front = sqrt(pow(point_return_front.getX() - 117, 2) + pow(point_return_front.getY(), 2));

    // 0 -----------------------
    // 0 move to middle position
    moveLegDiagonal(length_middle_front, frm.read() + (front_servo_middler_angle_change / 2), &frd, &frm, FRD_MIS, 0, true);
    moveLegDiagonal(length_middle_back, rlm.read() - (back_servo_middler_angle_change / 2), &rld, &rlm, RLD_MIS, 0, false);

    delay(320);

    // 0 move to future position
    moveLegDiagonal(front_target_length, frm.read() + (front_servo_middler_angle_change / 2), &frd, &frm, FRD_MIS, 0, true);
    moveLegDiagonal(back_target_length, rlm.read() - (back_servo_middler_angle_change / 2), &rld, &rlm, RLD_MIS, 0, false);

    delay(320);

    // 0 move to original position
    moveLegDiagonal(length_return_front, frm.read() - (front_servo_middler_angle_change / 2), &frd, &frm, FRD_MIS, FRM_MIS, true);
    moveLegDiagonal(length_return_back, rlm.read() + (back_servo_middler_angle_change / 2), &rld, &rlm, RLD_MIS, RLM_MIS, false);

    delay(320);

    // 0 move to original position
    moveLegDiagonal(front_leg_length, 90 + -20, &frd, &frm, FRD_MIS, FRM_MIS, true);
    moveLegDiagonal(back_leg_length, 90 + 60, &rld, &rlm, RLD_MIS, RLM_MIS, false);

    delay(320);

    // 1 -----------------------
    // 1 move to middle position
    moveLegDiagonal(length_middle_front, flm.read() - (front_servo_middler_angle_change / 2), &fld, &flm, FLD_MIS, 0, false);
    moveLegDiagonal(length_middle_back, rrm.read() + (back_servo_middler_angle_change / 2), &rrd, &rrm, RRD_MIS, 0, true);

    delay(320);

    // 1 move to future position
    moveLegDiagonal(front_target_length, flm.read() - (front_servo_middler_angle_change / 2), &fld, &flm, FLD_MIS, 0, false);
    moveLegDiagonal(back_target_length, rrm.read() + (back_servo_middler_angle_change / 2), &rrd, &rrm, RRD_MIS, 0, true);

    delay(320);

    // 1 move to original position + 10
    moveLegDiagonal(length_return_front, flm.read() + (front_servo_middler_angle_change / 2), &fld, &flm, FLD_MIS, FLM_MIS, false);
    moveLegDiagonal(length_return_back, rrm.read() - (back_servo_middler_angle_change / 2), &rrd, &rrm, RRD_MIS, RRM_MIS, true);

    delay(320);

    // 1 move to original position
    moveLegDiagonal(front_leg_length, 90 + 20, &fld, &flm, FLD_MIS, FLM_MIS, false);
    moveLegDiagonal(back_leg_length, 90 + -60, &rrd, &rrm, RRD_MIS, RRM_MIS, true);

    delay(320);
  }
}

void moveLegsBkwd(float back_middle_length, float front_middle_length, float back_servo_middler_angle_change, float front_servo_middler_angle_change, float back_target_length, float front_target_length) {
  frm.write(90 + FRM_MIS);
  flm.write(90 + FLM_MIS);
  rlm.write(90 + RLM_MIS);
  rrm.write(90 + RRM_MIS);

  frd.write(90 + FRD_MIS);
  fld.write(90 + FLD_MIS);
  rld.write(90 + RLD_MIS);
  rrd.write(90 + RRD_MIS);

  moveLegDiagonalReverse(front_leg_length, 90 + 60, &frd, &frm, FRD_MIS, FRM_MIS, true);
  moveLegDiagonalReverse(front_leg_length, 90 + -60, &fld, &flm, FLD_MIS, FLM_MIS, false);
  moveLegDiagonalReverse(back_leg_length, 90 + -20, &rld, &rlm, RLD_MIS, RLM_MIS, false);
  moveLegDiagonalReverse(back_leg_length, 90 + 20, &rrd, &rrm, RRD_MIS, RRM_MIS, true);

  rrt.write(100 + RRT_MIS);
  rlt.write(80 + RLT_MIS);
  frt.write(100 + FRT_MIS);
  flt.write(80 + FLT_MIS);

  // defining middle positions for travel - note that final position of travel is point_back and point_front respectively
  Point point_middle_back(point_back.getX() + step_size / 2, point_back.getY() + step_size);
  Point point_middle_front(point_front.getX() + step_size / 2, point_front.getY() + step_size);
  // calculating distance to middle position from leg origin
  float length_middle_back = sqrt(pow(point_middle_back.getX(), 2) + pow(point_middle_back.getY(), 2));
  float length_middle_front = sqrt(pow(point_middle_front.getX() - 117, 2) + pow(point_middle_front.getY(), 2));

  // future position already defined by target lengths

  // defining return positions for travel - note that final position of travel is point_back and point_front respectively
  Point point_return_back(point_back.getX() + step_size / 2, point_back.getY() - step_size);
  Point point_return_front(point_front.getX() + step_size / 2, point_front.getY() - step_size);
  // calculating distance to middle position from leg origin
  float length_return_back = sqrt(pow(point_return_back.getX(), 2) + pow(point_return_back.getY(), 2));
  float length_return_front = sqrt(pow(point_return_front.getX() - 117, 2) + pow(point_return_front.getY(), 2));

  // 0 -----------------------
  // 0 move to middle position
  moveLegDiagonalReverse(length_middle_front, frm.read() - (front_servo_middler_angle_change / 4), &frd, &frm, FRD_MIS, 0, true);
  moveLegDiagonalReverse(length_middle_back, rlm.read() + (back_servo_middler_angle_change / 4), &rld, &rlm, RLD_MIS, 0, false);

  delay(320);

  // 0 move to future position
  moveLegDiagonalReverse(front_target_length, frm.read() - (front_servo_middler_angle_change / 4), &frd, &frm, FRD_MIS, 0, true);
  moveLegDiagonalReverse(back_target_length, rlm.read() + (back_servo_middler_angle_change / 4), &rld, &rlm, RLD_MIS, 0, false);

  delay(320);

  // 0 move to original position
  moveLegDiagonalReverse(length_return_front, frm.read() + (front_servo_middler_angle_change / 4), &frd, &frm, FRD_MIS, FRM_MIS, true);
  moveLegDiagonalReverse(length_return_back, rlm.read() - (back_servo_middler_angle_change / 4), &rld, &rlm, RLD_MIS, RLM_MIS, false);

  delay(320);

  // 0 move to original position
  moveLegDiagonalReverse(back_leg_length + 10, 90 + 60, &frd, &frm, FRD_MIS, FRM_MIS, true);
  moveLegDiagonalReverse(front_leg_length, 90 + -20, &rld, &rlm, RLD_MIS, RLM_MIS, false);

  delay(320);

  // 1 -----------------------
  // 1 move to middle position
  moveLegDiagonalReverse(length_middle_front, flm.read() + (front_servo_middler_angle_change / 4), &fld, &flm, FLD_MIS, 0, false);
  moveLegDiagonalReverse(length_middle_back, rrm.read() - (back_servo_middler_angle_change / 4), &rrd, &rrm, RRD_MIS, 0, true);

  delay(320);

  // 1 move to future position
  moveLegDiagonalReverse(front_target_length, flm.read() + (front_servo_middler_angle_change / 4), &fld, &flm, FLD_MIS, 0, false);
  moveLegDiagonalReverse(back_target_length, rrm.read() - (back_servo_middler_angle_change / 4), &rrd, &rrm, RRD_MIS, 0, true);

  delay(320);

  // 1 move to original position + 10
  moveLegDiagonalReverse(length_return_front, flm.read() - (front_servo_middler_angle_change / 4), &fld, &flm, FLD_MIS, FLM_MIS, false);
  moveLegDiagonalReverse(length_return_back, rrm.read() + (back_servo_middler_angle_change / 4), &rrd, &rrm, RRD_MIS, RRM_MIS, true);

  delay(320);

  // 1 move to original position
  moveLegDiagonalReverse(back_leg_length + 10, 90 + -60, &fld, &flm, FLD_MIS, FLM_MIS, false);
  moveLegDiagonalReverse(front_leg_length, 90 + 20, &rrd, &rrm, RRD_MIS, RRM_MIS, true);

  delay(320);
}

// uses 1 at top
void turnLeft(float back_middle_length, float front_middle_length, float back_servo_middler_angle_change, float front_servo_middler_angle_change, float back_target_length, float front_target_length) {
  // 0 move to original position
  moveLegDiagonal(front_leg_length, 90 + -20, &frd, &frm, FRD_MIS, FRM_MIS, true);
  moveLegDiagonal(back_leg_length, 90 + 60, &rld, &rlm, RLD_MIS, RLM_MIS, false);

  // 1 move to original position
  moveLegDiagonal(front_leg_length, 90 + 20, &fld, &flm, FLD_MIS, FLM_MIS, false);
  moveLegDiagonal(back_leg_length, 90 + -60, &rrd, &rrm, RRD_MIS, RRM_MIS, true);

  delay(1320);

  //rrt.write(110 + RRT_MIS);
  //frt.write(110 + FRT_MIS);

  // defining middle positions for travel - note that final position of travel is point_back and point_front respectively
  Point point_middle_back(point_back.getX() + step_size / 2, point_back.getY() + step_size);
  Point point_middle_front(point_front.getX() + step_size / 2, point_front.getY() + step_size);
  // calculating distance to middle position from leg origin
  float length_middle_back = sqrt(pow(point_middle_back.getX(), 2) + pow(point_middle_back.getY(), 2));
  float length_middle_front = sqrt(pow(point_middle_front.getX() - 117, 2) + pow(point_middle_front.getY(), 2));

  // future position already defined by target lengths

  // defining return positions for travel - note that final position of travel is point_back and point_front respectively
  Point point_return_back(point_back.getX() + step_size / 2, point_back.getY() - step_size);
  Point point_return_front(point_front.getX() + step_size / 2, point_front.getY() - step_size);
  // calculating distance to middle position from leg origin
  float length_return_back = sqrt(pow(point_return_back.getX(), 2) + pow(point_return_back.getY(), 2));
  float length_return_front = sqrt(pow(point_return_front.getX() - 117, 2) + pow(point_return_front.getY(), 2));

  // 0 -----------------------
  // 0 move to middle position
  moveLegDiagonal(length_middle_front, frm.read() + (front_servo_middler_angle_change / 2), &frd, &frm, FRD_MIS, 0, true);
  moveLegDiagonal(length_middle_back, rlm.read() + (back_servo_middler_angle_change / 2), &rld, &rlm, RLD_MIS, 0, false);

  delay(320);

  // 0 move to future position
  moveLegDiagonal(front_target_length, frm.read() + (front_servo_middler_angle_change / 2), &frd, &frm, FRD_MIS, 0, true);
  moveLegDiagonal(back_target_length, rlm.read() + (back_servo_middler_angle_change / 2), &rld, &rlm, RLD_MIS, 0, false);

  delay(320);

  // 0 move to original position
  moveLegDiagonal(length_return_front, frm.read() - (front_servo_middler_angle_change / 2), &frd, &frm, FRD_MIS, FRM_MIS, true);
  moveLegDiagonal(length_return_back, rlm.read() - (back_servo_middler_angle_change / 2), &rld, &rlm, RLD_MIS, RLM_MIS, false);

  delay(320);

  // 0 move to original position
  moveLegDiagonal(front_leg_length, 90 + -20, &frd, &frm, FRD_MIS, FRM_MIS, true);
  moveLegDiagonal(back_leg_length, 90 + 60, &rld, &rlm, RLD_MIS, RLM_MIS, false);

  delay(320);

  // 1 -----------------------
  // 1 move to middle position
  moveLegDiagonal(length_middle_front, flm.read() + (front_servo_middler_angle_change / 2), &fld, &flm, FLD_MIS, 0, false);
  moveLegDiagonal(length_middle_back, rrm.read() + (back_servo_middler_angle_change / 2), &rrd, &rrm, RRD_MIS, 0, true);

  delay(320);

  // 1 move to future position
  moveLegDiagonal(front_target_length, flm.read() + (front_servo_middler_angle_change / 2), &fld, &flm, FLD_MIS, 0, false);
  moveLegDiagonal(back_target_length, rrm.read() + (back_servo_middler_angle_change / 2), &rrd, &rrm, RRD_MIS, 0, true);

  delay(320);

  // 1 move to original position + 10
  moveLegDiagonal(length_return_front, flm.read() - (front_servo_middler_angle_change / 2), &fld, &flm, FLD_MIS, FLM_MIS, false);
  moveLegDiagonal(length_return_back, rrm.read() - (back_servo_middler_angle_change / 2), &rrd, &rrm, RRD_MIS, RRM_MIS, true);

  delay(320);

  // 1 move to original position
  moveLegDiagonal(front_leg_length, 90 + 20, &fld, &flm, FLD_MIS, FLM_MIS, false);
  moveLegDiagonal(back_leg_length, 90 + -60, &rrd, &rrm, RRD_MIS, RRM_MIS, true);

  delay(320);
}


// uses 0 at top
void turnRight(float back_middle_length, float front_middle_length, float back_servo_middler_angle_change, float front_servo_middler_angle_change, float back_target_length, float front_target_length) {
  // 0 move to original position
  moveLegDiagonal(front_leg_length, 90 + -20, &frd, &frm, FRD_MIS, FRM_MIS, true);
  moveLegDiagonal(back_leg_length, 90 + 60, &rld, &rlm, RLD_MIS, RLM_MIS, false);

  // 1 move to original position
  moveLegDiagonal(front_leg_length, 90 + 20, &fld, &flm, FLD_MIS, FLM_MIS, false);
  moveLegDiagonal(back_leg_length, 90 + -60, &rrd, &rrm, RRD_MIS, RRM_MIS, true);

  delay(1320);

  //rlt.write(70 + RLT_MIS);
  //flt.write(70 + FLT_MIS);

  // defining middle positions for travel - note that final position of travel is point_back and point_front respectively
  Point point_middle_back(point_back.getX() + step_size / 2, point_back.getY() + step_size);
  Point point_middle_front(point_front.getX() + step_size / 2, point_front.getY() + step_size);
  // calculating distance to middle position from leg origin
  float length_middle_back = sqrt(pow(point_middle_back.getX(), 2) + pow(point_middle_back.getY(), 2));
  float length_middle_front = sqrt(pow(point_middle_front.getX() - 117, 2) + pow(point_middle_front.getY(), 2));

  // future position already defined by target lengths

  // defining return positions for travel - note that final position of travel is point_back and point_front respectively
  Point point_return_back(point_back.getX() + step_size / 2, point_back.getY() - step_size);
  Point point_return_front(point_front.getX() + step_size / 2, point_front.getY() - step_size);
  // calculating distance to middle position from leg origin
  float length_return_back = sqrt(pow(point_return_back.getX(), 2) + pow(point_return_back.getY(), 2));
  float length_return_front = sqrt(pow(point_return_front.getX() - 117, 2) + pow(point_return_front.getY(), 2));

  // 1 -----------------------
  // 1 move to middle position
  moveLegDiagonal(length_middle_front, flm.read() - (front_servo_middler_angle_change / 2), &fld, &flm, FLD_MIS, 0, false);
  moveLegDiagonal(length_middle_back, rrm.read() - (back_servo_middler_angle_change / 2), &rrd, &rrm, RRD_MIS, 0, true);

  delay(320);

  // 1 move to future position
  moveLegDiagonal(front_target_length, flm.read() - (front_servo_middler_angle_change / 2), &fld, &flm, FLD_MIS, 0, false);
  moveLegDiagonal(back_target_length, rrm.read() - (back_servo_middler_angle_change / 2), &rrd, &rrm, RRD_MIS, 0, true);

  delay(320);

  // 1 move to original position + 10
  moveLegDiagonal(length_return_front, flm.read() + (front_servo_middler_angle_change / 2), &fld, &flm, FLD_MIS, FLM_MIS, false);
  moveLegDiagonal(length_return_back, rrm.read() + (back_servo_middler_angle_change / 2), &rrd, &rrm, RRD_MIS, RRM_MIS, true);

  delay(320);

  // 1 move to original position
  moveLegDiagonal(front_leg_length, 90 + 20, &fld, &flm, FLD_MIS, FLM_MIS, false);
  moveLegDiagonal(back_leg_length, 90 + -60, &rrd, &rrm, RRD_MIS, RRM_MIS, true);

  delay(320);

  // 0 -----------------------
  // 0 move to middle position
  moveLegDiagonal(length_middle_front, frm.read() - (front_servo_middler_angle_change / 2), &frd, &frm, FRD_MIS, 0, true);
  moveLegDiagonal(length_middle_back, rlm.read() - (back_servo_middler_angle_change / 2), &rld, &rlm, RLD_MIS, 0, false);

  delay(320);

  // 0 move to future position
  moveLegDiagonal(front_target_length, frm.read() - (front_servo_middler_angle_change / 2), &frd, &frm, FRD_MIS, 0, true);
  moveLegDiagonal(back_target_length, rlm.read() - (back_servo_middler_angle_change / 2), &rld, &rlm, RLD_MIS, 0, false);

  delay(320);

  // 0 move to original position
  moveLegDiagonal(length_return_front, frm.read() + (front_servo_middler_angle_change / 2), &frd, &frm, FRD_MIS, FRM_MIS, true);
  moveLegDiagonal(length_return_back, rlm.read() + (back_servo_middler_angle_change / 2), &rld, &rlm, RLD_MIS, RLM_MIS, false);

  delay(320);

  // 0 move to original position
  moveLegDiagonal(front_leg_length, 90 + -20, &frd, &frm, FRD_MIS, FRM_MIS, true);
  moveLegDiagonal(back_leg_length, 90 + 60, &rld, &rlm, RLD_MIS, RLM_MIS, false);

  delay(320);
}

/*
  moveLegDiagonal(back_middle_length, rrm.read() + back_servo_middler_angle_change, &rrd, &rrm, RRD_MIS, RRM_MIS, true);
  moveLegDiagonal(back_middle_length, rlm.read() - back_servo_middler_angle_change, &rld, &rlm, RLD_MIS, RLM_MIS, false);
  moveLegDiagonal(front_middle_length, frm.read() + front_servo_middler_angle_change, &frd, &frm, FRD_MIS, FRM_MIS, true);
  moveLegDiagonal(front_middle_length, flm.read() - front_servo_middler_angle_change, &fld, &flm, FLD_MIS, FLM_MIS, false);

  moveLegDiagonal(front_leg_length, 90 + 20, &fld, &flm, FLD_MIS, FLM_MIS, false);
  moveLegDiagonal(front_leg_length, 90 + -20, &frd, &frm, FRD_MIS, FRM_MIS, true);
  moveLegDiagonal(back_leg_length, 90 + 50, &rld, &rlm, RLD_MIS, RLM_MIS, false);
  moveLegDiagonal(back_leg_length, 90 + -50, &rrd, &rrm, RRD_MIS, RRM_MIS, true);
*/

/*
* @desired_len = input length of the leg you want in mm (downer servo will accomodate to this length)
* @desired_angle = input angle of the middle servo
* @servo_downer = input the downer servo (name of it ends in d)  -- both of these servos should start in the same two letters e.g. fr
* @servo_middler = input the middle servo (name of it ends in m) -/
* @mis_downer = input misalignment constant for downer servo
* @mis_middler = input misalignment constant for middle servo
* @servo_side = input false if left, input true if right
*/
void moveLegDiagonal(float desired_len, float desired_angle, Servo* servo_downer, Servo* servo_middler, int mis_downer, int mis_middler, bool servo_side) {
  float servo_middler_angle = desired_angle + mis_middler;
  float servo_downer_angle = ((pow(desired_len, 2) - pow(DOWNER_JOINT, 2) - pow(UPPER_JOINT, 2)) / (-2 * DOWNER_JOINT * UPPER_JOINT));
  if (servo_downer_angle < -1.0) {
    servo_downer_angle = servo_downer_angle + 1;
  } else {
    if (servo_downer_angle > 1.0) {
      servo_downer_angle = servo_downer_angle - 1;
    }
  }

  servo_downer_angle = acos(servo_downer_angle) * (180.0 / PI);

  if (servo_side) {
    servo_downer_angle = 270 - servo_downer_angle + mis_downer;
  } else {
    servo_downer_angle = servo_downer_angle + mis_downer - 90;
  }

  servo_downer->write(servo_downer_angle);
  servo_middler->write(servo_middler_angle);
}

/*
* @desired_len = input length of the leg you want in mm (downer servo will accomodate to this length)
* @desired_angle = input angle of the middle servo
* @servo_downer = input the downer servo (name of it ends in d)  -- both of these servos should start in the same two letters e.g. fr
* @servo_middler = input the middle servo (name of it ends in m) -/
* @mis_downer = input misalignment constant for downer servo
* @mis_middler = input misalignment constant for middle servo
* @servo_side = input false if left, input true if right
*/
void moveLegDiagonalReverse(float desired_len, float desired_angle, Servo* servo_downer, Servo* servo_middler, int mis_downer, int mis_middler, bool servo_side) {
  float servo_middler_angle = desired_angle + mis_middler;
  float servo_downer_angle = ((pow(desired_len, 2) - pow(DOWNER_JOINT, 2) - pow(UPPER_JOINT, 2)) / (-2 * DOWNER_JOINT * UPPER_JOINT));
  if (servo_downer_angle < -1.0) {
    servo_downer_angle = servo_downer_angle + 1;
  } else {
    if (servo_downer_angle > 1.0) {
      servo_downer_angle = servo_downer_angle - 1;
    }
  }

  servo_downer_angle = acos(servo_downer_angle) * (180.0 / PI);

  if (servo_side) {
    servo_downer_angle = servo_downer_angle + mis_downer - 90;
  } else {
    servo_downer_angle = 270 - servo_downer_angle + mis_downer;
  }

  servo_downer->write(servo_downer_angle);
  servo_middler->write(servo_middler_angle);
}

void moveAllServosToPosition(int position) {
  fld.write(position + FLD_MIS);
  flm.write(position + FLM_MIS);
  flt.write(position + FLT_MIS);
  frd.write(position + FRD_MIS);
  frm.write(position + FRM_MIS);
  frt.write(position + FRT_MIS);
  rld.write(position + RLD_MIS);
  rlm.write(position + RLM_MIS);
  rlt.write(position + RLT_MIS);
  rrd.write(position + RRD_MIS);
  rrm.write(position + RRM_MIS);
  rrt.write(position + RRT_MIS);
}

void moveAllServosToPositionAuto() {
  int position = fld.read();
  fld.write(position + FLD_MIS);
  position = flm.read();
  flm.write(position + FLM_MIS);
  position = flt.read();
  flt.write(position + FLT_MIS);
  position = frd.read();
  frd.write(position + FRD_MIS);
  position = frm.read();
  frm.write(position + FRM_MIS);
  position = frt.read();
  frt.write(position + FRT_MIS);
  position = rld.read();
  rld.write(position + RLD_MIS);
  position = rlm.read();
  rlm.write(position + RLM_MIS);
  position = rlt.read();
  rlt.write(position + RLT_MIS);
  position = rrd.read();
  rrd.write(position + RRD_MIS);
  position = rrm.read();
  rrm.write(position + RRM_MIS);
  position = rrt.read();
  rrt.write(position + RRT_MIS);
}

void restLayingPosition(int position) {
  int left = position;
  int right = position;
  // fl
  fld.write(position - 90);
  flm.write(position);
  flt.write(position);
  //fr
  frd.write(position - 8 + 90);
  frm.write(90);
  frt.write(position);
  // rl
  rld.write(position - 8 - 90);
  rlm.write(position + 15);
  rlt.write(position - 10);
  // rr
  rrd.write(position + 90);
  rrm.write(position - 4 - 15);
  rrt.write(position - 4);
}

void restLayingPositionAuto(int position) {
  int left = position;
  int right = position;
  // Front-Left
  fld.write(position - 90 + FLD_MIS);
  flm.write(position + FLM_MIS);
  flt.write(position + FLT_MIS);
  // Front-Right
  frd.write(position + 90 + FRD_MIS);
  frm.write(position + FRM_MIS);
  frt.write(position + FRT_MIS);
  // Rear-Left
  rld.write(position - 90 + RLD_MIS);
  rlm.write(position + 15 + RLM_MIS);
  rlt.write(position + RLT_MIS);
  // Rear-Right
  rrd.write(position + 90 + RRD_MIS);
  rrm.write(position - 15 + RRM_MIS);
  rrt.write(position + RRT_MIS);
}

void restSittingPosition(int position) {
  // rl
  rld.write(position - 8 - 60);
  rlm.write(position);
  rlt.write(position - 10);
  // rr
  rrd.write(position + 60);
  rrm.write(position - 4);
  rrt.write(position - 4);
  // fl
  fld.write(position);
  flm.write(position + 25);
  flt.write(position - 5);
  //fr
  frd.write(position - 8);
  frm.write(position - 25);
  frt.write(position + 5);
}

void recoverLeft() {
  int position = 90;

  // rl
  rld.write(position + RLD_MIS);
  rlm.write(position - 90 + RLM_MIS);
  rlt.write(position - 90 + RLT_MIS);
  // rr
  rrd.write(position + 90 + RRD_MIS);
  rrm.write(position + RRM_MIS);
  rrt.write(position + RRT_MIS);
  // fl
  fld.write(position + FLD_MIS);
  flm.write(position - 90 + FLM_MIS);
  flt.write(position - 90 + FLT_MIS);
  //fr
  frd.write(position + 90 + FRD_MIS);
  frm.write(position + FRM_MIS);
  frt.write(position + FRT_MIS);

  delay(3200);

  frt.write(position + 45 + FLT_MIS);
  rrt.write(position + 45 + RLT_MIS);

  delay(3200);

  restLayingPositionAuto(90);
}

void recoverRight() {
  int position = 90;

  // rl
  rld.write(position - 90 + RLD_MIS);
  rlm.write(position + RLM_MIS);
  rlt.write(position + RLT_MIS);
  // rr
  rrd.write(position + RRD_MIS);
  rrm.write(position + 90 + RRM_MIS);
  rrt.write(position + 90 + RRT_MIS);
  // fl
  fld.write(position - 90 + FLD_MIS);
  flm.write(position + FLM_MIS);
  flt.write(position + FLT_MIS);
  //fr
  frd.write(position + FRD_MIS);
  frm.write(position + 90 + FRM_MIS);
  frt.write(position + 90 + FRT_MIS);

  delay(3200);

  flt.write(position - 45 + FLT_MIS);
  rlt.write(position - 45 + RLT_MIS);

  delay(3200);

  restLayingPositionAuto(90);
}