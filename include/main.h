#ifndef main_h // ifndef main_h to prevent double declaration of any identifiers such as types, enums and static variables
#define main_h // ifndef main_h 

// debug setup
#define DEBUG_VAR
#define DEBUG_ACT
//#define DEBUG_FCT
#define DEBUG_STATE

#ifdef DEBUG_VAR
  #define DEBUG_PRINT_VAR(x) Serial.print(x)
  #define DEBUG_PRINTLN_VAR(x) Serial.println(x)
#else
  #define DEBUG_PRINTLN_VAR(x)
  #define DEBUG_PRINT_VAR(x) 
#endif

#ifdef DEBUG_ACT
  #define DEBUG_PRINT_ACT(x) Serial.println(x)
  #define DEBUG_PRINTLN_ACT(x) Serial.println(x)
#else
  #define DEBUG_PRINT_ACT(x)
  #define DEBUG_PRINTLN_ACT(x)
#endif

#ifdef DEBUG_FCT
  #define DEBUG_PRINT_FCT(x) Serial.println(x)
  #define DEBUG_PRINTLN_FCT(x) Serial.println(x)
 #else
  #define DEBUG_PRINTLN_FCT(x)
#endif

#ifdef DEBUG_STATE
  #define DEBUG_PRINTLN_STATE(x) Serial.print(">Machine State: "); Serial.println(x)
#else
  #define DEBUG_PRINTLN_STATE(x)
#endif // debug setup

//pour la gestion du webserver
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>

const char* ssid = "Partage_julien";
const char* password = "poooorus15";

AsyncWebServer server(80);
WebSocketsServer webSocket(81);

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>settings of the robot</title>
</head>
<body>
  <h2>motors speed forward</h2>
  <label>speed left: <input type="number" id="speed_left" step="0.1" value="40"></label><br>
  <label>speed right: <input type="number" id="speed_right" step="0.1" value="55"></label><br>
  <button onclick="send_speed()">Send settings </button>
  
  <h2>motors speed rotation</h2>
  <label>speed left: <input type="number" id="rotation_left" step="0.1" value="40"></label><br>
  <label>speed right: <input type="number" id="rotation_right" step="0.1" value="55"></label><br>
  <button onclick="send_rotation()">Send settings </button>

  <h2>distance</h2>
  <label>distance: <input type="number" id="distance" step="0.1" value="40"></label><br>
  <button onclick="send_distance()">Send settings </button>

  <h2>Distance PID setting</h2>
  <label>Kp: <input type="number" id="kp" step="0.1" value="1.0"></label><br>
  <label>Ki: <input type="number" id="ki" step="0.1" value="0.5"></label><br>
  <label>Kd: <input type="number" id="kd" step="0.1" value="0.1"></label><br><br>
  <button onclick="sendPID()">Send settings </button>

  <h2>Balance PID setting</h2>
  <label>Kp_wheel: <input type="number" id="kp_wheel" step="0.1" value="0.45"></label><br>
  <label>Ki_wheel: <input type="number" id="ki_wheel" step="0.1" value="0.005"></label><br>
  <label>Kd_wheel: <input type="number" id="kd_wheel" step="0.1" value="0.2"></label><br><br>
  <button onclick="sendPID_balance()">Send settings</button>
  <p id="status">En attente...</p>
  <br>
  <table align="center">
    <tr>
      <td></td>
      <td><button onclick="set_command(1)" id="for">forward</button></td>
      <td></td>
    </tr>
    <tr>
      <td><button onclick="set_command(4)" id="rig">left</button></td>
      <td><button onclick="set_command(5)" id="sto">stop</button></td>
      <td><button onclick="set_command(3)" id="lef">right</button></td>
    </tr>
    <tr>
      <td></td>
      <td><button onclick="set_command(2)" id="bac">back</button></td>
      <td></td>
    </tr>
  </table>
  

<script>
  var ws = new WebSocket("ws://" + location.hostname + ":81/");
  ws.onmessage = function(event) {
    document.getElementById("status").innerText = "Réponse: " + event.data;
  };

  function send_speed() {
    var speed_left = parseFloat(document.getElementById("speed_left").value);
    var speed_right = parseFloat(document.getElementById("speed_right").value);
    var msg = JSON.stringify({ speed_left: speed_left, speed_right: speed_right });
    ws.send(msg);
  }

  function send_rotation() {
    var rotation_left = parseFloat(document.getElementById("roation_left").value);
    var rotaion_right = parseFloat(document.getElementById("rotation_right").value);
    var msg = JSON.stringify({ rotation_left: rotation_left, rotation_right: rotation_right });
    ws.send(msg);
  }

  function send_distance() {
    var distance = parseFloat(document.getElementById("distance").value);
    var msg = JSON.stringify({distance: distance});
    ws.send(msg);
  }

  function sendPID() {
    var kp = parseFloat(document.getElementById("kp_wheel").value);
    var ki = parseFloat(document.getElementById("ki_wheel").value);
    var kd = parseFloat(document.getElementById("kd_wheel").value);
    var msg = JSON.stringify({ kp: kp, ki: ki, kd: kd });
    ws.send(msg);
  }

  function sendPID_balance() {
    var kp = parseFloat(document.getElementById("kp").value);
    var ki = parseFloat(document.getElementById("ki").value);
    var kd = parseFloat(document.getElementById("kd").value);
    var msg = JSON.stringify({ kp_wheel: kp, ki_wheel: ki, kd_wheel: kd });
    ws.send(msg);
  }

  function set_command(comm) {
    var msg = JSON.stringify({command: comm});
    ws.send(msg);
  }

</script>
</body>
</html>
)rawliteral";


// include software header files
#include <Arduino.h>

//Pin Settings and hardware header files

// NeoPixel LED pins and header
#include "Adafruit_NeoPixel.h"
#define NEOPIXEL_PIN 23
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_RGB + NEO_KHZ800); //setup Neopixel

// ezButton pins and header
#include "ezButton.h"
#define button_debounce_time 100 
ezButton button_command(5); 
ezButton button_left(17);
ezButton button_backwards(19);
ezButton button_forwards(16);
ezButton button_right(18);
ezButton button_stop (4);

// camera
#include "HUSKYLENS.h"
#include "Wire.h"
HUSKYLENS huskylens;
bool objectDetected = false;
int objectID = -1;
int objectX = 0;
int objectY = 0;
int objectWidth = 0;
int objectHeight = 0;


// States
#define VOID_ST 0
#define INIT_ST 1
#define START_EXEC_ST 2
#define EXEC_ST 3
#define READ_COMM_ST 4
#define STOP_ST 5
#define FORWARD_ST 6
#define TURN_RIGHT_ST 7
#define TURN_LEFT_ST 8
#define BACK_ST 9
#define TUNE_ST 10
#define WAIT_ST 11

// Movement Commands
#define MAX_NR_COMMANDS 20
#define STOP 0
#define TURN_LEFT 1
#define TURN_RIGHT 3
#define FORWARD 2
#define BACKWARD 4
#define WAIT 5
#define ROTATION_TICKS 1920

// commands
int nr_comm;
int comm_index;         // the index of the action that is being executed...
int recorded_button[MAX_NR_COMMANDS];
int button_index = 0;
int mov;                // Programed data from buttons
unsigned long button_command_count;  // Nr. of times command button is pressed
unsigned long button_stop_count = 0; // Nr. of times stop button is pressed

int on_execute_test_st; // state control variable
int on_execute_comm_st; // state control variable

// TODO - state in movement
int machine_state;
int last_machine_state;
int stop_next_state;

// SetPoints for PID
#define SETPOINT_RUN 3600 // to be replaced as integers 


// PID
#include "PID_simple.h"

unsigned long time_now;
double input_distance, output_distance;
double val_outputL;
double val_outputR;
double enc_readL;
double enc_readR;
double measurment_time;
double last_time_now = 0;
double last_encL, last_encR;
double Setpoint = SETPOINT_RUN;
double delta_wheel = 1;
double wheel_balance = 1;
double k_delta = 0.7;
double computed_speedR, computed_speedL;
double last_speedL, last_speedR;
double delta_fix = 0;
double kp_wheel = 0.25, ki_wheel = 0.005, kd_wheel = 0.3;
double kp_wheel = 0.25, ki_wheel = 0.005, kd_wheel = 0.3;
double delta_goal = 1;
double kp = 0.25, ki =0, kd = 0; // changes in ki & kd resulted in strange behaviour
double kp = 0.25, ki =0, kd = 0; // changes in ki & kd resulted in strange behaviour
int kspeed = 1;
volatile int counterPID;
int freq = 100;
int freq = 100;
int motor_command_count = 0;
double value_fix = wheel_balance;

// tune turn movement
#define num_setpoint_values_turn 7 // number of possible tuning setpoints in equivalent distances
int setpoint_values_turn[num_setpoint_values_turn];
int setpoint_turn_min = 600;
int setpoint_turn_max = 900;
int tune_counter_turn;
int SETPOINT_TURN = 452; 
int SETPOINT_TURN = 452; 

// tune forward/backward movement regarding differences in motors
#define num_setpoint_values_move 7 // number of possible tuning setpoints in equivalent distances
float setpoint_values_move[num_setpoint_values_move];
float setpoint_move_min = -2.00;
float setpoint_move_max = 2.00;
int tune_counter_move;
// initial straight run
float setpoint_straight_run;     // increase to go right 

// Encoders Interrupt function variables and table
volatile double encoder1_pos;
volatile double encoder2_pos;
volatile double enc1_last = 0;
volatile double enc2_last = 0;

byte encoder1_state, encoder2_state;
int  encoder_table[] = { 0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0 };
//int state_table[1000];
int enc_dir_1 = 0;
int enc_dir_2 = 0;
int enc_count = 0;
// Motors
#include "ESP32MotorControl.h"
// Initialize motors library
ESP32MotorControl MotorControl = ESP32MotorControl();

// initial motor speed  
int default_speedL = 40; // because azobopi floated to right side     
int default_speedR = 55;
int speedL = default_speedL, speedR = default_speedR;
// initial motor speed  
int default_speedL = 40; // because azobopi floated to right side     
int default_speedR = 55;
int speedL = default_speedL, speedR = default_speedR;

// motor speed for turning -> set lower fixed speed for turning
int turnspeedL = 60;
int turnspeedR = 60;

// time motors are stopped
#define STOP_DELAY 1000
#define WAIT_DELAY 2000 // time the robo is waiting in delay state
unsigned long time_wait; // waiting timer
bool reset_time_wait = 1; // bool to reset waiting timer

// Wheels
#define WHEEL_DIAMETER 66 // wheel diameter in mm
#define WHEEL_CIRCUMFERENCE (3.14 * WHEEL_DIAMETER)
#define WHEELS_DISTANCE 120
#define CURVE_CIRCUMFERENCE (3.14 * WHEELS_DISTANCE)

// Encoders pins
//left
#define ENC1_A 35
#define ENC1_B 34

//right
#define ENC2_A 36
#define ENC2_B 39

// Timer & Mutex for encoders and PID counters
hw_timer_t  *timer      = NULL;
portMUX_TYPE timerMux   = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE counterMux = portMUX_INITIALIZER_UNLOCKED;

// Initialize PID control for each motor
PID pidleft(&Setpoint, &enc_readL, &val_outputL, kp, ki, kd);
PID pidright(&Setpoint, &enc_readR, &val_outputR, kp, ki, kd);
PID pid_delta(&delta_goal, &delta_wheel, &delta_fix, kp_wheel, ki_wheel, kd_wheel);
PID pid_distance(&Setpoint, &input_distance, &output_distance, kp, ki, kd);
// OLED DISPLAY SSD1306

#include <SPI.h> // inlucde libraries for use of OLED
#include <Wire.h> // inlucde libraries for use of OLED
#include <Adafruit_GFX.h> // inlucde libraries for use of OLED
#include <Adafruit_SSD1306.h> // inlucde libraries for use of OLED

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Display bitmaps

#define bitmap_height   128 // define bitmap size
#define bitmap_width    64  // define bitmap size

#include "displayuaclogo.h"
#include "smileys.h"
#endif 

// Speaker setup

#define PIN_SPEAKER 12
#include "sounds.h"