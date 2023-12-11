#include <ESP32Servo.h>
#include "Wire.h"
#include <MPU6050_light.h>
#include <string.h>

#define     HIGH        1
#define     LOW         0

//=== NAMING THE PORTS ===
#define     SLC         32
#define     SR          39
#define     SC          34
#define     SL          35
#define     SRC         36
#define     E_CH1       33
#define     CHA_M1      25
#define     E_CH2       26
#define     CHA_M2      27
#define     BUZZER      14
#define     BL_LCD      12
#define     SERVO       13
#define     DB4         15
#define     DB5         2
#define     DB6         0
#define     DB7         4
#define     EN_TRIG     16
#define     RS          17
#define     CS_SENSORS  5
#define     CLK         18
#define     MISO_ECHO   19
#define     SDA_MPU     21
#define     RX0         3
#define     TX0         1
#define     SCL_MPU     22
#define     MOSI        23
//=========================

//============================= LCD COMMANDS =============================
const int DATA[] = {4, 0, 2, 15};
#define     DATA_SIZE 4

// Commands: [- - - RS D7 D6 D5 D4]
#define     FUNCTION_SET 	      0x20 	// 0010 0000 - 4 bits
#define     DISPLAY_CONTROL     0x0C 	// 0000 1100
#define     CLEAR_DISPLAY 	    0x01 	// 0000 0001
#define     RETURN_HOME 	      0x02 	// 0000 0010
#define     ENTRY_MODE_SET 	    0x06 	// 0000 0110

// States
#define     LCD_DISPLAYON 		  0x04
#define     LCD_DISPLAYOFF 		  0x00
#define     DISPLAY_FUNCTION 	  0x08
#define     ENTER 				      0xA8  //1010 1000
#define     DISPLAY_SHIFT_LEFT 	0x18  //0001 1000
#define     DISPLAY_SHIFT_RIGHT 0x1C  //0001 1100

void pulseEnable();
void initLCD();
void write4bits(int value);
void write8bits(int value);

void writeData(const char* value){
  char c;
  for (int i = 0; i < strlen(value); i++){
    c = value[i];
    
    digitalWrite(RS, HIGH);
    write8bits(c);
    digitalWrite(RS, LOW);
  }
}

void display_sensors_values_on_LCD();
// ============================= END LCD COMMANDS =============================

// Private Objects and Variables for giroscope
MPU6050 mpu(Wire);
unsigned long timer = 0;

// DEFINES OF VALUES RELATED TO ENGINE CONTROL
#define     PWM1_Ch     2       // Direita
#define     PWM2_Ch     3       // Esquerda
#define     PWM_Res     8
#define     PWM_Freq    1000

#define     LINE_THRESHOLD_VALUE 500      // line reading cut-off value
#define     WALL_THRESHOLD_VALUE 30       // wall reading cut-off value
#define     ANGLE_TURN_VALUE 83           // angle that robot must turn based on giroscope value
#define     TIME_TO_RESTART 15            // seconds
#define     NUM_SENSORS 5

unsigned int sensors[NUM_SENSORS]       = {0};      // an array to hold sensor values
unsigned int sensors_max[NUM_SENSORS]   = {0};      // used to calibrate the initial values of the IR sensors
unsigned int sensors_min[NUM_SENSORS]   = {4096};   // used to calibrate the initial values of the IR sensors

float Kp = 0.008, Ki = 0, Kd = 0;                   // values of the PID constants values

char path[100] = "";

unsigned char path_length = 0;                      // the length of the path


float moving_average(float *buffer, int tamanho);

float read_ultrasonic_sensor();

void follow_segment();

void turn_off_motors();

void read_line();

void turn(char dir);

char select_turn(unsigned char found_left, unsigned char found_straight, unsigned char found_right);

void simplify_path();

void set_motors(int right_motor, int left_motor);

void lcd_load_custom_character();

const int SIZE_MOVING_AVERAGE = 5;
float buffer[SIZE_MOVING_AVERAGE] = {0};

Servo servo;                        // create servo object to control a servo
int pos = 0;                        // variable to store the servo position

float average_distance = 0;         // average distance read by the ultrasonic sensor

void setup() {
  Serial.begin(115200);
  Wire.begin();
  byte status = mpu.begin();
  while(status != 0){ }             // stop everything if could not connect to MPU6050
  delay(500);
  mpu.calcOffsets();                // gyro and accelero

  pinMode(EN_TRIG, OUTPUT);
  pinMode(MISO_ECHO, INPUT);

  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);

  servo.attach(SERVO, 500, 2500);   // attaches the servo on pin SERVO to the servo object
  servo.write(90);                  // Position the servo in the central position
  delay(500);

  pinMode(SLC       , INPUT);
  pinMode(SR        , INPUT);
  pinMode(SC        , INPUT);
  pinMode(SL        , INPUT); 
  pinMode(SRC       , INPUT);
  pinMode(CS_SENSORS, OUTPUT);

  digitalWrite(CS_SENSORS, HIGH);

  pinMode(E_CH1, OUTPUT);
  pinMode(E_CH2, OUTPUT);

  ledcAttachPin(CHA_M1, PWM1_Ch);
  ledcSetup(PWM1_Ch, PWM_Freq, PWM_Res);

  ledcAttachPin(CHA_M2, PWM2_Ch);
  ledcSetup(PWM2_Ch, PWM_Freq, PWM_Res);

  // ========================== LCD ==========================
  pinMode(RS, OUTPUT);
  pinMode(EN_TRIG, OUTPUT);
  
  for (int i = 0; i < DATA_SIZE; i++){
    pinMode(DATA[i], OUTPUT);
  }

  initLCD();
  
  // Always good to clear and return home
  write8bits(CLEAR_DISPLAY);
  write8bits(RETURN_HOME);
  delay(2);                         // 1.52ms delay needed for the Return Home command

  writeData("Thiago Lahass");
  write8bits(ENTER);
  writeData("Eng Computacao");
}
    
void loop() {
  //======================== APPLYING LEARNING ALGORITHM ========================
  while(1){
    delay(100);

    follow_segment();

    read_line();
    // If we no longer find a visible line, we finish the maze and can restart from the optimized path
    if(sensors[1] < LINE_THRESHOLD_VALUE && sensors[2] < LINE_THRESHOLD_VALUE && sensors[3] < LINE_THRESHOLD_VALUE ){
      Serial.println("No lines found, DONE!!!");
      break;
    }

    // set the motors bit more slowly
    set_motors(158, 94); // (128 + 30, 128 - 34) - apparently the motor on the right runs a little more with the same pwm than the one on the left, hence the difference
    read_line();

    // If the crossing sensors are already on the line, leave immediately
    // if not, walk a little further until you find the crossing line.
    while(sensors[0] < LINE_THRESHOLD_VALUE && sensors[4] < LINE_THRESHOLD_VALUE){
      read_line();
    }

    // stop the motors
    turn_off_motors();
    delay(100);

    //================== FOLLOWING THE MAZE ACCORDING TO THE WALLS ==================
    //================================ READ THE WALLS ===============================
    unsigned char found_left      = 0;
    unsigned char found_straight  = 0;
    unsigned char found_right     = 0;

    servo.write(90);                                    // Servo facing forward
    delay(500);
    average_distance = read_ultrasonic_sensor();
    display_sensors_values_on_LCD();
    
    if(average_distance > WALL_THRESHOLD_VALUE){
      found_straight = 1;
      Serial.println("found_straight");
    }

    servo.write(180);                                   // Servo facing left
    delay(500);
    average_distance = read_ultrasonic_sensor();
    display_sensors_values_on_LCD();

    if( average_distance > WALL_THRESHOLD_VALUE ){      // If the left is free, we can take it immediately
      found_left = 1;
      Serial.println("found_left");
    } 
    else{
      servo.write(0);                                   // Servo facing right
      delay(500);
      average_distance = read_ultrasonic_sensor();
      display_sensors_values_on_LCD();

      if(average_distance > WALL_THRESHOLD_VALUE){
        found_right = 1;
        Serial.println("found_right");
      }
    }

    servo.write(90);                                    // Servo facing forward
    //============================ END OF THE READ WALLS SECTION ==============================

    // Intersection identification is complete.

    unsigned char dir = select_turn(found_left, found_straight, found_right);
    Serial.print("Direção: ");
    Serial.println((char)dir);
    
    // Make the turn indicated by the path.
    turn(dir);
    
    // Store the intersection in the path variable.
    path[path_length] = dir;
    path_length ++;

    Serial.print("Path: ");
    Serial.println(path);

    // simplify the path
    simplify_path();
    Serial.print("Simplified Path: ");
    Serial.println(path);
  }
  
  // Now enter an infinite loop - we can re-run the maze as many times as we want to.
  while(1) {
    // Always good to clear and return home
    write8bits(CLEAR_DISPLAY);
    write8bits(RETURN_HOME);
    delay(2);                               // 1.52ms delay needed for the Return Home command

    writeData("DONE!!!");
    write8bits(ENTER);
    writeData("Restart please...");
    delay(1000);

    
    // Always good to clear and return home
    write8bits(CLEAR_DISPLAY);
    write8bits(RETURN_HOME);
    delay(2);                               // 1.52ms delay needed for the Return Home command
    writeData("Restarting in...");
    
    int j = 0;
    for(j = 0; j < TIME_TO_RESTART; j++) {
      
      char line[25];
      line[0] = '\0';
      dtostrf(j, 6, 2, line);
      strcat(line, " s");

      // writeData("Restarting in...");
      write8bits(ENTER);
      writeData(line);
      delay(1000);
    }

    int i;
    for(i = 0; i < path_length; i++) {
      // SECOND MAIN LOOP BODY
      follow_segment();
      
      // Drive straight while slowing down, as before.
      set_motors(158, 94);  // (128 + 30, 128 - 34) - apparently the motor on the right runs a little more with the same pwm than the one on the left, hence the difference
      read_line();

      // If the crossing sensors are already on the line, leave immediately
      // if not, walk a little further until you find the crossing line.
      while(sensors[0] < LINE_THRESHOLD_VALUE && sensors[4] < LINE_THRESHOLD_VALUE){
        read_line();
      }

      // stop the motors
      turn_off_motors();
      delay(100);
    
      // Make a turn according to the instruction stored in
      // path[i].
      turn(path[i]);
    }
    // Follow the last segment up to the finish.
    follow_segment();
    
    // Now we should be at the finish! Restart the loop.
  }
}

// Function to calculate the moving average
float moving_average(float *buffer, int size) {
    float sum = 0.0;
    for (int i = 0; i < size; ++i) {
        sum += buffer[i];
    }
    return sum / size;
}

// Function to obtain reading from the ultrasonic sensor
float read_ultrasonic_sensor() {
    float duration, distance;
    for(int i = 0; i < SIZE_MOVING_AVERAGE; i++){
        // Send a short pulse to the Trig pin to activate measurement
        digitalWrite(EN_TRIG, LOW);
        delayMicroseconds(10);
        digitalWrite(EN_TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(EN_TRIG, LOW);

        // Measure pulse duration on Echo pin
        duration = pulseIn(MISO_ECHO, HIGH);

        // Convert duration to distance (considering speed of sound ~343m/s)
        distance = duration * 0.0343 / 2;

        buffer[i] = distance;
    }

    // Calculation of the moving average
    float average_distance = moving_average(buffer, SIZE_MOVING_AVERAGE);

    return average_distance;
}


// FOLLOWING A SEGMENT USING ON-OFF CONTROL (not used here, for testing only)
void follow_segment_on_off() {
    while(1) {
        read_line();

        if( sensors[1] > LINE_THRESHOLD_VALUE ){        // must rotate to right
          set_motors(128+42, 128-22);
        }
        else if( sensors[3] > LINE_THRESHOLD_VALUE ){   // must rotate to left
          set_motors(128+22, 128-42);
        }
        else if( sensors[2] > LINE_THRESHOLD_VALUE  ){
          set_motors(128+42, 128-42);
        }
        else{
          set_motors(128, 128);
        }
    }
}


// FOLLOWING A SEGMENT USING PID CONTROL
void follow_segment() {
    int     last_proportional_left   = 0;
    long    integral_left            = 0;

    int     last_proportional_right  = 0;
    long    integral_right           = 0;

    unsigned long initial_time, atual_time;
    initial_time = millis();

    while(1) {
        read_line();

        // The "proportional" term should be 0 when we are on the line.
        float proportional_left  = sensors[1];
        float proportional_right = sensors[3];

        // Compute the left derivative (change) and integral (sum) of the position.
        int derivative_left = proportional_left - last_proportional_left;
        integral_left += proportional_left;

        // Compute the right derivative (change) and integral (sum) of the position.
        int derivative_right = proportional_right - last_proportional_right;
        integral_right += proportional_right;

        // Remember the last position.
        last_proportional_left = proportional_left;
        last_proportional_right = proportional_right;

        // Compute the difference between the two motor power settings,
        // m1 - m2. If this is a positive number the robot will turn
        // to the left. If it is a negative number, the robot will
        // turn to the right, and the magnitude of the number determines
        // the sharpness of the turn.
        int power_difference_left  = proportional_left*Kp + integral_left*Ki + derivative_left*Kd;
        int power_difference_right = proportional_right*Kp + integral_right*Ki + derivative_right*Kd;

        set_motors(167 + power_difference_left, 85 - power_difference_right); // direita, esquerda

        // We use the inner three sensors (1, 2, and 3) for
        // determining whether there is a line straight ahead, and the
        // sensors 0 and 4 for detecting lines going to the left and
        // right.
        atual_time = millis();

        if( atual_time > initial_time + 1000){
            if(sensors[1] > LINE_THRESHOLD_VALUE && sensors[2] > LINE_THRESHOLD_VALUE && sensors[3] > LINE_THRESHOLD_VALUE ) {
              // There is no line visible ahead, and we didn't see any
              // intersection. Must be a dead end.
              Serial.println("Intersection 3 lines found");
              return;
            }
            else if(sensors[0] > LINE_THRESHOLD_VALUE || sensors[4] > LINE_THRESHOLD_VALUE){
                // Found an intersection.
                if( sensors[0] > LINE_THRESHOLD_VALUE ){
                    Serial.println("Intersection left found");
                }
                if( sensors[0] > LINE_THRESHOLD_VALUE ){
                    Serial.println("Intersection right found");
                }
                return;
            }
            else if(sensors[1] < LINE_THRESHOLD_VALUE && sensors[2] < LINE_THRESHOLD_VALUE && sensors[3] < LINE_THRESHOLD_VALUE ){
              Serial.println("No lines found, DONE!!!");
              turn_off_motors();
              return;
            }
        }
    }
}


void set_motors(int right_motor, int left_motor){
    digitalWrite(E_CH1, HIGH);
    digitalWrite(E_CH2, HIGH);
    ledcWrite(PWM1_Ch, right_motor);  // Right motor channel CHA_M1
    ledcWrite(PWM2_Ch, left_motor);   // Left  motor channel CHA_M2
    return;
}

void turn_off_motors(){
    digitalWrite(E_CH1, LOW);
    digitalWrite(E_CH2, LOW);
    return;
}

// Turns according to the parameter dir, which should be 'L', 'R', 'S' (straight), or 'B' (back).
void turn (char dir) {
  mpu.update();
  float initial_position = mpu.getAngleZ();
  float current_position = initial_position;

  Serial.print("initial_position: ");
  Serial.println(initial_position);

    switch(dir) {
        case 'L':
            // Turn left with giroscope
            Serial.println("Esquerda");
            set_motors(128+45, 128+45);
            while( current_position < initial_position + ANGLE_TURN_VALUE ){
              mpu.update();
              current_position = mpu.getAngleZ();
              // Serial.println(current_position);
            }

            turn_off_motors();
            break;

            // Turn left with infra-red line sensors
            // set_motors(128+35, 128+35);
            // Serial.println(mpu.getAngleZ());
            // while( sensors[2] > LINE_THRESHOLD_VALUE ){
            //   read_line();
            // }
            // while(sensors[2] < LINE_THRESHOLD_VALUE){
            //   read_line();
            // }
            // set_motors(128, 128);
            // break;

        case 'R':
            // Turn right with giroscope
            Serial.println("Direita");
            set_motors(128-45, 128-45);
            while( current_position > initial_position - ANGLE_TURN_VALUE ){
              mpu.update();
              current_position = mpu.getAngleZ();
              Serial.print("current_position: ");
              Serial.println(current_position);
            }

            turn_off_motors();
            break;

            // // Turn right with line sensors
            // set_motors(128-35, 128-35);
            // while( sensors[2] > LINE_THRESHOLD_VALUE ){
            //   read_line();
            // }
            // while(sensors[2] < LINE_THRESHOLD_VALUE){
            //   read_line();
            // }
            // set_motors(128, 128);
            // break;
        case 'B':
          // Turn around with giroscope
            Serial.println("Back");
            set_motors(128+45, 128+45);
            while( current_position < initial_position + 2 * ANGLE_TURN_VALUE + 5 ){
              mpu.update();
              current_position = mpu.getAngleZ();
              // Serial.println(current_position);
            }

            turn_off_motors();
            break;

            // // Turn around.
            // set_motors(128+35, 128+35);
            // while( sensors[2] > LINE_THRESHOLD_VALUE ){
            //   read_line();
            // }
            // while(sensors[2] < LINE_THRESHOLD_VALUE){
            //   read_line();
            // }
            // set_motors(128, 128);
            // break;
        case 'S':
            // Don't do anything!
            Serial.println("Straight");
            break;
    }
}

// ================= "left hand on the wall" strategy =================
// This function decides which way to turn during the learning phase of
// maze solving. It uses the variables found_left, found_straight, and
// found_right, which indicate whether there is an exit in each of the
// three directions, applying the "left hand on the wall" strategy.
char select_turn(unsigned char found_left, unsigned char found_straight, unsigned char found_right) {
    // Make a decision about how to turn. The following code
    // implements a left-hand-on-the-wall strategy, where we always
    // turn as far to the left as possible.
    if      (found_left)        return 'L';
    else if (found_straight)    return 'S';
    else if (found_right)       return 'R';
    else                        return 'B';
}


// Path simplification. The strategy is that whenever we encounter a
// sequence xBx, we can simplify it by cutting out the dead end. For
// example, LBL -> S, because a single S bypasses the dead end
// represented by LBL.
void simplify_path() {
    // only simplify the path if the second-to-last turn was a 'B'
    if(path_length < 3 || path[path_length-2] != 'B') return;
    
    int total_angle = 0;
    int i;
    
    for( i = 1; i <= 3; i++) {
        switch( path[path_length - i] ) {
            case 'R':
                total_angle += 90;
                break;
            case 'L':
                total_angle += 270;
                break;
            case 'B':
                total_angle += 180;
                break;
        }
    }

    // Get the angle as a number between 0 and 360 degrees.
    total_angle = total_angle % 360;

    // Replace all of those turns with a single one.
    switch(total_angle) {
        case 0:
            path[path_length - 3] = 'S';
            break;
        case 90:
            path[path_length - 3] = 'R';
            break;
        case 180:
            path[path_length - 3] = 'B';
            break;
        case 270:
            path[path_length - 3] = 'L';
            break;
    }

    // The path is now two steps shorter.
    path_length -= 2;
}


// This takes a sensor reading and returns an estimate of
// the robot’s position with respect to the line, as a number between 0 and 4000. A value of
// 0 means that the line is to the left of sensor 0, value of 1000 means that the line is directly
// under sensor 1, 2000 means that the line is directly under sensor 2, and so on.

// Here is a simplified version of the code that reads the sensors

// Through experiments, the values vary between:
// 100% WHITE =~ 0
// 100% BLACK =~ 2700
void read_line(){
  sensors[0] = analogRead(SLC);
  sensors[1] = analogRead(SL);
  sensors[2] = analogRead(SC);
  sensors[3] = analogRead(SR);
  sensors[4] = analogRead(SRC);

  // Serial.print(sensors[0]);
  // Serial.print(" ");

  // Serial.print(sensors[1]);
  // Serial.print(" ");

  // Serial.print(sensors[2]);
  // Serial.print(" ");

  // Serial.print(sensors[3]);
  // Serial.print(" ");

  // Serial.println(sensors[4]);
}

// ============================= LCD COMMANDS =============================
/* ------------------------------------------------------
	Pulses the Enable pin to send data to the display
*/
void pulseEnable(){
  // Making sure the pin is LOW at first
  digitalWrite(EN_TRIG, LOW);
  delayMicroseconds(1);
  
  // Pulse the Enable pin
  digitalWrite(EN_TRIG, HIGH);
  delayMicroseconds(1);
  digitalWrite(EN_TRIG, LOW);
  delayMicroseconds(100);
}

/* ------------------------------------------------------
	Initializes the display as in Figure 24 of the 
    HD44780U datasheet requests
*/
void initLCD(){
  // Waiting at first
  delay(40);
  
  //Serial.println("Function set 4 bits 0b0010.");
  digitalWrite(RS, LOW);
  
  // Function set the interface with 4 bits
  write4bits(FUNCTION_SET >> 4);
  delayMicroseconds(4500); // A little more than 4.1 ms
  
  // Now, we set:
  // - Number of lines in the display (2 lines: 16x2)
  // - Size of the pixel matrix (5x8)
  // RS remains 0 (only is 1 when writing)
  //Serial.println("Function set 4 bits 0b0010 1000.");
  write8bits(FUNCTION_SET | DISPLAY_FUNCTION);
  
  // Display OFF
  //Serial.println("Display ON/OFF control 0b0000 1100.");
  write8bits(DISPLAY_CONTROL); 
  
  // Entry mode set
  //Serial.println("Entry mode set 0b0000 0110.");
  write8bits(ENTRY_MODE_SET); 
  
  // Clearing and returning home
  write8bits(CLEAR_DISPLAY);
  write8bits(RETURN_HOME);
  delay(2); // 1.52ms delay needed for the Return Home command
  
  // Now you're free to use the display
}

/* ------------------------------------------------------
	Actually sends the commands to the display
*/
void write4bits(int value){
  for(int i = 0; i < 4; i++){
    // Only the value corresponding to the bit of interest
  	digitalWrite(DATA[i], (value>>(3-i)) & 0x1);
  }
  pulseEnable();
}

/* ------------------------------------------------------
	Writes first half of data, than second half
*/
void write8bits(int value){
  // Sends first half of the data (upper part):
  write4bits(value>>4);
  // Sends last half of the data (lower part):
  write4bits(value);
}


void display_sensors_values_on_LCD(){
  write8bits(CLEAR_DISPLAY);
  write8bits(RETURN_HOME);
  delay(2);                                       // 1.52ms delay needed for the Return Home command

  char line1[25], line2[25];
  line1[0] = '\0';
  line2[0] = '\0';
  dtostrf(average_distance, 6, 2, line1);
  strcat(line1, " cm");

  for (int i = 1; i < NUM_SENSORS - 1; ++i) {
    char sensorValueStr[5];
    dtostrf(sensors[i], 4, 0, sensorValueStr);    // Converts the sensor value to a string

    strcat(line2, sensorValueStr);
    strcat(line2, " ");
  }

  if (strlen(line2) > 0) {
    line2[strlen(line2) - 1] = '\0';
  }

  writeData(line1);                               // display the distance captured by the ultrasonic sensor in [cm]
  write8bits(ENTER);
  writeData(line2);                               // display the values captured by the infra-red line sensors in the range 0 - 4096
}

