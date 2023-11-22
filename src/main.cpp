#include <Servo.h>

#define HIGH 1
#define LOW  0

#define NUM_SENSORS 5

//=== NOMEANDO AS PORTAS ===
//#define   VCC_3_3     1
//#define   EN          2
#define     SLC         3
#define     SR          4
#define     SC          5
#define     SL          6
#define     SRC         7
#define     E_CH1       8
#define     CHA_M1      9
#define     E_CH2       10
#define     CHA_M2      11
#define     BUZZER      12
#define     BL_LCD      13
//#define   GND         14
#define     SERVO       15
//#define   SD2         16
//#define   SD3         17
//#define   CMD         18
//#define   VCC_5       19
//#define   CLK         20
//#define   SD0         21
//#define   SD1         22
#define     DB4         23
#define     DB5         24
#define     DB6         25
#define     DB7         26
#define     EN_TRIG     27
#define     RS          28
#define     CS_SENSORS  29
#define     CLK         30
#define     MISO_ECHO   31
//#define   GND         32
#define     SDA_MPU     33
#define     RX0         34
#define     TX0         35
#define     SCL_MPU     36
#define     MOSI        37
//#define   GND         38



// The set_motors() function in the Pololu AVR Library (see Section 6 for more information) lets you
// set the duty cycle, and it uses 8-bit precision: a value of 255 corresponds to 100% duty cycle. For
// example, to get 67% on M1 and 33% on M2, you would call

// set_motors(171,84);

// To get a slowly decreasing PWM sequence like the one shown in the graph, you would need to write
// a loop that gradually decreases the motor speed over time.


unsigned int sensors[NUM_SENSORS]       = {0};      // an array to hold sensor values
unsigned int sensors_max[NUM_SENSORS]   = {1023};   // used to calibrate the initial values of the IR sensors
unsigned int sensors_min[NUM_SENSORS]   = {0};      // used to calibrate the initial values of the IR sensors

float Kp = 1/20, Ki = 1/10000, Kd = 3/2; // values of the PID constants values


char path[100] = "";

unsigned char path_length = 0; // the length of the path


float mediaMovel(float *buffer, int tamanho);

float lerSensorUltrasonico();

void follow_segment();

void read_line();

void turn (char dir);

char select_turn(unsigned char found_left, unsigned char found_straight, unsigned char found_right);

void simplify_path();

// os valores de right_motor e left_motor devem ser os valores de duty-cycles deles
// exemplos:
// max velocidade: (100, 100)
// parado: (50, 50)
void set_motors(int right_motor, int left_motor);

unsigned int read_battery_millivolts();

void calibrate_line_sensors();

void mapping_sensors_values_to_calibrated_values();

void initialize();

void lcd_load_custom_character();

void print_character();

bool button_is_pressed();

void display_path();



const int TAMANHO_MEDIA_MOVEL = 5;
float buffer[TAMANHO_MEDIA_MOVEL] = {0};

int posicao_servo = 0;	
int direcao_atual = 0;	// 0 indica anti-horario
const int VEL_SERVO = 15; 
Servo servo;

void setup() {
    Serial.begin(9600);
    pinMode(EN_TRIG, OUTPUT);
    pinMode(MISO_ECHO, INPUT);

    servo.attach(SERVO, 500, 2500);
  	// posiciona o servo na posicao central
    while( posicao_servo < 90 ){
      posicao_servo += VEL_SERVO;
      servo.write(posicao_servo);
      delay(100);
    }
}
    
void loop() {


    //============ LEITURA E ATUALIZAÇÃO DA DISTANCIA DO ULTRASOM ==============
    // Leitura do sensor ultrassônico
    // Atualização do buffer para média móvel
    for (int i = TAMANHO_MEDIA_MOVEL - 1; i > 0; --i) {
      buffer[i] = buffer[i - 1];
    }
    buffer[0] = lerSensorUltrasonico();

    // Cálculo da média móvel
    float media = mediaMovel(buffer, TAMANHO_MEDIA_MOVEL);

    // Impressão da distância
    Serial.print("Distancia: ");
    Serial.println(media);

    //========== FIM LEITURA E ATUALIZAÇÃO DA DISTANCIA DO ULTRASOM ============


    //========== UPDATE POSICAO SERVO ==========
    // anti-horario
    if( direcao_atual == 0 ){
      if(posicao_servo >= 180){
          direcao_atual = 1;
      }
      else{
          posicao_servo += VEL_SERVO; 
      }
    }
    else{ //horario
      if(posicao_servo <= 0){
          direcao_atual = 0;
      }
      else{
          posicao_servo -= VEL_SERVO; 
      }
    }
  
  	servo.write(posicao_servo);
  
  	// Impressão da posicao
    Serial.print("Posicao: ");
    Serial.println(posicao_servo);

    //========== FIM UPDATE POSICAO SERVO ==========


    // read the sensor:
    read_line();
    //calibrate the values
    calibrate_line_sensors();

    // AQUI TEMOS OS VALORES JÁ CALIBRADOS


    delay(100);  // Aguarda 0.1 segundo        

    while(1) {
        // FIRST MAIN LOOP BODY
        // (when we find the goal, we use break; to get out of this)
        
        follow_segment();
        
        // Drive straight a bit. This helps us in case we entered the intersection at an angle.
        // Note that we are slowing down - this prevents the robot from tipping forward too much.
        set_motors(50,50);
        delay(50);

        // These variables record whether the robot has seen a line to the
        // left, straight ahead, and right, whil examining the current intersection.
        unsigned char found_left        = 0;
        unsigned char found_straight    = 0;
        unsigned char found_right       = 0;

        unsigned int sensors[5];

        // Now read the sensors and check the intersection type.        
        read_line(sensors);

        // Check for left and right exits.
        if(sensors[0] > 100) found_left  = 1;
        if(sensors[4] > 100) found_right = 1;

        // Drive straight a bit more - this is enough to line up our wheels with the intersection.
        set_motors(40,40);
        delay(200);

        // Check for a straight exit.
        read_line(sensors);

        if(sensors[1] > 200 || sensors[2] > 200 || sensors[3] > 200) found_straight = 1;

        // Check for the ending spot.
        // If all three middle sensors are on dark black, we have
        // solved the maze.
        if(sensors[1] > 600 && sensors[2] > 600 && sensors[3] > 600) break;

        // Intersection identification is complete.
        // If the maze has been solved, we can follow the existing
        // path. Otherwise, we need to learn the solution.
        unsigned char dir = select_turn(found_left, found_straight, found_right);
        
        // Make the turn indicated by the path.
        turn(dir);
        
        // Store the intersection in the path variable.
        path[path_length] = dir;
        path_length ++;
        
        // You should check to make sure that the path_length does not
        // exceed the bounds of the array. We'll ignore that in this example.
        // Simplify the learned path.
        simplify_path();

        // Display the path on the LCD.
        display_path();
    }
    
    // Now enter an infinite loop - we can re-run the maze as many times as we want to.
    while(1) {
        // Beep to show that we finished the maze.
        // Wait for the user to press a button...
        int i;
        for(i = 0; i < path_length; i++) {
            // SECOND MAIN LOOP BODY
            
            follow_segment();
            
            // Drive straight while slowing down, as before.
            set_motors(50,50);
            delay(50);
            
            set_motors(40,40);
            delay(200);
            
            // Make a turn according to the instruction stored in
            // path[i].
            turn(path[i]);
        }
        // Follow the last segment up to the finish.
        follow_segment();
        
        // Now we should be at the finish! Restart the loop.
    }


}



// Função para calcular a média móvel
float mediaMovel(float *buffer, int tamanho) {
    float soma = 0.0;
    for (int i = 0; i < tamanho; ++i) {
        soma += buffer[i];
    }
    return soma / tamanho;
}

// Função para obter a leitura do sensor ultrassônico
float lerSensorUltrasonico() {
    // Envie um pulso curto para o pino Trig para ativar a medição
    digitalWrite(EN_TRIG, LOW);
    delayMicroseconds(1);
    digitalWrite(EN_TRIG, HIGH);
    delayMicroseconds(1);
    digitalWrite(EN_TRIG, LOW);

    // Meça a duração do pulso no pino Echo
    float duration = pulseIn(MISO_ECHO, HIGH);

    // Converta a duração em distância (considerando velocidade do som ~343m/s)
    float distance = duration * 0.0343 / 2;

    return distance;
}



// FOLLOWING A SEGMENT USING PID CONTROL
void follow_segment() {
    int     last_proportional   = 0;
    long    integral            = 0;

    while(1) {
        // Normally, we will be following a line. The code below is
        // similar to the 3pi-linefollower-pid example, but the maximum
        // speed is turned down to 60 for reliability.

        // Get the position of the line.
        unsigned int sensors[5];
        unsigned int position = read_line(sensors);

        // The "proportional" term should be 0 when we are on the line.
        int proportional = ((int)position) - 2000;

        // Compute the derivative (change) and integral (sum) of the position.
        int derivative = proportional - last_proportional;
        integral += proportional;

        // Remember the last position.
        last_proportional = proportional;

        // Compute the difference between the two motor power settings,
        // m1 - m2. If this is a positive number the robot will turn
        // to the left. If it is a negative number, the robot will
        // turn to the right, and the magnitude of the number determines
        // the sharpness of the turn.
        int power_difference = proportional*Kp + integral*Ki + derivative*Kd;

        // Compute the actual motor settings. We never set either motor to a negative value.
        const int max = 60;         // the maximum speed
        
        if (power_difference > max)     power_difference = max;
        if (power_difference < -max)    power_difference = -max;

        if (power_difference < 0)   set_motors(max + power_difference,  max                     );
        else                        set_motors(max,                     max - power_difference  );

        // We use the inner three sensors (1, 2, and 3) for
        // determining whether there is a line straight ahead, and the
        // sensors 0 and 4 for detecting lines going to the left and
        // right.
        if(sensors[1] < 100 && sensors[2] < 100 && sensors[3] < 100) {
            // There is no line visible ahead, and we didn't see any
            // intersection. Must be a dead end.
            return;
        }
        else if(sensors[0] > 200 || sensors[4] > 200) {
            // Found an intersection.
            return;
        }
    }
}


// Turns according to the parameter dir, which should be 'L', 'R', 'S' (straight), or 'B' (back).
void turn (char dir) {
    switch(dir) {
        case 'L':
            // Turn left.
            set_motors(-80,80);
            delay(200);
            break;
        case 'R':
            // Turn right.
            set_motors(80,-80);
            delay(200);
            break;
        case 'B':
            // Turn around.
            set_motors(80,-80);
            delay(400);
            break;
        case 'S':
            // Don't do anything!
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


// averages ten samples and returns the battery voltage in mV:
unsigned int read_battery_millivolts() {
    
}


// This takes a sensor reading and returns an estimate of
// the robot’s position with respect to the line, as a number between 0 and 4000. A value of
// 0 means that the line is to the left of sensor 0, value of 1000 means that the line is directly
// under sensor 1, 2000 means that the line is directly under sensor 2, and so on.

// Here is a simplified version of the code that reads the sensors
void read_line(){
    sensors[0] = analogRead(SLC);
    sensors[1] = analogRead(SL);
    sensors[2] = analogRead(SC);
    sensors[3] = analogRead(SR);
    sensors[4] = analogRead(SRC);
}

void calibrate_line_sensors(){
    
    // calibrate during the robot make a turn
    // while ( making a turn ) {
    while (millis() < 5000) {
        
        //read the IR sensors
        read_line();

        // record the maximum sensors[0] value
        if (sensors[0] > sensors_max[0]) {
            sensors_max[0] = sensors[0];
        }
        // record the minimum sensors[0] value
        if (sensors[0] < sensors_min[0]) {
            sensors_min[0] = sensors[0];
        }

        // record the maximum sensors[1] value
        if (sensors[1] > sensors_max[1]) {
            sensors_max[1] = sensors[1];
        }
        // record the minimum sensors[1] value
        if (sensors[1] < sensors_min[1]) {
            sensors_min[1] = sensors[1];
        }

        // record the maximum sensors[2] value
        if (sensors[2] > sensors_max[2]) {
            sensors_max[2] = sensors[2];
        }
        // record the minimum sensors[2] value
        if (sensors[2] < sensors_min[2]) {
            sensors_min[2] = sensors[2];
        }

        // record the maximum sensors[3] value
        if (sensors[3] > sensors_max[3]) {
            sensors_max[3] = sensors[3];
        }
        // record the minimum sensors[3] value
        if (sensors[3] < sensors_min[3]) {
            sensors_min[3] = sensors[3];
        }

        // record the maximum sensors[4] value
        if (sensors[4] > sensors_max[4]) {
            sensors_max[4] = sensors[4];
        }
        // record the minimum sensors[4] value
        if (sensors[4] < sensors_min[4]) {
            sensors_min[4] = sensors[4];
        }
    }
}

void mapping_sensors_values_to_calibrated_values(){
    // in case the sensor value is outside the range seen during calibration
    sensors[0] = constrain(sensors[0], sensors_min[0], sensors_max[0]);
    sensors[1] = constrain(sensors[1], sensors_min[1], sensors_max[1]);
    sensors[2] = constrain(sensors[2], sensors_min[2], sensors_max[2]);
    sensors[3] = constrain(sensors[3], sensors_min[3], sensors_max[3]);
    sensors[4] = constrain(sensors[4], sensors_min[4], sensors_max[4]);
    // apply the calibration to the sensor reading
    sensors[0] = map(sensors[0], sensors_min[0], sensors_max[0], 0, 255);
    sensors[1] = map(sensors[1], sensors_min[1], sensors_max[1], 0, 255);
    sensors[2] = map(sensors[2], sensors_min[2], sensors_max[2], 0, 255);
    sensors[3] = map(sensors[3], sensors_min[3], sensors_max[3], 0, 255);
    sensors[4] = map(sensors[4], sensors_min[4], sensors_max[4], 0, 255);
}
