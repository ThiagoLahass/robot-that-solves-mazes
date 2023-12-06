// #include <Servo.h>
#include "Wire.h"
#include <MPU6050_light.h>


// Private Objects and Variables
MPU6050 mpu(Wire);
unsigned long timer = 0;


#define HIGH 1
#define LOW  0

#define NUM_SENSORS 5

//=== NOMEANDO AS PORTAS ===
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


// DEFINES DE VALORES RELACIONADOS A CONTROLE DE MOTORES
#define PWM1_Ch     0 // Direita
#define PWM2_Ch     1 // Esquerda
#define PWM_Res     8
#define PWM_Freq    1000
#define MAX_SPEED   200
#define BASE_SPEED  170


// VALORES OBTIDOS ATRAVES DE EXPERIMENTOS
#define BLACK_VALUE 2500        // totalmente em cima da linha guia preta
#define WHITE_VALUE 0           // totalmente em cima do branco
#define THRESHOLD_VALUE 500     // valor de corte


unsigned int sensors[NUM_SENSORS]       = {0};      // an array to hold sensor values
unsigned int sensors_max[NUM_SENSORS]   = {0};   // used to calibrate the initial values of the IR sensors
unsigned int sensors_min[NUM_SENSORS]   = {4096};      // used to calibrate the initial values of the IR sensors

float Kp = 0.008, Ki = 0, Kd = 0; // values of the PID constants values


char path[100] = "";

unsigned char path_length = 0; // the length of the path


float mediaMovel(float *buffer, int tamanho);

float lerSensorUltrasonico();

void follow_segment();

void read_line();

void turn (char dir);

char select_turn(unsigned char found_left, unsigned char found_straight, unsigned char found_right);

void simplify_path();

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

// int posicao_servo = 0;	
// int direcao_atual = 0;	// 0 indica anti-horario
// const int VEL_SERVO = 15; 
// Servo servo;

void setup() {

    Serial.begin(115200);
    Wire.begin();
    byte status = mpu.begin();
    while(status!=0){ } // stop everything if could not connect to MPU6050
    delay(1000);
    // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
    mpu.calcOffsets(); // gyro and accelero

    pinMode(EN_TRIG, OUTPUT);
    pinMode(MISO_ECHO, INPUT);

    pinMode(BUZZER, OUTPUT);
    digitalWrite(BUZZER, LOW);

    // servo.attach(SERVO, 500, 2500);
  	// // posiciona o servo na posicao central
    // while( posicao_servo < 90 ){
    //   posicao_servo += VEL_SERVO;
    //   servo.write(posicao_servo);
    //   delay(100);
    // }

    pinMode(SLC   , INPUT);
    pinMode(SR    , INPUT);
    pinMode(SC    , INPUT);
    pinMode(SL    , INPUT); 
    pinMode(SRC   , INPUT);
    pinMode(CS_SENSORS, OUTPUT);

    digitalWrite(CS_SENSORS, HIGH);

    pinMode(E_CH1, OUTPUT);
    pinMode(E_CH2, OUTPUT);

    ledcAttachPin(CHA_M1, PWM1_Ch);
    ledcSetup(PWM1_Ch, PWM_Freq, PWM_Res);

    ledcAttachPin(CHA_M2, PWM2_Ch);
    ledcSetup(PWM2_Ch, PWM_Freq, PWM_Res);

    calibrate_line_sensors();

    Serial.print("MAX: ");
    Serial.print(sensors_max[0]);
    Serial.print(" ");

    Serial.print(sensors_max[1]);
    Serial.print(" ");

    Serial.print(sensors_max[2]);
    Serial.print(" ");

    Serial.print(sensors_max[3]);
    Serial.print(" ");

    Serial.println(sensors_max[4]);

    Serial.print("MIN: ");
    Serial.print(sensors_min[0]);
    Serial.print(" ");

    Serial.print(sensors_min[1]);
    Serial.print(" ");

    Serial.print(sensors_min[2]);
    Serial.print(" ");

    Serial.print(sensors_min[3]);
    Serial.print(" ");

    Serial.println(sensors_min[4]);
}
    
void loop() {

  delay(100);

  // follow_segment_on_off();

  follow_segment();

  set_motors(158, 98);
  read_line();

  while(sensors[0] < THRESHOLD_VALUE && sensors[4] < THRESHOLD_VALUE){
    read_line();
  }

  set_motors(128, 128);

  delay(100);

  turn('L');


  // mapping_sensors_values_to_calibrated_values();

    // Serial.print(sensors[0]);
    // Serial.print(" ");

    // Serial.print(sensors[1]);
    // Serial.print(" ");

    // Serial.print(sensors[2]);
    // Serial.print(" ");

    // Serial.print(sensors[3]);
    // Serial.print(" ");

    // Serial.println(sensors[4]);
    
  // delay(5000);

    

  // follow_segment();


    // //============ LEITURA E ATUALIZAÇÃO DA DISTANCIA DO ULTRASOM ==============
    // // Leitura do sensor ultrassônico
    // // Atualização do buffer para média móvel
    // for (int i = TAMANHO_MEDIA_MOVEL - 1; i > 0; --i) {
    //   buffer[i] = buffer[i - 1];
    // }
    // buffer[0] = lerSensorUltrasonico();

    // // Cálculo da média móvel
    // float media = mediaMovel(buffer, TAMANHO_MEDIA_MOVEL);

    // // Impressão da distância
    // Serial.print("Distancia: ");
    // Serial.println(media);

    // //========== FIM LEITURA E ATUALIZAÇÃO DA DISTANCIA DO ULTRASOM ============


    // //========== UPDATE POSICAO SERVO ==========
    // // anti-horario
    // if( direcao_atual == 0 ){
    //   if(posicao_servo >= 180){
    //       direcao_atual = 1;
    //   }
    //   else{
    //       posicao_servo += VEL_SERVO; 
    //   }
    // }
    // else{ //horario
    //   if(posicao_servo <= 0){
    //       direcao_atual = 0;
    //   }
    //   else{
    //       posicao_servo -= VEL_SERVO; 
    //   }
    // }
  
  	// servo.write(posicao_servo);
  
  	// // Impressão da posicao
    // Serial.print("Posicao: ");
    // Serial.println(posicao_servo);

    // //========== FIM UPDATE POSICAO SERVO ==========


    // // read the sensor:
    // read_line();
    // //calibrate the values
    // calibrate_line_sensors();

    // // AQUI TEMOS OS VALORES JÁ CALIBRADOS


    // delay(100);  // Aguarda 0.1 segundo        

    // while(1) {
    //     // FIRST MAIN LOOP BODY
    //     // (when we find the goal, we use break; to get out of this)
        
    //     follow_segment();
        
    //     // Drive straight a bit. This helps us in case we entered the intersection at an angle.
    //     // Note that we are slowing down - this prevents the robot from tipping forward too much.
    //     set_motors(50,50);
    //     delay(50);

    //     // These variables record whether the robot has seen a line to the
    //     // left, straight ahead, and right, whil examining the current intersection.
    //     unsigned char found_left        = 0;
    //     unsigned char found_straight    = 0;
    //     unsigned char found_right       = 0;

    //     unsigned int sensors[5];

    //     // Now read the sensors and check the intersection type.        
    //     read_line(sensors);

    //     // Check for left and right exits.
    //     if(sensors[0] > 100) found_left  = 1;
    //     if(sensors[4] > 100) found_right = 1;

    //     // Drive straight a bit more - this is enough to line up our wheels with the intersection.
    //     set_motors(40,40);
    //     delay(200);

    //     // Check for a straight exit.
    //     read_line(sensors);

    //     if(sensors[1] > 200 || sensors[2] > 200 || sensors[3] > 200) found_straight = 1;

    //     // Check for the ending spot.
    //     // If all three middle sensors are on dark black, we have
    //     // solved the maze.
    //     if(sensors[1] > 600 && sensors[2] > 600 && sensors[3] > 600) break;

    //     // Intersection identification is complete.
    //     // If the maze has been solved, we can follow the existing
    //     // path. Otherwise, we need to learn the solution.
    //     unsigned char dir = select_turn(found_left, found_straight, found_right);
        
    //     // Make the turn indicated by the path.
    //     turn(dir);
        
    //     // Store the intersection in the path variable.
    //     path[path_length] = dir;
    //     path_length ++;
        
    //     // You should check to make sure that the path_length does not
    //     // exceed the bounds of the array. We'll ignore that in this example.
    //     // Simplify the learned path.
    //     simplify_path();

    //     // Display the path on the LCD.
    //     display_path();
    // }
    
    // // Now enter an infinite loop - we can re-run the maze as many times as we want to.
    // while(1) {
    //     // Beep to show that we finished the maze.
    //     // Wait for the user to press a button...
    //     int i;
    //     for(i = 0; i < path_length; i++) {
    //         // SECOND MAIN LOOP BODY
            
    //         follow_segment();
            
    //         // Drive straight while slowing down, as before.
    //         set_motors(50,50);
    //         delay(50);
            
    //         set_motors(40,40);
    //         delay(200);
            
    //         // Make a turn according to the instruction stored in
    //         // path[i].
    //         turn(path[i]);
    //     }
    //     // Follow the last segment up to the finish.
    //     follow_segment();
        
    //     // Now we should be at the finish! Restart the loop.
    // }
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


// FOLLOWING A SEGMENT USING ON-OFF CONTROL
void follow_segment_on_off() {
    while(1) {
        read_line();

        if( sensors[1] > THRESHOLD_VALUE ){  // deve rodar p/ direita
          
          set_motors(128+42, 128-22);
        }
        else if( sensors[3] > THRESHOLD_VALUE ){  // deve rodar p/ esquerda
          set_motors(128+22, 128-42);
        }
        else if( sensors[2] > THRESHOLD_VALUE  ){
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

    while(1) {
        read_line();

        // The "proportional" term should be 0 when we are on the line.
        float proportional_left  = sensors[1];
        float proportional_right = sensors[3];

        // Serial.print(proportional_left);
        // Serial.print(" ");
        // Serial.println(proportional_right);

        // Compute the derivative (change) and integral (sum) of the position.
        int derivative_left = proportional_left - last_proportional_left;
        integral_left += proportional_left;

        // Compute the derivative (change) and integral (sum) of the position.
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

        // Serial.print(power_difference_left);
        // Serial.print(" ");
        // Serial.println(power_difference_right);

        set_motors(180 + power_difference_left, 76 - power_difference_right); // direita, esquerda


        // // Compute the actual motor settings. We never set either motor to a negative value.
        // if (power_difference > MAX_SPEED)     power_difference = MAX_SPEED;
        // if (power_difference < -MAX_SPEED)    power_difference = -MAX_SPEED;

        // if (power_difference < 0)   set_motors(MAX_SPEED + power_difference,  MAX_SPEED                     );
        // else                        set_motors(MAX_SPEED,                     MAX_SPEED - power_difference  );

        // // We use the inner three sensors (1, 2, and 3) for
        // // determining whether there is a line straight ahead, and the
        // // sensors 0 and 4 for detecting lines going to the left and
        // // right.
        if( sensors[1] > THRESHOLD_VALUE && sensors[2] > THRESHOLD_VALUE && sensors[3] > THRESHOLD_VALUE ) {
            // There is no line visible ahead, and we didn't see any
            // intersection. Must be a dead end.
            return;
        }
        // else if( sensors[0] > THRESHOLD_VALUE || sensors[4] > THRESHOLD_VALUE ) {
        //     // Found an intersection.
        //     return;
        // }
    }
}


void set_motors(int right_motor, int left_motor){
    digitalWrite(E_CH1, HIGH);
    digitalWrite(E_CH2, HIGH);
    ledcWrite(PWM1_Ch, right_motor);  // Canal motor direito  CHA_M1
    ledcWrite(PWM2_Ch, left_motor);   // Canal motor esquerdo CHA_M2
    return;
}


// Turns according to the parameter dir, which should be 'L', 'R', 'S' (straight), or 'B' (back).
void turn (char dir) {
  mpu.update();
  float posicao_inicial = mpu.getAngleZ();
  float posicao_atual;
  Serial.println(posicao_inicial);

    switch(dir) {
        case 'L':
            // Turn left with giroscope
            set_motors(128+45, 128+45);
            while( posicao_atual < posicao_inicial + 85 ){
              mpu.update();
              posicao_atual = mpu.getAngleZ();
              Serial.println(posicao_atual);
            }
            set_motors(128, 128);
            break;


            // Turn left with infra-red line sensors
            // set_motors(128+35, 128+35);
            // Serial.println(mpu.getAngleZ());
            // while( sensors[2] > THRESHOLD_VALUE ){
            //   read_line();
            // }
            // while(sensors[2] < THRESHOLD_VALUE){
            //   read_line();
            // }
            // set_motors(128, 128);
            // break;

        case 'R':

            // Turn right with giroscope
            set_motors(128-35, 128-35);
            while( posicao_atual < posicao_inicial - 85 ){
              mpu.update();
              posicao_atual = mpu.getAngleZ();
              Serial.println(posicao_atual);
            }
            set_motors(128, 128);
            break;

            // // Turn right with line sensors
            // set_motors(128-35, 128-35);
            // while( sensors[2] > THRESHOLD_VALUE ){
            //   read_line();
            // }
            // while(sensors[2] < THRESHOLD_VALUE){
            //   read_line();
            // }
            // set_motors(128, 128);
            // break;
        case 'B':

          // Turn around with giroscope
            set_motors(128+35, 128+35);
            while( posicao_atual < posicao_inicial + 175 ){
              mpu.update();
              posicao_atual = mpu.getAngleZ();
              Serial.println(posicao_atual);
            }
            set_motors(128, 128);
            break;

            // // Turn around.
            // set_motors(128+35, 128+35);
            // while( sensors[2] > THRESHOLD_VALUE ){
            //   read_line();
            // }
            // while(sensors[2] < THRESHOLD_VALUE){
            //   read_line();
            // }
            // set_motors(128, 128);
            // break;
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

// Por experimentos os valores estão variando entre:
// 100 % BRANCO =~ 0
// 100 % PRETO  =~ 2700
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

void calibrate_line_sensors(){
    
    // calibrate during the robot make a turn
    mpu.update();
    float posicao_inicial = mpu.getAngleZ();
    float posicao_atual;

    // Turn left with giroscope
    set_motors(128+35, 128+35);
    while( posicao_atual < posicao_inicial + 350 ){
        mpu.update();
        posicao_atual = mpu.getAngleZ();
        Serial.println(posicao_atual);

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
    set_motors(128, 128);
}

void mapping_sensors_values_to_calibrated_values(){
    // apply the calibration to the sensor reading
    sensors[0] = map(sensors[0], sensors_min[0], sensors_max[0], 0, 255);
    sensors[1] = map(sensors[1], sensors_min[1], sensors_max[1], 0, 255);
    sensors[2] = map(sensors[2], sensors_min[2], sensors_max[2], 0, 255);
    sensors[3] = map(sensors[3], sensors_min[3], sensors_max[3], 0, 255);
    sensors[4] = map(sensors[4], sensors_min[4], sensors_max[4], 0, 255);

    Serial.print(sensors[0]);
    Serial.print(" ");

    Serial.print(sensors[1]);
    Serial.print(" ");

    Serial.print(sensors[2]);
    Serial.print(" ");

    Serial.print(sensors[3]);
    Serial.print(" ");

    Serial.println(sensors[4]);
}
