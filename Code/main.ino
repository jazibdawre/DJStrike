/*
IR Mode:
   LEFT            RIGHT
         _________
        |         |
    A2  O         O  A1
        |         |
        |   CAR   |
        |         |
    B2  O         O  B1
        |_________|

*/

//Modes
void obstacle_detection_mode();
void line_follower_mode();
void voice_controlled_mode();
void manual_mode();

//Sensor Connection
const int right_sensor_pin = A6;
const int left_sensor_pin = A7;
const int obstacle_sensor_pin = A2;
const int motorA1      = 24;  
const int motorA2      = 28; 
const int motorB1      = 14; 
const int motorB2      = 14; 

int left_sensor_state;
int right_sensor_state;
int obstacle_sensor_state;
int forward_sensor_state;
void setup() {
  //Mode Selection
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  //Manual Mode
  pinMode(34, INPUT);
  pinMode(38, INPUT);
  pinMode(44, INPUT);
  //Acting as VCC power to IR sensors (Mega only pins!)
  pinMode(52, OUTPUT);
  pinMode(53, OUTPUT);
  digitalWrite(52, HIGH);
  digitalWrite(53, HIGH);
  //Acting as VCC power to ESP8266 (Mega only pins!)
  pinMode(48, OUTPUT);
  pinMode(49, OUTPUT);
  digitalWrite(48, HIGH);
  digitalWrite(49, LOW);
  //Motor control pins
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  if(digitalRead(2)==HIGH) {
    Serial.println("Manual Mode");
    manual_mode();
  } else if(digitalRead(3)==HIGH) {
    //Serial.println("IR Mode");
    line_follower_mode();
  } else if(digitalRead(4)==HIGH) {
    Serial.println("Voice Controlled Mode");
    voice_controlled_mode();
  }  if(digitalRead(5)==HIGH) {
    Serial.println("Obstacle Mode");
    obstacle_detection_mode();
  }
}

void obstacle_detection_mode() {
  obstacle_sensor_state = analogRead(obstacle_sensor_pin);
  
  //Only for testing
  Serial.print("Obstacle IR: ");
  Serial.println(obstacle_sensor_state);
  
  if (obstacle_sensor_state > 500) {
    Serial.println("OBSTACLE!\nStop");
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, LOW);
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, LOW);
  }
}

void manual_mode() {
    
  left_sensor_state = digitalRead(38);
  right_sensor_state = digitalRead(34);
  forward_sensor_state = digitalRead(44);
  
  //Only for testing
  Serial.print("Left IR: ");
  Serial.print(left_sensor_state);
  Serial.print(" | Right IR: ");
  Serial.print(right_sensor_state);
  Serial.print(" | Forward IR: ");
  Serial.println(forward_sensor_state);
  
  if (forward_sensor_state==HIGH) {
    Serial.println("Forward");
    digitalWrite(motorB2, HIGH);
    digitalWrite(motorB1, HIGH);
    if (left_sensor_state==HIGH) {
      Serial.println("Turning Left");
      digitalWrite(motorA1, LOW);
      digitalWrite(motorA2, HIGH);
    }
    if (right_sensor_state==HIGH) {
      Serial.println("Turning Right");
      digitalWrite(motorA1, HIGH);
      digitalWrite(motorA2, LOW);
    }
  } else {
    Serial.println("Stop");
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, LOW);
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, LOW);
  }
}

void voice_controlled_mode() {
  //Left to implement
}

void line_follower_mode() {
  
  left_sensor_state = analogRead(left_sensor_pin);
  right_sensor_state = analogRead(right_sensor_pin);
  
  //Only for testing
  Serial.print("Left IR: ");
  Serial.print(left_sensor_state);
  Serial.print(" | Right IR: ");
  Serial.println(right_sensor_state);

  if (right_sensor_state < 500 && left_sensor_state > 500) {
    //Serial.println("Turning Left");
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, HIGH);
    digitalWrite(motorB1, HIGH);  //Change to LOW for friction turning
    digitalWrite(motorB2, HIGH);
  }
  if (right_sensor_state > 500 && left_sensor_state < 500) {
    //Serial.println("Turning Right");
    digitalWrite(motorA1, HIGH);
    digitalWrite(motorA2, LOW);
    digitalWrite(motorB1, HIGH);
    digitalWrite(motorB2, HIGH);  //Change to LOW for friction turning
  }

  if (right_sensor_state < 500 && left_sensor_state < 500) {
    //Serial.println("Forward");
    digitalWrite(motorA2, LOW);  //Change to HIGH for 4WD
    digitalWrite(motorA1, LOW);  //Change to HIGH for 4WD
    digitalWrite(motorB2, HIGH);
    digitalWrite(motorB1, HIGH);
  }

  if (right_sensor_state > 500 && left_sensor_state > 500) {
    //Serial.println("Stop");
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, LOW);
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, LOW);
  }
}