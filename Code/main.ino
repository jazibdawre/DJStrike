// Sensor connections
const int obstacle_sensor_middle_trigger_pin = 32;
const int obstacle_sensor_middle_echo_pin = 33;
const int obstacle_sensor_left_trigger_pin = 34;
const int obstacle_sensor_left_echo_pin = 35;
const int obstacle_sensor_right_trigger_pin = 36;
const int obstacle_sensor_right_echo_pin = 37;
const int manual_forward_sensor_pin = 38;
const int manual_reverse_sensor_pin = 39;
const int manual_right_sensor_pin = 40;
const int manual_left_sensor_pin = 41;
const int ir_right_sensor_pin = A6;
const int ir_left_sensor_pin = A7;

// Sensor states
int manual_forward_sensor_state;
int manual_reverse_sensor_state;
int manual_left_sensor_state;
int manual_right_sensor_state;
int ir_left_sensor_state;
int ir_right_sensor_state;
int obstacle_sensor_middle_state;
int obstacle_sensor_left_state;
int obstacle_sensor_right_state;

void setup()
{
  pinMode(obstacle_sensor_middle_trigger_pin, OUTPUT);
  pinMode(obstacle_sensor_middle_echo_pin, INPUT);
  pinMode(obstacle_sensor_left_trigger_pin, OUTPUT);
  pinMode(obstacle_sensor_left_echo_pin, INPUT);
  pinMode(obstacle_sensor_right_trigger_pin, OUTPUT);
  pinMode(obstacle_sensor_right_echo_pin, INPUT);
  pinMode(manual_forward_sensor_pin, INPUT);
  pinMode(manual_reverse_sensor_pin, INPUT);
  pinMode(manual_right_sensor_pin, INPUT);
  pinMode(manual_left_sensor_pin, INPUT);
  //Acting as VCC power to IR sensors (Mega only pins!)
  pinMode(52, OUTPUT);
  pinMode(53, OUTPUT);
  Serial.begin(9600);
}

char obstacle_detection_mode()
{
  int duration;

  digitalWrite(obstacle_sensor_middle_trigger_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(obstacle_sensor_middle_trigger_pin, LOW);
  duration = pulseIn(obstacle_sensor_middle_echo_pin, HIGH);
  obstacle_sensor_middle_state = (duration * 0.034 / 2);

  digitalWrite(obstacle_sensor_left_trigger_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(obstacle_sensor_left_trigger_pin, LOW);
  duration = pulseIn(obstacle_sensor_left_echo_pin, HIGH);
  obstacle_sensor_left_state = (duration * 0.034 / 2);

  digitalWrite(obstacle_sensor_right_trigger_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(obstacle_sensor_right_trigger_pin, LOW);
  duration = pulseIn(obstacle_sensor_right_echo_pin, HIGH);
  obstacle_sensor_right_state = (duration * 0.034 / 2);

  if (obstacle_sensor_middle_state > 500)
  {
    return '1';
  }
  else if (obstacle_sensor_left_state > 500)
  {
    return '3';
  }
  else if (obstacle_sensor_right_state > 500)
  {
    return '4';
  }
  else
  {
    return '0';
  }
}

char manual_mode()
{

  manual_left_sensor_state = digitalRead(manual_left_sensor_pin);
  manual_right_sensor_state = digitalRead(manual_right_sensor_pin);
  manual_forward_sensor_state = digitalRead(manual_forward_sensor_pin);
  manual_reverse_sensor_state = digitalRead(manual_reverse_sensor_pin);

  if (manual_forward_sensor_state == HIGH)
  {
    return '1';
  }
  else if (manual_reverse_sensor_state == HIGH)
  {
    return '2';
  }
  else if (manual_left_sensor_state == HIGH)
  {
    return '3';
  }
  else if (manual_right_sensor_state == HIGH)
  {
    return '4';
  }
  else
  {
    return '0';
  }
}

char line_follower_mode()
{

  ir_left_sensor_state = analogRead(ir_left_sensor_pin);
  ir_right_sensor_state = analogRead(ir_right_sensor_pin);

  if (ir_right_sensor_state < 500 && ir_left_sensor_state > 500)
  {
    return '3';
  }
  if (ir_right_sensor_state > 500 && ir_left_sensor_state < 500)
  {
    return '4';
  }

  if (ir_right_sensor_state < 500 && ir_left_sensor_state < 500)
  {
    return '1';
  }

  if (ir_right_sensor_state > 500 && ir_left_sensor_state > 500)
  {
    return '0';
  }
}

void loop()
{
  Serial.println(manual_mode() + ':' + line_follower_mode() + ':' + obstacle_detection_mode());
}