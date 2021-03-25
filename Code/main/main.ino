// Sensor connections
const int obstacle_sensor_middle_echo_pin = 27;
const int obstacle_sensor_middle_trigger_pin = 29;

const int obstacle_sensor_left_echo_pin = 31;
const int obstacle_sensor_left_trigger_pin = 33;

const int obstacle_sensor_right_echo_pin = 30;
const int obstacle_sensor_right_trigger_pin = 32;

const int manual_forward_sensor_pin = 41;
const int manual_reverse_sensor_pin = 39;
const int manual_left_sensor_pin = 45;
const int manual_right_sensor_pin = 43;

const int ir_right_sensor_pin = A10;
const int ir_left_sensor_pin = A11;

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

  //Serial.println(String(obstacle_sensor_left_state) + " - " + String(obstacle_sensor_middle_state) + " - " + String(obstacle_sensor_right_state));

  if (obstacle_sensor_middle_state > 80)
  {
    return '1';
  }
  else if (obstacle_sensor_left_state > 80)
  {
    return '3';
  }
  else if (obstacle_sensor_right_state > 80)
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

  // Serial.println(String(ir_left_sensor_state)+" - "+String(ir_right_sensor_state));

  if (ir_right_sensor_state < 175 && ir_left_sensor_state > 175)
  {
    return '4';
  }
  if (ir_right_sensor_state > 175 && ir_left_sensor_state < 175)
  {
    return '3';
  }

  if (ir_right_sensor_state < 175 && ir_left_sensor_state < 175)
  {
    return '0';
  }

  if (ir_right_sensor_state > 175 && ir_left_sensor_state > 175)
  {
    return '1';
  }
}

void loop()
{
  Serial.println(String(manual_mode()) + ":" + String(line_follower_mode()) + ":" + String(obstacle_detection_mode()));
}
