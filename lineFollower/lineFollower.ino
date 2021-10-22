/*===================================================================

  This code is used to develop a Line Follower Robot using an IR Array with the Usagee of a PID Algorithm

  DEVELOPER: Pasindu Liyanage
  DATE: 2020.04.27
  =====================================================================*/

float Kp = 10, Ki = 0, Kd = 0;
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;
int sensor[5] = {0, 0, 0, 0, 0};
int initial_motor_speed = 120;
int rcnt = 0;

void read_sensor_values(void);
void calculate_pid(void);
void motor_control(void);

void forward();
void left();
void right();
int brake();
void reverse();


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  initalize_pins();
}

void loop() {
  // put your main code here, to run repeatedly:
  read_sensor_values();
  calculate_pid();
  motor_control();
}

void read_sensor_values() {
  int i = 0;
  
  for (i = 0; i < 5; i++) {
    sensor[i] = analogRead(i);
    if (sensor[i] < 100) {
      sensor[i] = 1;
    } else if (sensor[i] > 900) {
      sensor[i] = 0;
    }
  }

  if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1))
    error = 4;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1))
    error = 3;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 0))
    error = 2;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0))
    error = 1;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))
    error = 0;
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))
    error = -1;
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))
    error = -2;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))
    error = -3;
  else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))
    error = -4;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) {
    if ((error == -4) || (error == -5)) {
      error = -5;
      //count = count + 1;
    } else if ((error == 4) || (error == 5)) {
      error = 5;
      // count = count + 1;
    }
  }

  Serial.print(sensor[0]);
  Serial.print("  |  ");
  Serial.print(sensor[1]);
  Serial.print("  |  ");
  Serial.print(sensor[2]);
  Serial.print("  |  ");
  Serial.print(sensor[3]);
  Serial.print("  |  ");
  Serial.print(sensor[4]);
  Serial.println("  |  ");

}


void initalize_pins() {
  pinMode(9, OUTPUT); //PWM Pin 1
  pinMode(10, OUTPUT); //PWM Pin 2

  pinMode(3, OUTPUT); //Right Motor Pin 1
  pinMode(4, OUTPUT); //Right Motor Pin 2

  pinMode(5, OUTPUT); //Left Motor Pin 1
  pinMode(6, OUTPUT); //Left Motor Pin 2
}

void calculate_pid() {
  P = error;
  I = I + error;
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  previous_error = error;
}

void motor_control()
{
  // Calculating the effective motor speed:
  int left_motor_speed = initial_motor_speed - PID_value + 10;
  int right_motor_speed = initial_motor_speed + PID_value;

  // The motor speed should not exceed the max PWM value
  constrain(left_motor_speed, 0, 255);
  constrain(right_motor_speed, 0, 255);

  analogWrite(10, left_motor_speed);  //Left Motor Speed
  analogWrite(9, right_motor_speed); //Right Motor Speed
 
  Serial.print(left_motor_speed);  //Left Motor Speed
  Serial.print(" | ");
  Serial.println(right_motor_speed); //Right Motor Speed

  //following lines of code are to make the bot move forward
  digitalWrite(3, LOW);    //RIGHT motor forward
  digitalWrite(4, HIGH);
  digitalWrite(5, LOW);    //LEFT motor forward
  digitalWrite(6, HIGH);
}


void forward() {
  digitalWrite(3, LOW);    //RIGHT motor forward
  digitalWrite(4, HIGH);
  digitalWrite(5, LOW);    //LEFT motor forward
  digitalWrite(6, HIGH);
  delayMicroseconds(500);
}

void left() {
  digitalWrite(3, LOW);    //RIGHT motor forward
  digitalWrite(4, HIGH);
  digitalWrite(5, HIGH);    //LEFT motor forward
  digitalWrite(6, LOW);
}

void right() {
  digitalWrite(3, HIGH);    //RIGHT motor forward
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);    //LEFT motor forward
  digitalWrite(6, HIGH);
}

int brake() {
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  return (1);
}

void reverse() {
  digitalWrite(4, LOW);    //RIGHT motor forward
  digitalWrite(3, HIGH);
  digitalWrite(6, LOW);    //LEFT motor forward
  digitalWrite(5, HIGH);
  delayMicroseconds(500);
}
