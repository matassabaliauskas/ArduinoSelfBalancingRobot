// MOTORS

#define MOTOR_A_FORWARD 3
#define MOTOR_A_BACKWARDS 5
#define MOTOR_B_FORWARD 6
#define MOTOR_B_BACKWARDS 11

int a_start = 40;
int b_start = 40;

void motor_init()
{
  pinMode(MOTOR_A_FORWARD, OUTPUT);
  pinMode(MOTOR_A_BACKWARDS, OUTPUT);
  pinMode(MOTOR_B_FORWARD, OUTPUT);
  pinMode(MOTOR_B_BACKWARDS, OUTPUT);

  digitalWrite(MOTOR_A_FORWARD, LOW);
  digitalWrite(MOTOR_A_BACKWARDS, LOW);
  digitalWrite(MOTOR_B_FORWARD, LOW);
  digitalWrite(MOTOR_B_BACKWARDS, LOW);
}

void motor_init(int a, int b)
{
  a_start = a;
  b_start = b;
  motor_init();
}

void set_motor_A(int velocity)
{
  // int speed = map(abs(velocity), 0, 255, 55, 255);
  int speed = map(abs(velocity), 0, 255, a_start, 255);
  if (velocity == 0)
  {
    digitalWrite(MOTOR_A_BACKWARDS, LOW);
    digitalWrite(MOTOR_A_FORWARD, LOW);
  }
  else if (velocity > 0)
  {
    analogWrite(MOTOR_A_BACKWARDS, speed);
    digitalWrite(MOTOR_A_FORWARD, LOW);
  }
  else
  {
    analogWrite(MOTOR_A_FORWARD, speed);
    digitalWrite(MOTOR_A_BACKWARDS, LOW);
  }
}

void set_motor_B(int velocity)
{
  // int speed = map(abs(velocity), 0, 255, 47, 255);
  int speed = map(abs(velocity), 0, 255, b_start, 255);
  if (velocity == 0)
  {
    digitalWrite(MOTOR_B_BACKWARDS, LOW);
    digitalWrite(MOTOR_B_FORWARD, LOW);
  }
  else if (velocity > 0)
  {
    analogWrite(MOTOR_B_BACKWARDS, speed);
    digitalWrite(MOTOR_B_FORWARD, LOW);
  }
  else
  {
    analogWrite(MOTOR_B_FORWARD, speed);
    digitalWrite(MOTOR_B_BACKWARDS, LOW);
  }
}

void calibrate_motors(){
  set_motor_A(0);
  set_motor_B(0);
  Serial.begin(9600);
  Serial.println("Starting motor callibration");
  Serial.println("Each motor will start speeding up and you should write down the number that is sent just before the motor starts spinning");
  delay(2000);
  for (int i = 0; i < 255; i++)
  {
    set_motor_A(i);
    Serial.println(i);
    delay(100);
  }
  set_motor_A(0);
  for (int i = 0; i < 255; i++)
  {
    set_motor_B(i);
    Serial.println(i);
    delay(100);
  }
  set_motor_B(0);
  Serial.println("Test finished.");
  Serial.println("Press the restart button if you wish to run again.");
}

// filter and PID

// filter values
float accelTilt = 0;
float gyroTilt = 0;
float gyroTiltRate = 0;
float previousTilt = 0;
float currentTilt = 0;

float gyroX;
float accelY, accelZ;

// PID values
float Kp_oppie = 0.0;
float Kd_oppie = 0.0;
float Ki_oppie = 0.0;
// can't cheat by looking at defaults :p 

float setPoint_oppie = 0;
float move;
float error;
float errorSum;

void init_pid(float p, float i, float d)
{
  Kp_oppie = p;
  Ki_oppie = i;
  Kd_oppie = d;
}

void set_set_point(float point)
{
  setPoint_oppie = point;
}

int getOutput()
{
  return constrain(move, -255, 255);
}

void updateAcceleration(float z, float y)
{
  accelZ = z;
  accelY = y;
}

void updateRotation(float x)
{
  gyroX = x;
}

float getAccelerationAngle()
{
  return accelTilt;
}

float getGyroscopeAngle()
{
  return gyroTilt;
}

float getFilteredAngle()
{
  return currentTilt;
}

#define PID_LOOP_TIME 0.005

void init_filter()
{
  // initialize Timer1
  cli();      // disable global interrupts
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  // set compare match register to set sample time 5ms
  OCR1A = 9999;
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei(); // enable global interrupts
}

ISR(TIMER1_COMPA_vect)
{
  accelTilt = atan2f(-accelZ, accelY) * RAD_TO_DEG;
  gyroTiltRate = gyroX * PID_LOOP_TIME;
  gyroTilt = gyroTilt + gyroTiltRate;
  // currentTilt = 0.9934*(previousTilt+gyroTiltRate)+0.0066*accelTilt;
  currentTilt = 0.9930 * (previousTilt + gyroTiltRate) + 0.0070 * accelTilt;

  error = currentTilt - setPoint_oppie;
  // error = error*abs(error);
  errorSum += error;
  errorSum = constrain(errorSum, -300, 300);

  move = Kp_oppie * error + Ki_oppie * (errorSum)*PID_LOOP_TIME + Kd_oppie * (currentTilt - previousTilt) / PID_LOOP_TIME;

  if (currentTilt < setPoint_oppie - 65 || currentTilt > setPoint_oppie + 65)
  {
    move = 0;
  }

  previousTilt = currentTilt;
}