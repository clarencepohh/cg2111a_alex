#define LF 6   // Left forward pin
#define LR 5   // Left reverse pin
#define RF 10  // Right forward pin
#define RR 11  // Right reverse pin

#define FW_Right_constant 0.97
#define FW_Left_constant 1

#define BW_Right_constant 0.95
#define BW_Left_constant 1

#define LT_Right_constant 1
#define LT_Left_constant 1

#define RT_Right_constant 1
#define RT_Left_constant 1

#define run_time 5
#define motor_power 100
#define mode 1
//mode select: 1 = FORWARD, 2 = BACK, 3 = LEFT, 4 = RIGHT

// Convert percentages to PWM values
int pwmVal(float speed) {
  if (speed < 0.0)
    speed = 0;

  if (speed > 100.0)
    speed = 100.0;

  return (int)((speed / 100.0) * 255.0);
}

void forward(float speed) {

  int val = pwmVal(speed);
  analogWrite(LF, val * FW_Left_constant);
  analogWrite(RF, val * FW_Right_constant);
  analogWrite(LR, 0);
  analogWrite(RR, 0);
}

void reverse(float speed) {

  int val = pwmVal(speed);
  analogWrite(LR, val * BW_Left_constant);
  analogWrite(RR, val * BW_Right_constant);
  analogWrite(LF, 0);
  analogWrite(RF, 0);
}

void left(float speed) {
  int val = pwmVal(speed);
  analogWrite(LR, val * LT_Left_constant);
  analogWrite(RF, val * LT_Right_constant);
  analogWrite(LF, 0);
  analogWrite(RR, 0);
}

void right(float speed) {
  int val = pwmVal(speed);

  analogWrite(RR, val * RT_Right_constant);
  analogWrite(LF, val * RT_Left_constant);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
}

// Stop Alex. To replace with bare-metal code later.
void stop() {
  analogWrite(LF, 0);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
  analogWrite(RR, 0);
}

void setup() {
  // put your setup code here, to run once:
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
}
int i = 0;
void loop() {
  if (i < run_time) {
    switch(mode) {
      case 1: forward(motor_power);
      case 2: reverse(motor_power);
      case 3: left(motor_power);
      case 4: right(motor_power);
    }
  } else {
    stop();
  }

  i++;
}
