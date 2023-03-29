#include "serialize.h"
#include <stdarg.h>
#include <math.h>
#include "packet.h"
#include "constants.h"

typedef enum
{
  STOP=0,
  FORWARD=1,
  BACKWARD=2,
  LEFT=3,
  RIGHT=4
} TDirection;

volatile TDirection dir = STOP;

/*
 * Alex's configuration constants
 */

// Number of ticks per revolution from the 
// wheel encoder.

#define COUNTS_PER_REV      170.0

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          20.5

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  6   // Left forward pin
#define LR                  5   // Left reverse pin
#define RF                  10  // Right forward pin
#define RR                  9  // Right reverse pin

// PI, for calculating turn circumference
#define PI 3.141592654

// Vincent's length and breadth in cm
#define ALEX_LENGTH 20//longer than 15
#define ALEX_BREADTH 12.7

//Alex's diagonal. We compute and store this once since it's expensive to computer and really doesn't change
float alexDiagonal = 0.0;
// Alex's turning circumference, calculated once
float alexCirc = 0.0;

/*
 *    Alex's State Variables
 */

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks; 
volatile unsigned long rightReverseTicks;

volatile unsigned long leftForwardTicksTurns; 
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns; 
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

//Variable to keep track of whether we have moved a commanded distance
unsigned long deltaDist;
unsigned long newDist;

//Variable to keep track of our turning angle
unsigned long deltaTicks;
unsigned long targetTicks;
/*
 * 
 * Alex Communication Routines.
 * 
 */
 
TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".
    
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);
    
}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
  TPacket statusPacket;
  statusPacket.packetType=PACKET_TYPE_RESPONSE;
  statusPacket.command=RESP_STATUS;
  statusPacket.params[0]=leftForwardTicks;
  statusPacket.params[1]=rightForwardTicks;
  statusPacket.params[2]=leftReverseTicks;
  statusPacket.params[3]=rightReverseTicks;
  statusPacket.params[4]=leftForwardTicksTurns;
  statusPacket.params[5]=rightForwardTicksTurns;
  statusPacket.params[6]=leftReverseTicksTurns;
  statusPacket.params[7]=rightReverseTicksTurns;
  statusPacket.params[8]=forwardDist;
  statusPacket.params[9]=reverseDist;

  sendResponse(&statusPacket);
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprintf (char *format, ...) {
  va_list args;
  char buffer[128];
  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.
  
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
  
}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.
  
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);  
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.
  
  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);

}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);  
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs. 
  DDRD &= (~(1 << 2) & ~(1 << 3));
  PORTD = (1 << 2) | (1 << 3);
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR() //INT0
{
  if (dir == FORWARD) {
    leftForwardTicks++;
    forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  else if (dir == BACKWARD) {
    leftReverseTicks++;
    reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  else if (dir == LEFT) {
    leftReverseTicksTurns++;
  }
  else if (dir == RIGHT) {
    leftForwardTicksTurns++;
  }
  
  /*
  Serial.print("LEFT: ");
  Serial.println(leftForwardTicks);
  Serial.print("Distance Travelled on LEFT: ");
  Serial.println((double)leftTicks / COUNTS_PER_REV * WHEEL_CIRC);
  */
}

void rightISR() //INT1
{
  if (dir == FORWARD) {
    rightForwardTicks++;
  }
  else if (dir == BACKWARD) {
    rightReverseTicks++;
  }
  else if (dir == LEFT) {
    rightForwardTicksTurns++;
  }
  else if (dir == RIGHT) {
    rightReverseTicksTurns++;
  }
  
  /*
  Serial.print("RIGHT: ");
  Serial.println(rightTicks);
  Serial.print("Distance Travelled on RIGHT: ");
  Serial.println((double)rightTicks / COUNTS_PER_REV * WHEEL_CIRC);
  */
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.
  EICRA = 0b00001010;
  EIMSK = 0b00000011;
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.

ISR(INT0_vect) //vector for INT0
{
  leftISR();
}

ISR(INT1_vect) //vector for INT1
{
  rightISR();
}

// Implement INT0 and INT1 ISRs above.

/*
 * Setup and start codes for serial communications
 * 
 */
// Set up the serial connection. For now we are using 
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
  // To replace later with bare-metal.
  Serial.begin(9600);
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.
  
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{

  int count=0;

  while(Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
  Serial.write(buffer, len);
}

void disableADC() {
  PRR |= (1 << PRADC);
}

/*
 * Alex's motor drivers.
 * 
 */

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  /* Our motor set up is:  
   *    A1IN - Pin 5, PD5, OC0B
   *    A2IN - Pin 6, PD6, OC0A
   *    B1IN - Pin 10, PB2, OC1B
   *    B2In - pIN 11, PB3, OC2A
   */
  DDRD |= (1 << 5) | (1 << 6); // Set pins 5 and 6 to output
  DDRB |= (1 << 2) | (1 << 1); // Set pins 10 and 9 to output
}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{
  // Set up timer 0 for PWM on pins 5 and 6
  // TCNT0 = 0;
  // TCCR0A = 0b10100001; // Clear when up-counting, set when down-counting
  // TCCR0B = 0b00000011; // Phase correct PWM, prescaler = 64
  // TIMSK0 = 0b110; // Enable PWM interrupts
  // OCR0A = 0;
  // OCR0B = 0;

  // // Set up timer 1 for PWM on pins 9 and 10
  // TCNT1 = 0;
  // TCCR1A = 0b10100001;
  // TCCR1B = 0b00000011;
  // TIMSK1 |= 0b110;
  // OCR1AH = 0;
  // OCR1AL = 0;
  // OCR1BH = 0;
  // OCR1BL = 0;

}

void analog_write(int pin, int val) {
  analogWrite(pin, val);
  // if (pin == 5) {
  //   OCR0B = val;
  // }
  // else if (pin == 6) {
  //   OCR0A = val;
  // }
  // else if (pin == 9) {
  //   OCR1AH = 0;
  //   OCR1AL = val;
  // }
  // else if (pin == 10) {
  //   OCR1BH = 0;
  //   OCR1BL = val;
};

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if(speed < 0.0)
    speed = 0;

  if(speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward(float dist, float speed)
{
  dir = FORWARD;

  int val = pwmVal(speed);

  if (dist > 0) deltaDist = dist;
  else deltaDist = 9999999;

  newDist = forwardDist + deltaDist;

  // For now we will ignore dist and move
  // forward indefinitely. We will fix this
  // in Week 9.

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
  
  analogWrite(LF, val);
  analogWrite(RF, val * 0.95);
  analogWrite(LR, 0);
  analogWrite(RR, 0);
}

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed)
{
  dir = BACKWARD;
  
  int val = pwmVal(speed);

  if (dist > 0) deltaDist = dist;
  else deltaDist = 9999999;

  newDist = reverseDist + deltaDist;

  // For now we will ignore dist and 
  // reverse indefinitely. We will fix this
  // in Week 9.

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
  analogWrite(LR, val);
  analogWrite(RR, val * 0.95);
  analogWrite(LF, 0);
  analogWrite(RF, 0);
}

unsigned long ComputeDeltaTicks(float ang)
{
  unsigned long ticks = (unsigned long) ((ang * alexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));

  return ticks/3;
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void left(float ang, float speed)
{
  dir = LEFT;
  
  int val = pwmVal(speed);

  if(ang == 0){
    deltaTicks = 9999999;
  } else {
    deltaTicks = ComputeDeltaTicks(ang);
  }

  targetTicks = leftReverseTicksTurns + deltaTicks;
  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn left we reverse the left wheel and move
  // the right wheel forward.
  analogWrite(LR, val);
  analogWrite(RF, val);
  analogWrite(LF, 0);
  analogWrite(RR, 0);
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed)
{
  dir = RIGHT;
  
  int val = pwmVal(speed);

  if (ang == 0){
    deltaTicks = 9999999;
  } else {
    deltaTicks = ComputeDeltaTicks(ang);
  }

  targetTicks = rightReverseTicksTurns + deltaTicks;
  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn right we reverse the right wheel and move
  // the left wheel forward.
  analogWrite(RR, val);
  analogWrite(LF, val);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
}

// Stop Alex. To replace with bare-metal code later.
void stop()
{
  dir = STOP;
  
  analogWrite(LF, 0);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
  analogWrite(RR, 0);
}

/*
 * Alex's setup and run codes
 * 
 */

// Clears all our counters
void clearCounters()
{
  leftForwardTicks = 0; 
  rightForwardTicks = 0;
  leftReverseTicks = 0; 
  rightReverseTicks = 0;

  leftForwardTicksTurns = 0; 
  rightForwardTicksTurns = 0;
  leftReverseTicksTurns = 0; 
  rightReverseTicksTurns = 0;
}

// Clears one particular counter
void clearOneCounter(int which)
{
  clearCounters();
}
// Intialize Vincet's internal states

void initializeState()
{
  clearCounters();
}

// LF = OC0A
// LR = OC0B
// RF = OC1B
// RR = OC1A

void handleCommand(TPacket *command)
{
  int speed;
  int val;
  int delay_ms;
  switch(command->command)
  {
    // For movement commands, param[0] = speed, param[1] delay.
    case COMMAND_RUSH:
        sendOK();
        speed = command->params[0];
        val = pwmVal(speed);
        analog_write(LF, val*0.96);
        analog_write(RF, val);
        analog_write(LR, 0);
        analog_write(RR, 0);
      break;

    case COMMAND_FORWARD:
        sendOK();
        speed = command->params[0];
        val = pwmVal(speed);
        delay_ms = command->params[1];
        // OCR0A = val;
        // OCR1B = val;
        // delay(delay_ms);
        // OCR0A = 0;
        // OCR1B = 0;

        analog_write(LF, val);
        analog_write(RF, val);
        analog_write(LR, 0);
        analog_write(RR, 0);
        delay(delay_ms);
        analog_write(LF, 0);
        analog_write(RF, 0);
        analog_write(LR, 0);
        analog_write(RR, 0);

        //forward((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_REVERSE:
        sendOK();
        speed = command->params[0];
        val = pwmVal(speed);
        delay_ms = command->params[1];

        analog_write(LF, 0);
        analog_write(RF, 0);
        analog_write(LR, 200);
        analog_write(RR, 200);
        delay(delay_ms);
        analog_write(LF, 0);
        analog_write(RF, 0);
        analog_write(LR, 0);
        analog_write(RR, 0);
        //reverse((float) command->params[0], (float) command->params[1]);
      break;
    // For rotate commands, param[0] = angle, param[1] undefined.
    case COMMAND_TURN_RIGHT:
        sendOK();
        speed = command->params[0];
        val = pwmVal(speed);
        delay_ms = command->params[1];

        analog_write(LF, val);
        analog_write(RF, 0);
        analog_write(LR, 0);
        analog_write(RR, val);
        delay(delay_ms);
        analog_write(LF, 0);
        analog_write(RF, 0);
        analog_write(LR, 0);
        analog_write(RR, 0);
        //right((float) 1.6*(command->params[0]), (float) command->params[1]);
      break;
    case COMMAND_TURN_LEFT:
        sendOK();
        speed = command->params[0];
        val = pwmVal(speed);
        delay_ms = command->params[1];

        analog_write(LF, 0);
        analog_write(RF, val);
        analog_write(LR, val);
        analog_write(RR, 0);
        delay(delay_ms);
        analog_write(LF, 0);
        analog_write(RF, 0);
        analog_write(LR, 0);
        analog_write(RR, 0);
        // left((float) (command->params[0]), (float) command->params[1]);
      break;
    case COMMAND_STOP:
        sendOK();
        analog_write(LF, 0);
        analog_write(RF, 0);
        analog_write(LR, 0);
        analog_write(RR, 0);
        //stop();
      break;
    case COMMAND_GET_STATS:
        sendStatus();
      break;
    case COMMAND_CLEAR_STATS:
        sendOK();
        clearOneCounter(command->params[0]);
      break;
    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {
     

        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}

void setup() {

  //Compute the diagonal
  alexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  alexCirc = PI * alexDiagonal;
  
  // put your setup code here, to run once:

  cli();
  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  startMotors();
  enablePullups();
  disableADC();
  initializeState();
  sei();
}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void loop() {

// Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2

// forward(0, 100);

// Uncomment the code below for Week 9 Studio 2


 // put your main code here, to run repeatedly:
  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);
  
  if(result == PACKET_OK)
    handlePacket(&recvPacket);
  else
    if(result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else
      if(result == PACKET_CHECKSUM_BAD)
      {
        sendBadChecksum();
      } 
      
  if (deltaDist > 0) {
    if (dir==FORWARD) {
      if (forwardDist > newDist) {
        deltaDist=0;
        newDist=0;
        stop();   
      }
    } else if (dir==BACKWARD) {
      if (reverseDist > newDist) {
        deltaDist=0;
        newDist=0;
        stop();   
      }
    } else if (dir == STOP) {
      deltaDist=0;
      newDist=0;
      stop();
    }
  }

  if(deltaTicks > 0){
    if (dir == LEFT){
      if (leftReverseTicksTurns >= targetTicks){
        deltaTicks = 0; 
        targetTicks = 0;
        stop();
      }
    } else if (dir == RIGHT){
      if (rightReverseTicksTurns >= targetTicks){
        deltaTicks = 0; 
        targetTicks = 0;
        stop();
      }
    } else if (dir == STOP){
      deltaTicks = 0; 
      targetTicks = 0;
      stop();
    }
  }
}
