#include <Wire.h>
#include <PixyI2C.h>

#define X_CENTER    160L  
#define Y_CENTER    100L 
#define RCS_MIN_POS     0L 
#define RCS_MAX_POS     1000L 
#define RCS_CENTER_POS  ((RCS_MAX_POS-RCS_MIN_POS)/2)

#define BRAKE 0
#define Forward  1
#define Backward 2

#define MOTOR_1 0 
#define MOTOR_2 1

const byte PWM1 = 5;  // PWM control (speed) for motor A
const byte PWM2 = 6; // PWM control (speed) for motor B
//const byte DIRA = 12; // Direction control for motor A
//const byte DIRB = 13; // Direction control for motor B

//MOTOR 1
#define MOTOR_A1_PIN 7
#define MOTOR_B1_PIN 8

//MOTOR 2
#define MOTOR_A2_PIN 4
#define MOTOR_B2_PIN 9


#define CURRENT_SEN_1 A2
#define CURRENT_SEN_2 A3

#define EN_PIN_1 A0
#define EN_PIN_2 A1

class ServoLoop
{
public:
    ServoLoop(int32_t proportionalGain, int32_t derivativeGain);
  
    void update(int32_t error);
  
    int32_t m_pos;
    int32_t m_prevError;
    int32_t m_proportionalGain;
    int32_t m_derivativeGain;
};
  
// ServoLoop Constructor
ServoLoop::ServoLoop(int32_t proportionalGain, int32_t derivativeGain)
{
    m_pos = RCS_CENTER_POS;
    m_proportionalGain = proportionalGain;
    m_derivativeGain = derivativeGain;
 
    m_prevError = 0x80000000L;
}
  
// ServoLoop Update 
// Calculates new output based on the measured
// error and the current state.
void ServoLoop::update(int32_t error)
{
    long int velocity;
    char buf[32];
    if (m_prevError!=0x80000000)
    {
        velocity = (error*m_proportionalGain + (error - m_prevError)*m_derivativeGain)>>10;
  
        m_pos += velocity;
        if (m_pos>RCS_MAX_POS) 
        {
            m_pos = RCS_MAX_POS; 
        }
        else if (m_pos<RCS_MIN_POS)
        {
            m_pos = RCS_MIN_POS;
        }
    }
    m_prevError = error;
}

ServoLoop panLoop(200, 200);  // Servo loop for pan
ServoLoop tiltLoop(200, 200);
PixyI2C pixy(0x55);

void setup() {
   Serial.begin(9600);
   Serial2.begin(9600);
   Serial.print("Starting...\n");
   pixy.init();
   setupArdumoto(); //ฟังก์ชั่นกำหนด Pin และโหมดการทำงานของ Motor
}

uint32_t lastBlockTime = 0;

void loop() {
  uint16_t blocks;
    blocks = pixy.getBlocks();
    // If we have blocks in sight, track and follow them
    if (blocks)
    {
        int trackedBlock = TrackBlock(blocks);
        FollowBlock(trackedBlock);
        lastBlockTime = millis();
    }  
    else if (millis() - lastBlockTime > 100)
    {
        stopArdumoto(MOTOR_1);
        stopArdumoto(MOTOR_2);
       // ScanForBlocks();
    }
}

int oldX, oldY, oldSignature;
int TrackBlock(int blockCount)
{
    int trackedBlock = 0;
    long maxSize = 0;
    Serial.print("blocks =");
    Serial.println(blockCount);
    for (int n = 0; n < blockCount; n++)
    {
        if ((oldSignature == 0) || (pixy.blocks[n].signature == oldSignature))
        {
            long newSize = pixy.blocks[n].height * pixy.blocks[n].width;
            if (newSize > maxSize)
            {
                trackedBlock = n;
                maxSize = newSize;
            }
        }
    }
    int32_t panError = X_CENTER - pixy.blocks[trackedBlock].x;
    int32_t tiltError = pixy.blocks[trackedBlock].y - Y_CENTER;
    panLoop.update(panError);
    tiltLoop.update(tiltError);
    pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
    oldX = pixy.blocks[trackedBlock].x;
    oldY = pixy.blocks[trackedBlock].y;
    oldSignature = pixy.blocks[trackedBlock].signature;
    return trackedBlock;
}

int32_t size = 500;
void FollowBlock(int trackedBlock)
{
    int32_t followError = RCS_CENTER_POS - panLoop.m_pos;  // How far off-center are we looking now?
  
    // Size is the area of the object.
    // We keep a running average of the last 8.
    size += pixy.blocks[trackedBlock].width * pixy.blocks[trackedBlock].height; 
    size -= size >> 3;
  
    // Forward speed decreases as we approach the object (size is larger)
    int forwardSpeed = constrain(500 - (size/256), -250, 250);
    // Steering differential is proportional to the error times the forward speed
    int32_t differential = (followError + (followError * forwardSpeed))>>8;
 
    // Adjust the left and right speeds by the steering differential.
    int leftSpeed = constrain(forwardSpeed + differential, -250, 250);
    int rightSpeed = constrain(forwardSpeed - differential, -250, 250);
    // And set the motor speeds
        if(leftSpeed > 0)
        {
          driveArdumoto(MOTOR_2, Forward, leftSpeed-210);
        }
        if(rightSpeed > 0)
        {
          driveArdumoto(MOTOR_1, Forward, rightSpeed-210);
        }
        if(leftSpeed < 0)
        {
          driveArdumoto(MOTOR_2, Backward, ((leftSpeed*-1)-210));
        }
        if(rightSpeed < 0)
        {
          driveArdumoto(MOTOR_1, Backward, ((rightSpeed-1)-210));
        }
}

void driveArdumoto(byte motor, byte direct, int spd)
{
  if (motor == MOTOR_1)
  {
    
   if(direct == Forward)
    {
      digitalWrite(MOTOR_A1_PIN, LOW); 
      digitalWrite(MOTOR_B1_PIN, HIGH);
    }
    else if(direct == Backward)
    {
      digitalWrite(MOTOR_A1_PIN, HIGH);
      digitalWrite(MOTOR_B1_PIN, LOW);      
    }
    else
    {
      digitalWrite(MOTOR_A1_PIN, LOW);
      digitalWrite(MOTOR_B1_PIN, LOW);            
    }
    analogWrite(PWM1, spd);
  }
  else if (motor == MOTOR_2)
  {
    if(direct == Forward)
    {
      digitalWrite(MOTOR_A2_PIN, LOW);
      digitalWrite(MOTOR_B2_PIN, HIGH);
    }
    else if(direct == Backward)
    {
      digitalWrite(MOTOR_A2_PIN, HIGH);
      digitalWrite(MOTOR_B2_PIN, LOW);      
    }
    else
    {
      digitalWrite(MOTOR_A2_PIN, LOW);
      digitalWrite(MOTOR_B2_PIN, LOW);            
    }
    analogWrite(PWM2, spd);
  }  
}

void stopArdumoto(byte motor)
{
  driveArdumoto(motor,BRAKE,0);
}

void setupArdumoto()
{
  // All pins should be setup as outputs:
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  
  pinMode(MOTOR_A1_PIN, OUTPUT);
  pinMode(MOTOR_B1_PIN, OUTPUT);

  pinMode(MOTOR_A2_PIN, OUTPUT);
  pinMode(MOTOR_B2_PIN, OUTPUT);
  
  pinMode(CURRENT_SEN_1, OUTPUT);
  pinMode(CURRENT_SEN_2, OUTPUT);  

  pinMode(EN_PIN_1, OUTPUT);
  pinMode(EN_PIN_2, OUTPUT);
 
  // Initialize all pins as low:
  digitalWrite(PWM1, LOW);
  digitalWrite(PWM2, LOW);
  digitalWrite(EN_PIN_1, HIGH);
  digitalWrite(EN_PIN_2, HIGH);
}
