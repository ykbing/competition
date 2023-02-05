#include <MsTimer2.h>
#include <SPI.h>

#define MOTOR_F_PWM 2
#define MOTOR_F_ENA 3
#define MOTOR_F_ENB 4

#define MOTOR_R_PWM 5
#define MOTOR_R_ENA 6
#define MOTOR_R_ENB 7

#define ENC1_ADD 22
#define ENC2_ADD 23

volatile int32_t encoderPos1 = 0;
volatile int32_t encoderPos1_old = 0;
volatile int32_t encoderPos2 = 0;
volatile int32_t encoderPos2_old = 0;

int32_t target_Pos = 0;
int32_t pos_error = 0;
int32_t pos_error_d = 0;
int32_t pos_error_old = 0;
int32_t pos_error_sum = 0;

char data_buffer[5] = {0};
char motor_direction;

//PID gain for angular position control
float P = 3.5;
float Pd = 6.5;
float Pi = 2.5;

int position_control_pid_pwm = 0;

/////////ENCODER/////////
void initEncoders() {
  
  // Set slave selects as outputs
  pinMode(ENC1_ADD, OUTPUT);
  pinMode(ENC2_ADD, OUTPUT); 
  
  // Raise select pins
  // Communication begins when you drop the individual select signsl
  digitalWrite(ENC1_ADD,HIGH);
  digitalWrite(ENC2_ADD,HIGH);
 
  SPI.begin();
  
  // Initialize encoder 1
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(ENC1_ADD,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(ENC1_ADD,HIGH);       // Terminate SPI conversation 

  // Initialize encoder 2
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(ENC2_ADD,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(ENC2_ADD,HIGH);       // Terminate SPI conversation 
}

long readEncoder(int encoder_no) 
{  
  // Initialize temporary variables for SPI read
  unsigned int count_1, count_2, count_3, count_4;
  long count_value;   
  
  digitalWrite(ENC1_ADD + encoder_no-1,LOW);      // Begin SPI conversation
   // digitalWrite(ENC4_ADD,LOW);      // Begin SPI conversation
  SPI.transfer(0x60);                     // Request count
  count_1 = SPI.transfer(0x00);           // Read highest order byte
  count_2 = SPI.transfer(0x00);           
  count_3 = SPI.transfer(0x00);           
  count_4 = SPI.transfer(0x00);           // Read lowest order byte
  digitalWrite(ENC1_ADD+encoder_no-1,HIGH);     // Terminate SPI conversation 
  //digitalWrite(ENC4_ADD,HIGH);      // Begin SPI conversation
// Calculate encoder count
  count_value= ((long)count_1<<24) + ((long)count_2<<16) + ((long)count_3<<8 ) + (long)count_4;
  
  return count_value;
}

void clearEncoderCount(int encoder_no) {    
  // Set encoder1's data register to 0
  digitalWrite(ENC1_ADD+encoder_no-1,LOW);      // Begin SPI conversation  
  // Write to DTR
  SPI.transfer(0x98);    
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(ENC1_ADD+encoder_no-1,HIGH);     // Terminate SPI conversation 
  
  delayMicroseconds(100);  // provides some breathing room between SPI conversations
  
  // Set encoder1's current data register to center
  digitalWrite(ENC1_ADD+encoder_no-1,LOW);      // Begin SPI conversation  
  SPI.transfer(0xE0);    
  digitalWrite(ENC1_ADD+encoder_no-1,HIGH);     // Terminate SPI conversation 
}
/////////MOTOR/////////
void front_motor_control(int motor1_pwm)
{
   if (motor1_pwm > 0) // forward
  {
    digitalWrite(MOTOR_F_ENA, HIGH);
    digitalWrite(MOTOR_F_ENB, LOW);
    analogWrite(MOTOR_F_PWM, motor1_pwm);
  }
  else if (motor1_pwm < 0) // backward
  {
    digitalWrite(MOTOR_F_ENA, LOW);
    digitalWrite(MOTOR_F_ENB, HIGH);
    analogWrite(MOTOR_F_PWM, -motor1_pwm);
  }
  else
  {
    digitalWrite(MOTOR_F_ENA, LOW);
    digitalWrite(MOTOR_F_ENB, LOW);
    digitalWrite(MOTOR_F_PWM, 0);
  }
}

void rear_motor_control(int motor2_pwm)
{
   if (motor2_pwm > 0) // forward
  {
    digitalWrite(MOTOR_R_ENA, LOW);
    digitalWrite(MOTOR_R_ENB, HIGH);
    analogWrite(MOTOR_R_PWM, motor2_pwm);
  }
  else if (motor2_pwm < 0) // backward
  {
    digitalWrite(MOTOR_R_ENA, HIGH);
    digitalWrite(MOTOR_R_ENB, LOW);
    analogWrite(MOTOR_R_PWM, -motor2_pwm);
  }
  else
  {
    digitalWrite(MOTOR_R_ENA, LOW);
    digitalWrite(MOTOR_R_ENB, LOW);
    digitalWrite(MOTOR_R_PWM, 0);
  }
}
/*
void motor_control(void)
{
  position_control_pid_pwm = (P * pos_error) + (Pd * pos_error_d) + (Pi * pos_error_sum); 
  position_control_pid_pwm = (position_control_pid_pwm>=255) ? 255 : position_control_pid_pwm; 
  position_control_pid_pwm = (position_control_pid_pwm<=-255) ? -255 : position_control_pid_pwm; 
  
  front_motor_control(position_control_pid_pwm);
  rear_motor_control(position_control_pid_pwm);  
}*/

void control()
{  
  
  // PID for position control
  pos_error = target_Pos - encoderPos1;
  pos_error_d = pos_error - pos_error_old;
  pos_error_sum += pos_error;
  pos_error_sum = (pos_error_sum > 50) ? 50 : pos_error_sum;
  pos_error_sum = (pos_error_sum < -50) ? -50 : pos_error_sum;  
  if(fabs(pos_error)<=2) pos_error_sum = 0;

  position_control_pid_pwm = (P * pos_error) + (Pd * pos_error_d) + (Pi * pos_error_sum); 
  position_control_pid_pwm = (position_control_pid_pwm>=255) ? 255 : position_control_pid_pwm; 
  position_control_pid_pwm = (position_control_pid_pwm<=-255) ? -255 : position_control_pid_pwm; 

  front_motor_control(position_control_pid_pwm);
  rear_motor_control(position_control_pid_pwm); 
  
  pos_error_old = pos_error;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);

  pinMode(MOTOR_F_PWM, OUTPUT);
  pinMode(MOTOR_F_ENA, OUTPUT);  
  pinMode(MOTOR_F_ENB, OUTPUT);

  pinMode(MOTOR_R_PWM, OUTPUT);
  pinMode(MOTOR_R_ENA, OUTPUT);  
  pinMode(MOTOR_R_ENB, OUTPUT);

  initEncoders();          
  clearEncoderCount(1); 
  clearEncoderCount(2);
  
  MsTimer2::set(50, control); 
  MsTimer2::start();
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  int data;
  String s;
  char control_mode;
  
  encoderPos1 = readEncoder(1);
  encoderPos2 = readEncoder(2);
  
  if (Serial.available() > 0) 
  {
    delay(4);
    encoderPos1 = 0;
    encoderPos2 = 0;
    target_Pos = 0;
    
    control_mode = Serial.read();
   if (control_mode == 's') 
    {
      // Insert header into array
      //data_buffer[0] = 's';
      // Read remaining 3 characters of data and insert into array
      Serial.println("Henes motor");
      motor_direction = Serial.read();
      for (int i = 0; i < 4; i++) 
      {
        data_buffer[i] = Serial.read();
      }

       Serial.write(data_buffer[0]);
       Serial.write(data_buffer[1]);
       Serial.write(data_buffer[2]);
       Serial.write(data_buffer[3]);
        
       s=String(data_buffer);
       data = s.toInt();
      if(motor_direction =='r') data = -data;
      Serial.print("  : ");      
      Serial.println(data);
      delay(1000);
      target_Pos = data;
    }
  }
  Serial.print("encoderPos1:");
  Serial.print(encoderPos1);
  Serial.print(" ");
  Serial.print("encoderPos2:");
  Serial.print(encoderPos2);
  Serial.print(" ");
  Serial.print("pos_error:");
  Serial.print(pos_error);
  Serial.print(" ");
  Serial.print("pos_error_d:");
  Serial.print(pos_error_d);
  Serial.print(" ");
  Serial.print("position_control_pid_pwm:");
  Serial.println(position_control_pid_pwm);
  
}
