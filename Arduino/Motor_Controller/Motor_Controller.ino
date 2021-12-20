//Motor 1 pins
int in1_front = 37;
int in2_front = 39;
int enable_a_front = 35;
int motor1_ENCA = 20;
int motor1_ENCB = 25;

//Motor 2 pins
int in3_front = 45;
int in4_front = 47;
int enable_b_front = 49;
int motor2_ENCA = 21;
int motor2_ENCB = 27;

//encoder-odom values
int motor1_pos = 0;
int motor2_pos = 0;


//Motor 3 pins
int in3_rear = 9;
int in4_rear = 8;
int enable_b_rear = 7;
int motor4_ENCA = 3;
int motor4_ENCB = 5;

//Motor 4 pins
int in1_rear = 11;
int in2_rear = 10;
int enable_a_rear = 6;
int motor3_ENCA = 2;
int motor3_ENCB = 4;

//encoder-odom values
int motor3_pos = 0;
int motor4_pos = 0;



void setup() {
 Serial.begin(9600); 

  // Set all the motor control pins to outputs
  pinMode(in1_rear, OUTPUT);
  pinMode(in2_rear, OUTPUT);
  pinMode(in3_rear, OUTPUT);
  pinMode(in4_rear, OUTPUT);  
  pinMode(in1_front, OUTPUT);
  pinMode(in2_front, OUTPUT);
  pinMode(in3_front, OUTPUT);
  pinMode(in4_front, OUTPUT);  

  // Set all the encoder pins to inputs
  pinMode(motor1_ENCA, INPUT);
  pinMode(motor1_ENCB, INPUT);
  
  pinMode(motor2_ENCA, INPUT);
  pinMode(motor2_ENCB, INPUT);

  pinMode(motor3_ENCA, INPUT);
  pinMode(motor3_ENCB, INPUT);
  
  pinMode(motor4_ENCA, INPUT);
  pinMode(motor4_ENCB, INPUT);
  
  // Turn off motors - Initial state
  digitalWrite(in1_rear, LOW);
  digitalWrite(in2_rear, LOW);
  digitalWrite(in3_rear, LOW);
  digitalWrite(in4_rear, LOW);
  digitalWrite(in1_front, LOW);
  digitalWrite(in2_front, LOW);
  digitalWrite(in3_front, LOW);
  digitalWrite(in4_front, LOW);

  // Prepare interrupt pins
  attachInterrupt(digitalPinToInterrupt(motor1_ENCA), encoderInterrupt1, RISING);
  attachInterrupt(digitalPinToInterrupt(motor2_ENCA), encoderInterrupt2, RISING);
  attachInterrupt(digitalPinToInterrupt(motor3_ENCA), encoderInterrupt3, RISING);
  attachInterrupt(digitalPinToInterrupt(motor4_ENCA), encoderInterrupt4, RISING);

}

void loop() {
    fwd();
    print_encoder_pos(2000);
    
    stopMovement();
    print_encoder_pos(2000);
    
    bwd();
    print_encoder_pos(2000);
    
    stopMovement();
    print_encoder_pos(2000);

    rotateRight();
    print_encoder_pos(2000);

    stopMovement();
    print_encoder_pos(2000);
    
    rotateLeft();
    print_encoder_pos(2000);

    stopMovement();
    print_encoder_pos(2000);
}

void print_encoder_pos(int ms)
{
  for (int i=0; i < ms/20; i++)
  {
      Serial.print(motor1_pos);
      Serial.print(',');
      Serial.print(motor2_pos);
      Serial.print(',');
      Serial.print(motor3_pos);
      Serial.print(',');
      Serial.println(motor4_pos);
      delay(20);  
  }
}

//moving motor 1
void move_m1(int speed)
{
  if (speed > 0) //forward
  {
    digitalWrite(in1_front, LOW);
    digitalWrite(in2_front, HIGH);
  }
  else if (speed < 0) // backward
  {
    digitalWrite(in1_front, HIGH);
    digitalWrite(in2_front, LOW);
  }
    speed = abs(speed);
    analogWrite(enable_a_front, speed);  
}


//moving motor 2
void move_m2(int speed)
{
  if (speed > 0) //forward
  {
    digitalWrite(in3_front, LOW);
    digitalWrite(in4_front, HIGH);
  }
  else if (speed < 0) // backward
  {
    digitalWrite(in3_front, HIGH);
    digitalWrite(in4_front, LOW);
  }
    speed = abs(speed);
    analogWrite(enable_b_front, speed);  
}


//moving motor 3
void move_m3(int speed)
{
  if (speed > 0) //forward
  {
    digitalWrite(in3_rear, LOW);
    digitalWrite(in4_rear, HIGH);
  }
  else if (speed < 0) // backward
  {
    digitalWrite(in3_rear, HIGH);
    digitalWrite(in4_rear, LOW);
  }
    speed = abs(speed);
    analogWrite(enable_b_rear, speed);  
}


//moving motor 4
void move_m4(int speed)
{
  if (speed > 0) //forward
  {
    digitalWrite(in1_rear, LOW);
    digitalWrite(in2_rear, HIGH);
  }
  else if (speed < 0) // backward
  {
    
    digitalWrite(in1_rear, HIGH);
    digitalWrite(in2_rear, LOW);
  }
    speed = abs(speed);
    analogWrite(enable_a_rear, speed);  
}

void move_command(int m1_speed, int m2_speed, int m3_speed, int m4_speed)
{
  move_m1(m1_speed);
  move_m2(m2_speed);
  move_m3(m3_speed);
  move_m4(m4_speed);
}

void fwd()
{
  move_command(150, 150, 150, 150);
}

void bwd()
{
  move_command(-150, -150, -150, -150);  
}

void rotateRight()
{
  move_command(-150, 150, -150, 150);
}

void rotateLeft()
{
  move_command(150, -150, 150, -150);  
}
void stopMovement()
{
  move_command(0, 0, 0, 0);
}
void relax()
{
    digitalWrite(in1_rear, LOW);
    digitalWrite(in2_rear, LOW);
    digitalWrite(in3_rear, LOW);
    digitalWrite(in4_rear, LOW);
    digitalWrite(in1_front, LOW);
    digitalWrite(in2_front, LOW);
    digitalWrite(in3_front, LOW);
    digitalWrite(in4_front, LOW);

    analogWrite(enable_a_front, 0);
    analogWrite(enable_b_front, 0);
    analogWrite(enable_a_rear, 0);
    analogWrite(enable_b_rear, 0);    
}

void encoderInterrupt1(){
  int b = digitalRead(motor1_ENCB);
  if(b > 0){
    motor1_pos++;
  }
  else{
    motor1_pos--;
  }
}

void encoderInterrupt2(){
  int b = digitalRead(motor2_ENCB);
  if(b > 0){
    motor2_pos--;
  }
  else{
    motor2_pos++;
  }
}
void encoderInterrupt3(){
  int b = digitalRead(motor3_ENCB);
  if(b > 0){
    motor3_pos++;
  }
  else{
    motor3_pos--;
  }
}

void encoderInterrupt4(){
  int b = digitalRead(motor4_ENCB);
  if(b > 0){
    motor4_pos--;
  }
  else{
    motor4_pos++;
  }
}
