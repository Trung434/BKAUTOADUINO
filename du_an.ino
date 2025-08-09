float Kp=60, ki=0, kd=30;
float error=0, P=0, I=0, D=0, PID_value=0;
float previous_error = 0 ;
int gia_tri_ban_dau = 30;
int PID_phai,PID_trai;
int angle;
#define sensor1 2
#define sensor2 A4
#define sensor3 A3
#define sensor4 12
#define sensor5 13
#define In1 4
#define In2 5
#define In3 8
#define In4 9
#define ENA 6
#define ENB 10
#define R_S A1 //ir sensor Right
#define echo A2    //Echo pin
#define trigger A3 //Trigger pin
void read_sensor_values(void);
void calculate_pid(void);
void motor_control(void);
void dung();
void chay_thang();

void setup()
{
  #include <Servo.h>
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);
   pinMode(sensor1, INPUT);
    pinMode(sensor2, INPUT);
     pinMode(sensor3, INPUT);
      pinMode(sensor4, INPUT);
       pinMode(sensor5, INPUT);
  Serial.begin(9600);
}
void loop()
{
  read_sensor_values();
  calculate_pid();
  motor_control();
}

void read_sensor_values()
{  
  if(digitalRead(sensor5)==1) error=4;
  else if((digitalRead(sensor4)==1)&&(digitalRead(sensor5)==1)) error=3;
  else if(digitalRead(sensor4)==1) error=2;
  else if((digitalRead(sensor3)==1)&&(digitalRead(sensor4)==1)) error=1;
  else if(digitalRead(sensor3)==1) error=0; // vào line giữa
  else if((digitalRead(sensor2)==1)&&(digitalRead(sensor3)==1)) error=-1;
  else if(digitalRead(sensor2)==1) error=-2;
  else if((digitalRead(sensor1)==1)&&(digitalRead(sensor2)==1)) error=-3;
  else if(digitalRead(sensor1)==1) error=-4;
}
void calculate_pid()
{
  P = error;
  I = I+error;
  D = error-previous_error;
  PID_value = (Kp*P) + (ki*I) + (kd*D);
  previous_error = error;
}
void motor_control()
{
  // Luôn cho hai bánh tiến
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  digitalWrite(In3, LOW);
  digitalWrite(In4, HIGH);

  int base_speed = 90; // tốc độ tối thiểu để bánh vẫn quay
  int max_speed = 255;

  PID_phai = gia_tri_ban_dau - PID_value;
  PID_trai = gia_tri_ban_dau + PID_value;

  // Giữ trong khoảng [base_speed, max_speed]
  PID_phai = constrain(PID_phai, base_speed, max_speed);
  PID_trai = constrain(PID_trai, base_speed, max_speed);

  analogWrite(ENB, PID_phai);
  analogWrite(ENA, PID_trai);
}
