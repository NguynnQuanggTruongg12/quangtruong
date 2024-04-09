#include <avr/interrupt.h>
#include <avr/io.h>


#define in_pwm1 4
#define in_pwm2 5

#define pul2 9
#define dir2 8

#define pul1 7
#define dir1 6

long int V2,V3, x=(3969/289);
volatile long int stepCount_1 = 0;
volatile int stepCount2=0;
volatile bool state1=false;
volatile bool state2=false;
long int vitri=0, vitrimm=0,goc_dc=0, error=0,the2=0,the3=0;
float theta[3] = {0, 0, 0};
float theta1=0,theta2=0,theta3=0;


///////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
void chay(){
  if(digitalRead(13)==1) {vitri--; }  
  else vitri++;
}
void quay(int Speed){
  if(Speed>=60) Speed=60;
  if(Speed<=-60) Speed=-60;
  if(Speed>0){
  analogWrite(in_pwm2,0); analogWrite(in_pwm1,Speed);}
  if(Speed<0){
  analogWrite(in_pwm2,-Speed);analogWrite(in_pwm1,0);}
}

int pid(long int error, long int kp, long int ki, long int kd )
{
  long int derror,ierror;
  long int temp;
  static int errortr=0;    
  derror=error-errortr;
  ierror+=error;
  if(ierror>=10) ierror=10;
  if(ierror<=-10) ierror=-10;
  errortr=error;
  temp = kp*error+ki*ierror+kd*derror;
  if(temp >=60) temp =60;
  if(temp <=-60) temp =-60;
  return temp;
}


////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(9600);
    attachInterrupt(0,chay,RISING);
    noInterrupts();           // disable all interrupts  
  // setup_timer1();
    while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  interrupts(); 
  
  pinMode(pul1,OUTPUT);
  pinMode(dir1,OUTPUT);
  pinMode(pul2,OUTPUT);
  pinMode(dir2,OUTPUT);  
  
  setup_timer2();
  setup_timer3();

}
void serialEvent() {
  String test1 = "";

  if(Serial.available() > 0){
    test1 = Serial.readStringUntil(',');// Read the1
    // Serial.println(test1);
    if(test1 != " ")
    {
      goc_dc = (float)(test1.toFloat());
      Serial.print(" the1: " );
      Serial.println(goc_dc);
    }
    test1 = Serial.readStringUntil(',');// Read the1
    if(test1 != " ")
    {
      the2=(float)(test1.toFloat());
      Serial.print(" the2: ");
      Serial.println(the2);
      setup_timer1();
    }
    test1 = Serial.readStringUntil(',');// Read the1
    if(test1 != " ")
    {
      the3=(float)(test1.toFloat());
      Serial.print(" the3: ");
      Serial.println(the3);
    }
}
}

void pid_1(){
    vitrimm=goc_dc*((2688*4)/360);
  error=vitrimm-vitri;
  quay(pid(error,7.5,0.1,0.1));
}
void step_1(){
  V2=(the2*6400*x*3)/360;
  Serial.println(V2);
  state1=!state1;
  digitalWrite(pul1,state1);
  if (the2 >0) digitalWrite(dir1,1);
  else digitalWrite(dir1,0);
  stepCount_1++;
  if (stepCount_1/2 >= abs(V2)) {
    stepCount_1 = 0;
    the2=0;
    V2=0;
    TCCR1A = 0;
    TCCR1B = 0;
    TIMSK1 = 0;
}
}
void step_2(){
  V3=(the3*3200*3)/360;
  if (the3 >0) digitalWrite(dir2,1);
  else digitalWrite(dir2,0);
  state2=!state2;
  digitalWrite(pul2,state2);
  stepCount2++;
  if (stepCount2/2 >= abs(V3)) {
    stepCount2 = 0;
    the3=0;
    V3=0;
    // Serial.println("Ngắt timer3");
    TCCR3A = 0;
    TCCR3B = 0;
    TIMSK3 = 0;
  }
}
void setup_timer1() {
    /* Reset Timer/Counter1 */
    TCCR1A = 0;
    TCCR1B = 0;
    TIMSK1 = 0;

    /* Setup Timer/Counter1 in CTC mode with prescaler = 64 */
    TCCR1B |= (1 << WGM12) | (1 << CS11) | (0<< CS10);      //     1/8
    OCR1A = 70; // Set the compare match value
    TIMSK1 |= (1 << OCIE1A); 
}

void setup_timer2() {
    /* Reset Timer/Counter2 */
    TCCR2A = 0;
    TCCR2B = 0;
    TIMSK2 = 0;

    /* Setup Timer/Counter2 in CTC mode with prescaler = 64 */
    TCCR2A |= (1 << WGM21); // Choose CTC mode
    TCCR2B |=  (1 << CS21) | (1 << CS20); // prescaler = 64
    OCR2A = 70; // Set the compare match value
    TIMSK2 |= (1 << OCIE2A); // Enable Output Compare Interrupt for Timer2 channel A
}

void setup_timer3() {
    /* Reset Timer/Counter3 */
    TCCR3A = 0;
    TCCR3B = 0;
    TIMSK3 = 0;

    /* Setup Timer/Counter3 in CTC mode with prescaler = 64 */
    TCCR3B |= (1 << WGM32) | (1 << CS31) | (1 << CS30); // Corrected register name and prescaler values          1/64
    OCR3A = 400; // Set the compare match value
    TIMSK3 |= (1 << OCIE3A);
}

ISR (TIMER2_COMPA_vect) 
{ 
  // Serial.println("timer2");
  pid_1();
}
ISR (TIMER1_COMPA_vect) 
{
  // Serial.println("timer1");
  // step_1();
  V2=(the2*6400*x*3)/360;
  Serial.println(V2);
  state1=!state1;
  digitalWrite(pul1,state1);
  if (the2 >0) digitalWrite(dir1,1);
  else digitalWrite(dir1,0);
  stepCount_1++;
  if (stepCount_1/2 >= abs(V2)) {
    stepCount_1 = 0;
    the2=0;
    V2=0;
    TCCR1A = 0;
    TCCR1B = 0;
    TIMSK1 = 0;
}


}
ISR (TIMER3_COMPA_vect) 
{
  // Serial.println("timer3");
  step_2();

}
void loop()
{
  
  
  // Serial.println("1");
  // Serial.println(stepCount/((6400*3)/360));
}