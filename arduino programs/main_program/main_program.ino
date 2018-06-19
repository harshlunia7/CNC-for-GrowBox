#define mxen_a 5
#define mxa1 25
#define mxa2 24
#define v_pump_1 12
#define v_pump_2 13
#define v_pump_en 15
int encoderPin1 = 21;
int encoderPin2 = 20;

long count_x=0;
unsigned long timep,timec,etime;
byte state_x ,statep_x;
boolean A,B;
volatile int QEM[16]={0,-1,0,1,1,0,-1,0,0,1,0,-1,-1,0,1,0};
volatile int index_x;
float rot_deg_x=0;
int CPR_x=1680;
float x_current=0,x_past=0,x_target;
void state_change_x()
{
 A=digitalRead(encoderPin1);
 B=digitalRead(encoderPin2);

  if( (A==HIGH) && (B==HIGH) ) state_x=0;
  if( (A==HIGH) && (B==LOW) ) state_x=1;
  if( (A==LOW) && (B==LOW) ) state_x=2;
  if( (A==LOW) && (B==HIGH) ) state_x=3;

 index_x=4*state_x+statep_x;
 count_x=count_x+QEM[index_x];
 statep_x=state_x;
}


void setup() 
{
  Serial.begin (9600);

  pinMode(mxen_a,OUTPUT);
  pinMode(mxa1,OUTPUT);
  pinMode(mxa2,OUTPUT);
  pinMode(encoderPin1, INPUT_PULLUP); 
  pinMode(encoderPin2, INPUT_PULLUP);

 pinMode(v_pump_1,OUTPUT);
  pinMode(v_pump_2,OUTPUT);
  pinMode(v_pump_en,OUTPUT);
  
 timep=micros();
 A=digitalRead(encoderPin1);
 B=digitalRead(encoderPin2);

  if( (A==HIGH) && (B==HIGH) ) statep_x=0;
  if( (A==HIGH) && (B==LOW) ) statep_x=1;
  if( (A==LOW) && (B==LOW) ) statep_x=2;
  if( (A==LOW) && (B==HIGH) ) statep_x=3;
 
  
  attachInterrupt(2, state_change_x, CHANGE); 
  attachInterrupt(3, state_change_x, CHANGE);

}

void loop()
{ 

     /* digitalWrite(mxa1,HIGH);
      digitalWrite(mxa2,LOW);
      analogWrite(mxen_a,255);
      
      digitalWrite(v_pump_1,LOW);
      digitalWrite(v_pump_2,HIGH);
      analogWrite(v_pump_en,255); */
        Serial.print("please enter the distance in mm ");
while(Serial.available()==0){}
         x_current=Serial.parseInt();
        
       go_to_x(x_current);

       Serial.print("   you are now at :  "  );
       Serial.println(x_current);
       
      timec=micros();
      etime=timec-timep;
      if(etime>1000000) 
      {
        // Serial.println(count);
          //rot_deg=count*360/(CPR);
          //Serial.println(rot_deg);
         timep=timec;  
      }
  
}

void go_to_x(float x_current)
{
int  x=0;
count_x=0;
            x_target=x_current-x_past;
           Serial.print("  x_target:");
          Serial.print(x_target);
          
        while(x<x_target && x_target>0)  //forward or in positive direction  or clockwise
        {
           digitalWrite(mxa1,HIGH);
           digitalWrite(mxa2,LOW);
           analogWrite(mxen_a,255);
          rot_deg_x=count_x*360/(CPR_x);
          x=rot_deg_x*(20*7.85398)/(360);        //  x=deg*no of teeth*pitch/360
        } 
          
      while(x>x_target && x_target<0)  //backward or in negative direction  or anti-clockwise
        {
           digitalWrite(mxa1,LOW);
           digitalWrite(mxa2,HIGH);
           analogWrite(mxen_a,255);
          rot_deg_x=count_x*360/(CPR_x);
          x=rot_deg_x*(20*7.85398)/(360);        //  x=deg*no of teeth*pitch/360
        } 
        
           digitalWrite(mxa1,LOW);
           digitalWrite(mxa2,LOW);
          
           Serial.print("   distance moved :");
          Serial.print(x);
  x_past=x_current;
}

