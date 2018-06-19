#define m1en_a 9
#define m1a1 8
#define m1a2 7

int encoderPin1 = 2;
int encoderPin2 = 3;

long count=0;
unsigned long timep,timec,etime;
byte state ,statep;
boolean A,B;
volatile int QEM[16]={0,-1,0,1,1,0,-1,0,0,1,0,-1,-1,0,1,0};
volatile int index;
float rot_deg=0;
int CPR=1680;

void state_change()
{
 A=digitalRead(encoderPin1);
 B=digitalRead(encoderPin2);

  if( (A==HIGH) && (B==HIGH) ) state=0;
  if( (A==HIGH) && (B==LOW) ) state=1;
  if( (A==LOW) && (B==LOW) ) state=2;
  if( (A==LOW) && (B==HIGH) ) state=3;

 index=4*state+statep;
 count=count+QEM[index];
 statep=state;
}


void setup() 
{
  Serial.begin (9600);

  pinMode(m1en_a,OUTPUT);
  pinMode(m1a1,OUTPUT);
  pinMode(m1a2,OUTPUT);
  pinMode(encoderPin1, INPUT_PULLUP); 
  pinMode(encoderPin2, INPUT_PULLUP);

 timep=micros();
 A=digitalRead(encoderPin1);
 B=digitalRead(encoderPin2);

  if( (A==HIGH) && (B==HIGH) ) statep=0;
  if( (A==HIGH) && (B==LOW) ) statep=1;
  if( (A==LOW) && (B==LOW) ) statep=2;
  if( (A==LOW) && (B==HIGH) ) statep=3;
 
  
  attachInterrupt(0, state_change, CHANGE); 
  attachInterrupt(1, state_change, CHANGE);

}

void loop()
{ 

      digitalWrite(m1a1,HIGH);
      digitalWrite(m1a2,LOW);
      analogWrite(m1en_a,255);

      timec=micros();
      etime=timec-timep;
      if(etime>1000000) 
      {
        // Serial.println(count);
          rot_deg=count*360/(CPR);
          Serial.println(rot_deg);
         timep=timec;  
      }
  
}



