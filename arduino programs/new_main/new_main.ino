#define solenoidPin 10  

#define mxen_a 4
#define mxa1 22
#define mxa2 23
#define encoderPin1_x 2
#define encoderPin2_x 3

#define myen_b 5
#define myb1 25
#define myb2 24
#define encoderPin1_y 21
#define encoderPin2_y 20

#define mzen_a 6
#define mza1 27
#define mza2 26
#define encoderPin1_z 18
#define encoderPin2_z 19

#define v_pump_en 7
#define v_pump_b1 28
#define v_pump_b2 29




long count_x=0,count_y=0,count_z=0;
unsigned long timep,timec,etime;
byte state_x ,statep_x,state_y,statep_y,state_z,statep_z;
boolean A_x,B_x,A_y,B_y,A_z,B_z;
volatile int QEM[16]={0,-1,0,1,1,0,-1,0,0,1,0,-1,-1,0,1,0};
volatile int index_x,index_y,index_z;

float rot_deg_x=0,rot_deg_y=0,rot_deg_z=0;
int CPR_x=1680,CPR_y=1680,CPR_z=1680;

float x_current=0,x_past=0,x_target;
float y_current=0,y_past=0,y_target;
float z_current=0,z_past=600,z_target;

char data[14];
int str[14];

int water,seed;

void state_change_x()
{
 A_x=digitalRead(encoderPin1_x);
 B_x=digitalRead(encoderPin2_x);

  if( (A_x==HIGH) && (B_x==HIGH) ) state_x=0;
  if( (A_x==HIGH) && (B_x==LOW) ) state_x=1;
  if( (A_x==LOW) && (B_x==LOW) ) state_x=2;
  if( (A_x==LOW) && (B_x==HIGH) ) state_x=3;

 index_x=4*state_x+statep_x;
 count_x=count_x+QEM[index_x];
 statep_x=state_x;
}

void state_change_y()
{
 A_y=digitalRead(encoderPin1_y);
 B_y=digitalRead(encoderPin2_y);

  if( (A_y==HIGH) && (B_y==HIGH) ) state_y=0;
  if( (A_y==HIGH) && (B_y==LOW) ) state_y=1;
  if( (A_y==LOW) && (B_y==LOW) ) state_y=2;
  if( (A_y==LOW) && (B_y==HIGH) ) state_y=3;

 index_y=4*state_y+statep_y;
 count_y=count_y+QEM[index_y];
 statep_y=state_y;
}

void state_change_z()
{
 A_z=digitalRead(encoderPin1_z);
 B_z=digitalRead(encoderPin2_z);

  if( (A_z==HIGH) && (B_z==HIGH) ) state_z=0;
  if( (A_z==HIGH) && (B_z==LOW) ) state_z=1;
  if( (A_z==LOW) && (B_z==LOW) ) state_z=2;
  if( (A_z==LOW) && (B_z==HIGH) ) state_z=3;

 index_z=4*state_z+statep_z;
 count_z=count_z+QEM[index_z];
 statep_z=state_z;
}


void setup() 
{
  Serial.begin (9600);
   pinMode(solenoidPin, OUTPUT); 
   
  pinMode(mxen_a,OUTPUT);
  pinMode(mxa1,OUTPUT);
  pinMode(mxa2,OUTPUT);
  pinMode(encoderPin1_x, INPUT_PULLUP); 
  pinMode(encoderPin2_x, INPUT_PULLUP);
  
  pinMode(myen_b,OUTPUT);
  pinMode(myb1,OUTPUT);
  pinMode(myb2,OUTPUT);
  pinMode(encoderPin1_y, INPUT_PULLUP); 
  pinMode(encoderPin2_y, INPUT_PULLUP);
  
  pinMode(mzen_a,OUTPUT);
  pinMode(mza1,OUTPUT);
  pinMode(mza2,OUTPUT);
  pinMode(encoderPin1_z, INPUT_PULLUP); 
  pinMode(encoderPin2_z, INPUT_PULLUP);

  pinMode(v_pump_en,OUTPUT);
  pinMode(v_pump_b1,OUTPUT);
  pinMode(v_pump_b2,OUTPUT);
  
  
 timep=micros();
 
  A_x=digitalRead(encoderPin1_x);
  B_x=digitalRead(encoderPin2_x);

  if( (A_x==HIGH) && (B_x==HIGH) ) state_x=0;
  if( (A_x==HIGH) && (B_x==LOW) ) state_x=1;
  if( (A_x==LOW) && (B_x==LOW) ) state_x=2;
  if( (A_x==LOW) && (B_x==HIGH) ) state_x=3;
 
  A_y=digitalRead(encoderPin1_y);
  B_y=digitalRead(encoderPin2_y);

  if( (A_y==HIGH) && (B_y==HIGH) ) state_y=0;
  if( (A_y==HIGH) && (B_y==LOW) ) state_y=1;
  if( (A_y==LOW) && (B_y==LOW) ) state_y=2;
  if( (A_y==LOW) && (B_y==HIGH) ) state_y=3;
  
  A_z=digitalRead(encoderPin1_z);
  B_z=digitalRead(encoderPin2_z);

  if( (A_z==HIGH) && (B_z==HIGH) ) state_z=0;
  if( (A_z==HIGH) && (B_z==LOW) ) state_z=1;
  if( (A_z==LOW) && (B_z==LOW) ) state_z=2;
  if( (A_z==LOW) && (B_z==HIGH) ) state_z=3;
  
  attachInterrupt(0, state_change_x, CHANGE); 
  attachInterrupt(1, state_change_x, CHANGE);
  
  attachInterrupt(2, state_change_y, CHANGE); 
  attachInterrupt(3, state_change_y, CHANGE);
  
  
  attachInterrupt(4, state_change_z, CHANGE); 
  attachInterrupt(5, state_change_z, CHANGE);

}

void loop()
{
 
  
  for(int i=0;i<=13;i++)
  {
   while(Serial.available()==0) {}
   data[i]=Serial.read();
   if(data[i]=='!')
      {
          if(i!=13) Serial.println("Sorry, data lost or incorrect data");
          break;
      } 
  }
   Serial.print("Data received is :");
  for(int i=0;i<=13;i++)
  {
   Serial.print(data[i]);
  }
  
  Serial.println(""); 
  
  if(data[0]=='@' && data[13]=='!')  
  {
    get_parameters();
    
    Serial.print("x : ");
    Serial.println(x_current);
    Serial.print("y : ");
    Serial.println(y_current);
    Serial.print("z : ");
    Serial.println(z_current);
    Serial.print("water  : ");
    Serial.println(water);
    Serial.print("seed : ");
    Serial.println(seed);
    
     if(seed==1) start_seeder();
    go_to_x(x_current);
    go_to_y(y_current);
    go_to_z(z_current);
    
    if(seed==1)
    {
      digitalWrite(v_pump_b1,LOW); // pump  off after going to perfect location
      digitalWrite(v_pump_b2,LOW);
    }
   

       Serial.println("   you are now at :    "  );
       Serial.print("x = ");
       Serial.println(x_current);
       Serial.print("y = ");
       Serial.println(y_current);
       Serial.print("z = ");
       Serial.println(z_current);
       
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
     
 
  

//if(var=='y') { digitalWrite(solenoidPin, LOW);}
//if(var=='n')  { digitalWrite(solenoidPin, HIGH);}
}

void get_parameters()
{
 for(int i=0;i<=13;i++)
  {
    str[i]=(data[i])-48;  // conversion from ascii to int 
  }
    x_current=1000*str[1]+100*str[2]+10*str[3]+1*str[4];
    y_current=100*str[5]+10*str[6]+1*str[7];
    z_current=100*str[8]+10*str[9]+1*str[10];
    
    water=str[11];
    seed=str[12];
    
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
          
           Serial.print("   distance moved in x :");
          Serial.println(x);
  x_past=x_current;
}

void go_to_y(float y_current)
{
int  y=0;
count_y=0;
            y_target=y_current-y_past;
           Serial.print("  y_target:");
          Serial.print(y_target);
          
        while(y<y_target && y_target>0)  //forward or in positive direction  or clockwise
        {
           digitalWrite(myb1,HIGH);
           digitalWrite(myb2,LOW);
           analogWrite(myen_b,255);
          rot_deg_y=count_y*360/(CPR_y);
          y=rot_deg_y*(20*7.85398)/(360);        //  y=deg*no of teeth*pitch/360
        } 
          
      while(y>y_target && y_target<0)  //backward or in negative direction  or anti-clockwise
        {
           digitalWrite(myb1,LOW);
           digitalWrite(myb2,HIGH);
           analogWrite(myen_b,255);
          rot_deg_y=count_y*360/(CPR_y);
          y=rot_deg_y*(20*7.85398)/(360);        //  y=deg*no of teeth*pitch/360
        } 
        
           digitalWrite(myb1,LOW);
           digitalWrite(myb2,LOW);
          
           Serial.print("   distance moved in y :");
          Serial.println(y);
  y_past=y_current;
}


void go_to_z(float z_current)
{
int  z=0;
count_z=0;
            z_target=z_current-z_past;
           Serial.print("  z_target:");
          Serial.print(z_target);
          
        while(z<z_target && z_target>0)  //forward or in positive direction  or clockwise
        {
           digitalWrite(mza1,HIGH);
           digitalWrite(mza2,LOW);
           analogWrite(mzen_a,255);
          rot_deg_z=count_z*360/(CPR_z);
          z=rot_deg_z*(20*7.85398)/(360);        //  z=deg*no of teeth*pitch/360
        } 
          
      while(z>z_target && z_target<0)  //backward or in negative direction  or anti-clockwise
        {
           digitalWrite(mza1,LOW);
           digitalWrite(mza2,HIGH);
           analogWrite(mzen_a,255);
          rot_deg_z=count_z*360/(CPR_z);
          z=rot_deg_z*(20*7.85398)/(360);        //  z=deg*no of teeth*pitch/360
        } 
        
           digitalWrite(mza1,LOW);
           digitalWrite(mza2,LOW);
          
           Serial.print("   distance moved in z :");
          Serial.println(z);
  z_past=z_current;
}

void start_seeder()
{    
        go_to_x(50);   // going to location of seed compartment 
        go_to_y(225);
        go_to_z(0);
       digitalWrite(v_pump_b1,LOW);  // vaccum pump starts and picks up seed
      digitalWrite(v_pump_b2,HIGH);
      analogWrite(v_pump_en,255); 
      delay(5000);
      go_to_z(600);     //arm retracting 
  
}
