// MCP3204 ADC(12bit) Read using Arduino Mega

#define CS 10 
#define DI 11
#define DO 12
#define SPICLOCK  13//Clock 

int ch = 0; // select ch 0~
int readvalue; 

double ang =0.0;
double preset[5];
double avg_preset = 0;
unsigned long pre, cur;


void setup(){ 
 //set pin modes 
 pinMode(CS, OUTPUT); 
 pinMode(DI, OUTPUT); 
 pinMode(DO, INPUT); 
 pinMode(SPICLOCK, OUTPUT); 
 //disable device to start with 
 digitalWrite(CS,HIGH); 
 digitalWrite(DI,LOW); 
 digitalWrite(SPICLOCK,LOW); 

 Serial.begin(115200);

 // Initialization for removing bias
 for(int i=0;i<5;i++){
   readvalue = read_adc(ch); 
 ang = (double)readvalue*360/4095.0;
 preset[i] =ang;
 }
 avg_preset = (preset[0]+preset[1]+preset[2]+preset[3]+preset[4])/5;
 pre = micros();
} 

// reading absoulte angle through ezEncoder(magnetic angle sensor)
void loop() { 
  cur = micros();
  if (cur-pre > 2000){
 readvalue = read_adc(ch); 
 ang = (double)readvalue*360/4095.0;
 //Serial.println(ang-avg_preset); 
 Serial.println(cur-pre);
 pre = micros();
  }
} 

//function
int read_adc(int channel){
  int adcvalue = 0;
  byte commandbits = B11000000; //command bits - start, mode, chn (3), dont care (3)

  //allow channel selection
  commandbits|=((channel)<<3);

  digitalWrite(CS,LOW); //Select adc
  // setup bits to be written
  for (int i=7; i>=3; i--){
    digitalWrite(DI,commandbits&1<<i);
    //cycle clock
    digitalWrite(SPICLOCK,HIGH);
    digitalWrite(SPICLOCK,LOW);    
  }

  digitalWrite(SPICLOCK,HIGH);    //ignores 2 null bits
  digitalWrite(SPICLOCK,LOW);
  digitalWrite(SPICLOCK,HIGH);  
  digitalWrite(SPICLOCK,LOW);

  //read bits from adc
  for (int i=11; i>=0; i--){
    adcvalue+=digitalRead(DO)<<i;
    //cycle clock
    digitalWrite(SPICLOCK,HIGH);
    digitalWrite(SPICLOCK,LOW);
  }
  digitalWrite(CS, HIGH); //turn off device
  return adcvalue;
}
