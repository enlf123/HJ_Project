// MCP3204 ADC(12bit) Read

#define CS 4
#define MOSI 5
#define MISO  6
#define CLK 7
#define CH 1 // select ch 0~
int readvalue; 

double ang =0.0;
double preset[5];
double avg_preset = 0;

void setup(){ 
 //set pin modes 
 pinMode(CS, OUTPUT); 
 pinMode(MOSI, OUTPUT); 
 pinMode(MISO, INPUT); 
 pinMode(SPICLOCK, OUTPUT); 
 //disable device to start with 
 digitalWrite(CS,HIGH); 
 digitalWrite(MOSI,LOW); 
 digitalWrite(CLK,LOW); 

 Serial.begin(115200);

 // Initialization for removing bias
 for(int i=0;i<5;i++){
   readvalue = read_adc(1); 
 ang = (double)readvalue*360/4095.0;
 preset[i] =ang;
 }
 avg_preset = (preset[0]+preset[1]+preset[2]+preset[3]+preset[4])/5;
} 

// reading absoulte angle through ezEncoder(magnetic angle sensor)
void loop() { 
 readvalue = read_adc(1); 
 ang = (double)readvalue*360/4095.0;
 Serial.println(ang-avg_preset); 
 delay(10); 
} 

//function
int read_adc(int channel){
  int adcvalue = 0;
  byte commandbits = B11000000; //command bits - start, mode, chn (3), dont care (3)

  //allow channel selection
  commandbits|=((channel-1)<<3);

  digitalWrite(CS,LOW); //Select adc
  // setup bits to be written
  for (int i=7; i>=3; i--){
    digitalWrite(MOSI,commandbits&1<<i);
    //cycle clock
    digitalWrite(CLK,HIGH);
    digitalWrite(CLK,LOW);    
  }

  digitalWrite(CLK,HIGH);    //ignores 2 null bits
  digitalWrite(CLK,LOW);
  digitalWrite(CLK,HIGH);  
  digitalWrite(CLK,LOW);

  //read bits from adc
  for (int i=11; i>=0; i--){
    adcvalue+=digitalRead(MISO)<<i;
    //cycle clock
    digitalWrite(CLK,HIGH);
    digitalWrite(CLK,LOW);
  }
  digitalWrite(CS, HIGH); //turn off device
  return adcvalue;
}
