#include <EEPROM.h>
volatile byte Mode;

// LED strip
#include <NeoPixelBus.h>
const uint8_t PixelPin = 3;
const uint16_t PixelCount = 19+35+19; // 73
NeoPixelBus<NeoGrbFeature, NeoWs2812xMethod> strip(PixelCount, PixelPin);

// Some variables
byte brC = 8;         // brightnes 1 maximum (for encoder)
float Br;             // HSL Max 0.5/brC = level
unsigned int modeR = 30;     // cyclical change of modes in seconds (for first time)
byte modeL = 1;
float cLim = 768;    // color for 1/random() 3,6,12,24,48,96,192,384,768


////
// <
////

float Map(float x, float in_min, float in_max, float out_min, float out_max)
{
  if (in_min == in_max) return out_max;
  if (x >= in_max) return out_max;
  if (x <= in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float Brightness(byte b) {
  float x = Map(b, 1, 11, 1, 0);
  if (x == 0) x = 1. / b;
  Serial.println(x, 3);
  return x;
}

void setup() {
  Serial.begin(115200);

  pinMode(A0, INPUT);
  randomSeed(analogRead(1)*analogRead(2)*analogRead(3));

  //  delay (2000);
  Serial.println("Start");

  // this resets all the neopixels to an off state
  strip.Begin();
  strip.Show();

#if defined(__AVR_ATmega32U4__)     //Выключение светодиодов на Pro Micro
  delay (1000);                     //При питании по usb от компьютера нужна задержка перед выключением RXLED. Если питать от БП, то можно убрать эту строку.
  TXLED1;                           //на ProMicro выключим и TXLED
#define LED_BUILTIN 17
  digitalWrite(LED_BUILTIN, 1);
#endif

Br = Brightness(brC);
Mode = random(3);

/*
  while(1){
    for(byte i=0;i<PixelCount;i++){
//      RgbColor color = HsbColor(1/3., 1, Br);
      RgbColor color = RgbColor(0,255,0);
       strip.SetPixelColor(i, color);
    }
      strip.Show();
  }
// */ 

}

byte bb;
byte bMode;
byte PCL;
unsigned long timeA,timeB,timeL;
unsigned int cs;
unsigned int nRB=1,nDL;

void loop() {

  unsigned long tNow = millis();
  unsigned long tEnd = tNow;

  if ( tNow - timeA > cs ) {
    timeA = tNow;
    cs = 25;
    PCL = PixelCount;
    switch (Mode) {
      case 2:
        m3(0);
        break;
      case 1:
        m3(0);
        break;
      default:
        m5(0);
    }

    strip.Show();

  }
   

  // blink
  if ( tNow - timeB > 1000 ) {
    timeB = tNow;
    if (bb) {
      digitalWrite(LED_BUILTIN, 0);
      bb = 0;
    }
    else {
      digitalWrite(LED_BUILTIN, 1);
      bb = 1;
    }

//     Serial.print(modeL); Serial.print(" "); Serial.print(Mode); Serial.print(" "); Serial.println(modeR-(tNow-timeL)/1000);
  } //

  // random Mode
  if ( (tNow - timeL) / 1000 > modeR) {
    timeL = tNow;
    modeR = 900; //random(29,300);
    modeL = Mode;
    while( Mode == modeL ) Mode = random(1,3);
    nRB=1;
    Serial.print(Mode); Serial.print(" "); Serial.println(modeR);
  } 

} //end loop()


/////////////////////////////////////////////////////////////////////////////


void blank(byte x) {
  for (int i = 0; i < PCL; i++) {
    strip.SetPixelColor(i, 0);
  }
  if (x) strip.Show();
}

float lastRB,nextRB,getRB;
byte sW=0;
void m3(byte x){
    if(nRB==0 && nDL++<600){
      return;
    }
    if(nRB++ >= 50){
      nRB=0;
      nDL=0;
      if(x){
        if(++sW>1)sW=0;
        HsbColor px = RgbColor(strip.GetPixelColor(sW));
        getRB = px.H;
      } else {
        sW=0;
        getRB = lastRB;
      }
      byte c=1;
      while(abs(nextRB-getRB)<0.08 || abs(nextRB-lastRB)<0.08 || c==1){
        nextRB = random(cLim+1) / cLim;
        c=0;
      }
      lastRB = nextRB;
    }
    float nn = Map(nRB, 0, 50, 0, 1);
    RgbColor res = RgbColor::LinearBlend(HsbColor(getRB, 1, Br), HsbColor(nextRB, 1, Br), nn);
    for(byte i=0;i<PCL;i+=x+1){
        strip.SetPixelColor(i+sW, res);
    }
}

int stp=-1;
void m5(byte x) {
  if(nRB==1){
    nRB=0;
    for(byte i=0;i<PCL;i++){
      stp+=1;
      if(stp>96) stp=0; 
      float nn = Map(stp, 0, 96, 1, 0);
      RgbColor color = HsbColor(nn, 1, Br);
//      strip.SetPixelColor(i, color);
    }
  }
    
    stp+=1;
    if(stp>96) stp=0; 
    float nn = Map(stp, 0, 96, 1, 0);
    RgbColor res = HsbColor(nn, 1, Br);
    if(x==0){
      strip.ShiftRight(1);
    } else {
      strip.ShiftLeft(1);
    }
    strip.SetPixelColor((PCL-1)*x, res);
}
