#include <EEPROM.h>
volatile byte Mode;

// LED strip
#include <NeoPixelBus.h>
const uint8_t PixelPin = 9;
const uint16_t PixelCount = 60;
NeoPixelBus<NeoGrbwFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);

// Encoder
#define S1 2
#define S2 3
#define KEY 4
#define PWR 5
#include <EncButton.h>
EncButton<EB_TICK, S1, S2, KEY> enc;

// Some variables
byte brC = 6;         // brightnes 1 maximum (for encoder)
float Br;             // HSL Max 0.5/brC = level
unsigned int modeR = 180;     // cyclical change of modes in seconds (for first time)
byte modeL = 1;
float cLim = 768;    // color for 1/random() 3,6,12,24,48,96,192,384,768
unsigned int rainbow = 48;

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

  // Encoder
  pinMode(PWR, OUTPUT);
  digitalWrite(PWR, 1);
  attachInterrupt(digitalPinToInterrupt(S1), isr, CHANGE);  // D2
  attachInterrupt(digitalPinToInterrupt(S2), isr, CHANGE);  // D3

  EEPROM.get(1, brC);
  EEPROM.get(0, Mode);

  Br = Brightness(brC);


/*
  while(1){
    for(byte i=0;i<PixelCount;i++){
      RgbColor color = HsbColor(1/3., 1, Br);
//      RgbColor color = RgbColor(0,255,0);
       strip.SetPixelColor(i, color);
    }
      strip.Show();
  }
// */

}

void isr() {
  enc.tick();  // отработка в прерывании
}

byte bb;
byte bMode;
unsigned long timeA,timeB,timeL;
unsigned int cs;
unsigned int nRB, nDL;

void loop() {

  // обязательная функция отработки. Должна постоянно опрашиваться
  enc.tick();
  encoder();

  unsigned long tNow = millis();
  unsigned long tEnd = tNow;

  if ( tNow - timeA > cs ) {
    timeA = tNow;
    cs = 260;
    switch (Mode) {
      case 8:
        blank(0);
        break;
      case 7:
        m5(1);
        break;
      case 6:
        m4(1);
        break;
      case 5:
        m5(0);
        break;
      case 4:
        cs = 26;
        m3(1);
        break;
      case 3:
        m4(0);
        break;
      case 2:
        cs = 26;
        m3(0);
        break;
      case 1:
        m0();
        break;
      default:
        m2();
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
  if ( (tNow - timeL) / 1000 > modeR && Mode!=0 && Mode!=8) {
    timeL = tNow;
    modeR = random(29,300);
    modeL = Mode;
    while( Mode == modeL ) Mode = random(1,8);
    rainbow = random(6,PixelCount);
    nRB=1;
    Serial.print(Mode); Serial.print(" "); Serial.println(modeR);
  } 

} //end loop()


/////////////////////////////////////////////////////////////////////////////


void encoder() {
  if (enc.isRight()) {
    brC--;
    if (brC < 1) brC = 1;
    Br = Brightness(brC);
    EEPROM.put(1, brC);
  } else if (enc.isLeft()) {
    brC++;
    Br = Brightness(brC);
    EEPROM.put(1, brC);
  } else if (enc.isClick()) {
    Mode++;
    if (Mode > 8){ Mode=0; }
//    timeL = millis();    
    nRB=1;
    EEPROM.put(0, Mode);
  }
}


void blank(byte x) {
  for (int i = 0; i < PixelCount; i++) {
    strip.SetPixelColor(i, 0);
  }
  if (x) strip.Show();
}

void m0() {
    for(byte i=0;i<PixelCount;i++){
     if(random(3)==1){
      float RB = random(cLim+1) / cLim;
      RgbColor color = HsbColor(RB, 1, Br);
      strip.SetPixelColor(i, color);
     }
    }
}

// white
void m2() {
//    HsbColor cW = RgbColor(255,144,64);
    for(byte i=0;i<PixelCount;i++){
//      RgbColor color = HsbColor(cW.H, cW.S, Br);
      RgbwColor color = RgbwColor(Br*255);
      strip.SetPixelColor(i, color);
    }
}

float lastRB,nextRB,getRB;
byte sW=0;
void m3(byte x){
    if(nRB==0 && nDL++<300){
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
    for(byte i=0;i<PixelCount;i+=x+1){
        strip.SetPixelColor(i+sW, res);
    }
}

void m4(byte x){
  float RB;
  if(x==0){
    strip.ShiftRight(1);
  } else {
    strip.ShiftLeft(1);
  }
  if(nRB==1){
    nRB=0;
    for(byte i=0;i<PixelCount;i++){
      RB = random(cLim+1) / cLim;
      RgbColor color = HsbColor(RB, 1, Br);
      strip.SetPixelColor(i, color);
    }
  } else {
    RB = random(cLim+1) / cLim;
    RgbColor color = HsbColor(RB, 1, Br);
    strip.SetPixelColor((PixelCount-1)*x, color);
  }
}

int stp=-1;
void m5(byte x) {

  if(nRB==1){
    nRB=0;
    for(byte i=0;i<PixelCount;i++){
      stp+=1;
      if(stp>rainbow) stp=0;
      float nn = Map(stp, 0, rainbow, 1, 0);
      RgbColor color = HsbColor(nn, 1, Br);
      strip.SetPixelColor(i, color);
    }
  }
    
    stp+=1;
    if(stp>rainbow) stp=0; 
    float nn = Map(stp, 0, rainbow, 1, 0);
 //Serial.println(nn,3);
    RgbColor res = HsbColor(nn, 1, Br);
    if(x==0){
      strip.ShiftRight(1);
    } else {
      strip.ShiftLeft(1);
    }
    strip.SetPixelColor((PixelCount-1)*x, res);
}

byte mtr[][2]={ {1,4},{6,12},{14,16},{19,24},{27,29},{33,35},{39,49} };
byte lsW;
void m6(){
    if(nRB==0 && nDL++<500){
      return;
    }
    if(nRB==0) blank(0);
    float nn = Map(nRB, 0, 50, 0, Br);
    RgbwColor res = RgbwColor(nn*255);
    for(byte i=mtr[sW][0]-1;i<=mtr[sW][1]-1;i++){
        strip.SetPixelColor(i, res);
    }
    if(nRB++ >= 50){
      nRB=0;
      nDL=0;
      while(lsW == sW) sW=random(7);
      lsW=sW;
    }
}
