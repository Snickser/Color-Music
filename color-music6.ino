#define LIN_OUT 1
//#define LOG_OUT 1
#define FHT_N 128  //set to 256 point fht
#include <FHT.h>   //include the library
int colorMusic[5];
int LOWPASS[5];
unsigned long LOWP[FHT_N / 2];
unsigned long TLP[5];

#include <EEPROM.h>
byte Mode;

//#define DEBUG

byte cs = 23;        // millis
float pfMax = 1.08;
byte maxL = 5;       // levels for m2()
float modeR = 4;      // cyclical change of modes in minutes

// mic
#define AMP 7
#define analogInPin A0

//#define PROD

// LED strip
#include <NeoPixelBus.h>
const uint8_t PixelPin = 9;
byte pxRatio = 5;     // 1 for 30Led/m, 2 for 60Led/m, 4 for 100L/m, 8 for 144L/m
#ifdef PROD
const uint16_t PixelCount = 80;
#else
const uint16_t PixelCount = 100;
#endif
#ifdef PROD
byte brC = 3;         // brightnes 1 maximum, 10 minimum (for encoder)
#else
byte brC = 6;         // brightnes 1 maximum, 10 minimum (for encoder)
#endif
float Br; // HSL Max 0.5/brC = level
#ifdef PROD
NeoPixelBus<NeoRgbFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);
#else
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);
#endif

// Encoder
#define S1 2
#define S2 3
#define KEY 4
#define PWR 5
#include <EncButton.h>
EncButton<EB_TICK, S1, S2, KEY> enc;

// Some variables
HsbColor rnd;
float RB, RB2;
float bwL;
byte Nc, PCV;
unsigned long bwD;
int PCL;
float PCD;
float pfMin;
unsigned long timeL;

////
// <
////

float Map(float x, float in_min, float in_max, float out_min, float out_max)
{
  if (in_min == in_max) return out_max;
  if (x > in_max) return out_max;
  if (x < in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float Brightness(byte b) {
  float x = Map(b, 1, 9, 1, 0);
  if (x == 0) x = 1. / b;
  //  Serial.println(x, 3);
  return x;
}

void ModeSet(byte m) {
  timeL = millis();
  Nc = random(2);
  Mode = m;
  pfMin = 0.2;
  if (PixelCount % 2) {
    PCL = PixelCount / 2 + 1;
    PCV = 1;
  } else {
    PCL = PixelCount / 2;
    PCV = 0;
  }
  switch (m) {
    case 12:
      Serial.println("Mode 12: dance");
      //      EEPROM.put(0, 12);
      PCD = PCL * 1.0;
      break;
    case 11:
      Serial.println("Mode 11: flash");
      //      EEPROM.put(0, 12);
      PCL = 15; // level, not pixels
      PCD = PCL * 1.0;
      break;
    case 10:
      Serial.println("Mode 10: center + blend");
      //      EEPROM.put(0, 11);
      PCD = PCL * 1.5;
      break;
    case 9:
      Serial.println("Mode 9: linear + rainbow");
      //      EEPROM.put(0, 10);
      PCL = PixelCount;
      PCD = PCL * 0.5;
      break;
    case 8:
      Serial.println("Mode 8: center + random");
      //      EEPROM.put(0, 9);
      PCD = PCL * 1.5;
      break;
    case 7:
      Serial.println("Mode 7: flow");
      //      EEPROM.put(0, 8);
      PCL = 15; // devider, not pixels
      PCD = 1;
      pfMin = 0.1;
      break;
    case 6:
      Serial.println("Mode 6: freq five");
      //      EEPROM.put(0, 7);
      autoLowPass(2);
      break;
    case 5:
      Serial.println("Mode 5: center + rainbow");
      //      EEPROM.put(0, 6);
      PCD = PCL * 1.0;
      break;
    case 4:
      Serial.println("Mode 4: freq more");
      //      EEPROM.put(0, 5);
      autoLowPass(1);
      blank(1);
      break;
    case 3:
      Serial.println("Mode 3: linear + random");
      //      EEPROM.put(0, 4);
      PCL = PixelCount;
      PCD = PCL * 0.5;
      break;
    case 2:
      Serial.println("Mode 2: center 3");
      //      EEPROM.put(0, 3);
      PCL = 3 * maxL ;
      PCD = PCL * 0.5;
      pfMin = 0.02;
      break;
    case 1:
      Serial.println("Mode 1: linear");
      //     EEPROM.put(0, 2);
      PCL = PixelCount;
      PCD = PCL * 0.5;
      break;
    default:
      Serial.println("Mode 0: center");
      //     EEPROM.put(0, 1);
      PCD = PCL * 2.0;
      Mode = 0;
      pfMin = 0.3;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(analogInPin, INPUT);

  randomSeed(analogRead(analogInPin));

  //analogReference(INTERNAL);

  pinMode(AMP, OUTPUT);
  digitalWrite(AMP, 0);

#if defined (__AVR_ATmega32U4__) || defined(__AVR_ATmega328P__)
  // жуткая магия, меняем частоту оцифровки до 18 кГц
  // поднимаем частоту опроса аналогового порта до 38.4 кГц, по теореме
  // Котельникова (Найквиста) частота дискретизации будет 19.2 кГц
  // http://yaab-arduino.blogspot.ru/2015/02/fast-sampling-from-analog-input.html
  bitSet(ADCSRA, ADPS2);
  bitClear(ADCSRA, ADPS1);
  bitSet(ADCSRA, ADPS0);
#endif

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

  //  EEPROM.get(0, Mode);
  //  ModeSet(Mode);

  Br = Brightness(brC);
  ModeSet(6);

  /*
    while (1) {
    long aaa = millis();
    for (byte i = 0; i < 100; i++) {
    float nn = Map(i, 0, 99, 0, 1);
    RgbColor c = HsbColor::LinearBlend<NeoHueBlendClockwiseDirection>(HsbColor(0, 1, Br ), HsbColor(1, 1, Br ), nn);
    //    float cc = Map(i, 0, 99, 0, 360);
    //      HsbColor c = HsbColor(cc / 360., 1, Br );
    //      RgbColor c = RgbColor::LinearBlend(HsbColor(0, 1, Br ), HsbColor(1, 1, Br ), nn);
    //      c = colorGamma.Correct(c);
    //          RgbColor c (64,64,0);
    strip.SetPixelColor(i, c);
    }
    Serial.println(millis() - aaa);
    strip.Show();
    }
  */

}

void isr() {
  enc.tick();  // отработка в прерывании
}

int sLV, LV;
float eLV[5];
int cnt = 0;
float pmax = 1, pmin = 200;
float psum = 0;
float pavg;
float plast;
unsigned long timeB;
unsigned long timeF;
byte bb;
byte bMode;
float Lmin = 1. / 255.;
float Cmin = 1. / 360.;

void loop() {

  // обязательная функция отработки. Должна постоянно опрашиваться
  enc.tick();
  encoder();

  unsigned long tNow = millis();
  unsigned long tEnd = tNow + cs ;

  unsigned int v_max = 0;
  unsigned int v_min = 1024;
  unsigned int pcur;

  if (Mode == 4) {

    //    long aaa = millis();
    m4();
    //    Serial.println(millis() - aaa);

  } else if (Mode == 6) {

    //    long aaa = millis();
    m6();
    //        Serial.println(millis() - aaa);

  } else {

    // get audio
    while ( millis() < tEnd ) {
      int sensorValue = analogRead(analogInPin);
      if (sensorValue > v_max) {
        v_max = sensorValue;
      }
      if (sensorValue < v_min) {
        v_min = sensorValue;
      }
    }
    pcur = v_max - v_min;

    if (pcur > pmax) {
      pmax = pcur;
    } else {
      pmax *= 0.98;
    }
    if (pmax < PCL) {
      pmax = PCL;
    }

    if (pcur < pmin) {
      pmin = pcur;
    } else {
      pmin += pfMin;
    }

    if (cnt++ < 400) {
      psum += pmax;
      pavg = (psum / cnt) + PCD;
    } else {
      psum = (pavg - PCD) * 10;
      cnt = 10;
    }

    if (plast > pavg && pcur > pavg) {
      pavg += pcur;
      if (PCD > 0)PCD *= pfMax;
      else PCD += 5;
    }
    plast = pcur;

    float cr;
    if (pcur > pavg) {
      cr = pavg;
    }
    else cr = pcur;
    LV = round(Map(cr, pmin, pavg, 0, PCL));

    if (LV > sLV) {
      sLV = LV;
    } else {
      if (sLV > PCL * 0.4 && sLV > 30) {
        sLV -= 3;
      } else if (sLV > PCL * 0.3 || sLV > 10) {
        sLV -= 2;
      } else if (sLV > 0 ) {
        sLV--;
        if (sLV < PCL * 0.2) PCD -= 0.2; //*0.992;
      }
    }

#ifdef DEBUG
    Serial.print(pcur);
    Serial.print(" ");
    Serial.print(pmin);
    Serial.print(" ");
    Serial.print(pmax);
    Serial.print(" ");
    Serial.print(pavg);
    Serial.print(" ");
    Serial.print(sLV);
    Serial.print(" ");
    Serial.print(PCD);
    Serial.print(" ");
    Serial.print(PCL / 2);
    Serial.print(" ");
    Serial.println(PCL);
#endif

    long aaa = millis();

    switch (Mode) {
      case 12:
        blank(0);
        m12(sLV, LV);
        break;
      case 11:
        m11(LV);
        break;
      case 10:
        blank(0);
        m8(sLV, 1);
        break;
      case 9:
        blank(0);
        m5(sLV, 1);
        break;
      case 8:
        blank(0);
        m8(sLV, 0);
        break;
      case 7:
        m7(sLV, LV);
        break;
      case 5:
        blank(0);
        m5(sLV, 0);
        break;
      case 3:
        blank(0);
        m3(sLV);
        break;
      case 2:
        blank(0);
        m2(sLV);
        break;
      case 1:
        blank(0);
        m1(sLV);
        break;
      default:
        blank(0);
        m0(sLV);
    }

    //    Serial.println(millis() - aaa);

    strip.Show();

  } // end modes

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
  } //

  // random Mode
  if ( (tNow - timeL) / 1000 > 60 * modeR) {
    //    timeL = tNow;
    ModeSet(++Mode);
  }

} //end loop()


/////////////////////////////////////////////////////////////////////////////

void meter(byte n, byte c) {
  blank(1);
  for (byte i = 1; i <= n; i++) {
    if (i % 5 == 0) strip.SetPixelColor(i - 1, RgbColor(128, 0, 0));
    else strip.SetPixelColor(i - 1, RgbColor(128));
  }
  strip.Show();
  delay(500);
  EEPROM.put(c, n);
}

void encoder() {
  if (enc.isRight()) {
    switch (bMode) {
      case 5:
        digitalWrite(AMP, 1);
        break;
      case 4:
        pxRatio++;
        meter(pxRatio, 4);
        break;
      case 3:
        pfMax += 0.02;
        break;
      case 2:
        cs++;
        blank(0);
        meter(cs, 2);
        break;
      case 1:
        brC--;
        if (brC < 1) brC = 1;
        Br = Brightness(brC);
        meter(brC, 1);
        break;
      default:
        Mode++;
        ModeSet(Mode);
    }
  } else if (enc.isLeft()) {
    switch (bMode) {
      case 5:
        digitalWrite(AMP, 0);
        break;
      case 4:
        pxRatio--;
        if (pxRatio < 1) pxRatio = 1;
        meter(pxRatio, 4);
        break;
      case 3:
        pfMax -= 0.02;
        break;
      case 2:
        cs--;
        blank(0);
        meter(cs, 2);
        break;
      case 1:
        brC++;
        Br = Brightness(brC);
        meter(brC, 1);
        break;
      default:
        Mode--;
        ModeSet(Mode);
    }
  } else if (enc.isClick()) {
    bMode++;
    if (bMode > 5) bMode = 0;
  }
}

void analyzeAudio(byte n) {
  if (!n) n = 1;
  for (int i = 0 ; i < (FHT_N / n) ; i++) { // save FHT_N
    int k = analogRead(analogInPin);
    k -= 0x0200; // form into a signed int
    k <<= 6; // form into a 16b signed int
    fht_input[i] = k; // put real data into bins
  }
  fht_window(); // window the data for better frequency response
  fht_reorder(); // reorder the data before doing the fht
  fht_run(); // process the data in the fht
  //fht_mag_log();
  fht_mag_lin(); // take the output of the fht
}

void autoLowPass(byte m) {

  LOWPASS[0] = 0;
  LOWPASS[1] = 0;
  LOWPASS[2] = 0;
  LOWPASS[3] = 0;
  LOWPASS[4] = 0;

  for (int i = 0; i < FHT_N / 2; i++) {
    LOWP[i] = 0;
  }

  for (int i = 0; i < 50; i++) {
    analyzeAudio(m);
    if (m == 2) {
      for (int j = 2; j < 3; j++) {         // первые 2 канала - хлам
        if (fht_lin_out[j] > LOWPASS[0]) LOWPASS[0] = fht_lin_out[j];
      }
      for (int j = 3; j < 5; j++) {
        if (fht_lin_out[j] > LOWPASS[1]) LOWPASS[1] = fht_lin_out[j];
      }
      for (int j = 5; j < 10; j++) {
        if (fht_lin_out[j] > LOWPASS[2]) LOWPASS[2] = fht_lin_out[j];
      }
      for (int j = 10; j < 18; j++) {
        if (fht_lin_out[j] > LOWPASS[3]) LOWPASS[3] = fht_lin_out[j];
      }
      for (int j = 18; j < FHT_N / 2; j++) {
        if (fht_lin_out[j] > LOWPASS[4]) LOWPASS[4] = fht_lin_out[j];
      }
    } else {
      for (int j = 0; j < FHT_N / 2; j++) {
        if (fht_lin_out[j] > LOWP[j]) LOWP[j] = fht_lin_out[j];
      }
    }
  }
}

void blank(byte x) {
  for (int i = 0; i < PixelCount; i++) {
    strip.SetPixelColor(i, 0);
  }
  if (x) strip.Show();
}

void marker(int n, float c1, float c2, byte m) {
  float cr = 0.150 * pxRatio;
  RgbColor target;
  if (n > bwL) {
    bwL = n - 1;
    bwD = millis();
  } else {
    if (millis() - bwD > 700) {
      bwL -= cr;
      target = HsbColor(c2, 1, Br / 3 );
      strip.SetPixelColor(bwL + 1, target);
      if (m) strip.SetPixelColor(PixelCount - bwL - 1 - cr, target);
      target = target.Dim(63);
      //target = HsbColor(c2, 1, Br / 5 );
      strip.SetPixelColor(bwL + 2, target);
      if (m) strip.SetPixelColor(PixelCount - bwL - 2 - cr, target);
    }
  }
  target = HsbColor(c2, 1, Br );
  strip.SetPixelColor(bwL, target);
  if (m) strip.SetPixelColor(PixelCount - bwL - cr, target);

  target = HsbColor(c1, 1, Br * 2 );
  strip.SetPixelColor(n - 1, target);
  if (m) strip.SetPixelColor(PixelCount - n, target);
}


//////////////////////////////////////////////////////////////////
// Modes
//////////////////////////////////////////////////////////////////

void m6() {

  //  ModeSet(7);

  colorMusic[0] = 0;
  colorMusic[1] = 0;
  colorMusic[2] = 0;
  colorMusic[3] = 0;
  colorMusic[4] = 0;

  // get audio
  analyzeAudio(2);

  // lowpass filter
  //   for (int i = 0 ; i < FHT_N / 2 ; i++) {
  //      if (fht_lin_out[i] < 20 ) fht_lin_out[i] = 0;
  //    }

  // низкие частоты (0 и 1 зашумленные!)
  for (int i = 2; i < 3; i++) { // 3
    //colorMusic[0] += fht_lin_out[i];
    if (fht_lin_out[i] <= LOWPASS[0]) fht_lin_out[i] = 0;
    if (fht_lin_out[i] > colorMusic[0]) colorMusic[0] = fht_lin_out[i];
  }
  // средние частотыLOWPASS[0]
  for (int i = 3; i < 5; i++) { // 7
    //colorMusic[1] += fht_lin_out[i];
    if (fht_lin_out[i] <= LOWPASS[1]) fht_lin_out[i] = 0;
    if (fht_lin_out[i] > colorMusic[1]) colorMusic[1] = fht_lin_out[i];
  }
  // средние частоты
  for (int i = 5; i < 10; i++) { // 14
    //colorMusic[2] += fht_lin_out[i];
    if (fht_lin_out[i] <= LOWPASS[2]) fht_lin_out[i] = 0;
    if (fht_lin_out[i] > colorMusic[2]) colorMusic[2] = fht_lin_out[i];
  }
  // средние частоты
  for (int i = 10; i < 18; i++) { // 22
    //colorMusic[3] += fht_lin_out[i];
    if (fht_lin_out[i] <= LOWPASS[3]) fht_lin_out[i] = 0;
    if (fht_lin_out[i] > colorMusic[3]) colorMusic[3] = fht_lin_out[i];
  }
  // высокие частоты
  for (int i = 18; i < FHT_N / 2; i++) {
    //colorMusic[4] += fht_lin_out[i];
    if (fht_lin_out[i] <= LOWPASS[4]) fht_lin_out[i] = 0;
    if (fht_lin_out[i] > colorMusic[4]) colorMusic[4] = fht_lin_out[i];
  }

  pmax *= 0.992;
  //  pmin += 0.02;

  /*  colorMusic[0] /= 400;
    colorMusic[1] /= 500;
    colorMusic[2] /= 800;
    colorMusic[3] /= 1100;
    colorMusic[4] /= 1200;
  */
  for (byte i = 0; i < 5; i++) {
    if (colorMusic[i] > pmax) {
      pmax = colorMusic[i];
    }
    //    if (colorMusic[i] < pmin) {
    //      pmin = colorMusic[i];
    //    }
  }

  float LV[5];
  for (byte i = 0; i < 5; i++) {
    LV[i] = Map(colorMusic[i], 0, pmax, 0, Br );
    if (LV[i] > eLV[i]) {
      eLV[i] = LV[i];
      TLP[i] = millis();
    } else {
      if (eLV[i]) {
        eLV[i] -= 0.07 * Br;
        if (eLV[i] < 0) eLV[i] = 0;
      }
      if (millis() - TLP[i] < cs ) LOWPASS[i] += 2;
      else if (millis() - TLP[i] > 3000) {
        LOWPASS[i] -= 10;
        if (LOWPASS[i] < 0) LOWPASS[i] = 0;
      }
    }
  }

  //  #define DEBUG

#ifdef DEBUG
  if (colorMusic[0] || colorMusic[1] || colorMusic[2] || colorMusic[3] || colorMusic[4]) {
    for (int i = 0; i < 5; i++) {
      Serial.print(LOWPASS[i]);
      Serial.print(" ");
    }
    if (colorMusic[0] < 10) Serial.print(" ");
    Serial.print(colorMusic[0]);
    Serial.print(" ");
    if (colorMusic[1] < 10) Serial.print(" ");
    Serial.print(colorMusic[1]);
    Serial.print(" ");
    if (colorMusic[2] < 10) Serial.print(" ");
    Serial.print(colorMusic[2]);
    Serial.print(" ");
    if (colorMusic[3] < 10) Serial.print(" ");
    Serial.print(colorMusic[3]);
    Serial.print(" ");
    if (colorMusic[4] < 10) Serial.print(" ");
    Serial.print(colorMusic[4]);
    Serial.print(" ");
    Serial.print(pmin);
    Serial.print(" ");
    Serial.print(pmax, 3);
    Serial.println();
  }

#endif

  RgbColor target;

  PCL = PixelCount / 9 * 9;
  int cn = PixelCount - PCL;
  int m = PCL / 9;

  int n = 0;

  for (int p = 0; p < m; p++) {

    int cr = 0;

    n = map(1, 1, 9, 0, PCL - m);
    target = HsbColor(0.77, 1.0f, eLV[4]);
    strip.SetPixelColor(n + p + cr, target);
    if (cn > 1) {
      cr++;
      strip.SetPixelColor(n + p + cr, target);
    }

    n = map(2, 1, 9, 0, PCL - m);
    target = HsbColor(0, 1.0f, eLV[3]);
    strip.SetPixelColor(n + p + cr, target);
    if (cn > 3) {
      cr++;
      strip.SetPixelColor(n + p + cr, target);
    }

    n = map(3, 1, 9, 0, PCL - m);
    target = HsbColor(0.125, 1, eLV[2] / 1.3);
    strip.SetPixelColor(n + p + cr, target);
    if (cn > 5) {
      cr++;
      strip.SetPixelColor(n + p + cr, target);
    }

    n = map(4, 1, 9, 0, PCL - m);
    target = HsbColor(0.33f, 1.0f, eLV[1]);
    strip.SetPixelColor(n + p + cr, target);
    if (cn > 7) {
      cr++;
      strip.SetPixelColor(n + p + cr, target);
    }

    n = Map(5, 1, 9, 0, PCL - m);
    target = HsbColor(0.66f, 1.0f, eLV[0]);
    strip.SetPixelColor(n + p + cr, target);
    if (cn % 2) {
      cr++;
      strip.SetPixelColor(n + p + cr, target);
    }

    n = Map(6, 1, 9, 0, PCL - m);
    target = HsbColor(0.33f, 1.0f, eLV[1]);
    strip.SetPixelColor(n + p + cr, target);
    if (cn > 7) {
      cr++;
      strip.SetPixelColor(n + p + cr, target);
    }

    n = Map(7, 1, 9, 0, PCL - m);
    target = HsbColor(0.125, 1, eLV[2] / 1.3);
    strip.SetPixelColor(n + p + cr , target);
    if (cn > 5) {
      cr++;
      strip.SetPixelColor(n + p + cr, target);
    }

    n = Map(8, 1, 9, 0, PCL - m );
    target = HsbColor(0, 1.0f, eLV[3]);
    strip.SetPixelColor(n + p + cr, target);
    if (cn > 3) {
      cr++;
      strip.SetPixelColor(n + p + cr, target);
    }

    n = Map(9, 1, 9, 0, PCL - m );
    target = HsbColor(0.77, 1.0f, eLV[4]);
    strip.SetPixelColor(n + p + cr, target);
    if (cn > 1) {
      cr++;
      strip.SetPixelColor(n + p + cr, target);
    }

  }

  strip.Show();

}

void m4() {

  analyzeAudio(1);

  //  int LPF = 2;

  // lowpass filter
  for (int i = 0 ; i < FHT_N / 2 ; i++) {
    if (fht_lin_out[i] < LOWP[i] ) fht_lin_out[i] = 0;
    //    else fht_lin_out[i] /= LOWP[i];
    /*    if (fht_lin_out[i] < LOWPASS[0] && i <= 2) fht_lin_out[i] = 0;
        else if (fht_lin_out[i] < LOWPASS[1] && i == 3) fht_lin_out[i] = 0;
        else if (fht_lin_out[i] < LOWPASS[2] && i < 10) fht_lin_out[i] = 0;
        else if (fht_lin_out[i] < LOWPASS[3] && i < 18) fht_lin_out[i] = 0;
        else if (fht_lin_out[i] < LOWPASS[4]) fht_lin_out[i] = 0;
    */

    //#define DEBUG_EQ

#ifdef DEBUG_EQ
    if (i < 16) {
      if (fht_lin_out[i] < 10) Serial.print(" ");
      Serial.print(fht_lin_out[i], 1);
      Serial.print(" ");
    }
#endif

  }

#ifdef DEBUG_EQ
  Serial.println();
#endif

  int pmax = 0;
  HsbColor target;

  //  for (int i = FHT_N / 2 - 1; i > 1 ; i--) {
  for (int i = 0; i < FHT_N / 2; i++) {
    //    if (fht_lin_out[i] > pmax) pmax = fht_lin_out[i];
    int m = round(Map(i, 0, FHT_N / 2 - 1, PCL, PixelCount - 1));
    if (fht_lin_out[i] > LOWP[i]) {
      float L = Map(fht_lin_out[i], LOWP[i], 30, 0, Br );
      target = HsbColor( (i - 3) * (360. / (FHT_N / 2)) / 360., 1.0, L);
      RgbColor color = strip.GetPixelColor(m);
      HsbColor c = color;
      if (target.B < c.B && target.B > 0) target.B = Br;
      color = target;
      strip.SetPixelColor(m, color);
      strip.SetPixelColor(PixelCount - m - 1, color);
    } else {
      RgbColor color = strip.GetPixelColor(m);
      color.Darken(24 * Br);
      strip.SetPixelColor(m, color);
      strip.SetPixelColor(PixelCount - m - 1, color);
    }
  }
  strip.Show();
}

float RBn;
float slow;
void m5(int n, byte m) {
  int j = 0;
  if (m) j = PCL;
  RBn = 1. / (PCL / 1.4) ; //(PCL / (1 + m + pxRatio / 6.));
  int k = n + PCL - PCV;
  RgbColor target;

  //  if (n > 1) {
  //  if (millis() - timeF > (30 / (1 + m))) {
  //    timeF = millis();
  switch (Nc) {
    case 1:
      RB -= RBn;
      if (RB < 0) RB = (RB + 1.) - Cmin;
      break;
    default:
      RB += RBn;
      if (RB > 1) RB = (RB - 1.) + Cmin;
  }
  //  }


  float br = Map(n, 0, PCL, Br / 4, Br );
  for (int i = PCL - PCV; i < k - 1 ; i++) {
    float cc = RBn * (i - PCL);
    while (RB + cc > 1 || RB - cc < 0) cc--;
    switch (Nc) {
      case 1:
        target = HsbColor(RB + cc, 1, br);
        break;
      default:
        target = HsbColor(RB - cc, 1, br );
    }
    strip.SetPixelColor(i - j, target);
    if (!m) strip.SetPixelColor(PixelCount - i - 1, target);
  }
  //  }
  marker(k - j, 0, 0.67, 1 - m);
}

void m3(int n) {

  if (n > 2) {
    switch (Nc) {
      case 1:
        RB -= 0.0003;
        if (RB < 0) RB = (RB + 1.) - Cmin;
        break;
      default:
        RB += 0.0003;
        if (RB > 1) RB = (RB - 1.) + Cmin;
    }
  }
  float lv = Map(n, 0, PCL, Br / 4 , Br );
  RgbColor color = HsbColor(RB, 1, lv);
  for (int i = 0; i < n; i++) {
    float nn = Map(i - n * 0.6, 0, PCL, 0, 1);
    RgbColor res = RgbColor::LinearBlend(color, 25, nn);
    strip.SetPixelColor(i, res);
  }
  marker(n, 0, RB + 0.5, 0);
}

void m2(int n) {

  float minL = 0.5 / 24;
  RgbColor target;
  int m = PixelCount / 5;
  float L;

  if (n > 0) {
    L = Map(n, 1, maxL, minL, Br);
    target = HsbColor(0.67f, 1.0f, L );
    for (int i = m * 2; i < m * 3; i++) {
      strip.SetPixelColor(i, target);
    }
  }
  if (n > maxL) {
    L = Map(n - maxL, 1, maxL, minL, Br);
    target = HsbColor(0.33f, 1.0f, L );
    for (int i = m; i < m * 2; i++) {
      strip.SetPixelColor(i, target);
    }
    for (int i = m * 3; i < m * 4; i++) {
      strip.SetPixelColor(i, target);
    }
    target = HsbColor(0.50f, 1.0f, Br / 2 );
    for (int i = m * 2; i < m * 3; i++) {
      strip.SetPixelColor(i, target);
    }
  }
  if (n > maxL * 2) {
    L = Map(n - maxL * 2, 1, maxL, minL, Br );
    target = HsbColor(0, 1.0f, L );
    for (int i = 0; i < m; i++) {
      strip.SetPixelColor(i, target);
    }
    for (int i = m * 4; i < m * 5; i++) {
      strip.SetPixelColor(i, target);
    }
    target = HsbColor(0.130, 1.0f, Br / 2 );
    for (int i = m; i < m * 2; i++) {
      strip.SetPixelColor(i, target);
    }
    for (int i = m * 3; i < m * 4; i++) {
      strip.SetPixelColor(i, target);
    }
  }
}

void m1(int n) {
  int m = PCL / 5;
  float c, color[] = {0.56, 0.33, 0.130, 0.03, 0};

  float L = Map(n, 0, PCL, Br / 4, Br);
  for (int i = 0; i < n; i++) {
    if (i % m == 0) c = color[i / m];
    strip.SetPixelColor(i, HsbColor(c, 1, L));
  }
  marker(n, 0, 0.67, 0);
}

void m0(int n) {
  int v = PCL - PCV;
  int m = v / 5;
  int p = round(m / 2.);
  int k = n + v;
  float color[] = {0.33, 0.130, 0.03, 0, 0};
  float c = 0.56;

  if (n > 0) {
    float L = Map(n, 0, v, Br / 4, Br);
    for (int i = v; i < k; i++) {
      if ((i - v) % m == p) c = color[(i - v) / m];
      RgbColor res = HsbColor(c, 1, L);
      strip.SetPixelColor(i, res);
      strip.SetPixelColor(PixelCount - i - 1, res);
    }
  }
  marker(k, 0, 0.67, 1);
}

//byte cn;
int nLast;
void m7(int m, int n) {

  //  n = 10;

  byte nMin = 2;

  //  if (n > 7) {
  switch (Nc) {
    case 1:
      RB -= 0.03;
      if (RB < 0) RB = (RB + 1.) - Cmin;
      break;
    default:
      RB += 0.03;
      if (RB > 1) RB = (RB - 1.) + Cmin;
  }
  //  }

  //    Serial.println(nLast + String(" ") + n);

  if (n < nMin || n == nLast || n + 1 == nLast ) n = nMin;

  float cL = Map(n, nMin, PCL, Lmin, Br );
  strip.SetPixelColor(PixelCount / 2 , HsbColor(RB, 1, cL ));

  //    Serial.println(cL + String(" ") + n);

  //  if (millis() - timeF > 27) {
  //  if (n > nLast && n>8) cn = 1;
  //  if (cn++ > 0) {
  //    cn = 0;

  float L = round((Br / (PixelCount / 3)) * 256);
  while (n--) {
    //    timeF = millis();
    for (int i = 1; i <= PixelCount / 2 ; i++) {
      RgbColor color = strip.GetPixelColor(i);
      if (i < PixelCount / 4) {
        if (color.R > L || color.G > L || color.B > L) color.Darken(L);
        else if (color.R == L || color.G == L || color.B == L) color.Darken(L - 1);
      }
      if (i == 2 && color.CalculateBrightness()) {
        color = color.Brighten(map(m, nMin, PCL, 31, 15));
      }
      strip.SetPixelColor(i - 1, color);
      strip.SetPixelColor(PixelCount - i, color);
    }
    strip.Show();
    //    delay(5);
    timeF = millis();
    while (millis() - timeF < 4);
  }
  nLast = n;
}

float lastRB;
void m8(int n, byte m) {
  int k = n + PCL - PCV;

  if (millis() - timeF > 12 * 1000 && n < 3 || timeF == 0) {
    timeF = millis();
    lastRB = RB;
    while (abs(lastRB - RB) < 0.125) {
      RB = random(360) / 360.;
      while (abs(RB - RB2) < 0.25) {
        RB2 = random(100) / 100.;
      }
    }
  }
  float L, nn;
  RgbColor res;
  switch (m) {
    case 1:
      L = Map(n, 2, PCL, Br / 4, Br );
      for (int i = PCL - 1; i < k - 1; i++) {
        nn = Map(i - n * 0.07, PCL, k - 4, 0, 1);
        res = HsbColor::LinearBlend<NeoHueBlendShortestDistance>(HsbColor(RB, 1, L ), HsbColor(RB2, 1, L ), nn);
        strip.SetPixelColor(i, res);
        strip.SetPixelColor(PixelCount - i - 1, res);
      }
      marker(k, RB2 - 0.5, RB, 1);
      break;
    default:
      for (int i = PCL - 1; i < k - 1; i++) {
        L = Map(i - n * 0.3, PCL, k - 2, Br, Br / 4 );
        nn = Map(i - n * 0.3, PCL, k - 2, 1, 0);
        res = HsbColor(RB, nn, L );
        strip.SetPixelColor(i, res);
        strip.SetPixelColor(PixelCount - i - 1, res);
        //    Serial.println(bL, 5);
      }
      marker(k, RB - 0.5, RB, 1);
  }
}

int lastP;
void m11(int n) {
  int f = round(25 * Br);
  for (int i = 0; i < PixelCount; i++) {
    RgbColor color = strip.GetPixelColor(i);
    color.Darken(f);
    strip.SetPixelColor(i, color);
  }
  if (n > PCL * 0.15 && n > nLast) {
    int p;
    while ( abs(lastP - p) < PixelCount / 8 * 2) p = random(PixelCount / 14, PixelCount - PixelCount / 14);
    lastP = p;
    RB = Map(n, 0, PCL, 1, 0);
    float L = Map(n, PCL * 0.10, PCL, Br / 4, Br);
    RgbColor t1 = HsbColor(RB, 1, L);
    RgbColor t2 = HsbColor(RB, 1, L / 3);
    for (int i = 0; i < PixelCount / 7; i++) {
      float nn = Map(i, PixelCount / 7 * 0.25, PixelCount / 7 - 1, 0, 1);
      RgbColor res = RgbColor::LinearBlend(t1, t2, nn);
      strip.SetPixelColor(p + i, res);
      strip.SetPixelColor(p - i - 1 , res);
    }
  }
  nLast = n;
}

float mLast2, mLast1, mLast;
int dd = 10;
byte pp;
void m12(int n, int h) {

  //  n = 40;

  int k = n + PCL - PCV;
  int m;
  RgbColor L, L2;

  if (n == 0 || n > PCL * 0.95) {
    RB = random(100) / 100.;
  }

  //center
  m = PixelCount * 0.07;
  L = HsbColor(RB, 1, Map(h, m, PCL - m, 0, Br));
  for (int i = 0; i < m; i++) {
    float nn = Map(i, m * 0.2, m - 1, 0, 0.95);
    RgbColor res = RgbColor::LinearBlend(L, 0, nn);
    strip.SetPixelColor(PCL + i, res);
    strip.SetPixelColor(PCL - i - 1, res);
  }

  /*
    //edges
    //  m = PixelCount * 0.07;
    //  L = HsbColor(RB+0.5, 1, Map(h, 0, PCL, 0, Br));
    L = HsbColor(Map(n, 0, PCL - m, 0.9, 0), 1, Map(h, 0, PCL - m, 0, Br));
    for (int i = 0; i < m; i++) {
      float nn = Map(i, m * 0.2, m - 1, 0, 0.95);
      RgbColor res = RgbColor::LinearBlend(L, 0, nn);
      strip.SetPixelColor(i, res);
      strip.SetPixelColor(PixelCount - i - 1, res);
    }
  */

  //level
  m = PixelCount * 0.15;
  L = HsbColor(Map(n, 0, PCL - m, 0.9, 0), 1, Map(n, 0, PCL - m, Br / 4, Br));
  for (int i = 0; i < m; i++) {
    float nn = Map(i, m * 0.3, m - 1, 0, 0.95);
    RgbColor res = RgbColor::LinearBlend(L, 0, nn);
    strip.SetPixelColor(k + i, res);
    if (k - i > PCL - PCV ) {
      strip.SetPixelColor(PixelCount - k + i, res);
      strip.SetPixelColor(k - i - 1, res);
    }
    strip.SetPixelColor(PixelCount - k - i - 1, res);
  }

  // runner
  if (n == 0 && nLast != 0 && millis() - timeF > 1000) {
    mLast2 = PCL - PixelCount / 16;
    L = HsbColor(RB, 1, Br);
    while (mLast2++ <= PixelCount) {
      for (int i = 0; i < PixelCount / 8; i++) {
        float nn = Map(i, 0, PixelCount / 8 - 1, 1, 0);
        RgbColor res = RgbColor::LinearBlend(L, 0, nn);
        strip.SetPixelColor(mLast2 + i, res);
        strip.SetPixelColor(PixelCount - mLast2 - i - 1, res);
      }
      strip.Show();
      //      delay(8/pxRatio);
      timeF = millis();
      while (millis() - timeF < 8 / pxRatio);
    }
  }

  /*
      mLast2 += 3; //Map(k, PCL, PixelCount, 1, 5);
      if (mLast2 > PixelCount) {
        mLast2 = PCL;
        //    RB = random(360) / 360.;
      }
      L = HsbColor(RB, 1, Map(h, 0, PCL, Lmin, Br));
      for (int i = 0; i < PixelCount / 8; i++) {
        float nn = Map(i, 0, PixelCount / 8 - 1, 0, 1);
        RgbColor res = RgbColor::LinearBlend(L, 0, nn);
        //          strip.SetPixelColor(mLast2 + i, res);
        strip.SetPixelColor(PixelCount - mLast2 + i, res);
        strip.SetPixelColor(mLast2 - i - 1, res);
        //          strip.SetPixelColor(PixelCount - mLast2 - i - 1, res);
      }
  */


  /*
        mLast1 += Map(k, PCL, PixelCount, 2, pxRatio);
        if (mLast1 > PixelCount - PixelCount / 10) mLast1 = 0;
        L = 255 * Map(n, 1, PCL, 0, Br / 2);
        for (int i = 0; i < PixelCount / 10; i++) {
          float nn = Map(i, 0, PixelCount / 10 - 1, 0, 1);
          RgbColor res = RgbColor::LinearBlend(L, 0, nn);
          strip.SetPixelColor(mLast1 + i, res);
          strip.SetPixelColor(PixelCount - mLast1 + i, res);
          strip.SetPixelColor(mLast1 - i - 1, res);
          strip.SetPixelColor(PixelCount - mLast1 - i - 1, res);
        }
  */

  nLast = n;

}
