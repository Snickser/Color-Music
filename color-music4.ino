#include "arduinoFFT.h"
const uint16_t samples = 128; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 40000; //Hz, must be less than 10000 due to ADC
double vReal[samples];
double vImag[samples];
arduinoFFT FFT = arduinoFFT(vReal, vImag, samples, samplingFrequency); /* Create FFT object */
int colorMusic[5];
int LOWPASS[5];

#include <EEPROM.h>
byte Mode;

#include <NeoPixelBus.h>

byte maxL = 4;
byte cs = 25;
float pfMin;

#define DEBUG

// mic
int sensorValue;
const int analogInPin = A0;

// button
//const uint8_t ButtonPin = 4;

// LED
const uint16_t PixelCount = 40;
const uint8_t PixelPin = 8;
#define colorSaturation 64
byte Br = 8; // HSL Max 0.5/Br = level

HsbColor rnd;
float RB;
float bwL;
byte Nc;
unsigned long bwD;
byte PCL;
float PCD;

NeoPixelBus<NeoRgbFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);

RgbColor red(colorSaturation, 0, 0);
RgbColor green(0, colorSaturation / 2, 0);
RgbColor blue(0, 0, colorSaturation / 2);
RgbColor purple(colorSaturation, 0, colorSaturation);
RgbColor cyan(0, colorSaturation / 3, colorSaturation / 2);
RgbColor yellow(colorSaturation / 2, colorSaturation / 4, 0);
RgbColor orange(colorSaturation / 2, colorSaturation / 20, 0);
RgbColor white(colorSaturation);
RgbColor black(0);

float Map(float x, float in_min, float in_max, float out_min, float out_max)
{
  if (in_min == in_max) return out_max;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void ModeSet(byte m) {
  Mode = m;
  Nc = random(1);
  pfMin = 0.2;
  PCL = PixelCount;
  switch (m) {
    case 7:
      Serial.println("Mode 7: flow");
      EEPROM.put(0, 0);
      PCL = 10; // devider, not pixels
      PCD = PCL * 1.2;
      pfMin = 0.5;
      break;
    case 6:
      Serial.println("Mode 6: freq");
      EEPROM.put(0, 7);
      autoLowPass();
      break;
    case 5:
      Serial.println("Mode 5: center + rainbow");
      EEPROM.put(0, 6);
      if (PixelCount % 2) {
        PCL = PixelCount / 2 + 1;
      }
      else {
        PCL = PixelCount / 2;
      }
      PCD = PCL * 1.5;
      break;
    case 4:
      Serial.println("Mode 4: freq more");
      EEPROM.put(0, 5);
      autoLowPass();
      blank(1);
      break;
    case 3:
      Serial.println("Mode 3: linear + rainbow");
      EEPROM.put(0, 4);
      PCD = PCL * 2.1;
      break;
    case 2:
      Serial.println("Mode 2: center + 3 level");
      EEPROM.put(0, 3);
      PCL = 3 * maxL ;
      PCD = PCL * 0.5;
      pfMin = 0.02;
      break;
    case 1:
      Serial.println("Mode 1: linear");
      EEPROM.put(0, 2);
      PCD = PCL * 2.1;
      break;
    default:
      Serial.println("Mode 0: center");
      EEPROM.put(0, 1);
      if (PixelCount % 2) {
        PCL = PixelCount / 2 + 1;
      }
      else {
        PCL = PixelCount / 2;
      }
      PCD = PCL * 2.0;
      Mode = 0;
      pfMin = 0.3;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(analogInPin, INPUT);
  randomSeed(pow(analogRead(analogInPin), 3));
  //  analogReference(INTERNAL);

//  pinMode(ButtonPin, INPUT);

//  pinMode(7, OUTPUT);
//  digitalWrite(7, 1);
//  pinMode(9, OUTPUT);
//  digitalWrite(9, 0);

#if defined (__AVR_ATmega32U4__)
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

  EEPROM.get(0, Mode);
  ModeSet(Mode);
  ModeSet(4);

#if defined(__AVR_ATmega32U4__)   //Выключение светодиодов на Pro Micro
  delay (1000);                     //При питании по usb от компьютера нужна задержка перед выключением RXLED. Если питать от БП, то можно убрать эту строку.
  TXLED1;                           //на ProMicro выключим и TXLED
  digitalWrite(17, 1);
#endif

}

int sLV, LV;
float sLV1, sLV2, sLV3, sLV4, sLV5;
int cnt = 0;
float pmax = 1, pmin = 200;
float psum = 0;
float pavg;
float plast;
unsigned long timeL;
unsigned long timeB;
unsigned long timeF;
byte bb;

void loop() {

  int v_max = 0;
  int v_min = 1023;

  unsigned long tNow = millis();
  unsigned long tEnd = tNow + cs ;

  if (Mode == 4) {

    m4();

  } else if (Mode == 6) {

    m6();

  } else {

    // get audio
    while ( millis() < tEnd ) {
      sensorValue = analogRead(analogInPin);
      if (sensorValue > v_max) {
        v_max = sensorValue;
      }
      if (sensorValue < v_min) {
        v_min = sensorValue;
      }
    }
    int pcur = v_max - v_min;

    //Serial.println(tEnd - millis() );

    if (pcur > pmax) {
      //if ( pcur - pmax < 70 )
      pmax = pcur;
    } else {
      pmax *= 0.97;
    }
    if (pmax < PCL ) {
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
      PCD *= 1.10;
    }
    plast = pcur;

    float cr;
    if (pcur > pavg) {
      cr = pavg;
    }
    else cr = pcur;

    LV = round(Map(cr, pmin, pavg, 0, PCL));

    //    if (LV == 0 && ) PCD *= 0.99;

    if (LV > sLV) {
      sLV = LV;
    } else {
      if (sLV >= 1) {
        sLV--;
      } else {
        PCD *= 0.992;
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

    switch (Mode) {
      case 7:
        m7(sLV);
        break;
      case 5:
        m5(sLV);
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

  } // end modes

  // blink
  if ( tNow - timeB > 1000 ) {
    timeB = tNow;
    /*
        if (bb) {
          digitalWrite(17, 0);
          bb = 0;
        }
        else {
          digitalWrite(17, 1);
          bb = 1;
        }
    */
    /*
      if (digitalRead(ButtonPin)) {
      ModeSet(++Mode);
      timeL = tNow;
      }
    */
  }

  // random Mode
  if ( (tNow - timeL) / 1000 > 60 * 4) {
    timeL = tNow;
    ModeSet(++Mode);
  }

} //end loop()

void analyzeAudio() {

long aaa=millis();

  unsigned int sampling_period_us = round(1000000*(1.0/samplingFrequency));
  unsigned long microseconds = micros();
  for (int i = 0; i < samples; i++) {
    vReal[i] = analogRead(analogInPin);
    vImag[i] = 0;
    while (micros() - microseconds < sampling_period_us) {
      //empty loop
    }
    microseconds += sampling_period_us;
  }

  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  FFT.Compute(FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(); /* Compute magnitudes */

Serial.println(millis()-aaa);

}

void autoLowPass() {
  LOWPASS[0] = 0;
  LOWPASS[1] = 0;
  LOWPASS[2] = 0;
  LOWPASS[3] = 0;
  LOWPASS[4] = 0;
  for (int i = 0; i < 30; i++) {          // делаем 100 измерений
    analyzeAudio();                         // разбить в спектр
    for (byte j = 2; j < 3; j++) {         // первые 2 канала - хлам
      if (vReal[j] > LOWPASS[0]) LOWPASS[0] = vReal[j];
    }
    for (byte j = 3; j < 4; j++) {
      if (vReal[j] > LOWPASS[1]) LOWPASS[1] = vReal[j];
    }
    for (byte j = 4; j < 10; j++) {
      if (vReal[j] > LOWPASS[2]) LOWPASS[2] = vReal[j];
    }
    for (byte j = 10; j < 18; j++) {
      if (vReal[j] > LOWPASS[3]) LOWPASS[3] = vReal[j];
    }
    for (byte j = 18; j < samples / 2; j++) {
      if (vReal[j] > LOWPASS[4]) LOWPASS[4] = vReal[j];
    }
  }
}

void blank(byte c) {
  for (int i = 0; i < PixelCount; i++) {
    strip.SetPixelColor(i, 0);
  }
  if (c) strip.Show();
}

float RBn;
float slow;
void m5(int n) {

  int k = n + PCL;
  RBn = 1. / PCL ;

  if (n > 1) {
    if (millis() - timeF > 40) {
      timeF = millis();
      switch (Nc) {
        case 1:
          RB -= RBn;
          break;
        default:
          RB += RBn;
      }
    }

    //    Serial.println(RB, 4);

    switch (Nc) {
      case 1:
        if (RB < 0) RB = (RB + 1.) - 1. / 360.;
        break;
      default:
        if (RB > 1) RB = (RB - 1.) + 1. / 360.;
    }

    for (int i = PCL; i < k - 1; i++) {
      float br = Map(i, PCL, k - 2, 1. / Br / 4, 1. / Br );
      switch (Nc) {
        case 1:
          strip.SetPixelColor(i, HsbColor(RB + (RBn * (i - PCL)), 1., br ));
          strip.SetPixelColor(PixelCount - i - 1, HsbColor(RB + (RBn * (i - PCL)), 1., br ));
          break;
        default:
          strip.SetPixelColor(i, HsbColor(RB - (RBn * (i - PCL)), 1., br ));
          strip.SetPixelColor(PixelCount - i - 1, HsbColor(RB - (RBn * (i - PCL)), 1., br ));
      }
    }

  }

  for (int i = k + 1; i <= PixelCount; i++) {
    strip.SetPixelColor(i - 1, 0);
    strip.SetPixelColor(PixelCount - i, 0);
  }

  if (k > bwL) {
    bwL = k - 1;
    bwD = millis();
  } else {
    if (millis() - bwD > 700) {
      bwL -= 0.25;
      if (PixelCount > 10) {
        strip.SetPixelColor(bwL + 1, HsbColor(0.67f, 1., 1. / 96));
        strip.SetPixelColor(PixelCount - bwL - 1.25 , HsbColor(0.67f, 1., 1. / 96));
      }
    }
  }

  strip.SetPixelColor(bwL, HsbColor(0.67f , 1., 1. / (Br * 2)));
  strip.SetPixelColor(PixelCount - bwL - 0.25, HsbColor(0.67f , 1., 1. / (Br * 2)));

  strip.SetPixelColor(k - 1, HsbColor(0, 1.f, 1.f / (Br / 2)));
  strip.SetPixelColor(PixelCount - k, HsbColor(0, 1.f, 1.f / (Br / 2)));

  strip.Show();

}

void m3(int n) {

  if (n > 2) {
    switch (Nc) {
      case 1:
        RB -= 0.0015;
        if (RB < 0) RB = (RB + 1.) - 1. / 360.;
        break;
      default:
        RB += 0.0015;
        if (RB > 1) RB = (RB - 1.) + 1. / 360.;
    }
  }

  float lv = Map(n, 0, PCL, 1. / Br / 2 , 1. / Br );
  for (int i = 0; i < n; i++) {
    rnd = HsbColor(RB, 1.0f, lv );
    strip.SetPixelColor(i, rnd );
  }

  if (n > bwL) {
    bwL = n - 1;
    bwD = millis();
  } else {
    if (millis() - bwD > 700) {
      bwL -= 0.25;
      if (PixelCount > 10) {
        strip.SetPixelColor(bwL + 1, HsbColor(RB + 0.5, 1., 1. / 96 ));
      }
    }
  }
  strip.SetPixelColor(bwL, HsbColor(RB + 0.5, 1., 1. / (Br * 2)));
  strip.SetPixelColor(n - 1, red );
  strip.Show();

}

void m2(int n) {

  float minL = 0.5 / 24;
  HsbColor target;
  int m = PixelCount / 5;

  if (n > 0) {
    float aa = Map(n, 1, maxL, minL, 1.0 / Br);
    target = HsbColor(0.67f, 1.0f, aa );
    for (int i = m * 2; i < m * 3; i++) {
      strip.SetPixelColor(i, target);
    }
  }
  if (n > maxL) {
    float aa = Map(n - maxL, 1, maxL, minL, 1.0 / Br);
    target = HsbColor(0.33f, 1.0f, aa );
    for (int i = m; i < m * 2; i++) {
      strip.SetPixelColor(i, target);
    }
    for (int i = m * 3; i < m * 4; i++) {
      strip.SetPixelColor(i, target);
    }
    target = HsbColor(0.50f, 1.0f, 1.0 / (Br * 2) );
    for (int i = m * 2; i < m * 3; i++) {
      strip.SetPixelColor(i, target);
    }
  }
  if (n > maxL * 2) {
    float aa = Map(n - maxL * 2, 1, maxL, minL, 1.0 / (Br / 2));
    target = HsbColor(0, 1.0f, aa );
    for (int i = 0; i < m; i++) {
      strip.SetPixelColor(i, target);
    }
    for (int i = m * 4; i < m * 5; i++) {
      strip.SetPixelColor(i, target);
    }
    target = HsbColor(0.125, 1.0f, 1.0 / Br );
    for (int i = m; i < m * 2; i++) {
      strip.SetPixelColor(i, target);
    }
    for (int i = m * 3; i < m * 4; i++) {
      strip.SetPixelColor(i, target);
    }
    target = HsbColor(0.66, 0.9, 1. / (Br * 2) );
    for (int i = m * 2; i < m * 3; i++) {
      //      strip.SetPixelColor(i, target);
    }
  }
  strip.Show();
}

void m1(int n) {

  int m = PCL / 5;

  float L = Map(n, 0, PCL, 1. / Br / 4, 1. / Br);

  for (int i = 0; i < n; i++) {
    strip.SetPixelColor(i, HsbColor(0.56, 1, L));
  }
  for (int i = m; i < n; i++) {
    strip.SetPixelColor(i, HsbColor(0.33, 1, L));
  }
  for (int i = m * 2; i < n; i++) {
    strip.SetPixelColor(i, HsbColor(0.125, 1, L));
  }
  for (int i = m * 3; i < n; i++) {
    strip.SetPixelColor(i, HsbColor(0.025, 1, L));
  }
  for (int i = m * 4; i < n; i++) {
    strip.SetPixelColor(i, HsbColor(0, 1, L));
  }

  if (n > bwL) {
    bwL = n - 1;
    bwD = millis();
  } else {
    if (millis() - bwD > 700) {
      bwL -= 0.25;
      if (PixelCount > 10 ) {
        strip.SetPixelColor(bwL + 1, HsbColor(0.66f, 1., 1. / 96));
      }
    }
  }
  strip.SetPixelColor(bwL, HsbColor(0.66f, 1., 1. / (Br / 1)));
  strip.SetPixelColor(n - 1, HsbColor(0.f, 1., 1. / (Br / 2)));
  strip.Show();
}

void m0(int n) {

  int m = PCL / 5;
  int k = n + PCL;

  float L = Map(n, 0, PCL, 1. / Br / 4, 1. / Br);

  for (int i = PCL; i < k; i++) {
    strip.SetPixelColor(i, HsbColor(0.56, 1, L));
    strip.SetPixelColor(PixelCount - 1 - i, HsbColor(0.56, 1, L));
  }
  for (int i = PCL + m - 1; i < k; i++) {
    strip.SetPixelColor(i, HsbColor(0.33, 1, L));
    strip.SetPixelColor(PixelCount - 1 - i, HsbColor(0.33, 1, L));
  }
  for (int i = PCL + m * 2 - 1; i < k; i++) {
    strip.SetPixelColor(i, HsbColor(0.125, 1, L));
    strip.SetPixelColor(PixelCount - 1 - i, HsbColor(0.125, 1, L));
  }
  for (int i = PCL + m * 3 - 1; i < k; i++) {
    strip.SetPixelColor(i, HsbColor(0.03, 1, L));
    strip.SetPixelColor(PixelCount - 1 - i, HsbColor(0.03, 1, L));
  }
  for (int i = PCL + m * 4 - 1; i < k; i++) {
    strip.SetPixelColor(i, HsbColor(0, 1, L));
    strip.SetPixelColor(PixelCount - 1 - i, HsbColor(0, 1, L));
  }
  if (k > bwL) {
    bwL = k - 1;
    bwD = millis();
  } else {
    if (millis() - bwD > 700) {
      bwL -= 0.25;
      if (PixelCount > 10) {
        strip.SetPixelColor(bwL + 1, HsbColor(0.66f, 1., 1. / 96));
        strip.SetPixelColor(PixelCount - bwL - 1.25, HsbColor(0.66f, 1., 1. / 96));
      }
    }
  }
  strip.SetPixelColor(bwL, HsbColor(0.66f, 1., 1. / (Br / 1)));
  strip.SetPixelColor(PixelCount - bwL - 0.25, HsbColor(0.66f, 1., 1. / (Br / 1)));

  strip.SetPixelColor(k - 1, HsbColor(0, 1, 1. / (Br / 2)));
  strip.SetPixelColor(PixelCount - k, HsbColor(0, 1, 1. / (Br / 2)));

  strip.Show();
}

int nLast;
void m7(int n) {

  int nMin = 6;

  //  if (n > 7) {
  switch (Nc) {
    case 1:
      RB -= 0.031;
      if (RB < 0) RB = (RB + 1.) - 1. / 360.;
      break;
    default:
      RB += 0.031;
      if (RB > 1) RB = (RB - 1.) + 1. / 360.;
  }
  //  }

  //  Serial.println(nLast + String(" ") + n);

  if (n < nMin || n == nLast || n == nLast) n = nMin;
  nLast = n;

  float cr = Map(n, nMin, PCL, 1. / 255, 1. / Br );

  strip.SetPixelColor(PixelCount / 2 , HsbColor(RB, 1, cr ));

  if (millis() - timeF > 40) {
    timeF = millis();
    float L = 1. / Br / PixelCount / 3;
    for (int i = 1; i <= PixelCount / 2 ; i++) {
      RgbColor color = strip.GetPixelColor(i);
      HsbColor c = color;
      if (i < PixelCount * 0.4 ) {
        c.B -= L;
        if (c.B < 1. / 255) {
          c.B = 1. / 255;
        }
      }
      strip.SetPixelColor(i - 1, c);
      strip.SetPixelColor(PixelCount - i, c);
      //      Serial.println((i + 1) + String(" ") + (PixelCount - i - 1));
    }
  }

  strip.Show();
  //    delay(1000);
}

void m6() {

  colorMusic[0] = 0;
  colorMusic[1] = 0;
  colorMusic[2] = 0;
  colorMusic[3] = 0;
  colorMusic[4] = 0;

  // get audio
  analyzeAudio();

  // lowpass filter
  for (int i = 0 ; i < samples / 2 ; i++) {
    //    if (vReal[i] < 20 ) vReal[i] = 0;
  }

  // низкие частоты (0 и 1 зашумленные!)
  for (byte i = 2; i < 3; i++) { // 3
    if (vReal[i] < LOWPASS[0]) vReal[i] = 0;
    if (vReal[i] > colorMusic[0]) colorMusic[0] = vReal[i];
  }
  // средние частоты
  for (byte i = 3; i < 4; i++) { // 7
    if (vReal[i] < LOWPASS[1]) vReal[i] = 0;
    if (vReal[i] > colorMusic[1]) colorMusic[1] = vReal[i];
  }
  // средние частоты
  for (byte i = 4; i < 10; i++) { // 14
    if (vReal[i] < LOWPASS[2]) vReal[i] = 0;
    if (vReal[i] > colorMusic[2]) colorMusic[2] = vReal[i];
  }
  // средние частоты
  for (byte i = 10; i < 18; i++) { // 22
    if (vReal[i] < LOWPASS[3]) vReal[i] = 0;
    if (vReal[i] > colorMusic[3]) colorMusic[3] = vReal[i];
  }
  // высокие частоты
  for (byte i = 18; i < samples / 2; i++) {
    if (vReal[i] < LOWPASS[4]) vReal[i] = 0;
    if (vReal[i] > colorMusic[4]) colorMusic[4] = vReal[i];
  }

  pmax *= 0.992;
  //  pmin += 0.02;

  for (byte i = 0; i < 5; i++) {
    if (colorMusic[i] > pmax) {
      pmax = colorMusic[i];
    }
    if (colorMusic[i] < pmin) {
      pmin = colorMusic[i];
    }
  }

  float LV1 = Map(colorMusic[0], pmin, pmax, 0, 1.0 / (Br * 1));
  float LV2 = Map(colorMusic[1], pmin, pmax, 0, 1.0 / (Br * 1));
  float LV3 = Map(colorMusic[2], pmin, pmax, 0, 1.0 / (Br * 1));
  float LV4 = Map(colorMusic[3], pmin, pmax, 0, 1.0 / (Br * 1));
  float LV5 = Map(colorMusic[4], pmin, pmax, 0, 1.0 / (Br * 1));

  if (LV1 >= sLV1) {
    sLV1 = LV1;
  } else {
    if (sLV1) sLV1 -= 0.005;
    if (sLV1 < 0) sLV1 = 0;
  }
  if (LV2 >= sLV2) {
    sLV2 = LV2;
  } else {
    if (sLV2) sLV2 -= 0.005;
    if (sLV2 < 0) sLV2 = 0;
  }
  if (LV3 >= sLV3) {
    sLV3 = LV3;
  } else {
    if (sLV3) sLV3 -= 0.005;
    if (sLV3 < 0) sLV3 = 0;
  }
  if (LV4 >= sLV4) {
    sLV4 = LV4;
  } else {
    if (sLV4) sLV4 -= 0.005;
    if (sLV4 < 0) sLV4 = 0;
  }
  if (LV5 >= sLV5) {
    sLV5 = LV5;
  } else {
    if (sLV5) sLV5 -= 0.005;
    if (sLV5 < 0) sLV5 = 0;
  }

#define DEBUG

#ifdef DEBUG
  if (colorMusic[0] || colorMusic[1] || colorMusic[2] || colorMusic[3] || colorMusic[4]) {
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
    Serial.print(pmax);
    Serial.println();
  }
#endif

  HsbColor target;

  PCL = PixelCount / 9 * 9;
  int cn = PixelCount - PCL;
  int m = PCL / 9;

  //sLV1=sLV2=sLV3=sLV4=sLV5=0.2;

  int n = 0;

  for (int p = 0; p < m; p++) {

    //Serial.println(p);

    int cr = 0;

    n = map(1, 1, 9, 0, PCL - m);
    target = HsbColor(0.77, 1.0f, sLV5);
    strip.SetPixelColor((n + p + cr), target);
    if (cn > 1) {
      cr++;
      strip.SetPixelColor(n + p + cr, target);
    }

    n = map(2, 1, 9, 0, PCL - m);
    target = HsbColor(0, 1.0f, sLV4);
    strip.SetPixelColor(n + p + cr, target);
    if (cn > 3) {
      cr++;
      strip.SetPixelColor(n + p + cr, target);
    }

    n = map(3, 1, 9, 0, PCL - m);
    target = HsbColor(0.13, 1, sLV3);
    strip.SetPixelColor(n + p + cr, target);
    if (cn > 5) {
      cr++;
      strip.SetPixelColor(n + p + cr, target);
    }

    n = map(4, 1, 9, 0, PCL - m);
    target = HsbColor(0.33f, 1.0f, sLV2);
    strip.SetPixelColor(n + p + cr, target);

    n = Map(5, 1, 9, 0, PCL - m);
    target = HsbColor(0.66f, 1.0f, sLV1);
    strip.SetPixelColor(n + p + cr, target);
    if (cn % 2) {
      cr++;
      strip.SetPixelColor(n + p + cr, target);
    }

    n = Map(6, 1, 9, 0, PCL - m);
    target = HsbColor(0.33f, 1.0f, sLV2);
    strip.SetPixelColor(n + p + cr, target);

    n = Map(7, 1, 9, 0, PCL - m);
    target = HsbColor(0.13, 1.0f, sLV3);
    strip.SetPixelColor(n + p + cr , target);
    if (cn >= 6) {
      cr++;
      strip.SetPixelColor(n + p + cr, target);
    }

    n = Map(8, 1, 9, 0, PCL - m );
    target = HsbColor(0, 1.0f, sLV4);
    strip.SetPixelColor(n + p + cr, target);
    if (cn >= 4) {
      cr++;
      strip.SetPixelColor(n + p + cr, target);
    }

    n = Map(9, 1, 9, 0, PCL - m );
    target = HsbColor(0.77, 1.0f, sLV5);
    strip.SetPixelColor(n + p + cr, target);
    if (cn >= 2) {
      cr++;
      strip.SetPixelColor(n + p + cr, target);
    }

  }

  strip.Show();
}

void m4() {

  // long aaa = millis();
  analyzeAudio();
  // Serial.println(millis() - aaa);

  int LPF = 10;

  // lowpass filter
  for (int i = 2 ; i < samples / 2 ; i++) {
    if (vReal[i] < LPF ) vReal[i] = 0;
    //    else vReal[i] = vReal[i] * 5 ;

#define DEBUG_EQ

#ifdef DEBUG_EQ
    if (i < 16) {
      if (vReal[i] < 10) Serial.print(" ");
      Serial.print(vReal[i], 1);
      Serial.print(" ");
    }
#endif

  }

#ifdef DEBUG_EQ
  Serial.println();
#endif

  int pmax = 0;
  for (byte i = 2; i < samples / 2; i++) {

    if (vReal[i] > pmax) pmax = vReal[i];

    //  }

    HsbColor target;

    //  for (byte i = 2; i < samples / 2; i++) {
    int m = round(Map(i, 2, samples / 2 - 1, 0, PixelCount - 1));
    if (vReal[i] > LPF) {
      float LV = Map(vReal[i], LPF, pmax, 0, 1.0 / Br );
      target = HsbColor( (i - 2) * (360. / (samples / 2)) / 360., 1.0f, LV);
      RgbColor color = strip.GetPixelColor(m);
      HsbColor c = color;
      if (target.B > c.B) strip.SetPixelColor(m, target);
    } else {
      RgbColor color = strip.GetPixelColor(m);
      HsbColor c = color;
      if (c.B > 1. / Br / 4) {
        c.B -= 0.02;
      } else {
        c.B -= 0.02;
      }
      if (c.B < 0 ) c.B = 0;
      strip.SetPixelColor(m, c);
    }
  }
  strip.Show();
}
