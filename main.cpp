// Neopixel 8*8 matrix Audio Spectrum Visualizer
// Created by Kazuteru Yamada(yeisapporo).
//
// Analog input: A0
// Matrix output: 6

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN   (6)
#define LED_COUNT (8 * 8)

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

#define EZMTS_USE_TIMER (0)
#include <ezMTS.hpp>

ezMTS task(2, EZMTS_MILLISEC);
int taskIdMain;
int taskIdSpiral;
int taskIdReadAudio;
int taskIdDisplay;

#define FFT_NUM (16)
double rawAudioData[FFT_NUM] = {0};
double realSpec[FFT_NUM];
double imagSpec[FFT_NUM];
boolean flgFFTing = false;
boolean flgDisp = false;
unsigned char vRAM[8][8] = {0};

int8_t getAudioLevel2(int raw) {
  int8_t level;

  if(raw >= 8) {
    level = 7;
  } else if(raw >= 7) {
    level = 6;
  } else if(raw >= 6) {
    level = 5;
  } else if(raw >= 5) {
    level = 4;
  } else if(raw >= 4) {
    level = 3;
  } else if(raw >= 3) {
    level = 2;
  } else if(raw >= 2) {
    level = 1;
  } else if(raw >= 1) {
    level = 0;
  } else {
    level = -1;
  }

  return level;
}

void calcFFT(double *iW, double *oR, double *oI) {
  static double fReal[FFT_NUM], fImag[FFT_NUM];
  short N = FFT_NUM;
  short stg;
  short halfN = N / 2;
  short i, j, k, kp, m, h;
  double w1, w2, s1, s2, t1, t2;

  // number of stages
  i = N;
  stg = 0;
  while (i != 1) {
    i = i / 2;
    stg++;
  }

  // data input
  for (i = 0; i < N; i++) {
    double x = (double)i / N;
    // apply Blackman window
    fReal[i] = iW[i] * (0.42 - 0.5 * cos(2 * PI * x) + 0.08 * cos(4 * PI * x));
    // apply Nattoll window
    //fReal[i] = iW[i] * (0.355768 - 0.487396 * cos(2 * PI * x) + 0.144232 * cos(4 * PI * x) - 0.012604 * cos(6 * PI * x));
    fImag[i] = 0;
  }

  // bit-reveral sorting
  j = 0;
  for (i = 0; i <= N - 2; i++) {
    if (i < j) {
      t1 = fReal[j];
      fReal[j] = fReal[i];
      fReal[i] = t1;
    }
    k = halfN;
    while (k <= j) {
      j = j - k;
      k = k / 2;
    }
    j += k;
  }

  // butterfly calculation
  for (i = 1; i <= stg; i++) {
    m = pow(2, i);
    h = m / 2;
    for (j = 0; j < h; j++) {
      double w1in = j * (N / m);
      double w2in = w1in + halfN;
      w1 = cos(w1in / (FFT_NUM / (2 * PI)));
      w2 = sin(w2in / (FFT_NUM / (2 * PI)));
      for (k = j; k < N; k += m) {
        kp = k + h;
        s1 = fReal[kp] * w1 - fImag[kp] * w2;
        s2 = fReal[kp] * w2 + fImag[kp] * w1;
        t1 = fReal[k] + s1;
        fReal[kp] = fReal[k] - s1;
        fReal[k] = t1;
        t2 = fImag[k] + s2;
        fImag[kp] = fImag[k] - s2;
        fImag[k] = t2;
      }
    }
  }

  // result output
  for (i = 0; i < N; i++) {
    oR[i] = fReal[i];
    oI[i] = fImag[i];
  }
}

int readAudioData(void *dummy) {
  static int idx = 0;
  
  rawAudioData[idx++] = (analogRead(A0)) / 64;
  if(idx == FFT_NUM) {
    flgFFTing = true;
    calcFFT(rawAudioData, realSpec, imagSpec);
    for(int i = 0; i < FFT_NUM; i++) {
      realSpec[i] = sqrt(realSpec[i] * realSpec[i] + imagSpec[i] * imagSpec[i]);
    } 
    flgFFTing = false;
    idx = 0;
  }

  return 0;
}

int displaySpectrum(void *dummy) {
  int x = 0;
  int y = 0;
  static int top[8] = {0};
  static int topTimer[8];

  if(flgFFTing) {
    return 0;
  }

  flgDisp = true;
  for(x = 0; x < 8; x++) {
    int height = getAudioLevel2(realSpec[x]);
    for(int i = 0; i < 8; i++) {
      vRAM[x][i]  = 0;
    }
    if(height >= 0) {
      for(y = 0; y <= height; y++) {
        vRAM[x][y] = 255;
      }
    }
      if(top[x] < height) {
        top[x] = height;
        topTimer[x] = 80;
      } else {
        if(--topTimer[x] <= 0) {
          topTimer[x] = 4;
          top[x]--;
          if(top[x] <= -1) {
            top[x] = -1;
          }
        }
      }
  }
  strip.clear();
  for (x = 0; x < 8; x++) {
    for (y = 0; y < 8; y++) {
      strip.setPixelColor(x * 8 + y, vRAM[x][y]);
    }
      if(top[x] > 0) {
        if(top[x] == 7) {
          strip.setPixelColor(x * 8 + top[x], strip.Color(127, 0, 0));
        } else {
          strip.setPixelColor(x * 8 + top[x], strip.Color(127, 127, 127));
        }
      }
  }
  strip.show();
  flgDisp = false;

  return 0;
}

void setup() {
  Serial.begin(115200);
  pinMode(A0, INPUT);

  strip.begin();
  strip.show();
  strip.setBrightness(10);

  taskIdReadAudio = task.create(readAudioData);
  task.start(taskIdReadAudio, 4, EZMTS_AT_ONCE);
  taskIdDisplay = task.create(displaySpectrum);
  task.start(taskIdDisplay, 10, EZMTS_AT_ONCE);
}

void loop() {
}
