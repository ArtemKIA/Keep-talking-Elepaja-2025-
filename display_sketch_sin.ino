#include <Arduino.h>
#include <U8g2lib.h>

// I2C define
U8G2_SH1107_SEEED_128X128_F_HW_I2C u8g2(
  U8G2_R0,               // U8G2_R2 if 180° off
  U8X8_PIN_NONE,
  /*SCL clock=*/9,
  /*SDA data =*/8
);

// SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP
const int W = 128, H = 128; //Display dimension

const int cy = 64;          //Vertical center line
const float A = 40.0f;      //Amplitude
static float k = 10.0f;            //Cycles across the 128px width
const float phi = 0.0f;     //Phase for animation

const int POT_K_PIN = 4;

static inline int X(int x) {return x;}
static inline int Y(int y) {return y;}

void setup() {
  analogReadResolution(12);
  analogSetPinAttenuation(POT_K_PIN, ADC_11db);
  u8g2.setI2CAddress(0x3C << 1);
  u8g2.setBusClock(400000);
  u8g2.begin();
}

void loop() {
  u8g2.clearBuffer();

  int raw = analogRead(POT_K_PIN);          // 0..4095
  float k_target = 0.25f + (5.75f * raw / 4095.0f);  // 0.25 .. 6.0 cycles
  k = 0.85f * k + 0.15f * k_target;         // low-pass so it doesn’t jitter

  int prevx = 0, prevy = cy;
  for (int x=0; x<128; ++x) {
    float theta = 2.0f * PI * (k * x / (float)W) + phi; //rad
    int y = cy - (int)lrintf(A * sinf(theta));

    //u8g2.drawPixel(x, y); //single pixels
    u8g2.drawLine(prevx, prevy, x, y); //Draw + Connect dot samples

    prevx = x; prevy = y;
  }

  u8g2.drawFrame(X(0), Y(0), 128, 128);     // full-frame box to check usable area
  u8g2.sendBuffer();

  delay(10);
}