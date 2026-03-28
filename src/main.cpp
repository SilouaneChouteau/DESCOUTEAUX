#include <Arduino.h>
#include <U8g2lib.h>
#include <arduinoFFT.h>

// =========================================================
//  spec analyzer + proximity detector ~ nano atmega328
// =========================================================

// --- screens (_2_ = 2-page buffer = 256b each, fits in RAM) ---
U8G2_SSD1306_128X64_NONAME_2_SW_I2C displayLo(U8G2_R0, A5, A4, U8X8_PIN_NONE);
U8G2_SSD1306_128X64_NONAME_2_SW_I2C displayHi(U8G2_R0, A3, A2, U8X8_PIN_NONE);

// --- 7-seg display ---
const uint8_t segPins[]  = {2, 3, 4, 5, 6, 7, 8};
const uint8_t dizainePin = 9;
const uint8_t unitePin   = A1;

// --- HC-SR04 ---
const uint8_t trigPin = 10;
const uint8_t echoPin = 11;

// --- buzzer ---
const uint8_t buzzerPin = 12;

// --- proximity threshold (cm) ---
const uint8_t seuil = 10;

// =========================================================
//  FFT / audio
// =========================================================

#define SAMPLES        64
#define SAMPLING_FREQ  8000   // 125 Hz/bin

// int16 samples to save 256b vs float
int16_t vReal[SAMPLES];
int16_t vImag[SAMPLES];

// float buffer only for the FFT lib
float fReal[SAMPLES];
float fImag[SAMPLES];
ArduinoFFT<float> FFT = ArduinoFFT<float>(fReal, fImag, SAMPLES, SAMPLING_FREQ);

#define LO_BANDS  8
#define HI_BANDS  8

// uint16 bands/peaks to save RAM vs float
uint16_t loBands[LO_BANDS];
uint16_t hiBands[HI_BANDS];
uint16_t loPeaks[LO_BANDS];
uint16_t hiPeaks[HI_BANDS];

// bin tables in PROGMEM (flash, not RAM)
const uint8_t loBinStart[LO_BANDS] PROGMEM = { 1,  3,  5,  7,  9, 11, 13, 15};
const uint8_t loBinEnd  [LO_BANDS] PROGMEM = { 2,  4,  6,  8, 10, 12, 14, 16};
const uint8_t hiBinStart[HI_BANDS] PROGMEM = {16, 18, 20, 22, 24, 26, 28, 30};
const uint8_t hiBinEnd  [HI_BANDS] PROGMEM = {17, 19, 21, 23, 25, 27, 29, 31};

// labels in PROGMEM
const char loL0[] PROGMEM = "125";  const char loL1[] PROGMEM = "375";
const char loL2[] PROGMEM = "625";  const char loL3[] PROGMEM = "875";
const char loL4[] PROGMEM = "1.1k"; const char loL5[] PROGMEM = "1.4k";
const char loL6[] PROGMEM = "1.6k"; const char loL7[] PROGMEM = "1.9k";
const char* const loLabels[LO_BANDS] PROGMEM = {loL0,loL1,loL2,loL3,loL4,loL5,loL6,loL7};

const char hiL0[] PROGMEM = "2k";   const char hiL1[] PROGMEM = "2.2k";
const char hiL2[] PROGMEM = "2.5k"; const char hiL3[] PROGMEM = "2.8k";
const char hiL4[] PROGMEM = "3k";   const char hiL5[] PROGMEM = "3.2k";
const char hiL6[] PROGMEM = "3.5k"; const char hiL7[] PROGMEM = "3.8k";
const char* const hiLabels[HI_BANDS] PROGMEM = {hiL0,hiL1,hiL2,hiL3,hiL4,hiL5,hiL6,hiL7};

// temp buffer to read a label from PROGMEM
char labelBuf[6];

// integer constants instead of floats (saves global RAM)
#define PEAK_DECAY_NUM   15    // ~0.9375
#define PEAK_DECAY_DEN   16
#define MIN_FLOOR        10
#define HEADROOM_LO_NUM  8     // 1.6 = 8/5
#define HEADROOM_LO_DEN  5
#define HEADROOM_HI_NUM  2     // 2.0
#define HEADROOM_HI_DEN  1
#define SCALE_ATTACK_NUM 1
#define SCALE_ATTACK_DEN 12    // ~0.083
#define SCALE_DECAY_NUM  1
#define SCALE_DECAY_DEN  1000

uint16_t loMax = (uint16_t)(MIN_FLOOR * HEADROOM_LO_NUM / HEADROOM_LO_DEN);
uint16_t hiMax = (uint16_t)(MIN_FLOOR * HEADROOM_HI_NUM / HEADROOM_HI_DEN);

// --- screen layout ---
#define TITLE_H    11
#define LABEL_H    11
#define YAXIS_W    13
#define BAR_ZONE_X (YAXIS_W)
#define BAR_ZONE_W (128 - YAXIS_W)
#define BAR_ZONE_Y (TITLE_H)
#define BAR_ZONE_H (64 - TITLE_H - LABEL_H)   // 42px

// =========================================================
//  prototypes
// =========================================================
void sampleAudio();
void computeSpectrum();
void computeBands(const uint8_t* binStart, const uint8_t* binEnd,
                  uint16_t* bands, uint16_t* peaks, uint8_t n);
void updateDynMax(uint16_t* bands, uint8_t n, uint16_t& dynMax,
                  uint8_t hNum, uint8_t hDen);
void drawScreen(U8G2_SSD1306_128X64_NONAME_2_SW_I2C& disp,
                uint16_t* bands, uint16_t* peaks, uint16_t dynMax,
                uint8_t numBands, const char* title,
                const char* const* labels);
long    lireDistance();
void    eteindreSegments();
void    afficherD();
void    afficherG();
void    afficherDG();
void    afficherRien();

// =========================================================
//  setup
// =========================================================
void setup() {
  for (uint8_t i = 0; i < 7; i++) pinMode(segPins[i], OUTPUT);
  pinMode(dizainePin, OUTPUT);
  pinMode(unitePin,   OUTPUT);
  afficherRien();

  pinMode(trigPin,   OUTPUT);
  pinMode(echoPin,   INPUT);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  displayLo.setI2CAddress(0x3C * 2);
  displayLo.begin();
  displayLo.firstPage();
  do {
    displayLo.setFont(u8g2_font_6x10_tf);
    displayLo.drawStr(20, 35, "BASSES...");
  } while (displayLo.nextPage());

  displayHi.setI2CAddress(0x3D * 2);
  displayHi.begin();
  displayHi.firstPage();
  do {
    displayHi.setFont(u8g2_font_6x10_tf);
    displayHi.drawStr(20, 35, "AIGUS...");
  } while (displayHi.nextPage());

  for (uint8_t i = 0; i < LO_BANDS; i++) loPeaks[i] = 0;
  for (uint8_t i = 0; i < HI_BANDS; i++) hiPeaks[i] = 0;

  delay(800);
}

// =========================================================
//  loop
// =========================================================
void loop() {
  long distance = lireDistance();
  bool danger   = (distance > 0 && distance < seuil);

  if (danger) {
    // danger mode: buzz + DG on 7-seg
    digitalWrite(buzzerPin, HIGH);
    for (uint8_t i = 0; i < 80; i++) afficherDG();

  } else {
    // normal mode: spectrum analyzer on both OLEDs
    digitalWrite(buzzerPin, LOW);
    afficherRien();

    sampleAudio();
    computeSpectrum();
    computeBands(loBinStart, loBinEnd, loBands, loPeaks, LO_BANDS);
    computeBands(hiBinStart, hiBinEnd, hiBands, hiPeaks, HI_BANDS);
    updateDynMax(loBands, LO_BANDS, loMax, HEADROOM_LO_NUM, HEADROOM_LO_DEN);
    updateDynMax(hiBands, HI_BANDS, hiMax, HEADROOM_HI_NUM, HEADROOM_HI_DEN);

    drawScreen(displayLo, loBands, loPeaks, loMax, LO_BANDS,
               "BASSES Hz", loLabels);
    drawScreen(displayHi, hiBands, hiPeaks, hiMax, HI_BANDS,
               "AIGUS Hz",  hiLabels);
  }
}

// =========================================================
//  7-seg helpers
// =========================================================
void eteindreSegments() {
  for (uint8_t i = 0; i < 7; i++) digitalWrite(segPins[i], LOW);
}
void afficherRien() {
  digitalWrite(dizainePin, HIGH);
  digitalWrite(unitePin,   HIGH);
  eteindreSegments();
}
void afficherD() {
  // segments b c d e g
  const uint8_t seg[] = {0,1,1,1,1,0,1};
  for (uint8_t i = 0; i < 7; i++) digitalWrite(segPins[i], seg[i]);
}
void afficherG() {
  // segments a c d e f
  const uint8_t seg[] = {1,0,1,1,1,1,0};
  for (uint8_t i = 0; i < 7; i++) digitalWrite(segPins[i], seg[i]);
}
void afficherDG() {
  // multiplexed D / G
  afficherRien(); digitalWrite(dizainePin, LOW); afficherD(); delay(3);
  afficherRien(); digitalWrite(unitePin,   LOW); afficherG(); delay(3);
}

// =========================================================
//  HC-SR04
// =========================================================
long lireDistance() {
  digitalWrite(trigPin, LOW);  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long d = pulseIn(echoPin, HIGH, 30000UL);
  return d * 0.034 / 2;
}

// =========================================================
//  audio / FFT
// =========================================================
void sampleAudio() {
  const unsigned int period = 1000000UL / SAMPLING_FREQ;
  for (uint8_t i = 0; i < SAMPLES; i++) {
    unsigned long t = micros();
    vReal[i] = (int16_t)(analogRead(A0) - 512);
    vImag[i] = 0;
    while (micros() - t < period);
  }
}

void computeSpectrum() {
  // copy int16 -> float for the FFT lib
  for (uint8_t i = 0; i < SAMPLES; i++) {
    fReal[i] = (float)vReal[i];
    fImag[i] = 0.0f;
  }
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();
}

void computeBands(const uint8_t* binStart, const uint8_t* binEnd,
                  uint16_t* bands, uint16_t* peaks, uint8_t n) {
  for (uint8_t b = 0; b < n; b++) {
    uint8_t ks  = pgm_read_byte(&binStart[b]);
    uint8_t ke  = pgm_read_byte(&binEnd[b]);
    float   val = 0.0f;
    for (uint8_t k = ks; k <= ke; k++)
      if (fReal[k] > val) val = fReal[k];

    uint16_t v = (val < MIN_FLOOR) ? 0 : (uint16_t)val;
    bands[b] = v;
    if (v >= peaks[b]) peaks[b] = v;
    else peaks[b] = (uint16_t)((uint32_t)peaks[b] * PEAK_DECAY_NUM / PEAK_DECAY_DEN);
  }
}

void updateDynMax(uint16_t* bands, uint8_t n, uint16_t& dynMax,
                  uint8_t hNum, uint8_t hDen) {
  uint16_t frameMax = MIN_FLOOR;
  for (uint8_t i = 0; i < n; i++)
    if (bands[i] > frameMax) frameMax = bands[i];

  uint16_t target = (uint16_t)((uint32_t)frameMax * hNum / hDen);
  if (target > dynMax)
    dynMax += (uint16_t)((uint32_t)(target - dynMax) * SCALE_ATTACK_NUM / SCALE_ATTACK_DEN);
  else if (dynMax > target)
    dynMax -= (uint16_t)((uint32_t)(dynMax - target) * SCALE_DECAY_NUM / SCALE_DECAY_DEN);

  uint16_t minVal = (uint16_t)((uint32_t)MIN_FLOOR * hNum / hDen);
  if (dynMax < minVal) dynMax = minVal;
}

// =========================================================
//  OLED draw
//  fix: getBufferCurrTileRow() tells which 16px slice is
//  being rendered -> each element drawn only once
// =========================================================
void drawScreen(U8G2_SSD1306_128X64_NONAME_2_SW_I2C& disp,
                uint16_t* bands, uint16_t* peaks, uint16_t dynMax,
                uint8_t numBands, const char* title,
                const char* const* labels) {

  const uint8_t gap  = 2;
  const uint8_t barW = (BAR_ZONE_W - (numBands + 1) * gap) / numBands;
  const int yBottom  = BAR_ZONE_Y + BAR_ZONE_H - 1;  // 52
  const int yMid     = BAR_ZONE_Y + BAR_ZONE_H / 2;  // 32
  const int yTop     = BAR_ZONE_Y;                    // 11

  disp.firstPage();
  do {
    // current 16px window
    uint8_t tileRow = disp.getBufferCurrTileRow();
    int     py0     = tileRow * 8;
    int     py1     = py0 + 15;

    // title + separator (y 0..11)
    if (py0 <= TITLE_H) {
      disp.setFont(u8g2_font_6x10_tf);
      disp.drawStr(YAXIS_W, 9, title);
      disp.drawHLine(0, TITLE_H, 128);
    }

    disp.setFont(u8g2_font_4x6_tf);

    // vertical axis
    disp.drawVLine(YAXIS_W - 1, BAR_ZONE_Y, BAR_ZONE_H);

    // mid dotted line
    if (yMid >= py0 && yMid <= py1)
      for (uint8_t x = YAXIS_W; x < 128; x += 4) disp.drawPixel(x, yMid);

    // Y axis labels
    if ((yTop + 5) >= py0 && yTop <= py1)
      disp.drawStr(0, yTop + 5, "max");
    if ((yMid + 3) >= py0 && yMid <= py1)
      disp.drawStr(2, yMid + 3, "50");
    if ((yBottom - 1) >= py0 && (yBottom - 1) <= py1)
      disp.drawStr(4, yBottom - 1, "0");

    // bottom axis line
    if ((yBottom + 1) >= py0 && (yBottom + 1) <= py1)
      disp.drawHLine(YAXIS_W - 1, yBottom + 1, 128 - YAXIS_W + 1);

    // bars, peaks, freq labels
    for (uint8_t b = 0; b < numBands; b++) {
      int x = BAR_ZONE_X + gap + b * (barW + gap);

      // bar
      float norm = (float)bands[b] / dynMax;
      if (norm > 1.0f) norm = 1.0f;
      int barH = (int)(norm * (BAR_ZONE_H - 1));
      if (barH > 0) {
        int rStart = max(yBottom - barH, py0);
        int rEnd   = min(yBottom, py1);
        for (int row = rStart; row <= rEnd; row++)
          if ((yBottom - row) % 3 != 0)
            disp.drawHLine(x, row, barW);
      }

      // peak dot
      float normP = (float)peaks[b] / dynMax;
      if (normP > 1.0f) normP = 1.0f;
      int peakY = yBottom - (int)(normP * (BAR_ZONE_H - 1));
      if (peakY >= yTop && peakY <= yBottom) {
        if (peakY >= py0 && peakY <= py1)
          disp.drawHLine(x, peakY, barW);
        if (peakY > yTop && (peakY - 1) >= py0 && (peakY - 1) <= py1)
          disp.drawHLine(x, peakY - 1, barW);
      }

      // freq label - only on last slice (y=63)
      if (63 >= py0 && 63 <= py1) {
        strcpy_P(labelBuf, (char*)pgm_read_word(&labels[b]));
        uint8_t ll = strlen(labelBuf);
        int lx = x + barW / 2 - (ll * 4) / 2;
        if (lx < YAXIS_W) lx = YAXIS_W;
        disp.drawStr(lx, 63, labelBuf);
      }
    }

  } while (disp.nextPage());
}