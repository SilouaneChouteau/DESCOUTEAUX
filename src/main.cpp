#include <Arduino.h>
#include <U8g2lib.h>
#include <arduinoFFT.h>

U8G2_SSD1306_128X64_NONAME_2_SW_I2C ecranBas(U8G2_R0, A5, A4, U8X8_PIN_NONE);
U8G2_SSD1306_128X64_NONAME_2_SW_I2C ecranHaut(U8G2_R0, A3, A2, U8X8_PIN_NONE);

const uint8_t brochesSegments[] = {2, 3, 4, 5, 6, 7, 8};
const uint8_t brocheDizaine    = 9;
const uint8_t brocheUnite      = A1;
const uint8_t brocheTrig       = 10;
const uint8_t brocheEcho       = 11;
const uint8_t brocheBuzzer     = 12;
const uint8_t seuil            = 10;

#define ECHANTILLONS     64
#define FREQ_ECHANT      8000

int16_t vReel[ECHANTILLONS];
int16_t vImag[ECHANTILLONS];
float   fReel[ECHANTILLONS];
float   fImag[ECHANTILLONS];
ArduinoFFT<float> FFT = ArduinoFFT<float>(fReel, fImag, ECHANTILLONS, FREQ_ECHANT);

#define NB_BANDES_BAS   8
#define NB_BANDES_HAUT  8

uint16_t bandesBas[NB_BANDES_BAS];
uint16_t bandesHaut[NB_BANDES_HAUT];
uint16_t picsBas[NB_BANDES_BAS];
uint16_t picsHaut[NB_BANDES_HAUT];

const uint8_t debutBinBas[NB_BANDES_BAS]  PROGMEM = { 1,  3,  5,  7,  9, 11, 13, 15};
const uint8_t finBinBas[NB_BANDES_BAS]    PROGMEM = { 2,  4,  6,  8, 10, 12, 14, 16};
const uint8_t debutBinHaut[NB_BANDES_HAUT] PROGMEM = {16, 18, 20, 22, 24, 26, 28, 30};
const uint8_t finBinHaut[NB_BANDES_HAUT]   PROGMEM = {17, 19, 21, 23, 25, 27, 29, 31};

const char etBas0[] PROGMEM = "125";  const char etBas1[] PROGMEM = "375";
const char etBas2[] PROGMEM = "625";  const char etBas3[] PROGMEM = "875";
const char etBas4[] PROGMEM = "1.1k"; const char etBas5[] PROGMEM = "1.4k";
const char etBas6[] PROGMEM = "1.6k"; const char etBas7[] PROGMEM = "1.9k";
const char* const etiquettesBas[NB_BANDES_BAS] PROGMEM = {etBas0,etBas1,etBas2,etBas3,etBas4,etBas5,etBas6,etBas7};

const char etHaut0[] PROGMEM = "2k";   const char etHaut1[] PROGMEM = "2.2k";
const char etHaut2[] PROGMEM = "2.5k"; const char etHaut3[] PROGMEM = "2.8k";
const char etHaut4[] PROGMEM = "3k";   const char etHaut5[] PROGMEM = "3.2k";
const char etHaut6[] PROGMEM = "3.5k"; const char etHaut7[] PROGMEM = "3.8k";
const char* const etiquettesHaut[NB_BANDES_HAUT] PROGMEM = {etHaut0,etHaut1,etHaut2,etHaut3,etHaut4,etHaut5,etHaut6,etHaut7};

char tampEtiquette[6];

#define DECROISSANCE_PIC_NUM   15
#define DECROISSANCE_PIC_DEN   16
#define PLANCHER_MIN           10
#define MARGE_BAS_NUM          8
#define MARGE_BAS_DEN          5
#define MARGE_HAUT_NUM         2
#define MARGE_HAUT_DEN         1
#define ATTAQUE_ECHELLE_NUM    1
#define ATTAQUE_ECHELLE_DEN    12
#define DECROISSANCE_ECHELLE_NUM 1
#define DECROISSANCE_ECHELLE_DEN 1000

uint16_t maxDynBas  = (uint16_t)(PLANCHER_MIN * MARGE_BAS_NUM  / MARGE_BAS_DEN);
uint16_t maxDynHaut = (uint16_t)(PLANCHER_MIN * MARGE_HAUT_NUM / MARGE_HAUT_DEN);

#define TITRE_H    11
#define ETIQUETTE_H 11
#define AXEY_W     13
#define ZONE_X     (AXEY_W)
#define ZONE_W     (128 - AXEY_W)
#define ZONE_Y     (TITRE_H)
#define ZONE_H     (64 - TITRE_H - ETIQUETTE_H)

void echantillonnerAudio();
void calculerSpectre();
void calculerBandes(const uint8_t* debutBin, const uint8_t* finBin,
                    uint16_t* bandes, uint16_t* pics, uint8_t n);
void mettreAJourMaxDyn(uint16_t* bandes, uint8_t n, uint16_t& maxDyn,
                       uint8_t mNum, uint8_t mDen);
void dessinerEcran(U8G2_SSD1306_128X64_NONAME_2_SW_I2C& ecran,
                   uint16_t* bandes, uint16_t* pics, uint16_t maxDyn,
                   uint8_t nbBandes, const char* titre,
                   const char* const* etiquettes);
long    lireDistance();
void    eteindreSegments();
void    afficherD();
void    afficherG();
void    afficherDG();
void    afficherRien();

void setup() {
  for (uint8_t i = 0; i < 7; i++) pinMode(brochesSegments[i], OUTPUT);
  pinMode(brocheDizaine, OUTPUT);
  pinMode(brocheUnite,   OUTPUT);
  afficherRien();

  pinMode(brocheTrig,   OUTPUT);
  pinMode(brocheEcho,   INPUT);
  pinMode(brocheBuzzer, OUTPUT);
  digitalWrite(brocheBuzzer, LOW);

  ecranBas.setI2CAddress(0x3C * 2);
  ecranBas.begin();
  ecranBas.firstPage();
  do {
    ecranBas.setFont(u8g2_font_6x10_tf);
    ecranBas.drawStr(20, 35, "BASSES...");
  } while (ecranBas.nextPage());

  ecranHaut.setI2CAddress(0x3D * 2);
  ecranHaut.begin();
  ecranHaut.firstPage();
  do {
    ecranHaut.setFont(u8g2_font_6x10_tf);
    ecranHaut.drawStr(20, 35, "AIGUS...");
  } while (ecranHaut.nextPage());

  for (uint8_t i = 0; i < NB_BANDES_BAS;  i++) picsBas[i]  = 0;
  for (uint8_t i = 0; i < NB_BANDES_HAUT; i++) picsHaut[i] = 0;

  delay(800);
}

void loop() {
  long distance = lireDistance();
  bool danger   = (distance > 0 && distance < seuil);

  if (danger) {
    digitalWrite(brocheBuzzer, HIGH);
    for (uint8_t i = 0; i < 80; i++) afficherDG();
  } else {
    digitalWrite(brocheBuzzer, LOW);
    afficherRien();

    echantillonnerAudio();
    calculerSpectre();
    calculerBandes(debutBinBas,  finBinBas,  bandesBas,  picsBas,  NB_BANDES_BAS);
    calculerBandes(debutBinHaut, finBinHaut, bandesHaut, picsHaut, NB_BANDES_HAUT);
    mettreAJourMaxDyn(bandesBas,  NB_BANDES_BAS,  maxDynBas,  MARGE_BAS_NUM,  MARGE_BAS_DEN);
    mettreAJourMaxDyn(bandesHaut, NB_BANDES_HAUT, maxDynHaut, MARGE_HAUT_NUM, MARGE_HAUT_DEN);

    dessinerEcran(ecranBas,  bandesBas,  picsBas,  maxDynBas,  NB_BANDES_BAS,  "BASSES Hz", etiquettesBas);
    dessinerEcran(ecranHaut, bandesHaut, picsHaut, maxDynHaut, NB_BANDES_HAUT, "AIGUS Hz",  etiquettesHaut);
  }
}

void eteindreSegments() {
  for (uint8_t i = 0; i < 7; i++) digitalWrite(brochesSegments[i], LOW);
}
void afficherRien() {
  digitalWrite(brocheDizaine, HIGH);
  digitalWrite(brocheUnite,   HIGH);
  eteindreSegments();
}
void afficherD() {
  const uint8_t seg[] = {0,1,1,1,1,0,1};
  for (uint8_t i = 0; i < 7; i++) digitalWrite(brochesSegments[i], seg[i]);
}
void afficherG() {
  const uint8_t seg[] = {1,0,1,1,1,1,0};
  for (uint8_t i = 0; i < 7; i++) digitalWrite(brochesSegments[i], seg[i]);
}
void afficherDG() {
  afficherRien(); digitalWrite(brocheDizaine, LOW); afficherD(); delay(3);
  afficherRien(); digitalWrite(brocheUnite,   LOW); afficherG(); delay(3);
}

long lireDistance() {
  digitalWrite(brocheTrig, LOW);  delayMicroseconds(2);
  digitalWrite(brocheTrig, HIGH); delayMicroseconds(10);
  digitalWrite(brocheTrig, LOW);
  long d = pulseIn(brocheEcho, HIGH, 30000UL);
  return d * 0.034 / 2;
}

void echantillonnerAudio() {
  const unsigned int periode = 1000000UL / FREQ_ECHANT;
  for (uint8_t i = 0; i < ECHANTILLONS; i++) {
    unsigned long t = micros();
    vReel[i] = (int16_t)(analogRead(A0) - 512);
    vImag[i] = 0;
    while (micros() - t < periode);
  }
}

void calculerSpectre() {
  for (uint8_t i = 0; i < ECHANTILLONS; i++) {
    fReel[i] = (float)vReel[i];
    fImag[i] = 0.0f;
  }
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();
}

void calculerBandes(const uint8_t* debutBin, const uint8_t* finBin,
                    uint16_t* bandes, uint16_t* pics, uint8_t n) {
  for (uint8_t b = 0; b < n; b++) {
    uint8_t ks  = pgm_read_byte(&debutBin[b]);
    uint8_t ke  = pgm_read_byte(&finBin[b]);
    float   val = 0.0f;
    for (uint8_t k = ks; k <= ke; k++)
      if (fReel[k] > val) val = fReel[k];

    uint16_t v = (val < PLANCHER_MIN) ? 0 : (uint16_t)val;
    bandes[b] = v;
    if (v >= pics[b]) pics[b] = v;
    else pics[b] = (uint16_t)((uint32_t)pics[b] * DECROISSANCE_PIC_NUM / DECROISSANCE_PIC_DEN);
  }
}

void mettreAJourMaxDyn(uint16_t* bandes, uint8_t n, uint16_t& maxDyn,
                       uint8_t mNum, uint8_t mDen) {
  uint16_t maxTrame = PLANCHER_MIN;
  for (uint8_t i = 0; i < n; i++)
    if (bandes[i] > maxTrame) maxTrame = bandes[i];

  uint16_t cible = (uint16_t)((uint32_t)maxTrame * mNum / mDen);
  if (cible > maxDyn)
    maxDyn += (uint16_t)((uint32_t)(cible - maxDyn) * ATTAQUE_ECHELLE_NUM / ATTAQUE_ECHELLE_DEN);
  else if (maxDyn > cible)
    maxDyn -= (uint16_t)((uint32_t)(maxDyn - cible) * DECROISSANCE_ECHELLE_NUM / DECROISSANCE_ECHELLE_DEN);

  uint16_t valMin = (uint16_t)((uint32_t)PLANCHER_MIN * mNum / mDen);
  if (maxDyn < valMin) maxDyn = valMin;
}

void dessinerEcran(U8G2_SSD1306_128X64_NONAME_2_SW_I2C& ecran,
                   uint16_t* bandes, uint16_t* pics, uint16_t maxDyn,
                   uint8_t nbBandes, const char* titre,
                   const char* const* etiquettes) {

  const uint8_t ecart = 2;
  const uint8_t largBarre = (ZONE_W - (nbBandes + 1) * ecart) / nbBandes;
  const int yBas  = ZONE_Y + ZONE_H - 1;
  const int yMil  = ZONE_Y + ZONE_H / 2;
  const int yHaut = ZONE_Y;

  ecran.firstPage();
  do {
    uint8_t rangTuile = ecran.getBufferCurrTileRow();
    int     py0       = rangTuile * 8;
    int     py1       = py0 + 15;

    if (py0 <= TITRE_H) {
      ecran.setFont(u8g2_font_6x10_tf);
      ecran.drawStr(AXEY_W, 9, titre);
      ecran.drawHLine(0, TITRE_H, 128);
    }

    ecran.setFont(u8g2_font_4x6_tf);
    ecran.drawVLine(AXEY_W - 1, ZONE_Y, ZONE_H);

    if (yMil >= py0 && yMil <= py1)
      for (uint8_t x = AXEY_W; x < 128; x += 4) ecran.drawPixel(x, yMil);

    if ((yHaut + 5) >= py0 && yHaut <= py1)
      ecran.drawStr(0, yHaut + 5, "max");
    if ((yMil + 3) >= py0 && yMil <= py1)
      ecran.drawStr(2, yMil + 3, "50");
    if ((yBas - 1) >= py0 && (yBas - 1) <= py1)
      ecran.drawStr(4, yBas - 1, "0");

    if ((yBas + 1) >= py0 && (yBas + 1) <= py1)
      ecran.drawHLine(AXEY_W - 1, yBas + 1, 128 - AXEY_W + 1);

    for (uint8_t b = 0; b < nbBandes; b++) {
      int x = ZONE_X + ecart + b * (largBarre + ecart);

      float norme = (float)bandes[b] / maxDyn;
      if (norme > 1.0f) norme = 1.0f;
      int hautBarre = (int)(norme * (ZONE_H - 1));
      if (hautBarre > 0) {
        int rDebut = max(yBas - hautBarre, py0);
        int rFin   = min(yBas, py1);
        for (int ligne = rDebut; ligne <= rFin; ligne++)
          if ((yBas - ligne) % 3 != 0)
            ecran.drawHLine(x, ligne, largBarre);
      }

      float normePic = (float)pics[b] / maxDyn;
      if (normePic > 1.0f) normePic = 1.0f;
      int yPic = yBas - (int)(normePic * (ZONE_H - 1));
      if (yPic >= yHaut && yPic <= yBas) {
        if (yPic >= py0 && yPic <= py1)
          ecran.drawHLine(x, yPic, largBarre);
        if (yPic > yHaut && (yPic - 1) >= py0 && (yPic - 1) <= py1)
          ecran.drawHLine(x, yPic - 1, largBarre);
      }

      if (63 >= py0 && 63 <= py1) {
        strcpy_P(tampEtiquette, (char*)pgm_read_word(&etiquettes[b]));
        uint8_t ll = strlen(tampEtiquette);
        int lx = x + largBarre / 2 - (ll * 4) / 2;
        if (lx < AXEY_W) lx = AXEY_W;
        ecran.drawStr(lx, 63, tampEtiquette);
      }
    }

  } while (ecran.nextPage());
}
