#define ver "v1.25"
//bool DEBUG = false ; // Si débug, on affiche toutes les données sur le port série.
bool DEBUG = true ;

// Doit être définit avant d'inclure la libraire LilyGoWatch
#define LILYGO_WATCH_2020_V2             // T-Watch 2020 VERSION 2 EXCLUSIVEMENT
#include <LilyGoWatch.h>

#include "SL31intro.h"  //image du bateau de coté pour écran intro
#include "SL31160NB.h"  //image contours du bateau pour rosaces
//#include "SB240.h";     //image JPG de Sylvie w=209, h=240, convertie par http://www.rinkydinkelectronics.com/t_imageconverter565.php
#include "LOGOHO.h";    //Logo Hisse et Oh 225X225
#include "CT4.h";        //image compte tours

// Objet C++ qui permettra d'accéder aux fonctions du boitier
TTGOClass *ttgo = nullptr;
TFT_eSPI *tft = nullptr;   // Essayer:  TFT_eSPI * tft = ttgo->tft; ou TFT_eSPI *tft =  nullptr; (trouvé dans l'exemple de la gestion de l'alim
BMA *sensor;               // Sensor Accelerometre voir exemple "BasicUnit/BMA423_Feature"

// Gesion de l'Accelerometre pour la mise en veille/réveil par 2 tapes sur la montre
bool irqaccel = false;


// Gestion de l energie
AXP20X_Class *power;
char buf[128];
bool irq = false;

// Gestion de l'EEPROM pour stockage persistant de la luminosité
#include <EEPROM.h> //Gestion EEPROM

// VIBREUR HAPTIQUE
#define DRV2605_ADDRESS             0x5A
static Adafruit_DRV2605 *drv;

int tempo = 400 ; //Taux de rafraichissement d'une page en ms Surtout utile en simulation car les datas changent trop
int temps = 0;    // Stockage de la valeur de millis()
volatile byte allume = 1 ;          // Flag allume ou pas
volatile byte ChoixPage = 1;        // N° de la page à afficher
volatile byte ToucheDirection = 1;  // Sens selon l'endroit de l'ecran qui est touche
volatile byte NbMaxPage = 8;        //nb de pages affichables
byte Brightness = 128;              // Luminosité écran 0-255

//String  HEURE = "12:50";
float SOG = 0;
float SOGold = 0;    // sert pour effacer l ancien affichage
char  TTG[5] = {0};
char  ETA[5] = {0};
int   BTW = 0;
float DTW = 0;
int   CAP = 0;

int   TWA = 0;
int   AWA = 0;
float AWS = 0;
float TWS = 0;
int   GWD = 0;

int   SOC = 0;
float VOLTAGE = 0;
float AMPERAGE = 0;

int   TECHAP = 0;
int   THUILE = 0;
int   RPM    = 0;

float PROF = 0;

int CapDessin = 0;    // valeur du CAP utilisé pour le dessin
String  AMUREA = "B"; // Amure vent apparent B ou T
String  AMURET = "B"; // Amure vent réel


// Couleurs graduations et points cardinaux
int COLGRAD10 = TFT_RED;
int COLGRAD30 = TFT_RED;
int COLPCC = TFT_RED;
int COLPCF = TFT_RED;
int COLGRADROND = TFT_RED;

// diamètres de la rosace
int r3 = 85;
int r2 = 90;
int r1 = 110; // diametre exterieur

// Parametres fleches vent
int RI = 40; // Rayon exterieur
int RE = 85; // Rayon interieur
int EP = 11; // Epaisseur en degres divisé par 2
float EPF = EP * 0.0174533; // Conversion en radian de l'EPaisseur d'une Fleche

// Coordonnées X, Y, Z de chaque triangle dessinant une flèche-aiguille de vent
// flèche vent apparent
int X1AWA = 0;
int Y1AWA = 0;
int X2AWA = 0;
int Y2AWA = 0;
int X3AWA = 0;
int Y3AWA = 0;
int XLAWA = 0;
int YLAWA = 0;
// flèche vent réel
int X1TWA = 0;
int Y1TWA = 0;
int X2TWA = 0;
int Y2TWA = 0;
int X3TWA = 0;
int Y3TWA = 0;
int XLTWA = 0;
int YLTWA = 0;


uint8_t hh, mm, ss, mmonth, dday; // // variables H, M, S
uint16_t yyear; // Annee

// Definition des polices de caractères
#include "Free_Fonts.h" // LE FICHIER DOIT ETRE DANS LE MEME REPERTOIRE QUE LE SKETCH

// variables pour la récup des données
const byte numChars = 100; //80 à l'origine
char receivedChars[numChars];
char tempChars[numChars];        // TABLEAU TEMPORAIRE UTIISE POUR LE PARSING
// VARIABLE POUR LE PARSING
boolean newData = false;


// BLUETOOTH
// https://github.com/espressif/arduino-esp32/blob/master/libraries/BluetoothSerial/examples/SerialToSerialBTM/SerialToSerialBTM.ino
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;
String MACadd = "AA:BB:CC:11:22:33"; // mettre l'adresse du HC-05 séparateurs ":"  PAS SUR QUE CETTE LIGNE SERVE
uint8_t address[6]  = {0x98, 0xD3, 0x61, 0xF6, 0x5B, 0x84}; // mettre l'adresse du HC-05 avec séparateurs  "," au lieu de ":"
String name = "ALBA2BT";  // nom du module émetteur Bluetooth HC-05 branché sur l'Arduino Mega auquel la montre se connecte
const char *pin = "FXVT";       // mettre le code pin du HC-05
bool connected;

// ====================================================================== DESSIN DU LOGO BLUETOOTH
void LogoBT() {
  Serial.println ("Logo BT");
  int Xlogo = 180;
  int Ylogo = 6;
  int BTColor = TFT_BLUE;

  if (SerialBT.available() == 0) {  // si il n'y a rien dans le buffer de reception bluetooth, alors on affiche le logo BE en rouge
    BTColor = TFT_RED ;
  }
  else {
    BTColor = TFT_BLUE;             // sinon en Bleu
  }

  ttgo->tft->fillEllipse(Xlogo + 9, Ylogo + 12, 9, 12, BTColor);
  ttgo->tft->drawLine(Xlogo + 9, Ylogo + 3, Xlogo + 9, Ylogo + 22, TFT_WHITE);
  ttgo->tft->drawLine(Xlogo + 9, Ylogo + 3, Xlogo + 14, Ylogo + 7, TFT_WHITE);
  ttgo->tft->drawLine(Xlogo + 14, Ylogo + 7, Xlogo + 5, Ylogo + 14, TFT_WHITE);
  ttgo->tft->drawLine(Xlogo + 9, Ylogo + 22, Xlogo + 14, Ylogo + 17, TFT_WHITE);
  ttgo->tft->drawLine(Xlogo + 14, Ylogo + 17, Xlogo + 5, Ylogo + 7, TFT_WHITE);
}

// ====================================================================== VIBRATION
void QuickBuzz() { //Vibration moteur VOIR LES EXEMPLES DE LA TTGO V2
  Serial.print ("BUZZZ...");
  // définition de l'effet
  drv->setWaveform(0, 75);  // joue l'effet
  drv->setWaveform(1, 0);       // fin de l'effet
  // play the effect!
  drv->go();
  Serial.print ("FIN BUZ...");
}


// ====================================================================== GESTION DE L'ACCELEROMETRE POUR LA MISE EN VEILLE PAR DOUBLE TAPE SUR L'ECRAN
void accelerometre() { //https://foxpost.xyz/pages/ttgo_watch/

  if (irqaccel) {
    irqaccel = 0;
    bool  rlst;
    do {
      // Read the BMA423 interrupt status,
      // need to wait for it to return to true before continuing
      rlst =  sensor->readInterrupt();
    } while (!rlst);

    // Double-click interrupt
    if (sensor->isDoubleClick()) {
      if (allume == 1) {          // La montre est allumée alors on la met en veille
        ttgo->closeBL();          // stoppe le retroeclairage , economise 90mA
        ttgo->displaySleep();      // affichage en veille economise 8-10mA
        allume = 0;
      }
      else {                       // La montre est en veille alors on remet l affichage en route
        ttgo->openBL();
        ttgo->displayWakeup();
        allume = 1;
      }
    }
  }

  Serial.print(power->getBattDischargeCurrent());   Serial.println(" mA"); // pour debug, affichage de la conso sur le moniteur série pour voir la variation de conso écran eteint.

}

// ====================================================================== GESTION ECRAN TACTILE
void ToucheEcran() {
  int16_t x, y;
  ToucheDirection = 0;
  if (ttgo->getTouch(x, y)) {
    QuickBuzz(); // si on touche l'écran, on fait vibrer.
    // l'écrans est partagés en 9 cases, 3 lignes de 3 cases,
    while (ttgo->getTouch(x, y)) {} // On attend jusqu'à ce qu'on arretede toucher l'écran, sinon rebond
    if (x < 80   && y < 80)                       {
      ToucheDirection = 1;
    }
    if (x >= 80  && x < 160   && y < 80)            {
      ToucheDirection = 2;
    }
    if (x >= 160 && y < 80)                       {
      ToucheDirection = 3;
    }
    if (x < 80   && y >= 80   && y < 160)           {
      ToucheDirection = 4;
    }
    if (x >= 80  && x < 200   && y >= 80  && y < 160) {
      ToucheDirection = 5;
    }
    if ( x >= 160   && y > 80 && y <= 160)          {
      ToucheDirection = 6;
    }
    if (x < 80   && y >= 160)                     {
      ToucheDirection = 7;
    }
    if (x >= 80  && x < 160   && y >= 160)          {
      ToucheDirection = 8;
    }
    if (x >= 160 && y >= 160)                     {
      ToucheDirection = 9;
    }
    Serial.print ("ToucheEcran"); Serial.println (ToucheDirection);

    if (ToucheDirection == 4) {  // On passe à l'écran précédent à gauche
      --ChoixPage;
    }
    if (ToucheDirection == 6) { // On passe à l'écran suivant à droite
      ++ChoixPage;
    }
  }

  // On en profite pour tester si le bouton a été pressé pour la gestion de la mise en veille par le bouton
  if (irq) {
    irq = false;
    ttgo->power->readIRQ();
    if (ttgo->power->isPEKShortPressIRQ()) {
      // Clean power chip irq status
      ttgo->power->clearIRQ();

      // Set  touchscreen sleep
      ttgo->displaySleep();

      ttgo->powerOff();

      //Set all channel power off
      ttgo->power->setPowerOutPut(AXP202_LDO3, false);
      ttgo->power->setPowerOutPut(AXP202_LDO4, false);
      ttgo->power->setPowerOutPut(AXP202_LDO2, false);
      ttgo->power->setPowerOutPut(AXP202_EXTEN, false);
      ttgo->power->setPowerOutPut(AXP202_DCDC2, false);

      //  A DECOMMENTER SI Réveil en touchant l'écran
      // esp_sleep_enable_ext1_wakeup(GPIO_SEL_38, ESP_EXT1_WAKEUP_ALL_LOW);

      // A DECOMMENTER SI Réveil en appuis bref sur le bouton
      esp_sleep_enable_ext1_wakeup(GPIO_SEL_35, ESP_EXT1_WAKEUP_ALL_LOW);

      esp_deep_sleep_start();
    }
    ttgo->power->clearIRQ();
  }


  // On en profite pour tester si on a double tapé la montres pour la mise en veille de l'écran
  accelerometre ();

}

// ======================================================================
void setup() {
  Serial.begin(115200);
  Serial.print("TTGO-SailWatch-");
  Serial.println(ver);

  // Récupère l'objet ttgo
  ttgo = TTGOClass::getWatch();
  ttgo->begin();

  // initialize EEPROM avec une taille predefinie. On stocke dans l'eeprom la valeur de la luminosité
  EEPROM.begin(10);
  int BrightnessOrigine = EEPROM.read(0);  // Récupère la valeur de la luminosité stockée en EEPROM
  Serial.println (BrightnessOrigine);
  ttgo->openBL();                          // Allume l'écran
  ttgo->setBrightness(BrightnessOrigine);  // applique la luminosité  d'origine memorisee
  ttgo->tft->setRotation(0);               // Tourne l'ecran de 180°, bouton mieux placé

  // ===================================================  PAGE D'ACCUEIL AFFICHE L'IMAGE DU BATEAU LE NOM DU PROGRAMME ET LA VERSION
  ttgo->tft->setTextColor( TFT_BLUE, TFT_BLACK);
  ttgo->tft->drawXBitmap(70, 1, SL31introxbm, SL31intro_width, SL31intro_height, TFT_WHITE, TFT_BLACK ); // affichage de l'image NB du Bateau
  ttgo->tft->setTextSize(1);
  ttgo->tft->setFreeFont(FSSB18);
  ttgo->tft->setCursor(70, 190);
  ttgo->tft->println("TTGO");
  ttgo->tft->setCursor(30, 220);
  ttgo->tft->println("Sail Watch");
  ttgo->tft->setTextColor(TFT_WHITE, TFT_BLACK);
  ttgo->tft->setFreeFont(FSSB9);
  ttgo->tft->setCursor(95, 239);
  ttgo->tft->println(ver);

  // VIBREUR HAPTIQUE  Voir exemple TTGO Twatch librarie/BasicUnit/TWatchV2Special/DRV2605Basic
  drv = ttgo->drv;
  ttgo->enableDrv2650(true);
  drv->selectLibrary(1);
  drv->setMode(DRV2605_MODE_INTTRIG);

  // GETION DE L ACCELEROMETRE
  sensor = ttgo->bma;  //plus facile a ecrire
  // Accel parameter structure
  Acfg cfg;
  cfg.odr = BMA4_OUTPUT_DATA_RATE_100HZ; // frequence
  cfg.range = BMA4_ACCEL_RANGE_2G;       // Gamme d acceleration
  cfg.bandwidth = BMA4_ACCEL_NORMAL_AVG4; // MOYENNE
  cfg.perf_mode = BMA4_CONTINUOUS_MODE;   // FILTRAGE
  sensor->accelConfig(cfg);               // Configure l accelerometre BMA423
  sensor->enableAccel();

  pinMode(BMA423_INT1, INPUT);
  attachInterrupt(BMA423_INT1, [] {
    irqaccel = 1;                              // Set interrupt to set irq value to 1
  }, RISING);                             //It must be a rising edge


  sensor->enableFeature(BMA423_WAKEUP, true);   // Enable BMA423 isDoubleClick feature
  sensor->enableWakeupInterrupt();              // It corresponds to isDoubleClick interrupt


  power = ttgo->power;

  Serial.println("Activation de l'AXP202"); // Gestion de l'energie
  // ADC monitoring must be enabled to use the AXP202 monitoring function
  power->adc1Enable(
    AXP202_VBUS_VOL_ADC1 |
    AXP202_VBUS_CUR_ADC1 |
    AXP202_BATT_CUR_ADC1 |
    AXP202_BATT_VOL_ADC1,
    true);

  // ADC monitoring issu du sketch WakeupFormTouchscreen
  pinMode(AXP202_INT, INPUT_PULLUP);
  attachInterrupt(AXP202_INT, [] {
    irq = true;
  }, FALLING);
  //!Clear IRQ unprocessed  first
  ttgo->power->enableIRQ(AXP202_PEK_SHORTPRESS_IRQ, true);
  ttgo->power->clearIRQ();

  pinMode(TOUCH_INT, INPUT);  // voir si on peut enlever

  // ==================   BLUETOOTH  (code de l'exemple pour TTGO T-Watch SerialToSerialBTM

  SerialBT.begin("TTGO-MONTRE", true); //Nom du Bluetooth de la montre
  SerialBT.setPin(pin);
  Serial.println("The device started, now you can pair it with bluetooth!");

  // Boucle de tentative de connexion BlueTooth
  while (!SerialBT.connected(10)) {
    Serial.println("avant Connexion");
    connected = SerialBT.connect(address);
    Serial.println("apres Connexion");
    if (connected) {
      Serial.println("Connexion OK");
      LogoBT();
      ttgo->tft->setTextColor(TFT_GREEN, TFT_BLACK);
      ttgo->tft->setTextSize(1);
      ttgo->tft->setCursor(1, 15);
      ttgo->tft->println("Connexion OK");
      delay (200);
      break;
    }

    ToucheEcran();
    if (ToucheDirection == 5) {  // Si pendant la tentative de connexion au BT on touche le centre de l'écran (longtemps), alors on sort de la boucle de tentative de connexion
      break;
    }

    Serial.println("Failed to connect. Make sure remote device is available and in range, then restart app.");
    ttgo->tft->setTextColor(TFT_RED, TFT_BLACK);
    ttgo->tft->setTextSize(1);
    ttgo->tft->setCursor(1, 15);
    ttgo->tft->println("Pas de Bluetooth");

  }
  /*
    // disconnect() may take upto 10 secs max
    if (SerialBT.disconnect()) {
    Serial.println("Disconnected Succesfully!");
    }
    // this would reconnect to the name(will use address, if resolved) or address used with connect(name/address).
    SerialBT.connect();
    if (connected) {
    Serial.println("Connected Succesfully! 2");
    LogoBT();
    }
  */
}

// ====================================================================== BOUCLE PRINCIPALE
void loop() {
  Serial.println ("Debut loop");
  ToucheEcran();

  if (ChoixPage > NbMaxPage) {  // Si on est arrivé au dernier écran, on boucle sur l'écran 1
    ChoixPage = 1;
  }
  if (ChoixPage < 1) {          // Si on est arrivé au premier écran, on boucle sur le dernier écran
    ChoixPage = NbMaxPage;
  }
  Serial.print ("ChoixPage"); Serial.println (ChoixPage);
  switch (ChoixPage) { // Selon le choix de page
    case 1: // cas 1
      PageNavigtion ();
      break;
    case 2: // cas 2
      Vent();
      break;
    case 3: // cas 3
      DisplayPageBatterie();
      break;
    case 4: // cas 4
      DisplayPageMoteur();
      break;
    case 5: // cas 5
      MOTEUR();
      break;
    case 6: // cas 6
      PageMouillage();
      break;
    case 7: // cas 7
      SetBrightness();
      break;
    case 8: // cas 8
      photoSB();
      break;
  }
}

// ====================================================================== PAGE NAVIGATION
void PageNavigtion () {
  ttgo->tft->setTextFont(1);
  ttgo->tft->fillScreen(TFT_BLACK);
  ttgo->tft->drawRoundRect(0, 0, 240, 240, 5, TFT_RED);
  ttgo->tft->fillTriangle ( 115, 0, 125, 0,  120, 10, TFT_RED );
  // DESSINE COURONNE
  ttgo->tft->fillCircle(120, 120, r1, TFT_RED);
  ttgo->tft->fillCircle(120, 120, r1 - 20, TFT_BLACK);

  // Dessine la forme du bateau
  ttgo->tft->drawXBitmap(92, 40, SL31160NBxbm, SL31160NB_width, SL31160NB_height, TFT_BLACK, TFT_WHITE);

  // AFFICHAGE LABELs
  ttgo->tft->setTextColor(TFT_RED, TFT_BLACK);
  ttgo->tft->setTextSize(1);
  ttgo->tft->setFreeFont(FSSB9);
  ttgo->tft->setCursor(3, 38);
  ttgo->tft->println("ETA");
  ttgo->tft->setCursor(195, 38);
  ttgo->tft->println("BTW");
  ttgo->tft->setCursor(3, 215);
  ttgo->tft->println("TTG");
  ttgo->tft->setCursor(195, 215);
  ttgo->tft->println("DTW");


  while (ChoixPage == 1) {
    ToucheEcran();      // l'écran a-t-il été touché ?
    GetDatas();         // Récupère les données reçues en BT

    // Efface l'ancienne rosace
    COLGRAD10 = TFT_RED;
    COLGRAD30 = TFT_RED;
    COLPCC = TFT_RED;
    COLPCF = TFT_RED;
    COLGRADROND = TFT_RED;
    DrawCompasMobile ();

    // Dessine la nouvelle rosace
    CapDessin = CAP;
    COLGRAD10 = TFT_WHITE;
    COLGRAD30 = TFT_YELLOW;
    COLPCC = TFT_WHITE;
    COLPCF = TFT_RED;
    COLGRADROND = TFT_YELLOW;
    DrawCompasMobile ();

    // Dessine la forme du bateau
    ttgo->tft->drawXBitmap(92, 40, SL31160NBxbm, SL31160NB_width, SL31160NB_height, TFT_BLACK, TFT_WHITE);

    // Dessine la fleche du vent apparent
    FlecheVentAppNav ();

    // DESSINE RETICULE
    ttgo->tft->drawFastVLine(120, 22 , 195, TFT_WHITE) ;
    ttgo->tft->drawFastHLine(22, 120, 195, TFT_WHITE) ;

    // AFFICHAGE SOG
    ttgo->tft->setTextColor(TFT_BLACK, TFT_WHITE);
    ttgo->tft->setTextSize(2);
    //ttgo->tft->setFreeFont(FSS12);
    ttgo->tft->setCursor(104, 105);
    ttgo->tft->print("SOG");
    ttgo->tft->setCursor(104, 120);
    ttgo->tft->setTextSize(1);
    ttgo->tft->setFreeFont(FSSB18);
    ttgo->tft->setTextColor(TFT_BLACK, TFT_BLACK);
    ttgo->tft->setCursor(95, 150);
    ttgo->tft->print(SOGold, 1);
    ttgo->tft->setTextColor(TFT_WHITE , TFT_BLACK);
    ttgo->tft->setCursor(95, 150);
    ttgo->tft->print(SOG, 1);
    SOGold = SOG;

    // Affiche le CAP
    ttgo->tft->setTextFont(1);
    ttgo->tft->fillRect(95, 11, 55 , 24 , TFT_RED);
    ttgo->tft->setTextColor( TFT_WHITE , TFT_RED);
    ttgo->tft->setTextSize(3);
    if (CAP > 99) {                       // Centrage de l'affichage
      ttgo->tft->setCursor(95, 11);
    }
    else if (CAP > 9 ) {
      ttgo->tft->setCursor(103, 11);
    }
    else  {
      ttgo->tft->setCursor(113, 11);
    }
    ttgo->tft->println(CAP);            // affichage du CAP

    //  VALEURS NUMERIQUES NAVIGATION
    ttgo->tft->setTextFont(1);
    ttgo->tft->setFreeFont(FSS12);
    ttgo->tft->setTextColor(TFT_RED, TFT_BLACK);
    ttgo->tft->setTextSize(1);

    ttgo->tft->fillRect(5, 4, 63, 19, TFT_BLACK);
    ttgo->tft->setCursor(5, 20);
    ttgo->tft->println(ETA);

    ttgo->tft->fillRect(5, 220, 70, 19, TFT_BLACK);
    ttgo->tft->setCursor(5, 236);
    ttgo->tft->println(TTG);

    ttgo->tft->fillRect(195, 4, 38, 19, TFT_BLACK);
    ttgo->tft->setCursor(195, 20 );
    ttgo->tft->setTextColor(TFT_RED, TFT_BLACK);
    ttgo->tft->println(BTW);

    ttgo->tft->fillRect(180, 220, 53, 19, TFT_BLACK);
    ttgo->tft->setCursor(180, 236);
    ttgo->tft->setTextColor(TFT_RED, TFT_BLACK);
    ttgo->tft->println(DTW, 1);

    // Temporisation
    temps = millis();
    while (millis() < (temps + tempo)) {  }
  }
}

// ====================================================================== DESSINE COMPAS MOBILE
void DrawCompasMobile () {

  // DESSNE GRADUATIONS 10 DEGRES
  for (int a = CapDessin + 0 ; a < CapDessin + 360 ; a = a + 10) {
    float ar = a * 0.0174533;
    ttgo->tft->drawLine ((120 + (sin (ar) * r1)), (120 + (cos (ar) * r1)), (120 + (sin (ar) * r2)), (120 + (cos (ar) * r2)), COLGRAD10);
  }

  // DESSNE GRADUATIONS 30 DEGRES
  for (int a = CapDessin + 0 ; a < CapDessin + 360 ; a = a + 30) {
    float ar = a * 0.0174533;
    ttgo->tft->fillCircle((120 + (sin (ar) * (r1 - 10))), (120 + (cos (ar) * (r1 - 10))), 2, COLGRADROND);
  }

  // AFFICHAGE 4 POINTS CARDINAUX N,S,E,W
  int XPC = 0;
  int YPC = 0;
  ttgo->tft->setTextFont(1);
  ttgo->tft->setTextColor(COLPCC, COLPCF);
  ttgo->tft->setTextSize(2);
  XPC =  120 + (sin ((360 - CapDessin + 0) * 0.0174533) * (r1 - 10)) - 7;
  YPC = 120 - (cos ((360 - CapDessin + 0) * 0.0174533) * (r1 - 10)) - 7;;
  ttgo->tft->setCursor(XPC, YPC);
  ttgo->tft->println("N");
  XPC =  120 + (sin ((360 - CapDessin + 90) * 0.0174533) * (r1 - 10)) - 7;
  YPC = 120 - (cos ((360 - CapDessin + 90) * 0.0174533) * (r1 - 10)) - 7;;
  ttgo->tft->setCursor(XPC, YPC);
  ttgo->tft->println("E");
  XPC =  120 + (sin ((360 - CapDessin + 180) * 0.0174533) * (r1 - 10)) - 7;
  YPC = 120 - (cos ((360 - CapDessin + 180) * 0.0174533) * (r1 - 10)) - 7;;
  ttgo->tft->setCursor(XPC, YPC);
  ttgo->tft->println("S");
  XPC =  120 + (sin ((360 - CapDessin + 270) * 0.0174533) * (r1 - 10)) - 7;
  YPC = 120 - (cos ((360 - CapDessin + 270) * 0.0174533) * (r1 - 10)) - 7;;
  ttgo->tft->setCursor(XPC, YPC);
  ttgo->tft->println("W");

}

// ====================================================================== FLECHES VENT APPARENT NAVIGATION
void FlecheVentAppNav () {
  // Parametres fleches vent
  int r1 = 110;
  int RI = 70; // Rayon exterieur
  int RE = 90; // Rayon interieur
  int EP = 4; // Epaisseur en degres divisé par 2
  float EPF = EP * 0.0174533; // Conversion de l'épaisseur d'une flèche de degrès à radian

  // EFFACE LES ANCIENNES FLECHES
  ttgo->tft->fillTriangle ( X1AWA, Y1AWA, X2AWA, Y2AWA, X3AWA, Y3AWA,  TFT_BLACK);

  // DESSINE FLECHE VENT APPARENT
  float AWAF = (AWA - 90) * 0.0174533;
  X1AWA = 120 + cos (AWAF) * RI;;
  Y1AWA = 120 + sin (AWAF) * RI ;

  X2AWA = 120 + cos (AWAF - EPF) * RE;
  Y2AWA = 120 + sin (AWAF - EPF) * RE;
  X3AWA = 120 + cos (AWAF + EPF) * RE;
  Y3AWA = 120 + sin (AWAF + EPF) * RE;

  ttgo->tft->fillTriangle ( X1AWA, Y1AWA, X2AWA, Y2AWA, X3AWA, Y3AWA,  TFT_BLUE);
}

// ====================================================================== PAGE VENT
void     Vent() {
  ttgo->tft->setTextFont(1);
  // Parametres fleches vent
  int r1 = 110;
  int RI = 40; // Rayon eXterieur
  int RE = 85; // Rayon interieur
  int EP = 11; // Epaisseur en degres divisé par 2
  float EPF = EP * 0.0174533; // Conversion de l'épaisseur d'une flèche de degrès à radian

  // Valeurs affichées du AWA et TWA de 0 à 180 babord ou tribord et pas de 0 à 360
  int AWAval = 0;
  int TWAval = 0;

  int X1AWA = 0;
  int Y1AWA = 0;
  int X2AWA = 0;
  int Y2AWA = 0;
  int X3AWA = 0;
  int Y3AWA = 0;
  int XLAWA = 0;
  int YLAWA = 0;

  int X1TWA = 0;
  int Y1TWA = 0;
  int X2TWA = 0;
  int Y2TWA = 0;
  int X3TWA = 0;
  int Y3TWA = 0;
  int XLTWA = 0;
  int YLTWA = 0;

  // Dessine cadre
  ttgo->tft->fillScreen(TFT_BLACK);
  ttgo->tft->drawRoundRect(0, 0, 240, 240, 5, TFT_RED);
  // DESSINE COURONNE
  ttgo->tft->fillCircle(120, 120, r1, TFT_DARKGREY);

  // DESSINE GRADUATION 10 DEGRES
  for (int a = 0 ; a < 360 ; a = a + 10) {
    float ar = a * 0.0174533;
    ttgo->tft->drawLine (120, 120, (120 + (sin (ar) * r1)), (120 + (cos (ar) * r1)), TFT_WHITE);
  }
  ttgo->tft->fillCircle(120, 120, r1 - 10, TFT_BLACK);

  // DESSNE GRADUATION 30 DEGRES
  for (int a = 0 ; a < 360 ; a = a + 30) {
    float ar = a * 0.0174533;
    ttgo->tft->drawLine (120, 120, 120 + (sin (ar) * r1), 120 + (cos (ar) * r1), TFT_YELLOW);
  }
  ttgo->tft->fillCircle(120, 120, 95, TFT_BLACK);

  // AFFICHAGE LABELs
  ttgo->tft->setTextColor(TFT_RED, TFT_BLACK);
  ttgo->tft->setTextSize(1);
  ttgo->tft->setFreeFont(FSSB9);
  ttgo->tft->setCursor(3, 38);
  ttgo->tft->println("AWS");
  ttgo->tft->setCursor(195, 38);
  ttgo->tft->println("AWA");
  ttgo->tft->setCursor(3, 215);
  ttgo->tft->println("TWS");
  ttgo->tft->setCursor(195, 215);
  ttgo->tft->println("TWA");

  // DESSINE RETICULE
  ttgo->tft->drawFastVLine(120, 22 , 195, TFT_WHITE) ;
  ttgo->tft->drawFastHLine(40, 120, 150, TFT_WHITE) ;

  // DESSINE 0,90,180,270 DEGRES
  ttgo->tft->setTextFont(1);
  ttgo->tft->setTextSize(2);
  ttgo->tft->setTextColor(TFT_WHITE, TFT_DARKGREY);
  // ttgo->tft->setFreeFont(FSS12);
  ttgo->tft->setCursor(115, 6);
  ttgo->tft->print("0");
  ttgo->tft->setCursor(214, 112);
  ttgo->tft->print("90");
  ttgo->tft->setCursor(105, 220);
  ttgo->tft->print("180");
  ttgo->tft->setCursor(2, 112);
  ttgo->tft->print("270");

  while (ChoixPage == 2) {      // Tant que le choix de page = 2 = vent
    ToucheEcran();  // A-t-on touché l'écran ?
    GetDatas();     // récupération des données

    ttgo->tft->setTextFont(1);

    // conversion si les AWA et TWA dépassent les 180°
    if (AWA > 180) {
      AWAval = 360 - AWA;
      AMUREA = "B";
    }
    else {
      AMUREA = "T";
      AWAval = AWA;
    }
    if (TWA > 180) {
      TWAval = 360 - TWA;
      AMURET = "B";
    }
    else {
      AMURET = "T";
      TWAval = TWA;
    }

    // EFFACE LES ANCIENNES FLECHES
    ttgo->tft->fillTriangle ( X1AWA, Y1AWA, X2AWA, Y2AWA, X3AWA, Y3AWA,  TFT_BLACK);
    ttgo->tft->fillTriangle ( X1TWA, Y1TWA, X2TWA, Y2TWA, X3TWA, Y3TWA,  TFT_BLACK);

    // DESSINE RETICULE
    ttgo->tft->drawFastVLine(120, 22 , 195, TFT_WHITE) ;
    ttgo->tft->drawFastHLine(38, 120, 175, TFT_WHITE) ;

    // DESSINE FLECHE VENT APPARENT
    float AWAF = (AWA - 90) * 0.0174533;
    X1AWA = 120 ;
    Y1AWA = 120 ;

    X2AWA = 120 + cos (AWAF - EPF) * RE;
    Y2AWA = 120 + sin (AWAF - EPF) * RE;
    X3AWA = 120 + cos (AWAF + EPF) * RE;
    Y3AWA = 120 + sin (AWAF + EPF) * RE;

    ttgo->tft->fillTriangle ( X1AWA, Y1AWA, X2AWA, Y2AWA, X3AWA, Y3AWA,  TFT_BLUE);

    ttgo->tft->setTextColor(TFT_WHITE, TFT_BLUE);
    ttgo->tft->setTextSize(2);
    XLAWA = (120 + cos (AWAF) * (RE - 13)) - 7;
    YLAWA = (120 + sin (AWAF) * (RE - 13)) - 7;
    ttgo->tft->setCursor(XLAWA, YLAWA);
    ttgo->tft->print("A");

    // DESSINE FLECHE VENT REEL
    float TWAF = (TWA - 90) * 0.0174533;
    X1TWA = 120 + cos (TWAF) * RI;
    Y1TWA = 120 + sin (TWAF) * RI;
    X2TWA = 120 + cos (TWAF - EPF) * RE;
    Y2TWA = 120 + sin (TWAF - EPF) * RE;
    X3TWA = 120 + cos (TWAF + EPF) * RE;
    Y3TWA = 120 + sin (TWAF + EPF) * RE;
    ttgo->tft->fillTriangle ( X1TWA, Y1TWA, X2TWA, Y2TWA, X3TWA, Y3TWA,  TFT_ORANGE);

    ttgo->tft->setTextColor(TFT_BLACK, TFT_ORANGE);
    ttgo->tft->setTextSize(2);
    XLTWA = (120 + cos (TWAF) * (RE - 13)) - 5;
    YLTWA = (120 + sin (TWAF) * (RE - 13)) - 5;
    ttgo->tft->setCursor(XLTWA, YLTWA);
    ttgo->tft->print("T");

    // AFFICHAGE GWD
    ttgo->tft->setTextColor(TFT_BLACK, TFT_WHITE);
    ttgo->tft->setTextSize(2);
    ttgo->tft->setCursor(104, 30);
    ttgo->tft->print("GWD");
    if (GWD < 100) {
      ttgo->tft->setCursor(116, 45);
    }
    else {
      ttgo->tft->setCursor(104, 45);
    }
    ttgo->tft->print(GWD);

    // AFFICHAGE SOG
    ttgo->tft->setTextColor(TFT_BLACK, TFT_WHITE);
    ttgo->tft->setTextSize(2);
    //ttgo->tft->setFreeFont(FSS12);
    ttgo->tft->setCursor(104, 105);
    ttgo->tft->print("SOG");
    ttgo->tft->setCursor(104, 120);
    ttgo->tft->setTextSize(1);
    ttgo->tft->setFreeFont(FSSB18);
    ttgo->tft->setTextColor(TFT_BLACK, TFT_BLACK);
    ttgo->tft->setCursor(95, 150);
    ttgo->tft->print(SOGold, 1);
    ttgo->tft->setTextColor(TFT_WHITE , TFT_BLACK);
    ttgo->tft->setCursor(95, 150);
    ttgo->tft->print(SOG, 1);
    SOGold = SOG;

    //POLICE VALEURS NUMERIQUES
    ttgo->tft->setTextFont(1);
    ttgo->tft->setFreeFont(FSS12);
    ttgo->tft->setTextColor(TFT_RED, TFT_BLACK);
    ttgo->tft->setTextSize(1);

    ttgo->tft->fillRect(5, 4, 46, 19, TFT_BLACK);
    ttgo->tft->setCursor(5, 20);
    ttgo->tft->println(AWS, 1);

    ttgo->tft->fillRect(5, 220, 46, 19, TFT_BLACK);
    ttgo->tft->setCursor(5, 236);
    ttgo->tft->println(TWS, 1);

    ttgo->tft->fillRect(195, 4, 38, 19, TFT_BLACK);
    ttgo->tft->setCursor(195, 20 );
    if (AMUREA == "B") {
      ttgo->tft->setTextColor(TFT_RED, TFT_BLACK);
    }
    else {
      ttgo->tft->setTextColor(TFT_GREEN, TFT_BLACK);
    }
    ttgo->tft->println(AWAval);

    ttgo->tft->fillRect(195, 220, 38, 19, TFT_BLACK);
    ttgo->tft->setCursor(195, 236);
    if (AMUREA == "B") {
      ttgo->tft->setTextColor(TFT_RED, TFT_BLACK);
    }
    else {
      ttgo->tft->setTextColor(TFT_GREEN, TFT_BLACK);
    }
    ttgo->tft->println(TWAval);

    //delay (tempo);

    // Temporisation
    temps = millis();
    while (millis() < (temps + tempo)) {  }

  }
}


// ====================================================================== PAGE BATTERIE
void DisplayPageBatterie () {
  Serial.println ("DisplayPageBatterie");
  Cadres();
  DisplayLabelUnitePageBatterie();
  while (ChoixPage == 3) {
    DisplayEntete();
    ToucheEcran();
    GetDatas();
    DisplayDataBatterie();
  }
}

// ====================================================================== PAGE MOTEUR
void DisplayPageMoteur () {
  Serial.println ("DisplayPageMoteur");
  Cadres();
  DisplayLabelMoteur();
  while (ChoixPage == 4) {
    DisplayEntete();
    ToucheEcran();
    GetDatas();
    DisplayDataMoteur();
  }
}

// ====================================================================== MOTEUR AVEC IMAGE COMPTE TOURS
void MOTEUR () {

  int LA = 70;  //longueur aiguille
  int EPA = 7; //largeur base aiguille
  int ACT = 0; // Angle aiguille Compte tours

  int X1ACT = 0;
  int Y1ACT = 0;
  int X2ACT = 0;
  int Y2ACT = 0;
  int X3ACT = 0;
  int Y3ACT = 0;
  int XLACT = 0;
  int YLACT = 0;

  ttgo->tft->fillScreen(TFT_BLACK);                 // efface l'écran
  ttgo->tft->drawRoundRect(0, 0, 240, 240, 4, TFT_RED );                // efface l'écran
  ttgo->tft->setSwapBytes(true);

  while (ChoixPage == 5) {                          // On reste là tant que la page selectionnée est la N°7
    ToucheEcran();                                  // l'écran a-t-il été touché ?
    GetDatas();     // récupération des données
    ttgo->tft->pushImage(5, 39, 230, 200, CT4);          //affiche la photo de compte tours pushImage(x,y,largeur,hauteur,image)

    // DESSINE AIGUILLE COMPTE TOUR
    ACT = map (RPM, 0, 4000, 150, 390);
    float ACTF = (ACT ) * 0.0174533;
    X1ACT = 120 + cos (ACTF ) * LA;   // Extremité Aiguille
    Y1ACT = 140 + sin (ACTF ) * LA;

    X2ACT = 120 + cos (ACTF - 1.5708 ) * EPA;     // 1.5708 rad = 90 degres
    Y2ACT = 140 + sin (ACTF - 1.5708) * EPA;
    X3ACT = 120 + cos (ACTF + 1.5708) * EPA;
    Y3ACT = 140 + sin (ACTF + 1.5708) * EPA;
    ttgo->tft->fillTriangle ( X1ACT, Y1ACT, X2ACT, Y2ACT, X3ACT, Y3ACT,  TFT_RED);
    ttgo->tft->fillCircle(120, 140, 17, TFT_BLACK);

    ttgo->tft->setTextFont(1);
    ttgo->tft->setTextSize(1);
    ttgo->tft->setFreeFont(FSS18);
    //  ttgo->tft->setTextColor(TFT_BLACK, TFT_BLACK);
    //  ttgo->tft->setCursor(85, 195);
    //   ttgo->tft->println(SOGold);
    ttgo->tft->setTextColor(TFT_RED, TFT_BLACK);
    ttgo->tft->setCursor(95, 195);
    ttgo->tft->println(SOG, 1);
    //   SOGold = SOG;

    //  VALEURS NUMERIQUES MOTEUR
    ttgo->tft->fillRect(5, 1, 63, 38, TFT_BLACK);
    ttgo->tft->setCursor(5, 30);
    ttgo->tft->println(THUILE);

    ttgo->tft->fillRect(195, 1, 43, 38, TFT_BLACK);
    ttgo->tft->setCursor(195, 30);
    ttgo->tft->println(TECHAP);


    // Temporisation
    temps = millis();
    while (millis() < (temps + tempo)) {  }
  }
}

// ====================================================================== PAGE MOUILLAGE SOG + PROFONDEUR
void PageMouillage () {
  float SOGold = 0;
  float PROFold = 0;
  int i = 0;
  int nbvalmoy = 15;  // c'est à dire environ toutes les 6-7 secondes
  float SumPROF = 0;
  float moyPROF = 0;
  float moyPROFold = 0;

  ttgo->tft->fillScreen(TFT_WHITE);
  ttgo->tft->fillRoundRect(1, 1, 238, 38, 10, TFT_BLACK);
  ttgo->tft->drawRoundRect(2, 44, 236, 94, 10, TFT_BLACK);
  ttgo->tft->drawRoundRect(2, 140, 236, 86, 10, TFT_BLACK);
  DisplayEntete();
  Stepper();
  Serial.println ("fin cadre");

  // Affichage labels et unités
  ttgo->tft->setTextFont(2);
  ttgo->tft->setTextColor(TFT_BLACK, TFT_WHITE );
  ttgo->tft->setTextSize(2);
  ttgo->tft->setCursor(30, 105);  ttgo->tft->println("SOG       Kts");
  ttgo->tft->setCursor(30, 193);  ttgo->tft->println("DEPTH     m");

  // Boucle
  while (ChoixPage == 6) {
    DisplayEntete();
    ToucheEcran();  // A-t-on touché l'écran ?
    GetDatas();     // récupération des données
    ///Serial.println ("debut PROF");
    // ttgo->tft->setTextFont(0);
    // ttgo->tft->setTextSize(5);
    ttgo->tft->setTextFont(2);
    ttgo->tft->setTextSize(2);
    ttgo->tft->setFreeFont(FSSB18);
    ttgo->tft->setTextColor(TFT_WHITE, TFT_WHITE);              //Effacement de anciennes données
    ttgo->tft->setCursor(50, 100); ttgo->tft->println(SOGold, 1);
    ttgo->tft->setCursor(50, 193); ttgo->tft->println(PROFold, 1);
    ttgo->tft->setTextColor(TFT_BLACK, TFT_WHITE);            //Affichage de nouvelles données
    ttgo->tft->setCursor(50, 100); ttgo->tft->println(SOG, 1);
    ttgo->tft->setCursor(50, 193); ttgo->tft->println(PROF, 1);

    // Calcul de la tendance  de la profondeur et affichage des flèches
    if ( i <  nbvalmoy) {
      SumPROF = SumPROF + PROF;
      ++i;
      //Serial.println (i);
    }
    if ( i == nbvalmoy) {       // si on a atteint le nd de valeurs voulues pour calculer la moyenne
      moyPROF = SumPROF / i ;   // alors on calcule la une profondeur moyenne
      i = 0;
      SumPROF = 0;
      ttgo->tft->fillRect(185, 145, 40, 50, TFT_WHITE); // efface l'ancienne fleche
      Serial.print ("moyPROF "); Serial.println (moyPROF); Serial.print ("moyPROFold "); Serial.println (moyPROFold);

      if (moyPROF < moyPROFold) {  // LE FOND REMONTE, ON AFFICHE UNE FLECHE ROUGE
        //  Serial.println ("ROUGE ");
        ttgo->tft->fillRect(195, 145, 20, 30, TFT_RED);
        ttgo->tft->fillTriangle ( 185, 175, 225, 175, 205, 195, TFT_RED);
        moyPROFold = moyPROF ;
      }
      if (moyPROF > moyPROFold) {   // LE FOND DESCEND,  ON AFFICHE UNE FLECHE VERTE
        //     Serial.println ("VERT ");
        ttgo->tft->fillRect(195, 165, 20, 30, TFT_GREEN);
        ttgo->tft->fillTriangle ( 185, 165, 225, 165, 205, 145, TFT_GREEN);
        moyPROFold = moyPROF ;
      }
    }
    SOGold = SOG ;
    PROFold = PROF;

    // delay (tempo);

    // Temporisation
    temps = millis();
    while (millis() < (temps + tempo)) {  }

  }
}
// ====================================================================== REGLAGE LUMINOSITE
void SetBrightness() {
  int16_t x, y, xOld;

  Serial.println ("reglage luminosité");
  int BrightnessOrigine = EEPROM.read(0);
  //int BrightnessOrigine = 128;
  Brightness = BrightnessOrigine ;
  ttgo->tft->fillScreen(TFT_BLACK);
  ttgo->tft->setTextFont(2);
  ttgo->tft->setTextColor(TFT_WHITE, TFT_BLACK);
  ttgo->tft->setTextSize(2);
  ttgo->tft->setCursor(20, 60);
  ttgo->tft->println("LUMINOSITE:        ");
  AfficheLuminosite ();
  Stepper();

  while (ChoixPage == 7) {
    DisplayEntete();                                      // on affiche la ligne d'état en haut
    if (ttgo->getTouch(x, y)) {                           // si on touche l'écran
      if (x < 80   && y >= 80   && y < 160)           {
        break;                                      // si on touche l'écran à gauche on recule d'une page
      }
      if ( x >= 160   && y > 80 && y <= 160)          {
        break;                                   // si on touche l'écran à droite on avance d'une page

      }
      if ( y > 180 & y < 220) {                      // si en plus on touche dans la zone de la glissière
        if (x != xOld) {                              // et que la valeur sur le curseur est différent de la valeur précédente
          Brightness = map (x, 0, 239, 3, 255);           // alors on calcule la nouvelle valeur de la luminosité, de 3 à 255en fonction de la position du doigt de 0 à 239 sur l'écran
          AfficheLuminosite ();                           // et on affiche la nouvelle position du curseur et on affiche la nouvelle luminosité
        }
        xOld = x;
      }
    }
    // Sauve la luminosité en Eeprom si elle est différente de la luminosité d'origine
    if (Brightness != BrightnessOrigine) {
      EEPROM.write(0, Brightness);
      EEPROM.commit();
    }
  }
}
// Affichage du curseur tactile de la luminosité
void AfficheLuminosite () {
  ttgo->setBrightness(Brightness);       // 0~255
  ttgo->tft->fillRect(170, 60, 70, 60, TFT_BLACK);
  ttgo->tft->setTextColor(TFT_YELLOW, TFT_BLACK);
  ttgo->tft->setTextSize(2);
  ttgo->tft->setCursor(180, 60);
  ttgo->tft->println(Brightness);
  ttgo->tft->fillRoundRect(20, 180, 200, 40, 20, TFT_WHITE);
  ttgo->tft->fillRoundRect(24, 183, (196 * Brightness / 255), 34, 17, TFT_YELLOW);
}

// ====================================================================== AFFICHAGE CADRES
void Cadres() {
  // On utilise la librairie eTFT pour afficher du texte à l'écran
  ttgo->tft->fillScreen(TFT_RED);
  ttgo->tft->fillRoundRect(1, 1, 238, 38, 10, TFT_BLACK);
  ttgo->tft->fillRoundRect(2, 44, 236, 63, 10, TFT_BLACK);
  ttgo->tft->fillRoundRect(2, 110, 236, 63, 10, TFT_BLACK);
  ttgo->tft->fillRoundRect(2, 174, 236, 51, 10, TFT_BLACK);
  DisplayEntete();
  Stepper();
  Serial.println ("fin cadre");
}

// ====================================================================== AFFICHAGE LIBELLES BATTERIE
void DisplayLabelUnitePageBatterie() {
  // EffaceLabelUnite();
  // ttgo->tft->setTextFont(2);
  ttgo->tft->setTextSize(1);
  ttgo->tft->setFreeFont(FSS12);
  ttgo->tft->setTextColor(TFT_GREEN, TFT_BLACK);
  ttgo->tft->setCursor(10, 90);
  ttgo->tft->println(F("SOC                       %"));
  ttgo->tft->setCursor(10, 155);
  ttgo->tft->println(F("VOLT              "));
  ttgo->tft->setCursor(10, 210);
  ttgo->tft->println(F("AMP               "));
}

// ====================================================================== AFFICHAGE LIBELLES MOTEUR
void DisplayLabelMoteur() {
  // EffaceLabelUnite();
  // ttgo->tft->setTextFont(2);
  ttgo->tft->setTextSize(1);
  ttgo->tft->setFreeFont(FSS12);
  ttgo->tft->setTextColor(TFT_GREEN, TFT_BLACK);
  ttgo->tft->setCursor(10, 90);
  ttgo->tft->println(F("tECH                   degC"));
  ttgo->tft->setCursor(10, 155);
  ttgo->tft->println(F("tHUI                    degC"));
  ttgo->tft->setCursor(10, 210);
  ttgo->tft->println(F("RPM               "));
}

// ======================================================================AFFICHAGE DONNEES BATTERIE
void DisplayDataBatterie() {
  ttgo->tft->setTextFont(2);
  ttgo->tft->setTextSize(1);
  ttgo->tft->setFreeFont(FSSB24);
  ttgo->tft->fillRect(70, 50, 130, 50, TFT_BLACK);
  ttgo->tft->setCursor(90, 90);
  ttgo->tft->setTextColor(TFT_RED, TFT_BLACK);
  ttgo->tft->println(SOC);
  ttgo->tft->fillRect(70, 113, 130, 50, TFT_BLACK);
  ttgo->tft->setCursor(90, 155);
  ttgo->tft->setTextColor(TFT_RED, TFT_BLACK);
  ttgo->tft->println(VOLTAGE, 1);
  ttgo->tft->fillRect(70, 176, 130, 50, TFT_BLACK);
  ttgo->tft->setCursor(90, 210);
  ttgo->tft->setTextColor(TFT_RED, TFT_BLACK);
  ttgo->tft->println(AMPERAGE, 1);
  //delay (tempo);

  // Temporisation
  temps = millis();
  while (millis() < (temps + tempo)) {  }

}

// ====================================================================== AFFICHAGE DONNEES MOTEUR
void DisplayDataMoteur() {
  ttgo->tft->setTextFont(2);
  ttgo->tft->setTextSize(1);
  ttgo->tft->setFreeFont(FSSB24);
  ttgo->tft->fillRect(70, 50, 100, 50, TFT_BLACK);
  ttgo->tft->setCursor(90, 90);
  ttgo->tft->setTextColor(TFT_RED, TFT_BLACK);
  ttgo->tft->println(TECHAP, 0);
  ttgo->tft->fillRect(70, 113, 100, 50, TFT_BLACK);
  ttgo->tft->setCursor(90, 155);
  ttgo->tft->setTextColor(TFT_RED, TFT_BLACK);
  ttgo->tft->println(THUILE, 0);
  ttgo->tft->fillRect(70, 176, 140, 50, TFT_BLACK);
  ttgo->tft->setCursor(90, 210);
  ttgo->tft->setTextColor(TFT_RED, TFT_BLACK);
  ttgo->tft->println(RPM, 0);
  //delay (tempo);

  // Temporisation
  temps = millis();
  while (millis() < (temps + tempo)) {  }

}
// ====================================================================== RECUPERATION DES DONNEES
void GetDatas() {
  // +++++++++++++++++++++++++++++++++++++++++++++++
  // Example 5 - Receive with start- and end-markers combined with parsing
  // https://forum.arduino.cc/t/serial-input-basics-updated/382007/3

  //  <5.6,10h56,6h13,95,50,270,60,55,8.6,10.7,78,99,13.1,-3.5,33,65,2300,25.5>  // POUR DEBUGGAGE
  Serial.println("Entrez les données au format: <SOG, TTG, ETA, BTW, DTW, CAP, TWA, AWA, TWS, AWS, GWD, SOC, VOLTAGE, AMPERAGE, TECHAP, THUILE, RPM, PROF> ");
  Serial.println();

  recvWithStartEndMarkers(); //Recoit les données du port   BT (ou Serie POUR SIMULATION)
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    // this temporary copy is necessary to protect the original data
    //   because strtok() used in parseData() replaces the commas with \0
    parseData();      //parse les données de la sentence
    showParsedData(); // Affiche les données sur moniteur serie
    newData = false;
  }
  /*
    // ===== SIMULATION
    SOG = (random (0, 200) ) / 10;
    TTG = "10:03";
    ETA = "8:10";
    BTW = (random (0, 360) );
    DTW = (random (0, 1000) );
    CAP = CAP + 1;
    if (CAP > 360) {
      CAP = 0;
    }
    TWA = (random (0, 360) );
    AWA = (random (0, 360) );
    TWS = (random (0, 200) ) / 10;
    AWS = (random (0, 200) ) / 10;
    GWD = (random (0, 360) );

    SOC = (random (0, 100) );
    VOLTAGE = (random (0, 150) ) / 10;
    AMPERAGE = (random (0, 100) ) / 10;

    TECHAP = (random (0, 100) );
    THUILE = (random (0, 100) );
    RPM = (random (0, 3600) );

    PROF = (random (0, 100) );
  */
}

//============
void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;
  Serial.print("CARACTERES RECUS: "); Serial.println( SerialBT.available());
  //while (Serial.available() > 0 && newData == false) {  //saisie manuelle par moniteur série
  while (SerialBT.available() > 0 && newData == false) {  //RECEPTION SUR BLUETOOTH
    rc = SerialBT.read();
    /*
      Serial.print("On a reçu le caractère: ");
      Serial.println(rc);
    */
    if (recvInProgress == true) {  // ON EST EN TRAIN DE RECEVOIR DES DONNEES
      if (rc != endMarker) {       // SI LE CARACTERE RECU N EST PAS UN ">" QUI EST LA FIN DES DONNEES
        receivedChars[ndx] = rc;    // ON STOCKE LE CARECTERE RECU
        ndx++;                      // ON INCERMENTE D INDEX DES CARACTERES RECUS
        if (ndx >= numChars) {      // SI ON DEPASSE LA TAILLE MAX DE LA CHAINE DE CARATERE TOTALE DES DONNES
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
  // Serial.println("FIN recvWithStartEndMarkers ");
  // IMPRESSION DE LA LIGNE DE DONNEES RECUES
  for (int k = 0 ; k <= numChars  ; k++ ) {
    Serial.print (receivedChars[k]);
  }
  Serial.println ("");
}

//============
void parseData() {      // split the data into its parts   <SOG,TTG,ETA,BTW,DTW,CAP,TWA,AWA,TWS,AWS,GWD,SOC,VOLTAGE,AMPERAGE,TECHAP,THUILE,RPM,PROF>
  Serial.println("Debut PARSE DATA");
  char * strtokIndx; // this is used by strtok() as an index

  //Serial.println("avant strokindx");
  strtokIndx = strtok(tempChars, ", ");  // get the first part
  SOG = atof(strtokIndx);               // convert this part to a float
  // Serial.println("apres strokindx");

  strtokIndx = strtok(NULL, ", ");
  // TTG = atof(strtokIndx);     // convert this part to a float
  strcpy(TTG, strtokIndx);    // convert this part to a STRING  a remettre

  strtokIndx = strtok(NULL, ", "); // this continues where the previous call left off
  // ETA = atoi(strtokIndx);     // convert this part to an integer
  strcpy(ETA, strtokIndx);    // convert this part to a STRING

  strtokIndx = strtok(NULL, ", "); // this continues where the previous call left off
  BTW = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ", "); // this continues where the previous call left off
  DTW = atof(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ", "); // this continues where the previous call left off
  CAP = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ", "); // this continues where the previous call left off
  TWA = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ", "); // this continues where the previous call left off
  AWA = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ", ");
  TWS = atof(strtokIndx);     // convert this part to a float

  strtokIndx = strtok(NULL, ", "); // this continues where the previous call left off
  AWS = atof(strtokIndx);     // convert this part to an integer
  // strcpy(AWS, strtokIndx);    // convert this part to a STRING

  strtokIndx = strtok(NULL, ", "); // this continues where the previous call left off
  GWD = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ", "); // this continues where the previous call left off
  SOC = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ", ");
  VOLTAGE = atof(strtokIndx);     // convert this part to a float

  strtokIndx = strtok(NULL, ", ");
  AMPERAGE = atof(strtokIndx);     // convert this part to a float

  strtokIndx = strtok(NULL, ", "); // this continues where the previous call left off
  TECHAP = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ", "); // this continues where the previous call left off
  THUILE = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ", "); // this continues where the previous call left off
  RPM = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ", ");
  PROF = atof(strtokIndx);     // convert this part to a float
  /*
    char messageFromPC[numChars] = {0};
    int integerFromPC = 0;
    float floatFromPC = 0.0;

    strtokIndx = strtok(tempChars,", ");      // get the first part - the string
    strcpy(SOG, strtokIndx); // copy it to SOG

    strtokIndx = strtok(NULL, ", "); // this continues where the previous call left off
    //integerFromPC = atoi(strtokIndx);     // convert this part to an integer

    strtokIndx = strtok(NULL, ", ");
    //floatFromPC = atof(strtokIndx);     // convert this part to a float
  */
  Serial.println("Fin PARSE DATA");
}

//============

void showParsedData() {
  // <SOG,TTG,ETA,BTW,DTW,CAP,TWA,AWA,TWS,AWS,GWD,SOC,VOLTAGE,AMPERAGE,TECHAP,THUILE,RPM,PROF>
  if (DEBUG) {
    Serial.print("SOG ");
    Serial.println(SOG);
    Serial.print("TTG ");
    Serial.println(TTG);
    Serial.print("ETA ");
    Serial.println(ETA);
    Serial.print("BTW ");
    Serial.println(BTW);
    Serial.print("DTW ");
    Serial.println(DTW);
    Serial.print("CAP ");
    Serial.println(CAP);
    Serial.print("TWA ");
    Serial.println(TWA);
    Serial.print("AWA ");
    Serial.println(AWA);
    Serial.print("TWS ");
    Serial.println(TWS);
    Serial.print("AWS ");
    Serial.println(AWS);
    Serial.print("GWD ");
    Serial.println(GWD);
    Serial.print("SOC ");
    Serial.println(SOC);
    Serial.print("VOLTAGE ");
    Serial.println(VOLTAGE);
    Serial.print("AMPERAGE ");
    Serial.println(AMPERAGE);
    Serial.print("TECHAP ");
    Serial.println(TECHAP);
    Serial.print("THUILE ");
    Serial.println(THUILE);
    Serial.print("RPM ");
    Serial.println(RPM);
    Serial.print("PROF ");
    Serial.println(PROF);
  }
}

// ====================================================================== AFFICHAGE LIGNE D'ENTETE
void DisplayEntete() {
  DisplayVersion();
  AppBattery();
  DisplayTime();
  LogoBT();
}

// ====================================================================== AFFICHAGE DE LA VERSION SUR LIGNE D'ENTETE
void DisplayVersion() {
  ttgo->tft->setTextFont(2);
  ttgo->tft->setTextSize(1);
  ttgo->tft->setTextColor(TFT_WHITE);
  ttgo->tft->setCursor(8, 12);
  ttgo->tft->println(ver);
  // Serial.println(ver);
}
// ====================================================================== AFFICHAGE CHARGE BATTERIE SUR LIGNE D'ENTETE
void AppBattery() {       // AFFICHAGE CHARGE BATTERIE
  int per = ttgo->power->getBattPercentage(); // Recupere le % de charge de la batterie
  int batx = 60;
  int baty = 13;
  int batcolor = TFT_WHITE ;
  if (per < 33 ) {
    batcolor = TFT_RED ;   //Dessine la pile en rouge si < 33 %
  }
  else if (per < 50) {
    batcolor = TFT_ORANGE; //Dessine la pile en rouge si < 50 %
  }
  else {
    batcolor = TFT_WHITE; //Dessine la pile en rouge si entre 50 et 100 %
  }
  // Affichage du pourcentage
  ttgo->tft->setTextFont(2);
  ttgo->tft->setTextSize(1);
  ttgo->tft->setTextColor(batcolor, TFT_BLACK);
  ttgo->tft->setCursor(95, 12);
  ttgo->tft->print(per); ttgo->tft->println("%  ");
  ttgo->tft->fillRect(batx, baty, 28, 12, TFT_WHITE);
  ttgo->tft->fillRect(batx + 2, baty + 2, 24, 8, TFT_BLACK);
  ttgo->tft->fillRect(batx + 4, baty + 4, (per / 5), 4, batcolor);
  ttgo->tft->fillRect(batx + 28, baty + 4, 2, 6, TFT_WHITE);
}

// ====================================================================== AFFICHAGE DE L'HEURE SUR LIGNE D'ENTETE
void DisplayTime() {  //AFFICHAGE DE L'HEURE SUR LIGNE D'ENTETE

  RTC_Date tnow = ttgo->rtc->getDateTime(); // Récupère l'heure
  hh = tnow.hour;
  mm = tnow.minute;
  ss = tnow.second;
  dday = tnow.day;
  mmonth = tnow.month;
  yyear = tnow.year;

  // affiche l'heure
  ttgo->tft->setTextFont(2);
  ttgo->tft->setTextSize(1);
  ttgo->tft->setTextColor(TFT_WHITE, TFT_BLACK);
  ttgo->tft->setCursor(135, 12);
  if (hh < 10) {
    ttgo->tft->print(" ");
  }
  ttgo->tft->print(hh); ttgo->tft->print(": ");
  if (mm < 10) {
    ttgo->tft->print("0");  // ajoute un "0" si moins de 10mn
  }
  ttgo->tft->print(mm);
}
// ====================================================================== PETITS CERCLES EN BAS D ECRAN
void Stepper() {
  int Ycercle = 232;
  int Largeurecran = 240;
  int RayonCercle = 5;
  int EcartCercle = 3;
  int CouleurCercle = TFT_LIGHTGREY;
  int CentreCercle = ((240 - ((NbMaxPage * 2 * RayonCercle) + ((NbMaxPage - 1) * EcartCercle))) / 2) + RayonCercle;
  for (int i = 1; i < (NbMaxPage + 1) ; i++) {
    if (i == ChoixPage) {
      CouleurCercle = TFT_WHITE;
      ttgo->tft->fillCircle(CentreCercle, Ycercle, RayonCercle - 1, CouleurCercle);
      ttgo->tft->drawCircle(CentreCercle, Ycercle, RayonCercle + 1, TFT_BLACK);
    }
    else {
      CouleurCercle = TFT_SILVER;
    }
    ttgo->tft->fillCircle(CentreCercle, Ycercle, RayonCercle, CouleurCercle);
    CentreCercle = CentreCercle + (2 * RayonCercle) + EcartCercle;
  }
}


// ====================================================================== IMAGE SYLVIE
void photoSB () {
  ttgo->tft->fillScreen(TFT_BLACK);                 // efface l'écran
  ttgo->tft->setSwapBytes(true);
  ttgo->tft->pushImage(15, 15, 225, 225, LOGOHO);     //affiche la photo de SylviepushImage(x,y,209,240,image)
  while (ChoixPage == 8) {                          // On reste là tant que l'écran n'a pas été touché
    ToucheEcran();      // l'écran a-t-il été touché ?
  }
}

// FIN ======================================================================
