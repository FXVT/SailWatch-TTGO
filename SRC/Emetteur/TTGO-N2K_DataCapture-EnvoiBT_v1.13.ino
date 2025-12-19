// DEBUGGER LE TTG
#define ver "TTGO-N2K_DataCapture-EnvoiBT_v1.13"
#include <Arduino.h>
#define N2k_SPI_CS_PIN 9    // N° Pin  SPI select pour mcp_can Elecrow shield = 10 , Seedunio Shield = 9
#include <NMEA2000_CAN.h>
#include <N2kMessages.h>
#include <N2kMessagesEnumToStr.h>

bool    SIMULATION = false;     // choisir si on est en mode simulation (envoi de données bidons) ou pas
String  HEURE = "0";
int     HTime = 0.00000;
int     MNTime = 0;
double  ResteTime = 0.0000;

float   SOGD = 0;   // recu
String  TTG = "0";  // CALCULE
String  ETA = "12:50";   // recu
int     BTW = 0;   // recu
double  DTW = 0;    // recu
int     CAP = 0;   // recu

int   TWA = 0;;    // CALCULE
float AWA = 0;    // recu
float TWS = 0;    // CALCULE
float AWS = 0;    // recu
int   GWD = 0;    // CALCULE


int   SOC = 0;    // recu
float VOLTAGE = 0; // RECU
float AMPERAGE = 0;// RECU

int TECHAP = 0;   // recu
int THUILE = 0;   // recu
int RPM    = 0;   // recu

double PROF = 0;   // recu

// Variables Définies mais pas envoyées, utilisées pour les calculs
float COGD = 0;    // recu pour calculer GWD
int   TWD = 0;
float WCV = 0;   // WayPointClosingVelociy recu pour calculer TTG

String Donnees; // Phrase de donnees envoyee en bluetooth

// Led clignotante. définir la numéro de la broche
const int ledPin =  8;         // crée un identifiant pour la broche utilisée avec la LED
int ledState = LOW;             // ledState est utilisée pour fixer l'état de la LED (HIGH/LOW)

int PeriodEnvoi = 500 ;         // VOIR CE QUE CA DONNE SI ON PASSE A UNE SECONDE

typedef struct {
  unsigned long PGN;
  void (*Handler)(const tN2kMsg &N2kMsg);
} tNMEA2000Handler;

void SystemTime(const tN2kMsg &N2kMsg);
void EngineRapid(const tN2kMsg &N2kMsg);
void EngineDynamicParameters(const tN2kMsg &N2kMsg);
void TransmissionParameters(const tN2kMsg &N2kMsg);
void Speed(const tN2kMsg &N2kMsg);
void WaterDepth(const tN2kMsg &N2kMsg);
void COGSOG(const tN2kMsg &N2kMsg);
void Heading(const tN2kMsg &N2kMsg);
void DCBatStatus(const tN2kMsg &N2kMsg);

void Wind(const tN2kMsg &N2kMsg);
void DCStatus (const tN2kMsg &N2kMsg);
void NavigationInfo (const tN2kMsg &N2kMsg);

tNMEA2000Handler NMEA2000Handlers[] = {
  {126992L, &SystemTime},
  {127488L, &EngineRapid},
  {127489L, &EngineDynamicParameters},
  {127493L, &TransmissionParameters},
  {128259L, &Speed},
  {128267L, &WaterDepth},
  {129026L, &COGSOG},
  {127250L, &Heading},
  {130306L, &Wind},
  {127508L, &DCBatStatus},
  {127506L, &DCStatus},
  {129284L, &NavigationInfo},
  {0, 0}
};

Stream *OutputStream;

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);

void setup()
{
  Serial.begin(115200);
  Serial3.begin(115200); // HC-05 branché sur le Port Serie3 Tx=18 Rx=19 Passer a 38400 si trop d erreurs
  delay (500) ;

  Serial.print ("TTGO-N2K_DataCapture-EnvoiBT_v"); //Affiche la version du logiciel sur le moniteur serie
  Serial.println (ver);

  pinMode(ledPin , OUTPUT); // LED clignotante

  OutputStream = &Serial;

  NMEA2000.SetN2kCANReceiveFrameBufSize(100);  // On augmente le buffer, ça peut pas faire de mal

  // Do not forward bus messages at all
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);
  NMEA2000.SetForwardStream(OutputStream);
  // Set false below, if you do not want to see messages parsed to HEX withing library
  // NMEA2000.EnableForward(false); // Disable all msg forwarding to USB (=Serial)
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);
  //  NMEA2000.SetN2kCANMsgBufSize(2);
  NMEA2000.Open();
  OutputStream->println("Running...");
}

// Converti l'unité des données
template<typename T> void PrintLabelValWithConversionCheckUnDef(const char* label, T val, double (*ConvFunc)(double val) = 0, bool AddLf = false, int8_t Desim = -1 ) {
  OutputStream->print(label);
  if (!N2kIsNA(val)) {
    if ( Desim < 0 ) {
      if (ConvFunc) {
        OutputStream->print(ConvFunc(val));
      } else {
        OutputStream->print(val);
      }
    } else {
      if (ConvFunc) {
        OutputStream->print(ConvFunc(val), Desim);
      } else {
        OutputStream->print(val, Desim);
      }
    }
  } else OutputStream->print("not available");
  if (AddLf) OutputStream->println();
}

//*****************************************************************************
//NMEA 2000 message handler
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  int iHandler;

  // Find handler
  OutputStream->print("================================================  IN MAIN HANDLER: "); OutputStream->println(N2kMsg.PGN);
  for (iHandler = 0; NMEA2000Handlers[iHandler].PGN != 0 && !(N2kMsg.PGN == NMEA2000Handlers[iHandler].PGN); iHandler++);

  if (NMEA2000Handlers[iHandler].PGN != 0) {
    NMEA2000Handlers[iHandler].Handler(N2kMsg);
  }
}
//***************************************************************************** JE NE VOIS PAS A QUOI CA SERT, SI on l'enlève ça compile quand même.
void printLLNumber(Stream *OutputStream, unsigned long long n, uint8_t base = 10)
{
  unsigned char buf[16 * sizeof(long)]; // Assumes 8-bit chars.
  unsigned long long i = 0;

  if (n == 0) {
    OutputStream->print('0');
    return;
  }

  while (n > 0) {
    buf[i++] = n % base;
    n /= base;
  }

  for (; i > 0; i--)
    OutputStream->print((char) (buf[i - 1] < 10 ?
                                '0' + buf[i - 1] :
                                'A' + buf[i - 1] - 10));
}

// ==========================================================================================================================
void loop()
{
  if (SIMULATION) {                 // Si on est en mode simulation on envoie des données bidons
    GetDatas();                       // Simulation
  }
  else {
    NMEA2000.ParseMessages();   // Sinon on récupère les données du réseau NMEA2000
  }
  static unsigned long Updated = millis();
  if ( Updated + PeriodEnvoi < millis() ) {   // TEMPORISATION
    Updated = millis();
    PrepareEnvoiDataBT ();                    // CALULE CERTAINES DONNEES ET FAIT L'ENVOI EN BLUETOOTH
  }
  // Serial.println ("=================================================================================");
}

// ========================================================================================== GENERATION DE DONNEES DE SIMULATION
void GetDatas() {
  HEURE = "12:50";

  SOGD = (random (0, 200) ) / 10;
  WCV = SOGD ;
  // TTG = "10:03";                 // Pas la peine de le générer, il est calculé
  ETA = "8:10";
  BTW = (random (0, 360) );
  DTW = (random (0, 1000) );
  CAP = (random (0, 360) );

  // TWA = (random (0, 360) );      // Pas la peine de le générer, il est calculé
  AWA = (random (0, 360) );
  // TWS = (random (0, 200) ) / 10; // Pas la peine de le générer, il est calculé
  AWS = (random (0, 200) ) / 10;
  // GWD = (random (0, 360) );      // Pas la peine de le générer, il est calculé

  SOC = (random (0, 100) );
  VOLTAGE = (random (0, 150) ) / 10;
  AMPERAGE = (random (0, 100) ) / 10;

  TECHAP = (random (0, 100) );
  THUILE = (random (0, 100) );
  RPM = (random (0, 3600) );

  PROF = (random (0, 100) );

  //============== DONNES BIDON FIXES AVEC DECIMALES
  /*
    HEURE = "12:50";
    SOGD =5.07;
    WCV = SOGD ;
    // TTG = "10:03";                 // Pas la peine de le générer, il est calculé
    ETA = "8:12";
    BTW = (random (0, 360) );
    DTW = 123.45;
    CAP = (random (0, 360) );
    // TWA = (random (0, 360) );      // Pas la peine de le générer, il est calculé
    AWA = 264.56;
    // TWS = (random (0, 200) ) / 10; // Pas la peine de le générer, il est calculé
    AWS = 10.67;
    // GWD = (random (0, 360) );      // Pas la peine de le générer, il est calculé
    SOC = (random (0, 100) );
    VOLTAGE = 12.57;
    AMPERAGE = -10.45;
    TECHAP = (random (0, 100) );
    THUILE = (random (0, 100) );
    RPM = (random (0, 3600) );
    PROF = 2.43;
  */

}

//**************************************************************************** HEURE
void SystemTime(const tN2kMsg &N2kMsg) {
  Serial.println ("debut Systemtime");
  unsigned char SID;
  uint16_t SystemDate;
  double SystemTime;
  tN2kTimeSource TimeSource;

  if (ParseN2kSystemTime(N2kMsg, SID, SystemDate, SystemTime, TimeSource) ) {
    //OutputStream->println("System time:");
    //PrintLabelValWithConversionCheckUnDef("  SID: ", SID, 0, true);
    //PrintLabelValWithConversionCheckUnDef("  days since 1.1.1970: ", SystemDate, 0, true);
    PrintLabelValWithConversionCheckUnDef("  seconds since midnight: ", SystemTime, 0, true);
    //OutputStream->print("  time source: "); PrintN2kEnumType(TimeSource, OutputStream);
  } else {
    OutputStream->print("Failed to parse PGN: "); OutputStream->println(N2kMsg.PGN);
  }
  // SystemTime = 45321;  // POUR DEBUG
  HTime = (SystemTime / 3600);
  Serial.println (HTime);
  ResteTime = ((SystemTime / 3600) - HTime);
  Serial.println (ResteTime);
  MNTime = (ResteTime * 60);
  Serial.println (MNTime);
  if (MNTime < 10) {
    String HEURE (String (HTime) + ":0" + String (MNTime));
  }
  else {
    String HEURE (String (HTime) + ":" + String (MNTime));
  }
  Serial.println (HEURE);
  Serial.println ("fin Systemtime");
}

/*
   nombre de secondesdepuis minuit.Donc:
  ex: 56123 = ??
  tu divise par 3600, ca te donne 15.589 donc 15h
  t'arrondis le reste a 0.59 que tu multipllie ensuite par 60 pour avoir les min soit 35,4, arrondi - 35
  56123 = 15h35
*/

//*****************************************************************************130306 WIND
void Wind(const tN2kMsg &N2kMsg) {

  unsigned char SID;
  double WindSpeed;
  double WindAngle;
  tN2kWindReference WindReference;

  if (ParseN2kPGN130306(N2kMsg, SID, WindSpeed, WindAngle, WindReference) ) {
    // PrintLabelValWithConversionCheckUnDef("Wind Speed params: ", SID, 0, true);
    PrintLabelValWithConversionCheckUnDef("  Vitesse: ", WindSpeed, 0, true);
    PrintLabelValWithConversionCheckUnDef("  Angle: ", WindAngle, 0, true);
    // OutputStream->print("WindReference: "); OutputStream->println(WindReference);
  } else {
    OutputStream->print("Failed to parse PGN: "); OutputStream->println(N2kMsg.PGN);
  }
  float Windspeed = WindSpeed;
  AWS = (Windspeed * 1.9438); //Conversion ms en kts
  AWA = (WindAngle * 57.2958); // Converion Rad en Degres

  Serial.print ("Angle VENT Rad");  Serial.print (WindAngle);  Serial.print ("  AWA deg ");  Serial.println (AWA);

}

//***************************************************************************** RPM
void EngineRapid(const tN2kMsg &N2kMsg) {
  unsigned char EngineInstance;
  double EngineSpeed;
  double EngineBoostPressure;
  int8_t EngineTiltTrim;

  if (ParseN2kEngineParamRapid(N2kMsg, EngineInstance, EngineSpeed, EngineBoostPressure, EngineTiltTrim) ) {
    //PrintLabelValWithConversionCheckUnDef("Engine rapid params: ", EngineInstance, 0, true);
    PrintLabelValWithConversionCheckUnDef("  RPM: ", EngineSpeed, 0, true);
    //PrintLabelValWithConversionCheckUnDef("  boost pressure (Pa): ", EngineBoostPressure, 0, true);
    // PrintLabelValWithConversionCheckUnDef("  tilt trim: ", EngineTiltTrim, 0, true);
  } else {
    OutputStream->print("Failed to parse PGN: "); OutputStream->println(N2kMsg.PGN);
  }
  RPM = EngineSpeed;
}

//***************************************************************************** TEMPERATURE HUILE
void EngineDynamicParameters(const tN2kMsg &N2kMsg) {
  unsigned char EngineInstance;
  double EngineOilPress;
  double EngineOilTemp;
  double EngineCoolantTemp;
  double AltenatorVoltage;
  double FuelRate;
  double EngineHours;
  double EngineCoolantPress;
  double EngineFuelPress;
  int8_t EngineLoad;
  int8_t EngineTorque;
  tN2kEngineDiscreteStatus1 Status1;
  tN2kEngineDiscreteStatus2 Status2;

  if (ParseN2kEngineDynamicParam(N2kMsg, EngineInstance, EngineOilPress, EngineOilTemp, EngineCoolantTemp,
                                 AltenatorVoltage, FuelRate, EngineHours,
                                 EngineCoolantPress, EngineFuelPress,
                                 EngineLoad, EngineTorque, Status1, Status2) ) {
    // PrintLabelValWithConversionCheckUnDef("Engine dynamic params: ", EngineInstance, 0, true);
    // PrintLabelValWithConversionCheckUnDef("  oil pressure (Pa): ", EngineOilPress, 0, true);
    PrintLabelValWithConversionCheckUnDef("  oil temp (C): ", EngineOilTemp, &KelvinToC, true);
    PrintLabelValWithConversionCheckUnDef("  coolant temp (C): ", EngineCoolantTemp, &KelvinToC, true);
    //PrintLabelValWithConversionCheckUnDef("  altenator voltage (V): ", AltenatorVoltage, 0, true);
    //PrintLabelValWithConversionCheckUnDef("  fuel rate (l/h): ", FuelRate, 0, true);
    //PrintLabelValWithConversionCheckUnDef("  engine hours (h): ", EngineHours, &SecondsToh, true);
    //PrintLabelValWithConversionCheckUnDef("  coolant pressure (Pa): ", EngineCoolantPress, 0, true);
    //PrintLabelValWithConversionCheckUnDef("  fuel pressure (Pa): ", EngineFuelPress, 0, true);
    //PrintLabelValWithConversionCheckUnDef("  engine load (%): ", EngineLoad, 0, true);
    //PrintLabelValWithConversionCheckUnDef("  engine torque (%): ", EngineTorque, 0, true);
  } else {
    OutputStream->print("Failed to parse PGN: "); OutputStream->println(N2kMsg.PGN);
  }
  THUILE = (EngineOilTemp - 273.15); // Conversion F en °C

}

//***************************************************************************** TEMPERATURE ECHAPPEMENT
void TransmissionParameters(const tN2kMsg &N2kMsg) {
  unsigned char EngineInstance;
  tN2kTransmissionGear TransmissionGear;
  double OilPressure;
  double OilTemperature;
  unsigned char DiscreteStatus1;

  if (ParseN2kTransmissionParameters(N2kMsg, EngineInstance, TransmissionGear, OilPressure, OilTemperature, DiscreteStatus1) ) {
    // PrintLabelValWithConversionCheckUnDef("Transmission params: ", EngineInstance, 0, true);
    // OutputStream->print("  gear: "); PrintN2kEnumType(TransmissionGear, OutputStream);
    // PrintLabelValWithConversionCheckUnDef("  oil pressure (Pa): ", OilPressure, 0, true);
    PrintLabelValWithConversionCheckUnDef("  oil temperature (C): ", OilTemperature, &KelvinToC, true);
    // PrintLabelValWithConversionCheckUnDef("  discrete status: ", DiscreteStatus1, 0, true);
  } else {
    OutputStream->print("Failed to parse PGN: "); OutputStream->println(N2kMsg.PGN);
  }
  TECHAP =  (OilTemperature - 273.15); // Conversion F en °C
}
//***************************************************************************** CAP
void Heading(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  tN2kHeadingReference HeadingReference;
  double Heading;
  double Deviation;
  double Variation;

  if (ParseN2kHeading(N2kMsg, SID, Heading, Deviation, Variation, HeadingReference) ) {
    OutputStream->println("Heading:");
    // PrintLabelValWithConversionCheckUnDef("  SID: ", SID, 0, true);
    // OutputStream->print("  reference: "); PrintN2kEnumType(HeadingReference, OutputStream);
    PrintLabelValWithConversionCheckUnDef("  Heading (deg): ", Heading, &RadToDeg, true);
    // PrintLabelValWithConversionCheckUnDef("  Deviation (deg): ", Deviation, &RadToDeg, true);
    //PrintLabelValWithConversionCheckUnDef("  Variation (deg): ", Variation, &RadToDeg, true);
  } else {
    OutputStream->print("Failed to parse PGN: "); OutputStream->println(N2kMsg.PGN);
  }
  CAP = (Heading * 57.2958) ; // Conversion radian en degres
  Serial.print("CAP "); Serial.print(CAP);        // voir si c'est égal au Heading arrondi semble faux trop petit 124 au lieu de 124,79
}

//*****************************************************************************129026 SOG OK DISPO, COG A ZERO CAR N AVANCE PAS ?
void COGSOG(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  tN2kHeadingReference HeadingReference;
  double COG;
  double SOG;

  if (ParseN2kCOGSOGRapid(N2kMsg, SID, HeadingReference, COG, SOG) ) {
    OutputStream->println("COG/SOG:");
    //PrintLabelValWithConversionCheckUnDef("  SID: ", SID, 0, true);
    //OutputStream->print("  reference: "); PrintN2kEnumType(HeadingReference, OutputStream);
    PrintLabelValWithConversionCheckUnDef("  COG (deg): ", COG, &RadToDeg, true);
    PrintLabelValWithConversionCheckUnDef("  SOG (m/s): ", SOG, 0, true);
  } else {
    OutputStream->print("Failed to parse PGN: "); OutputStream->println(N2kMsg.PGN);
  }
  SOGD = (SOG * 1.9438);  //Conversion ms en kts
  COGD = (COG * 57.2958); // Conversion radian en degres  COG=toujours 360.00 !!!
}
//***************************************************************************** 129284 DTW ETA BTW
void NavigationInfo(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  double DistanceToWaypoint;
  tN2kHeadingReference BearingReference;
  bool PerpendicularCrossed;
  bool ArrivalCircleEntered;
  tN2kDistanceCalculationType CalculationType;
  double ETATime;
  int16_t ETADate;
  double BearingOriginToDestinationWaypoint;
  double BearingPositionToDestinationWaypoint;
  uint8_t OriginWaypointNumber;
  uint8_t DestinationWaypointNumber;
  double DestinationLatitude;
  double DestinationLongitude;
  double WaypointClosingVelocity;

  long HTimeETA;
  int MNTimeETA;

  /*
    if (ParseN2kNavigationInfo(N2kMsg, SID, DistanceToWaypoint, BearingReference, PerpendicularCrossed, ArrivalCircleEntered, CalculationType,
                               ETATime, ETADate, BearingOriginToDestinationWaypoint, BearingPositionToDestinationWaypoint, OriginWaypointNumber, DestinationWaypointNumber,
                               DestinationLatitude, DestinationLongitude, WaypointClosingVelocity))
  */
  if (ParseN2kPGN129284     (N2kMsg, SID, DistanceToWaypoint, BearingReference, PerpendicularCrossed, ArrivalCircleEntered, CalculationType,
                             ETATime, ETADate, BearingOriginToDestinationWaypoint, BearingPositionToDestinationWaypoint, OriginWaypointNumber, DestinationWaypointNumber,
                             DestinationLatitude, DestinationLongitude, WaypointClosingVelocity))
  {
    OutputStream->print("DTW: "); OutputStream->println(DistanceToWaypoint);
    OutputStream->print("ETATime: "); OutputStream->println(ETATime);
    OutputStream->print("BTW: "); OutputStream->println(BearingPositionToDestinationWaypoint);
    PrintLabelValWithConversionCheckUnDef("  WCV (m/s): ", WaypointClosingVelocity, 0, true);
  }
  else {
    OutputStream->print("Failed to parse PGN: "); OutputStream->println(N2kMsg.PGN);
  }

  DTW = DistanceToWaypoint / 1852;                // Convertir Metres en Miles
  WCV = ((WaypointClosingVelocity * 1.9438), 1);  //Conversion ms en kts
  BTW = (BearingPositionToDestinationWaypoint * 57.2958) ; //Convertir radian en degres

  // CONVERSION ETA EN SECONDE EN ETA AU FORMAT HH:MM
  HTimeETA = (ETATime / 3600);
  MNTimeETA = (ETATime - (HTimeETA * 3600)) / 60;
  if (MNTimeETA < 10) {
    ETA = (String (HTimeETA) + ":0" + String (MNTimeETA));
  }
  else {
    ETA = (String (HTimeETA) + ":" + String (MNTimeETA));
  }
}
//***************************************************************************** 127508  VOLTAGE AMPERAGE
void DCBatStatus (const tN2kMsg &N2kMsg) {
  unsigned char BatteryInstance;
  double BatteryVoltage;
  double BatteryCurrent;
  double BatteryTemperature;
  unsigned char SID;

  if (ParseN2kDCBatStatus(N2kMsg, BatteryInstance, BatteryVoltage, BatteryCurrent, BatteryTemperature, SID) ) {
    //OutputStream->print("Batterie instance: "); OutputStream->println(BatteryInstance);
    OutputStream->print("  - Voltage (V): "); OutputStream->println(BatteryVoltage);
    OutputStream->print("  - Conso (A): "); OutputStream->println(BatteryCurrent);
    //OutputStream->print("  - Temperature (F/C): "); OutputStream->println(BatteryTemperature);
    //OutputStream->print("  - SID: "); OutputStream->println(SID);
  } else {
    OutputStream->print("Failed to parse PGN: ");  OutputStream->println(N2kMsg.PGN);
  }
  VOLTAGE   = BatteryVoltage;
  AMPERAGE  = BatteryCurrent;
}
//***************************************************************************** 127506  SOC
void DCStatus(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  unsigned char DCInstance;
  tN2kDCType DCType;
  unsigned char StateOfCharge;
  unsigned char StateOfHealth;
  double TimeRemaining;
  double RippleVoltage;
  double Capacity;

  if (ParseN2kDCStatus(N2kMsg, SID, DCInstance, DCType, StateOfCharge, StateOfHealth, TimeRemaining, RippleVoltage, Capacity) ) {
    //OutputStream->print("DC instance: ");
    //OutputStream->println(DCInstance);
    //OutputStream->print("  - type: "); PrintN2kEnumType(DCType, OutputStream);
    OutputStream->print("  - state of charge (%): "); OutputStream->println(StateOfCharge);
    //OutputStream->print("  - state of health (%): "); OutputStream->println(StateOfHealth);
    //OutputStream->print("  - time remaining (h): "); OutputStream->println(TimeRemaining / 60);
    //OutputStream->print("  - ripple voltage: "); OutputStream->println(RippleVoltage);
    //OutputStream->print("  - capacity: "); OutputStream->println(Capacity);

  } else {
    OutputStream->print("Failed to parse PGN: ");  OutputStream->println(N2kMsg.PGN);
  }
  SOC = StateOfCharge;
}

//*****************************************************************************128259 SOW, SOG(NOT AVAILABLE) PAS SUR QU ON GARDE CAR SOG NO AVAILABLE  ET SOW INUTILE
void Speed(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  double SOW; double SOG;
  tN2kSpeedWaterReferenceType SWRT;

  if (ParseN2kBoatSpeed(N2kMsg, SID, SOW, SOG, SWRT) ) {
    OutputStream->print("Boat speed:");
    PrintLabelValWithConversionCheckUnDef(" SOW:", N2kIsNA(SOW) ? SOW : msToKnots(SOW));
    PrintLabelValWithConversionCheckUnDef(", SOG:", N2kIsNA(SOG) ? SOG : msToKnots(SOG));
    OutputStream->print(", ");
    PrintN2kEnumType(SWRT, OutputStream, true);
  }
  SOGD = ((SOG * 1.9438), 1); //Conversion ms en kts
}

//***************************************************************************** 128267 PROFONDEUR EAU
/*
  // MA VERSION
  void WaterDepth (const tN2kMsg &N2kMsg) {
  unsigned char SID;
  double DepthBelowTransducer;
  double Offset;
  double Range = N2kDoubleNA;

  if (ParseN2kWaterDepth(N2kMsg, SID, DepthBelowTransducer, Offset, Range) ) {
    OutputStream->print("Prof Eau:"); OutputStream->println(DepthBelowTransducer);
    OutputStream->print("Offset "); OutputStream->println(Offset);
  }
  PROF = (DepthBelowTransducer + Offset);
  }
*/

void WaterDepth(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  double DepthBelowTransducer;
  double Offset;

  if (ParseN2kWaterDepth(N2kMsg, SID, DepthBelowTransducer, Offset) ) {
    if ( N2kIsNA(Offset) || Offset == 0 ) {
      PrintLabelValWithConversionCheckUnDef("Prof Eau: Depth below transducer", DepthBelowTransducer);
      if ( N2kIsNA(Offset) ) {
        OutputStream->println(", offset not available");
      } else {
        OutputStream->println(", offset=0");
      }
    } else {
      if (Offset > 0) {
        OutputStream->print("Water depth:");
      } else {
        OutputStream->print("Depth below keel:");
      }
      if ( !N2kIsNA(DepthBelowTransducer) ) {
        OutputStream->println(DepthBelowTransducer + Offset);
      } else {
        OutputStream->println(" not available");
      }
    }
    PROF = (DepthBelowTransducer + Offset);
    Serial.print ("PROF: "); Serial.println (PROF);
  }
}

//***************************************************************************** PREPARATION / AFFICHAGE et ENVOI DES DONNEES EN BLUETOOTH
void PrepareEnvoiDataBT() {

  // ====================================== calcul du TTG
  int TTGH = DTW / WCV;     // Calcul des heures
  double TTGreste = (DTW / WCV) - TTGH;
  int TTGM = TTGreste * 60;  // Calcul des minutes
  if (TTGM < 10) {
    TTG = (String (TTGH) + ":0" + String (TTGM));
  }
  else {
    TTG = (String (TTGH) + ":" + String (TTGM));
  }
Serial.print ("DTW: "); Serial.println (DTW);
Serial.print ("WCV: "); Serial.println (WCV);
Serial.print ("TTG: "); Serial.println (TTG);
  // ====================================== calcul de GWD
  GWD = CAP + AWA ;
  if (GWD >= 360) {
    GWD = (GWD - 360);
  }

  // ====================================== calcul de TWS et TWA https://www.yachtd.com/news/trigonometry_and_encryption.html
  // ====================================== calcul de TWS
  float M =  -1.0; //Float -1.0

  if (SOGD == 0) {
    TWS = AWS; //TWS is equal to AWS when no speed
  }
  else {
    // ====================================== calcul de TWS
    TWS = sqrt((SOGD * SOGD) + (AWS * AWS) - (2 * SOGD * AWS * (cos(AWA / 180 * 3.14)))); // CALCUL VENT REEL
    // Serial.println (SOGD * SOGD);    Serial.println (AWS * AWS);    Serial.println (cos(AWA / 180 * 3.14));    Serial.println ("TWS origine ");     Serial.println (TWS);

    // ======================================= calcul de TWD
    if (TWS != 0) {
      float F = ((SOGD * SOGD)  + (TWS * TWS) - (AWS * AWS)) / (2 * TWS * SOGD);
      // POUR PB D'ARRONDI
      if (F < M) {
        F = M;
      }
      F = 3.1416 - acos( F ); //3.14 = 180°
      if (AWA > 180) {
        TWD = COGD - (F * 57.2958);
      }
      else {
        TWD = COGD + (F * 57.2958);
      }
    }
  }

  // VERIFIE SI TWD EST < OU > 360 degrees
  if (TWD > 360) {
    TWD = TWD - 360;
  }
  if (TWD < 0) {
    TWD = TWD + 360;
  }

  // ======================================= calcul de TWA and et verifie la plage de valeur
  TWA = TWD - COGD;
  if (TWA < 0) {
    TWA = TWA + 360;
  }
  // ========================================================================== Affichage données
  Serial.print("SOG ");
  Serial.println(SOGD);
  Serial.print("TTG ");
  Serial.println(TTG);
  Serial.print("ETA ");
  Serial.println(ETA);
  Serial.print("WCV ");
  Serial.println(WCV);
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

  // CONSTRUCTION DE LA CHAINE "Donnees" QUI CONCATENE TOUTES LES DONNEES A ENVOYER EN BLUETOOTH, SEPARATEUR VIRGULE ET <> COMME DELIMITEURS
  Donnees =  "<" + String (SOGD, 1) + "," + TTG + "," + ETA  + "," + BTW  + "," + DTW  + "," + CAP  + ","  + TWA + "," +  AWA + "," + String (TWS , 1)  + "," +   String (AWS, 1) + "," + GWD + "," + SOC + "," +  String (VOLTAGE, 1) + "," +  String (AMPERAGE, 1) + "," + TECHAP + "," + THUILE + "," + RPM + "," + PROF + ">" ;
  // Donnees = "<5.6,10h56,6h13,95,50,270,60,55,8.6,10.7,78,99,13.1,-3.5,33,65,2300,25.5>";    // POUR DEBUGGAGE
  Serial.println(Donnees);  // IMPRESSION SUR PORT SERIE POUR VERIFICATION
  Serial3.print(Donnees);   //envoi sur le port Serial3 vers le module BlueTooth HC-05 Esclave
  BlinkLed();               //Appel allumage voyant LED La led s'allume et s'eteind a chaque envoi de données
}

//------------------------------------------------------ LED CLIGNOTANTE
void BlinkLed () {
  if (ledState == LOW)
    ledState = HIGH;
  else
    ledState = LOW;
  // modifie l'état de la LED
  digitalWrite(ledPin, ledState);
}
