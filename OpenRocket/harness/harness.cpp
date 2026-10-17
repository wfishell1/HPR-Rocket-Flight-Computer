// Replays a simulated flight through HPR's original event logic (Event_Logic.ino).
//
// Input:  a sensors CSV made by Parser/openrocket_to_test.py
// Output: <input>_cpp_events.csv  every event flag, radio event, pyro firing, EEPROM write
//         <input>_cpp_trace.csv   the flight computer's estimates at every sample
//
// checkEvents() is compiled from the real Main Code/Event_Logic.ino, unchanged.
// The code it depends on from HPR_Rocket_Flight_PC_V4_7_2.ino (liftoff detection,
// velocity/altitude integration, barometer processing, pyro firing) is copied
// below with line references, minus the hardware calls.

#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>

#include "arduino_stubs.h"
#include "../../Main Code/declarations.h"

// copied from HPR_Rocket_Flight_PC_V4_7_2.ino:1742
void timerBuildPkt(){buildPkt = true;}

// copied from HPR_Rocket_Flight_PC_V4_7_2.ino:1587-1592
void firePyros(char event){
   pyroFire = true;
   if(pyro1.func == event){pins.firePin = pyro1.firePin; digitalWrite(pyro1.firePin, HIGH); pyro1.fireStatus = true; pyro1.fireStart = fltTime.timeCurrent;}
   if(pyro2.func == event){pins.firePin = pyro2.firePin; digitalWrite(pyro2.firePin, HIGH); pyro2.fireStatus = true; pyro2.fireStart = fltTime.timeCurrent;}
   if(pyro3.func == event){pins.firePin = pyro3.firePin; digitalWrite(pyro3.firePin, HIGH); pyro3.fireStatus = true; pyro3.fireStart = fltTime.timeCurrent;}
   if(pyro4.func == event){pins.firePin = pyro4.firePin; digitalWrite(pyro4.firePin, HIGH); pyro4.fireStatus = true; pyro4.fireStart = fltTime.timeCurrent;}}

// copied from HPR_Rocket_Flight_PC_V4_7_2.ino:1594-1612
void pulsePyro(){
  static boolean pyro1fire = false;
  static boolean pyro2fire = false;
  static boolean pyro3fire = false;
  static boolean pyro4fire = false;

  if(pyro1.fireStatus){
    if(pyro1fire){digitalWrite(pyro1.firePin, LOW); pyro1fire = false;}
    else{digitalWrite(pyro1.firePin, HIGH); pyro1fire = true;}}
  if(pyro2.fireStatus){
    if(pyro2fire){digitalWrite(pyro2.firePin, LOW); pyro2fire = false;}
    else{digitalWrite(pyro2.firePin, HIGH); pyro2fire = true;}}
  if(pyro3.fireStatus){
    if(pyro3fire){digitalWrite(pyro3.firePin, LOW); pyro3fire = false;}
    else{digitalWrite(pyro3.firePin, HIGH); pyro3fire = true;}}
  if(pyro4.fireStatus){
    if(pyro4fire){digitalWrite(pyro4.firePin, LOW); pyro4fire = false;}
    else{digitalWrite(pyro4.firePin, HIGH); pyro4fire = true;}}
}

// copied from HPR_Rocket_Flight_PC_V4_7_2.ino:1662-1738
void processBaroSamp(){

  //----------------------------
  //process preliftoff variables
  //----------------------------
  static float sumBaseAlt = 0.0F;
  static uint8_t baseAltPosn = 0;
  static float baseAltBuff[30]  = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
                                   0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
                                   0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
  if(events.preLiftoff || events.touchdown || events.timeOut){
    sumBaseAlt -= baseAltBuff[baseAltPosn];
    sumBaseAlt += baro.rawAlt;
    baseAltBuff[baseAltPosn] = baro.rawAlt;
    baseAltPosn++;
    const uint8_t sizeBaseAltBuff = sizeof(baseAltBuff) / sizeof(baseAltBuff[0]);
    if(baseAltPosn >= sizeBaseAltBuff){baseAltPosn = 0;}
    baro.baseAlt = sumBaseAlt / sizeBaseAltBuff;
    radio.baseAlt = (int16_t)baro.baseAlt;
    baro.newSamp = false;}

  //--------------------------
  //Update in-flight variables
  //--------------------------
  baro.Alt = baro.rawAlt - baro.baseAlt;
  //update the maximum altitude reporting variable
  if(baro.Alt > baro.maxAlt && !events.apogee){baro.maxAlt = baro.Alt;}

  //----------------------------
  //Smoothed barometric altitude
  //----------------------------
  static float rawAltSum = 0.0;
  static float rawAltBuff[10] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  static byte rawAltPosn = 0;
  //Update the rolling sum
  rawAltSum -= rawAltBuff[rawAltPosn];
  rawAltBuff[rawAltPosn] = baro.Alt;
  rawAltSum += rawAltBuff[rawAltPosn];
  //update the position counter
  rawAltPosn++;
  static const float sizeRawAltBuff = sizeof(rawAltBuff)/sizeof(rawAltBuff[0]);
  if(rawAltPosn >= (byte)sizeRawAltBuff){rawAltPosn = 0;}
  //calculate the smoothed barometric altitude
  baro.smoothAlt = rawAltSum / sizeRawAltBuff;

  //---------------------------
  //barometric derived velocity
  //---------------------------
  static uint8_t altAvgPosn = 0;
  static int16_t baroVelPosn = 0;
  static float altAvgBuff[30] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  static unsigned long baroTimeBuff[30] = {0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL,
                                    0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL,
                                    0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL};
  //if we are past apogee, use the maximum difference between samples in the buffer, otherwise set the difference to 10 samples
  if(events.apogee){baroVelPosn = altAvgPosn;}
  else{
    baroVelPosn = altAvgPosn - 10;
    if(baroVelPosn < 0){baroVelPosn = (int)((sizeof(altAvgBuff)/sizeof(altAvgBuff[0])) - (10 - altAvgPosn));}}
  //calculate the barometric derived velocity from the moving averages
  baro.Vel = (baro.smoothAlt - altAvgBuff[baroVelPosn])/((float)(baro.timeLastSamp - baroTimeBuff[baroVelPosn])*mlnth);
  //update variables
  if(baro.Vel > baro.maxVel){baro.maxVel = baro.Vel;}
  radio.vel = baro.Vel;
  radio.alt = baro.smoothAlt;
  altAvgBuff[altAvgPosn] = baro.smoothAlt;
  baroTimeBuff[altAvgPosn] = baro.timeLastSamp;
  altAvgPosn++;
  if(altAvgPosn >= (byte)(sizeof(altAvgBuff)/sizeof(altAvgBuff[0]))){altAvgPosn = 0;}
  if(events.apogeeFire || settings.testMode){fusionVel = baro.Vel; fusionAlt = baro.smoothAlt;}

  //----------------------------
  //Barometric touchdown trigger
  //----------------------------
  (events.mainDeploy && fabs(baro.Vel) < 1.0F ) ? baroTouchdown++ : baroTouchdown = 0;}

// the real, unmodified event logic
#include "../../Main Code/Event_Logic.ino"

//---------------------------------------------------------------------------
// Harness
//---------------------------------------------------------------------------

static std::ofstream eventsOut;
static double nowSeconds = 0.0;  // time since the start of the file

void logAction(const char* kind, const std::string& name, const std::string& detail) {
  double flightSeconds = fltTime.liftoff ? fltTime.timeCurrent * 1e-6 : 0.0;
  eventsOut << nowSeconds << ',' << flightSeconds << ',' << kind << ',' << name << ',' << detail << '\n';
  std::printf("%9.3f s  (flight %7.3f s)  %-10s %s %s\n", nowSeconds, flightSeconds, kind, name.c_str(), detail.c_str());
}

void digitalWrite(uint8_t pin, uint8_t value) {
  static const std::map<char, std::string> functions = {
      {'A', "apogee"}, {'M', "main"}, {'B', "booster separation"}, {'I', "sustainer ignition"},
      {'1', "airstart 1"}, {'2', "airstart 2"}, {'N', "none"}};
  pyroMap* pyros[] = {&pyro1, &pyro2, &pyro3, &pyro4};
  for (int i = 0; i < 4; i++) {
    if (pyros[i]->firePin == pin) {
      std::string detail = "pyro" + std::to_string(i + 1) + " (" + functions.at(pyros[i]->func) + ")";
      logAction("PYRO", value == HIGH ? "fire" : "off", detail);
      return;
    }
  }
}

static const char* radioEventName(uint8_t code) {
  static const char* names[] = {
      "Preflight", "Liftoff", "Booster_Burnout", "Apogee", "Fire_Apogee", "Separation", "Fire_Mains",
      "Under_Chute", "Eject_Booster", "Fire_Sustainer", "Sustainer_Ignition", "Sustainer_Burnout",
      "Fire_Airstart1", "Airstart1_Ignition", "Airstart1_Burnout", "Fire_Airstart2", "Airstart2_Ignition",
      "Airstart2_Burnout", "NoFire_Rotn_Limit", "NoFire_Alt_Limit", "NoFire_RotnAlt_Limit", "Booster_Apogee",
      "Fire_Booster_Apogee", "Booster_Separation", "Fire_Booster_Mains", "Booster_Under_Chute", "Time_Limit",
      "Touchdown", "Power_Loss_Restart", "Booster_Touchdown", "Booster_Preflight", "Booster_Time_Limit",
      "Booster_Power_Loss"};
  return code < sizeof(names) / sizeof(names[0]) ? names[code] : "unknown";
}

// every flag in the events struct, so changes can be logged
static std::vector<std::pair<const char*, bool*>> eventFlags() {
  return {
      {"preLiftoff", &events.preLiftoff}, {"inFlight", &events.inFlight}, {"postFlight", &events.postFlight},
      {"liftoff", &events.liftoff}, {"falseLiftoffCheck", &events.falseLiftoffCheck},
      {"boosterBurnout", &events.boosterBurnout}, {"boosterBurnoutCheck", &events.boosterBurnoutCheck},
      {"boosterSeparation", &events.boosterSeparation}, {"sustainerFireCheck", &events.sustainerFireCheck},
      {"sustainerFire", &events.sustainerFire}, {"sustainerIgnition", &events.sustainerIgnition},
      {"sustainerBurnout", &events.sustainerBurnout}, {"airStart1Check", &events.airStart1Check},
      {"airStart1Fire", &events.airStart1Fire}, {"airStart1Ignition", &events.airStart1Ignition},
      {"airStart1BurnoutCheck", &events.airStart1BurnoutCheck}, {"airStart1Burnout", &events.airStart1Burnout},
      {"airStart2Check", &events.airStart2Check}, {"airStart2Fire", &events.airStart2Fire},
      {"airStart2Ignition", &events.airStart2Ignition}, {"airStart2BurnoutCheck", &events.airStart2BurnoutCheck},
      {"airStart2Burnout", &events.airStart2Burnout}, {"apogee", &events.apogee},
      {"apogeeFire", &events.apogeeFire}, {"apogeeSeparation", &events.apogeeSeparation},
      {"mainDeploy", &events.mainDeploy}, {"touchdown", &events.touchdown}, {"timeOut", &events.timeOut}};
}

static void logChanges(const eventList& before, uint8_t radioBefore) {
  eventList copy = before;
  auto now = eventFlags();
  // point the same field list at the saved copy to compare field by field
  auto old = now;
  for (auto& f : old) f.second = reinterpret_cast<bool*>(reinterpret_cast<char*>(&copy) +
                                                        (reinterpret_cast<char*>(f.second) - reinterpret_cast<char*>(&events)));
  for (size_t i = 0; i < now.size(); i++) {
    if (*now[i].second != *old[i].second) logAction("FLAG", now[i].first, *now[i].second ? "true" : "false");
  }
  if (radio.event != radioBefore) logAction("RADIO_EVENT", radioEventName(radio.event), std::to_string(radio.event));
}

struct Options {
  std::string input;
  char profile = 'S';
  double baroHz = 25.0;
  double mainDeployFt = 700.0;    // Main Code/Settings.txt defaults
  double apogeeDelayS = 0.0;
  double altThresholdFt = 400.0;
  int maxAngleDeg = 20;
  int countsPerG = 2048;          // 16 g accelerometer, 16-bit output
};

static void usage() {
  std::cerr << "usage: harness <sensors.csv> [--profile S|2] [--baro-hz 25] [--main-ft 700]\n"
               "                [--apogee-delay 0] [--alt-threshold-ft 400] [--max-angle 20]\n";
  std::exit(1);
}

static Options parseArgs(int argc, char** argv) {
  Options o;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto next = [&]() -> std::string { if (i + 1 >= argc) usage(); return argv[++i]; };
    if (a == "--profile") o.profile = next()[0];
    else if (a == "--baro-hz") o.baroHz = std::stod(next());
    else if (a == "--main-ft") o.mainDeployFt = std::stod(next());
    else if (a == "--apogee-delay") o.apogeeDelayS = std::stod(next());
    else if (a == "--alt-threshold-ft") o.altThresholdFt = std::stod(next());
    else if (a == "--max-angle") o.maxAngleDeg = std::stoi(next());
    else if (a[0] == '-') usage();
    else o.input = a;
  }
  if (o.input.empty() || (o.profile != 'S' && o.profile != '2')) usage();
  return o;
}

// stands in for setup(): sensor scaling and the user settings from Settings.txt
static void configure(const Options& o) {
  settings.testMode = false;
  settings.fltProfile = o.profile;
  settings.mainDeployAlt = (float)(o.mainDeployFt / unitConvert);
  settings.apogeeDelay = (unsigned long)(o.apogeeDelayS * 1000000UL);
  settings.rcdTime = 900000000UL;
  settings.fireTime = 500000UL;
  settings.inflightRecover = 0;
  settings.boosterSeparationDelay = 500000UL;
  settings.sustainerFireDelay = 500000UL;
  settings.altThreshold = (int)(o.altThresholdFt / unitConvert);
  settings.maxAngle = o.maxAngleDeg * 10;  // stored in tenths of a degree (SD.ino:317)
  if (o.profile == '2') {
    settings.pyro1Func = 'B'; settings.pyro2Func = 'I'; settings.pyro3Func = 'A'; settings.pyro4Func = 'M';
  } else {
    settings.pyro1Func = 'N'; settings.pyro2Func = 'N'; settings.pyro3Func = 'A'; settings.pyro4Func = 'M';
  }
  // HPR_Rocket_Flight_PC_V4_7_2.ino:469-472
  pins.pyro1Fire = 1; pins.pyro2Fire = 2; pins.pyro3Fire = 3; pins.pyro4Fire = 4;
  pyro1.func = settings.pyro1Func; pyro1.firePin = pins.pyro1Fire;
  pyro2.func = settings.pyro2Func; pyro2.firePin = pins.pyro2Fire;
  pyro3.func = settings.pyro3Func; pyro3.firePin = pins.pyro3Fire;
  pyro4.func = settings.pyro4Func; pyro4.firePin = pins.pyro4Fire;

  // HPR_Rocket_Flight_PC_V4_7_2.ino:377-383
  accel.gainZ = 1.0f / o.countsPerG;
  g = (int16_t)(1.0 / accel.gainZ);
  accel.gainZ *= 9.80665;
  gTrigger = 2.5 * g;
  accel.ADCmax = (int)(0.98 * 32768);
  sensors.highG = 0;
}

struct Sample {
  uint32_t timeUs;
  double accelMps2, pressureMbar, offVertDeg;
};

static std::vector<Sample> readSensors(const std::string& path) {
  std::ifstream in(path);
  if (!in) { std::cerr << "cannot open " << path << '\n'; std::exit(1); }
  std::string line;
  std::getline(in, line);
  std::vector<std::string> header;
  { std::stringstream ss(line); std::string h; while (std::getline(ss, h, ',')) header.push_back(h); }
  auto col = [&](const char* name) {
    for (size_t i = 0; i < header.size(); i++) if (header[i] == name) return (int)i;
    std::cerr << "missing column " << name << '\n'; std::exit(1);
  };
  int cTime = col("time_us"), cAccel = col("accel_z_mps2"), cPress = col("baro_pressure_mbar"), cOff = col("off_vert_deg");

  std::vector<Sample> samples;
  while (std::getline(in, line)) {
    std::vector<std::string> f;
    std::stringstream ss(line); std::string v;
    while (std::getline(ss, v, ',')) f.push_back(v);
    samples.push_back({(uint32_t)std::stoul(f[cTime]), std::stod(f[cAccel]), std::stod(f[cPress]), std::stod(f[cOff])});
  }
  return samples;
}

int main(int argc, char** argv) {
  Options o = parseArgs(argc, argv);
  configure(o);
  std::vector<Sample> samples = readSensors(o.input);

  std::string base = o.input.substr(0, o.input.rfind(".csv"));
  eventsOut.open(base + "_cpp_events.csv");
  eventsOut << "time_s,flight_time_s,kind,name,detail\n";
  std::ofstream traceOut(base + "_cpp_trace.csv");
  traceOut << "time_s,flight_time_s,accel_z_counts,accelNow,accelVel,accelAlt,fusionVel,fusionAlt,"
              "baro_Alt,baro_Vel,offVert,altOK,rotnOK,radio_event\n";

  const uint32_t baroInterval = (uint32_t)(1e6 / o.baroHz);
  // micros() has been counting through setup() by the time loop() starts (startupTime = 10 s in Settings.txt).
  // Starting at 0 instead makes the first barometric velocity divide by zero.
  const uint32_t bootUs = 10000000UL;
  uint32_t nextBaro = 0;

  for (const Sample& s : samples) {
    nowSeconds = s.timeUs * 1e-6;
    eventList before = events;
    uint8_t radioBefore = radio.event;
    const uint32_t micros = s.timeUs + bootUs;

    // sensor reads (replaces getAccel/getBaro and the sampling in loop() :1048-1085)
    fltTime.tmClock = micros;
    long counts = std::lround(s.accelMps2 / 9.80665 * g);
    accel.z = (int16_t)std::max(-32768L, std::min(32767L, counts));
    if (micros >= nextBaro) {
      baro.pressure = (float)s.pressureMbar;
      baro.rawAlt = 44330 * (1.0 - pow(baro.pressure / (baro.seaLevelPressure), 0.1903));  // Device_Drivers.ino:2065
      baro.newSamp = true;
      baro.timeLastSamp = micros;
      nextBaro = micros + baroInterval;
    }

    // HPR_Rocket_Flight_PC_V4_7_2.ino:1096
    if(baro.newSamp){processBaroSamp();}

    // detect liftoff, HPR_Rocket_Flight_PC_V4_7_2.ino:1105-1116 (radio and GPS bookkeeping left out)
    if (!events.liftoff && accel.z > gTrigger && !events.touchdown && !events.timeOut) {
      fltTime.padTime = fltTime.tmClockPrev = fltTime.tmClock;
      fltTime.liftoff = fltTime.tmClock;
      events.preLiftoff = false;
      events.liftoff = true;
      events.inFlight = true;
      radio.event = Liftoff;
      radio.alt = 0;
      radioTimer.begin(timerBuildPkt, pktInterval.inflight/pktInterval.samplesPerPkt);
    }

    if (events.liftoff) {
      // HPR_Rocket_Flight_PC_V4_7_2.ino:1144-1146
      fltTime.dt = fltTime.tmClock - fltTime.tmClockPrev;
      fltTime.timeCurrent += fltTime.dt;
      fltTime.tmClockPrev = fltTime.tmClock;

      // HPR_Rocket_Flight_PC_V4_7_2.ino:1160-1162
      if(abs(accel.z) < accel.ADCmax){accelNow = (float)(accel.z - g) * accel.gainZ;}
      else if(sensors.highG != 0){accelNow = (float)(highG.z - high1G) *  highG.gainZ;}
      else{accelNow = (float)(accel.z - g) * accel.gainZ;}

      // HPR_Rocket_Flight_PC_V4_7_2.ino:1165-1194
      if(!events.apogeeFire){
        if(accelNow > maxG){maxG = accelNow;}
        accelVel += accelNow * (float)fltTime.dt * mlnth;
        accelAlt += accelVel * (float)fltTime.dt * mlnth;
        fusionVel += accelNow * (float)fltTime.dt * mlnth;
        if(baro.newSamp && baro.Vel < 300 && baro.Vel < baro.maxVel && accelNow < 0.2F && fusionVel < 300.0F && baro.Alt < 13000){
          fusionVel *= 0.99F;
          fusionVel += 0.01F * baro.Vel;}
        if(fusionVel > maxVelocity){maxVelocity = fusionVel;}
        fusionAlt += fusionVel * (float)fltTime.dt * mlnth;
        if(baro.newSamp &&  baro.Alt < 9000){
          fusionAlt *= 0.95;
          fusionAlt += 0.05 * baro.Alt;}
        if(!altOK && (fusionAlt > settings.altThreshold || settings.testMode)){altOK = true;}
      }

      // rotation: HPR integrates the gyro (Rotation.ino); the harness uses the simulated tilt instead.
      // offVert is in tenths of a degree, same as Rotation.ino:120-126
      offVert = (int)std::lround(s.offVertDeg * 10.0);
      rotnOK = (!rotationFault && offVert < settings.maxAngle) ? true : false;
      if (!rotationFault && offVert > 450){rotationFault = true; rotnOK = false;}

      // HPR_Rocket_Flight_PC_V4_7_2.ino:1215
      checkEvents();

      // the SD write clears the new-sample flag (SD.ino:419)
      baro.newSamp = false;

      // HPR_Rocket_Flight_PC_V4_7_2.ino:1241-1245: end of flight
      if (events.timeOut || events.touchdown) {
        events.liftoff = false;
        logChanges(before, radioBefore);
        break;
      }
    }

    logChanges(before, radioBefore);
    traceOut << nowSeconds << ',' << (fltTime.liftoff ? fltTime.timeCurrent * 1e-6 : 0.0) << ',' << accel.z << ','
             << accelNow << ',' << accelVel << ',' << accelAlt << ',' << fusionVel << ',' << fusionAlt << ','
             << baro.Alt << ',' << baro.Vel << ',' << offVert << ',' << altOK << ',' << rotnOK << ','
             << (int)radio.event << '\n';
  }

  std::printf("\nMax baro altitude: %.1f m (%.0f ft), max velocity: %.1f m/s\n", baro.maxAlt, baro.maxAlt * unitConvert, maxVelocity);
  std::printf("Wrote %s_cpp_events.csv and %s_cpp_trace.csv\n", base.c_str(), base.c_str());
  return 0;
}
