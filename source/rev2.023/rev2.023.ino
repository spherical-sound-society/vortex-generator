/*
   CONNECIONS
  Arduino CLK (pin D13) -> DAC SCK (pin 3)
  Arduino MOSI (pin D11) -> DAC SDI (pin 4)
  Arduino CS (pin D10) -> DAC CS (pin2)
  >>>>>>>Arduino pin D7 -> DAC LATCH (pin 5)>GND
  Arduino 5V -> DAC VDD (pin 1)
  Arduino GND -> DAC VSS (pin 7)
*/
// MCPDAC relies on SPI.
#include <SPI.h>
#include <MCPDAC.h>
#include <digitalWriteFast.h>
#include "avdweb_AnalogReadFast.h"



float expoTable[128] = {1.0, 1.0536326, 1.1101418, 1.1696815, 1.2324146, 1.2985122, 1.3681549, 1.4415325, 1.5188457, 1.6003054, 1.686134, 1.7765658, 1.8718476, 1.9722397, 2.078016, 2.1894655, 2.3068924, 2.430617, 2.5609775, 2.6983292, 2.8430479, 2.995528, 3.1561859, 3.3254602, 3.5038135, 3.6917324, 3.8897297, 4.0983458, 4.318151, 4.5497446, 4.793759, 5.050861, 5.321752, 5.607172, 5.9078994, 6.224755, 6.558605, 6.9103603, 7.2809806, 7.6714787, 8.08292, 8.516429, 8.973187, 9.454442, 9.961509, 10.495771, 11.058686, 11.651794, 12.27671, 12.93514, 13.628888, 14.359839, 15.129997, 15.941458, 16.796438, 17.697277, 18.646427, 19.646482, 20.700178, 21.81038, 22.980125, 24.212614, 25.511198, 26.879435, 28.321047, 29.839975, 31.440376, 33.126602, 34.903275, 36.775227, 38.747574, 40.825714, 43.0153, 45.32232, 47.753082, 50.314198, 53.012676, 55.855892, 58.851585, 62.00796, 65.3336, 68.83761, 72.529564, 76.4195, 80.5181, 84.83649, 89.38648, 94.18053, 99.23167, 104.55371, 110.16122, 116.06947, 122.29453, 128.85353, 135.76431, 143.04565, 150.71759, 158.801, 167.31784, 176.29156, 185.74658, 195.70859, 206.20499, 217.26433, 228.91682, 241.19414, 254.13007, 267.75977, 282.12033, 297.25122, 313.19363, 329.9909, 347.68924, 366.3368, 385.98425, 406.68567, 428.49738, 451.47864, 475.6927, 501.2054, 528.0862, 556.40894, 586.2507, 617.69293, 650.8212, 685.72656, 722.504, 761.2535
                       };
float logTable[128] = {
  796.8609, 756.29846, 717.80084, 681.2632, 646.585, 613.67206, 582.43475, 552.78723, 524.64886, 497.94278, 472.5964, 448.53995, 425.70804, 404.03854, 383.47186, 363.9521, 345.42615, 327.843, 311.15488, 295.3164, 280.284, 266.01678, 252.47592, 239.6242, 227.42668, 215.85004, 204.8628, 194.43471, 184.53745, 175.14409, 166.22878, 157.76727, 149.73657, 142.11456, 134.88054, 128.01482, 121.49852, 115.31391, 109.444145, 103.87317, 98.58574, 93.567474, 88.804665, 84.28426, 79.99398, 75.922066, 72.05745, 68.38954, 64.908325, 61.604332, 58.46852, 55.492313, 52.667614, 49.98669, 47.44224, 45.027313, 42.735302, 40.559967, 38.495365, 36.535847, 34.67608, 32.910973, 31.235722, 29.645746, 28.136696, 26.704468, 25.345144, 24.055008, 22.830547, 21.66841, 20.565434, 19.518602, 18.525053, 17.58208, 16.687109, 15.837689, 15.031512, 14.266367, 13.540174, 12.8509445, 12.196796, 11.57595, 10.986705, 10.427451, 9.896668, 9.392901, 8.914779, 8.460993, 8.030308, 7.621544, 7.2335873, 6.865379, 6.515914, 6.184237, 5.869443, 5.570673, 5.287112, 5.0179844, 4.7625556, 4.520129, 4.2900434, 4.071669, 3.8644104, 3.6677017, 3.4810066, 3.3038144, 3.1356416, 2.9760294, 2.8245418, 2.6807654, 2.5443075, 2.4147956, 2.2918763, 2.1752138, 2.0644898, 1.9594021, 1.8596634, 1.7650018, 1.6751586, 1.5898887, 1.5089593, 1.4321493, 1.3592492, 1.2900599, 1.2243925, 1.1620678, 1.1029155, 1.0467743
};

//analogic pins
#define A_AttackPin           A0
#define A_DecayPin            A1
#define B_AttackPin           A2
#define B_DecayPin            A3
#define ATT_A_CV_Pin          A4
#define DEC_A_CV_Pin          A5
#define ATT_B_CV_Pin          A6
#define DEC_B_CV_Pin          A7
#define A_GateInPin           2
#define B_GateInPin           3
#define A_EOC_Pin             9
#define B_EOC_Pin             12
#define RunglerOutA_Pin       5
#define RunglerOutB_Pin       6
#define RunglerOutC_Pin       8
#define Th4Clock_Pin     1
#define isChannelAInLoop_Pin  4
#define isChannelBInLoop_Pin  7
#define debounceRange         20
#define isDebugging   false // define vs bool
int lengthFactor = 3;

bool isBInterrupt = true;
bool isAInterrupt = true;


int A_Attack_Value  = 512;
int A_Decay_Value  = 512;
int ATT_A_CV = 512;
int DEC_A_CV = 512;
int ATT_B_CV = 512;
int DEC_B_CV = 512;
int B_Attack_Value  = 512;
int B_Decay_Value  = 512;
int B_Curve_Value  = 512;
int mixedAAttack = 512;
int mixedADecay = 512;
int mixedBAttack = 512;
int mixedBDecay = 512;

long A_OutputVoltage = 0;
long B_OutputVoltage = 0;
long A_EOC_millis = 0;
long B_EOC_millis = 0;
int A_State = 0;
int B_State = 0;
bool isLooperAOn = false;
bool isLooperBOn = false;
bool isAEOC = false;
bool isBEOC = false;

bool channelAHysteresys = false;
bool channelBHysteresys = false;

byte rungleRegister = 0;
byte th4Counter = 0;


//STATES 0=off, 1=attack phase, 2=decay phase
void setup()
{
  attachInterrupt(digitalPinToInterrupt(A_GateInPin), gateAInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(B_GateInPin), gateBInterrupt, RISING);
  // CS on pin 10, no LDAC pin (tie it to ground).
  MCPDAC.begin(10);

  // Set the gain to "HIGH" mode - 0 to 4096mV.
  MCPDAC.setGain(CHANNEL_A, GAIN_HIGH);
  MCPDAC.setGain(CHANNEL_B, GAIN_HIGH);

  // Do not shut down channels
  MCPDAC.shutdown(CHANNEL_A, false);
  MCPDAC.shutdown(CHANNEL_B, false);

  pinMode(A_EOC_Pin, OUTPUT);
  pinMode(B_EOC_Pin, OUTPUT);
  pinMode(RunglerOutA_Pin, OUTPUT);
  pinMode(RunglerOutB_Pin, OUTPUT);
  pinMode(RunglerOutC_Pin, OUTPUT);
  pinMode(A_GateInPin, INPUT);
  pinMode(B_GateInPin, INPUT);
  pinMode(isChannelAInLoop_Pin, INPUT_PULLUP);
  pinMode(isChannelBInLoop_Pin, INPUT_PULLUP);
  pinMode(Th4Clock_Pin, OUTPUT);



  if (isDebugging) {
    Serial.begin(9600);
  }
  bool isBInterrupt = true;
  bool isAInterrupt = true;
}

void gateAInterrupt() {
  isAInterrupt = true;
   Serial.println(">>> gateAInterrupt:");
}
void gateBInterrupt() {
  isBInterrupt = true;
   Serial.println(">>> gateBInterrupt:");
}

void loop()
{

  isLooperAOn = digitalReadFast(isChannelAInLoop_Pin);
  isLooperBOn = digitalReadFast(isChannelBInLoop_Pin);
  /*
    if (digitalReadFast(A_GateInPin)) {
      gateAInterrupt();
    }
    if (digitalReadFast(B_GateInPin)) {
      gateBInterrupt();
    }
  */
  // isLooperAOn = true;
  //8isLooperBOn = true;ç

  //Serial.print(" isLooperAOn:");
  //Serial.print(isLooperAOn);
  //Serial.print(" isLooperBOn:");
  //Serial.println(isLooperBOn);

  // check interruptions
  readAPots();
  readBPots();


  if (isAInterrupt) {
    A_State = 1;
    isAInterrupt = false;
  }
  if (isBInterrupt) {
    B_State = 1;
    isBInterrupt = false;
  }


  envelopeManager();
  //Serial.print(" A_OutputVoltage:");
  // Serial.println(A_OutputVoltage);

  //POSIBLEMENTE AQUI VA EL CODIGO DE CHANNEL B HYSTERESIS ue ahora esta dentro de rungler()

  //TODO 16384 esta hardcoded. se ha de calcular con respecto al lenghtfactor
  if (A_OutputVoltage > 1024 && channelAHysteresys) {
    rungler();
    channelAHysteresys = false;
  }
  if (A_OutputVoltage < 1024) {
    channelAHysteresys = true;
  }


  // Set the voltages from 0 to 4095
  MCPDAC.setVoltage(CHANNEL_A, A_OutputVoltage >> lengthFactor); //lenghtfactor no permite division >> con doubles
  MCPDAC.setVoltage(CHANNEL_B, B_OutputVoltage >> lengthFactor);

}
void envelopeManager() {
  //STATES 0=off, 1=attack phase, 2=decay phase

  if (A_State == 1) {
    A_OutputVoltage += mixedAAttack << lengthFactor;

    if (A_OutputVoltage >= 4095 << lengthFactor) {
      A_OutputVoltage = 4095 << lengthFactor;
      A_State = 2;
    }
  }

  if (isAEOC && (A_EOC_millis + 10 <= millis())) {
    isAEOC = false;
    digitalWriteFast( A_EOC_Pin, LOW);
  }
  if (A_State == 2) {
    A_OutputVoltage -= mixedADecay;
    if (A_OutputVoltage <= 0) {
      A_OutputVoltage = 0;
      A_State = 0;
      // TODO SEÑAL  EOC A
      A_EOC_millis = millis();
      isAEOC = true;
      digitalWriteFast( A_EOC_Pin, HIGH);
      if (isLooperAOn) {
        isAInterrupt = true;
      }
    }
  }
  //B CHANNEL

  if (B_State == 1) {
    B_OutputVoltage += mixedBAttack << lengthFactor;

    if (B_OutputVoltage >= 4095 << lengthFactor) {
      B_OutputVoltage = 4095 << lengthFactor;
      B_State = 2;
    }
  }
  if (isBEOC && (B_EOC_millis + 10 <= millis())) {
    isBEOC = false;
    digitalWriteFast( B_EOC_Pin, LOW);
  }

  if (B_State == 2) {
    B_OutputVoltage -= mixedBDecay;
    if (B_OutputVoltage <= 0) {
      B_OutputVoltage = 0;
      B_State = 0;
      // TODO SEÑAL  EOC B
      B_EOC_millis = millis();
      isBEOC = true;
      digitalWriteFast( B_EOC_Pin, HIGH);
      if (isLooperBOn) {
        isBInterrupt = true;
      }
    }
  }
}
void readAPots() {
  int value;
  //isAInterrupt = false;
  // TODO IMPLEMENTAR ANALOGREADFAST
  // SUMA LOS DOS DEBOUNCES, DIVIDE POR >>1 Y LUEGO CALCULA LOGTABLE
  A_Attack_Value = softDebounce(analogReadFast(A_AttackPin), A_Attack_Value);

  // A_Attack_Value = logTable[A_Attack_Value >> 3];

  ATT_A_CV = softDebounce(analogReadFast(ATT_A_CV_Pin), ATT_A_CV);
  //value = A_Attack_Value + (ATT_A_CV - 512);
  value = A_Attack_Value + ATT_A_CV;

  value = range(value);
  mixedAAttack = logTable[value >> 3];

  A_Decay_Value = softDebounce(analogReadFast(A_DecayPin), A_Decay_Value);
  DEC_A_CV = softDebounce(analogReadFast(DEC_A_CV_Pin), DEC_A_CV);
  //value = A_Decay_Value + (DEC_A_CV - 512);
  value = A_Decay_Value + DEC_A_CV;
  value = range(value);
  mixedADecay = logTable[value >> 3];

  if (isDebugging) {
    Serial.print("analogRead2: ");
    Serial.print(analogRead(2));

    Serial.print("   analogRead3: ");
    Serial.println(analogRead(3));

  }
}

int range(int val) {
  if (val < 0) {
    val = 0;
  }
  if (val > 1023) {
    val = 1023;
  }
  return (val);
}
void readBPots() {
  int value;
  //isAInterrupt = false;
  // TODO IMPLEMENTAR ANALOGREADFAST
  // SUMA LOS DOS DEBOUNCES, DIVIDE POR >>1 Y LUEGO CALCULA LOGTABLE
  B_Attack_Value = softDebounce(analogReadFast(B_AttackPin), B_Attack_Value);

  // A_Attack_Value = logTable[A_Attack_Value >> 3];

  ATT_B_CV = softDebounce(analogReadFast(ATT_B_CV_Pin), ATT_B_CV);
  //value = B_Attack_Value + (ATT_B_CV - 512);
  value = B_Attack_Value + ATT_B_CV;

  value = range(value);
  mixedBAttack = logTable[value >> 3];

  B_Decay_Value = softDebounce(analogReadFast(B_DecayPin), B_Decay_Value);
  DEC_B_CV = softDebounce(analogReadFast(DEC_B_CV_Pin), DEC_B_CV);
  //value = B_Decay_Value + (DEC_B_CV - 512);
  value = B_Decay_Value +DEC_B_CV;
  value = range(value);
  mixedBDecay = logTable[value >> 3];

  if (isDebugging) {
    Serial.print("analogRead2: ");
    Serial.print(analogRead(2));

    Serial.print("   analogRead3: ");
    Serial.println(analogRead(3));

  }

}


int  softDebounce(int  readCV, int  oldRead) {
  if (abs(readCV - oldRead) > debounceRange) {
    return readCV;
  }
  return oldRead;
}
