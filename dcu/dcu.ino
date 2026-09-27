/*
  Driver Control Unit (DCU)

  Hardware defaults (adjust to match the actual build):
    - Arduino with the same CAN library/interface used by the PCU and MCU
    - 20x4 I2C LCD, address 0x27
    - 4x4 matrix keypad, rows on pins 2-5 and columns on pins 6-9

  Keypad flow:
    1       Select driving program 1 (only program currently available)
    #       Start a trip from home (PCU) toward the MCU
    *       Return from the MCU to the PCU
    A       End the session and request return to the PCU

  CAN protocol is documented in moose_propulsion.dbc and paired with the
  MCTL_DCU/MSTA_PCU handling in pcu/pcu.ino.
*/

#include <CAN.h>
#include <SPI.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <stdio.h>
#include <string.h>
#include "j1939_address.h"

// ========================== HARDWARE CONFIGURATION ==========================
const uint8_t LCD_I2C_ADDRESS = 0x27;
const uint8_t LCD_COLUMNS = 20;
const uint8_t LCD_ROWS = 4;
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, LCD_COLUMNS, LCD_ROWS);

const byte KEYPAD_ROWS = 4;
const byte KEYPAD_COLS = 4;
char keypadMap[KEYPAD_ROWS][KEYPAD_COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte rowPins[KEYPAD_ROWS] = {2, 3, 4, 5};
byte colPins[KEYPAD_COLS] = {6, 7, 8, 9};
Keypad keypad = Keypad(makeKeymap(keypadMap), rowPins, colPins,
                       KEYPAD_ROWS, KEYPAD_COLS);

// ========================== CAN PROTOCOL ====================================
const uint32_t CAN_BAUD_RATE = 500E3;
// Provisional local addresses and project-specific J1939 PGNs.
const uint8_t J1939_SA_DCU = 0xA0;
const uint8_t J1939_SA_PCU = 0xA1;
const uint32_t J1939_PGN_MCTL_DCU = 0x00EF00;
const uint32_t J1939_PGN_MSTA_PCU = 0x00FF01;
J1939AddressClaim j1939Node;
const unsigned long COMMAND_PERIOD_MS = 100;
const unsigned long STATUS_TIMEOUT_MS = 500;

enum DcuCommand : uint8_t {
  COMMAND_HOLD = 0,
  COMMAND_HOME = 1,
  COMMAND_TO_MCU = 2,
  COMMAND_TO_PCU = 3
};

enum PcuState : uint8_t {
  PCU_BOOT_WAIT = 0,
  PCU_WAIT_FOR_LIDAR = 1,
  PCU_HOMING = 2,
  PCU_AT_HOME = 3,
  PCU_TO_MCU = 4,
  PCU_AT_MCU = 5,
  PCU_TO_HOME = 6,
  PCU_FAULT = 7
};

// ========================== DRIVE PROGRAMS =================================
struct DriveProgram {
  uint8_t id;
  const char* name;
};

// Add programs here as PCU drive profiles are implemented.
const DriveProgram programs[] = {
  {1, "Standard"}
};
const uint8_t PROGRAM_COUNT = sizeof(programs) / sizeof(programs[0]);

// ========================== RUNTIME STATE ===================================
bool programSelected = false;
uint8_t selectedProgram = 0;
uint8_t desiredCommand = COMMAND_HOLD;
uint8_t commandSequence = 0;
bool sessionEnding = false;

uint8_t pcuState = PCU_BOOT_WAIT;
uint16_t pcuDistanceCm = 0;
int16_t pcuSpeedCmPerSecond = 0;
bool pcuStatusReceived = false;
unsigned long lastPcuStatusMs = 0;
unsigned long lastCommandTxMs = 0;
unsigned long lastDisplayUpdateMs = 0;
char lastDisplayLine1[21] = "";
char lastDisplayLine2[21] = "";
char lastDisplayLine3[21] = "";
char lastDisplayLine4[21] = "";

void setCommand(uint8_t command) {
  if (desiredCommand != command) {
    desiredCommand = command;
    commandSequence++;
  }
}

void selectProgram(uint8_t programId) {
  for (uint8_t i = 0; i < PROGRAM_COUNT; i++) {
    if (programs[i].id == programId) {
      selectedProgram = programId;
      programSelected = true;
      sessionEnding = false;
      setCommand(COMMAND_HOME);
      return;
    }
  }
}

void processKey(char key) {
  if (!programSelected) {
    if (key >= '1' && key <= '9') {
      selectProgram((uint8_t)(key - '0'));
    }
    return;
  }

  // A always requests a controlled return to the PCU once a program is chosen.
  if (key == 'A') {
    sessionEnding = true;
    setCommand(COMMAND_HOME);
    return;
  }

  const bool statusFresh = pcuStatusReceived &&
      (millis() - lastPcuStatusMs <= STATUS_TIMEOUT_MS);
  if (!statusFresh) return;

  if (key == '#' && pcuState == PCU_AT_HOME) {
    sessionEnding = false;
    setCommand(COMMAND_TO_MCU);
  } else if (key == '*' && pcuState == PCU_AT_MCU) {
    setCommand(COMMAND_TO_PCU);
  }
}

void pollCan() {
  const int packetSize = CAN.parsePacket();
  if (packetSize <= 0) return;

  const uint32_t packetId = (uint32_t)CAN.packetId();
  const bool extendedFrame = CAN.packetExtended();
  uint8_t payload[8];
  int bytesRead = 0;
  while (CAN.available()) {
    const int value = CAN.read();
    if (bytesRead < (int)sizeof(payload)) payload[bytesRead] = (uint8_t)value;
    bytesRead++;
  }

  j1939Node.handleFrame(packetId, extendedFrame, payload, bytesRead);
  if (!j1939Node.mayTransmitApplication()) return;

  const uint32_t pgn = j1939GetPgn(packetId);
  const uint8_t sourceAddress = (uint8_t)(packetId & 0xFF);
  if (!extendedFrame || pgn != J1939_PGN_MSTA_PCU ||
      sourceAddress != J1939_SA_PCU || packetSize != 8 ||
      bytesRead != 8 || payload[0] > PCU_FAULT) return;

  pcuState = payload[0];
  pcuDistanceCm = (uint16_t)payload[1] | ((uint16_t)payload[2] << 8);
  const uint16_t speedRaw = (uint16_t)payload[3] | ((uint16_t)payload[4] << 8);
  const int32_t speedSigned = speedRaw <= 32767
      ? (int32_t)speedRaw
      : (int32_t)speedRaw - 65536L;
  pcuSpeedCmPerSecond = (int16_t)speedSigned;
  pcuStatusReceived = true;
  lastPcuStatusMs = millis();
}

void sendCommandHeartbeat() {
  const unsigned long now = millis();
  if (now - lastCommandTxMs < COMMAND_PERIOD_MS) return;
  lastCommandTxMs = now;
  if (!j1939Node.mayTransmitApplication()) return;

  const uint32_t id = j1939MakeId(3, J1939_PGN_MCTL_DCU,
                                  J1939_SA_PCU, j1939Node.address());
  CAN.beginExtendedPacket(id);
  CAN.write(desiredCommand);
  CAN.write(selectedProgram);
  CAN.write(commandSequence);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.endPacket();
}

void copyDisplayLine(char destination[21], const char* source) {
  uint8_t i = 0;
  while (i < 20 && source[i] != '\0') {
    destination[i] = source[i];
    i++;
  }
  while (i < 20) destination[i++] = ' ';
  destination[20] = '\0';
}

void showLines(const char* line1, const char* line2,
               const char* line3 = "", const char* line4 = "") {
  char paddedLine1[21];
  char paddedLine2[21];
  char paddedLine3[21];
  char paddedLine4[21];
  copyDisplayLine(paddedLine1, line1);
  copyDisplayLine(paddedLine2, line2);
  copyDisplayLine(paddedLine3, line3);
  copyDisplayLine(paddedLine4, line4);

  if (strcmp(paddedLine1, lastDisplayLine1) == 0 &&
      strcmp(paddedLine2, lastDisplayLine2) == 0 &&
      strcmp(paddedLine3, lastDisplayLine3) == 0 &&
      strcmp(paddedLine4, lastDisplayLine4) == 0) return;

  strcpy(lastDisplayLine1, paddedLine1);
  strcpy(lastDisplayLine2, paddedLine2);
  strcpy(lastDisplayLine3, paddedLine3);
  strcpy(lastDisplayLine4, paddedLine4);
  lcd.setCursor(0, 0);
  lcd.print(paddedLine1);
  lcd.setCursor(0, 1);
  lcd.print(paddedLine2);
  lcd.setCursor(0, 2);
  lcd.print(paddedLine3);
  lcd.setCursor(0, 3);
  lcd.print(paddedLine4);
}

void showTravelScreen(const char* title, const char* prompt) {
  char distanceLine[21];
  char speedLine[21];
  snprintf(distanceLine, sizeof(distanceLine), "PCU range: %u cm",
           (unsigned int)pcuDistanceCm);
  snprintf(speedLine, sizeof(speedLine), "PCU speed: %d cm/s",
           (int)pcuSpeedCmPerSecond);
  showLines(title, distanceLine, speedLine, prompt);
}

void updateDisplay() {
  const unsigned long now = millis();
  if (now - lastDisplayUpdateMs < 200) return;
  lastDisplayUpdateMs = now;

  if (!programSelected) {
    showLines("Select drive program", "1: Standard", "", "");
    return;
  }

  if (!pcuStatusReceived || now - lastPcuStatusMs > STATUS_TIMEOUT_MS) {
    showLines("Waiting for PCU", "Check CAN status", "", "");
    return;
  }

  switch (pcuState) {
    case PCU_BOOT_WAIT:
    case PCU_WAIT_FOR_LIDAR:
      showLines("Checking lidar", "Please wait...", "", "");
      break;
    case PCU_HOMING:
      showTravelScreen("Returning home", "A: end session");
      break;
    case PCU_AT_HOME:
      if (sessionEnding) showLines("Session ended", "Target is at home", "", "#: start again");
      else showLines("Target is at home", "#: travel to MCU", "", "A: end session");
      break;
    case PCU_TO_MCU:
      showTravelScreen("Moving to MCU", "A: return home");
      break;
    case PCU_AT_MCU:
      showLines("Stopped at MCU", "*: return to PCU", "", "A: end session");
      break;
    case PCU_TO_HOME:
      showTravelScreen("Returning home", "A: end session");
      break;
    case PCU_FAULT:
    default:
      showLines("PCU fault", "Check target/system", "", "A: request home");
      break;
  }
}

void setup() {
  Serial.begin(115200);

  Wire.begin();
  lcd.init();
  lcd.backlight();
  showLines("Moose range", "Starting CAN...");

  if (!CAN.begin(CAN_BAUD_RATE)) {
    showLines("CAN start failed", "Check interface");
    while (true) { }
  }

  // Identity 1 is unique within this project. NAME contains a test-only
  // manufacturer-code placeholder until one is assigned by SAE.
  j1939Node.begin(J1939_SA_DCU, j1939MakeTestName(1));

  commandSequence = 0;
  Serial.println("DCU ready");
}

void loop() {
  pollCan();

  const char key = keypad.getKey();
  if (key != NO_KEY) processKey(key);

  sendCommandHeartbeat();
  updateDisplay();
}
