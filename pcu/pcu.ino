#include <CAN.h>
#include <SPI.h>
#include "j1939_address.h"

// Provisional local addresses and project-specific J1939 PGNs.
const uint8_t J1939_SA_DCU = 0xA0;
const uint8_t J1939_SA_PCU = 0xA1;
const uint8_t J1939_SA_MCU = 0xA2;
const uint32_t J1939_PGN_LMSD_MCU = 0x00FF00;
const uint32_t J1939_PGN_MCTL_DCU = 0x00EF00;
const uint32_t J1939_PGN_MSTA_PCU = 0x00FF01;
J1939AddressClaim j1939Node;
const unsigned long CAN_TIMEOUT_MS = 500;
const unsigned long STATUS_PERIOD_MS = 100;

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

// ========================== TFMINI LIDAR ===================================
typedef struct {
  int distance;
  int strength;
  int temp;
  boolean receiveComplete;
} TF;

TF Lidar = {0, 0, 0, false};
int previousLocalDistance = 0;
unsigned long previousLocalSampleMs = 0;
unsigned long lastLocalSampleMs = 0;
int localDistanceCm = 0;
int localSpeedCmPerSecond = 0;
bool haveLocalSample = false;

int remoteDistanceCm = 0;
int remoteSpeedCmPerSecond = 0;
unsigned long lastRemoteSampleMs = 0;
bool haveRemoteSample = false;

// ========================== MOTOR / SAFETY =================================
const int a1 = 7;
const int a2 = 8;
const int pa = 6;
const uint8_t MOTOR_PWM_PROGRAM_1 = 50;
const int END_DISTANCE_CM = 10;
const int SPEED_ZERO_THRESHOLD_CM_S = 1;
const unsigned long STOP_CONFIRM_MS = 1000;

uint8_t requestedCommand = COMMAND_HOLD;
uint8_t requestedProgram = 0;
uint8_t lastCommandSequence = 0;
unsigned long lastDcuCommandMs = 0;
bool haveDcuCommand = false;
uint8_t pcuState = PCU_BOOT_WAIT;
unsigned long stationaryStartMs = 0;
bool stationaryTimerRunning = false;
unsigned long lastStatusTxMs = 0;

void motor_forward_constant(uint8_t pwm) {
  digitalWrite(a1, HIGH);
  digitalWrite(a2, LOW);
  analogWrite(pa, pwm);
}

void motor_backward_constant(uint8_t pwm) {
  digitalWrite(a1, LOW);
  digitalWrite(a2, HIGH);
  analogWrite(pa, pwm);
}

void motor_brake() {
  digitalWrite(a1, LOW);
  digitalWrite(a2, LOW);
  analogWrite(pa, 0);
}

void resetStationaryTimer() {
  stationaryTimerRunning = false;
}

bool stoppedForOneSecond(int speedCmPerSecond) {
  if (abs(speedCmPerSecond) >= SPEED_ZERO_THRESHOLD_CM_S) {
    resetStationaryTimer();
    return false;
  }

  if (!stationaryTimerRunning) {
    stationaryStartMs = millis();
    stationaryTimerRunning = true;
  }
  return millis() - stationaryStartMs >= STOP_CONFIRM_MS;
}

void getLidarData(TF* lidar) {
  static uint8_t index = 0;
  static uint8_t rx[9];

  while (Serial.available()) {
    const uint8_t value = (uint8_t)Serial.read();
    if (index == 0 && value != 0x59) continue;
    if (index == 1 && value != 0x59) {
      index = 0;
      continue;
    }

    rx[index++] = value;
    if (index == sizeof(rx)) {
      uint8_t checksum = 0;
      for (uint8_t i = 0; i < 8; i++) checksum += rx[i];
      if (rx[8] == checksum) {
        const uint16_t distance = rx[2] | ((uint16_t)rx[3] << 8);
        const uint16_t strength = rx[4] | ((uint16_t)rx[5] << 8);
        if (distance > 0 && strength > 0) {
          lidar->distance = distance;
          lidar->strength = strength;
          lidar->temp = ((rx[6] | ((uint16_t)rx[7] << 8)) / 8) - 256;
          lidar->receiveComplete = true;
        }
      }
      index = 0;
    }
  }
}

void updateLocalLidar() {
  getLidarData(&Lidar);
  if (!Lidar.receiveComplete) return;
  Lidar.receiveComplete = false;

  const unsigned long now = millis();
  const int currentDistance = Lidar.distance;
  if (haveLocalSample && now > previousLocalSampleMs) {
    const unsigned long elapsedMs = now - previousLocalSampleMs;
    const long distanceChange = (long)previousLocalDistance - currentDistance;
    long speed = distanceChange * 1000L / (long)elapsedMs;
    if (speed > 32767L) speed = 32767L;
    if (speed < -32768L) speed = -32768L;
    localSpeedCmPerSecond = (int)speed;
  } else {
    localSpeedCmPerSecond = 0;
  }

  previousLocalDistance = currentDistance;
  previousLocalSampleMs = now;
  localDistanceCm = currentDistance;
  lastLocalSampleMs = now;
  haveLocalSample = true;
}

bool localLidarFresh() {
  return haveLocalSample && millis() - lastLocalSampleMs <= CAN_TIMEOUT_MS;
}

bool remoteLidarFresh() {
  return haveRemoteSample && millis() - lastRemoteSampleMs <= CAN_TIMEOUT_MS;
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
  const uint8_t destinationAddress = (uint8_t)((packetId >> 8) & 0xFF);

  if (extendedFrame && pgn == J1939_PGN_LMSD_MCU &&
      sourceAddress == J1939_SA_MCU && packetSize == 8 && bytesRead == 8) {
    const uint16_t distance = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
    const uint16_t speedRaw = (uint16_t)payload[2] | ((uint16_t)payload[3] << 8);
    const int32_t speed = speedRaw <= 32767
        ? (int32_t)speedRaw
        : (int32_t)speedRaw - 65536L;
    remoteDistanceCm = (int)distance;
    remoteSpeedCmPerSecond = (int)speed;
    lastRemoteSampleMs = millis();
    haveRemoteSample = true;
    return;
  }

  if (extendedFrame && pgn == J1939_PGN_MCTL_DCU &&
      sourceAddress == J1939_SA_DCU && destinationAddress == J1939_SA_PCU &&
      packetSize == 8 && bytesRead == 8 &&
      payload[0] <= COMMAND_TO_PCU) {
    requestedCommand = payload[0];
    requestedProgram = payload[1];
    lastCommandSequence = payload[2];
    lastDcuCommandMs = millis();
    haveDcuCommand = true;
  }
}

void updateMotion() {
  if (!j1939Node.mayTransmitApplication() || !haveDcuCommand ||
      millis() - lastDcuCommandMs > CAN_TIMEOUT_MS) {
    motor_brake();
    pcuState = PCU_FAULT;
    resetStationaryTimer();
    return;
  }

  if (requestedCommand == COMMAND_HOLD) {
    motor_brake();
    pcuState = localLidarFresh() ? PCU_WAIT_FOR_LIDAR : PCU_BOOT_WAIT;
    resetStationaryTimer();
    return;
  }

  if (requestedProgram != 1) {
    motor_brake();
    pcuState = PCU_FAULT;
    resetStationaryTimer();
    return;
  }

  if (!localLidarFresh()) {
    motor_brake();
    pcuState = PCU_WAIT_FOR_LIDAR;
    resetStationaryTimer();
    return;
  }

  const uint8_t pwm = MOTOR_PWM_PROGRAM_1;
  const bool atHomeRange = localDistanceCm <= END_DISTANCE_CM;

  if (requestedCommand == COMMAND_HOME || requestedCommand == COMMAND_TO_PCU) {
    if (atHomeRange) {
      motor_brake();
      pcuState = stoppedForOneSecond(localSpeedCmPerSecond)
          ? PCU_AT_HOME
          : PCU_HOMING;
    } else {
      resetStationaryTimer();
      pcuState = PCU_TO_HOME;
      motor_backward_constant(pwm);
    }
    return;
  }

  // A trip toward the MCU is only allowed after the PCU has verified home.
  if (requestedCommand == COMMAND_TO_MCU) {
    if (!remoteLidarFresh()) {
      motor_brake();
      pcuState = PCU_WAIT_FOR_LIDAR;
      resetStationaryTimer();
      return;
    }

    if (pcuState != PCU_TO_MCU && pcuState != PCU_AT_MCU) {
      if (!atHomeRange) {
        motor_brake();
        pcuState = PCU_FAULT;
        resetStationaryTimer();
        return;
      }

      motor_brake();
      if (!stoppedForOneSecond(localSpeedCmPerSecond)) {
        pcuState = PCU_HOMING;
        return;
      }
      pcuState = PCU_AT_HOME;
    }

    if (remoteDistanceCm <= END_DISTANCE_CM) {
      motor_brake();
      pcuState = stoppedForOneSecond(remoteSpeedCmPerSecond)
          ? PCU_AT_MCU
          : PCU_TO_MCU;
    } else {
      resetStationaryTimer();
      pcuState = PCU_TO_MCU;
      motor_forward_constant(pwm);
    }
  }
}

void transmitStatus() {
  const unsigned long now = millis();
  if (now - lastStatusTxMs < STATUS_PERIOD_MS) return;
  lastStatusTxMs = now;
  if (!j1939Node.mayTransmitApplication()) return;

  const uint16_t distance = localLidarFresh() ? (uint16_t)localDistanceCm : 0xFFFF;
  int speed = localLidarFresh() ? localSpeedCmPerSecond : 0;
  if (speed > 32767) speed = 32767;
  if (speed < -32768) speed = -32768;
  const uint16_t speedRaw = (uint16_t)(int16_t)speed;

  CAN.beginExtendedPacket(j1939MakeId(6, J1939_PGN_MSTA_PCU,
                                      J1939_GLOBAL_ADDRESS, j1939Node.address()));
  CAN.write(pcuState);
  CAN.write((uint8_t)(distance & 0xFF));
  CAN.write((uint8_t)(distance >> 8));
  CAN.write((uint8_t)(speedRaw & 0xFF));
  CAN.write((uint8_t)(speedRaw >> 8));
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.endPacket();
}

void setup() {
  Serial.begin(115200);
  pinMode(a1, OUTPUT);
  pinMode(a2, OUTPUT);
  pinMode(pa, OUTPUT);
  motor_brake();

  if (!CAN.begin(500E3)) {
    while (true) {
      motor_brake();
    }
  }

  // Identity 2 is unique within this project. NAME contains a test-only
  // manufacturer-code placeholder until one is assigned by SAE.
  j1939Node.begin(J1939_SA_PCU, j1939MakeTestName(2));
}

void loop() {
  pollCan();
  updateLocalLidar();
  updateMotion();
  transmitStatus();
}
