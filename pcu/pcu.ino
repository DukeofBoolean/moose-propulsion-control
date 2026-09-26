#include <CAN.h>
#include <SPI.h>

// ========================== GLOBAL VARIABLES ==========================
volatile int remote_distance = 0;  // in cm
volatile int remote_speed = 0;     // in cm/s
volatile int local_distance = 0;   // in cm
volatile int local_speed = 0;      // in cm/s
volatile bool new_remote_data = false;
volatile bool new_local_data = false;

// ========================== LOCAL LIDAR (TFmini) =====================
typedef struct {
  int distance;
  int strength;
  int temp;
  boolean receiveComplete;
} TF;

TF Lidar = {0, 0, 0, false};
int previous_distance = 0;
unsigned long previous_time = 0;
const unsigned long sampling_interval = 100; // ms

// Motor control pins
const int a1 = 7, a2 = 8, pa = 6;

// Safety timeout
const unsigned long TIMEOUT_MS = 500; // 500 ms
unsigned long lastUpdateTime = 0;

// Stationary detection
unsigned long stationaryStartTime = 0;
bool stationaryTimerRunning = false;

// Motor speed constant
const uint8_t MOTOR_PWM = 50;

// Thresholds
const int DISTANCE_THRESHOLD = 10; // cm
const int SPEED_ZERO_THRESHOLD = 1; // cm/s
const int STARTUP_DIFF_THRESHOLD = 10; // cm

// ========================== MOTOR STATES ==========================
enum MotorState {
  INIT,
  FORWARD,
  BRAKE_REMOTE,
  WAIT_REMOTE,
  REVERSE,
  BRAKE_LOCAL,
  WAIT_LOCAL
};

MotorState state = INIT;

// ========================== TFmini LIDAR DATA ACQUISITION ==========
void getLidarData(TF* lidar) {
  static char i = 0;
  char j = 0;
  int checksum = 0;
  static int rx[9];

  while (Serial.available()) {
    rx[i] = Serial.read();
    if (rx[0] != 0x59) {
      i = 0;
    } else if (i == 1 && rx[1] != 0x59) {
      i = 0;
    } else if (i == 8) {
      for (j = 0; j < 8; j++) checksum += rx[j];
      if (rx[8] == (checksum % 256)) {
        lidar->distance = rx[2] + rx[3] * 256;
        lidar->strength = rx[4] + rx[5] * 256;
        lidar->temp = (rx[6] + rx[7] * 256) / 8 - 256;
        lidar->receiveComplete = true;
      }
      i = 0;
    } else {
      i++;
    }
  }
}

// ========================== MOTOR CONTROL FUNCTIONS ==================
void motor_forward_constant(uint16_t omega) {
  digitalWrite(a1, HIGH);
  digitalWrite(a2, LOW);
  analogWrite(pa, omega);
}

void motor_backward_constant(uint16_t omega) {
  digitalWrite(a1, LOW);
  digitalWrite(a2, HIGH);
  analogWrite(pa, omega);
}

void motor_brake() {
  digitalWrite(a1, LOW);
  digitalWrite(a2, LOW);
  analogWrite(pa, 0);
}

// ========================== SETUP ==========================
void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(a1, OUTPUT);
  pinMode(a2, OUTPUT);
  pinMode(pa, OUTPUT);

  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }

  Serial.println("System Ready: CAN + Motor + Local Lidar on Hardware Serial");
}

// ========================== LOCAL LIDAR PROCESSING ==================
void localLidar() {
  getLidarData(&Lidar);

  if (Lidar.receiveComplete) {
    int current_distance = Lidar.distance;
    Lidar.receiveComplete = false;

    unsigned long current_time = millis();
    if (current_time - previous_time >= sampling_interval) {
      int time_elapsed = current_time - previous_time;
      int distance_change = previous_distance - current_distance; // positive if approaching
      int current_speed = 0;

      if (time_elapsed > 0) {
        current_speed = (distance_change * 1000) / time_elapsed; // cm/s
      }

      previous_distance = current_distance;
      previous_time = current_time;

      local_distance = current_distance;
      local_speed = current_speed;
      new_local_data = true;
    }
  }
}

// ========================== MAIN LOOP ==========================
void loop() {
  // Poll CAN packets
  int packetSize = CAN.parsePacket();
  if (packetSize > 0) {
    const bool expected_frame = CAN.packetId() == 0x12 && packetSize == 4;
    uint8_t payload[4];
    int bytes_read = 0;
    while (CAN.available()) {
      int value = CAN.read();
      if (bytes_read < (int)sizeof(payload)) {
        payload[bytes_read] = (uint8_t)value;
      }
      bytes_read++;
    }

    if (expected_frame && bytes_read == 4) {
      const uint16_t distance_cm = ((uint16_t)payload[0] << 8) | payload[1];
      const uint16_t speed_raw = ((uint16_t)payload[2] << 8) | payload[3];
      const int32_t signed_speed = speed_raw <= 32767
          ? (int32_t)speed_raw
          : (int32_t)speed_raw - 65536L;

      remote_distance = (int)distance_cm;
      remote_speed = (int)signed_speed;
      new_remote_data = true;
      lastUpdateTime = millis();
    }
  }

  // Update local Lidar data
  localLidar();

  // Get copies of volatile variables
  int remote_dist = remote_distance;
  int remote_spd = remote_speed;
  bool has_new_remote = new_remote_data;
  new_remote_data = false;

  int local_dist = local_distance;
  int local_spd = local_speed;
  bool has_new_local = new_local_data;
  new_local_data = false;

  // Motor State Machine
  switch (state) {
    case INIT:
      motor_brake();
      Serial.println("State: INIT - Waiting for lidar data...");
      if (remote_dist > 0 && local_dist > 0) {
        if (abs(remote_dist - local_dist) < STARTUP_DIFF_THRESHOLD) {
          state = FORWARD;
        } else if (remote_dist > local_dist) {
          state = FORWARD;
        } else {
          state = REVERSE;
        }
        Serial.print("Init complete. Starting in state: ");
        Serial.println(state == FORWARD ? "FORWARD" : "REVERSE");
      }
      break;

    case FORWARD:
      motor_forward_constant(MOTOR_PWM);
      if (remote_dist < DISTANCE_THRESHOLD) {
        state = BRAKE_REMOTE;
        motor_brake();
        Serial.println("Reached remote end → BRAKE_REMOTE");
      }
      break;

    case BRAKE_REMOTE:
      motor_brake();
      if (abs(remote_spd) < SPEED_ZERO_THRESHOLD) {
        if (!stationaryTimerRunning) {
          stationaryStartTime = millis();
          stationaryTimerRunning = true;
        }
        if (millis() - stationaryStartTime > 1000) {
          stationaryTimerRunning = false;
          state = REVERSE;
          Serial.println("Object stationary 1s → REVERSE");
        }
      } else {
        stationaryTimerRunning = false;
      }
      break;

    case REVERSE:
      motor_backward_constant(MOTOR_PWM);
      if (local_dist < DISTANCE_THRESHOLD) {
        state = BRAKE_LOCAL;
        motor_brake();
        Serial.println("Reached local end → BRAKE_LOCAL");
      }
      break;

    case BRAKE_LOCAL:
      motor_brake();
      if (abs(local_spd) < SPEED_ZERO_THRESHOLD) {
        if (!stationaryTimerRunning) {
          stationaryStartTime = millis();
          stationaryTimerRunning = true;
        }
        if (millis() - stationaryStartTime > 1000) {
          stationaryTimerRunning = false;
          state = FORWARD;
          Serial.println("Object stationary 1s → FORWARD");
        }
      } else {
        stationaryTimerRunning = false;
      }
      break;
  }

  // Safety timeout: brake if no new remote data for TIMEOUT_MS
  if (millis() - lastUpdateTime > TIMEOUT_MS && state != INIT) {
    motor_brake();
    Serial.println("Motor: BRAKE (timeout)");
  }
}
