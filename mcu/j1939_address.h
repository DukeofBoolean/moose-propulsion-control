#ifndef MOOSE_J1939_ADDRESS_H
#define MOOSE_J1939_ADDRESS_H

#include <Arduino.h>
#include <CAN.h>

// Small J1939-81 address-claim helper for this fixed-address test network.
// The manufacturer code in the NAME is provisional and must be replaced with
// an SAE-assigned code before connecting these ECUs to another J1939 network.
const uint8_t J1939_NULL_ADDRESS = 0xFE;
const uint8_t J1939_GLOBAL_ADDRESS = 0xFF;
const uint32_t J1939_PGN_ADDRESS_CLAIMED = 0x00EE00;
const uint32_t J1939_PGN_REQUEST = 0x00EA00;
const unsigned long J1939_ADDRESS_CLAIM_WAIT_MS = 250;

inline uint32_t j1939MakeId(uint8_t priority, uint32_t pgn,
                            uint8_t destination, uint8_t source) {
  uint32_t id = ((uint32_t)(priority & 0x07) << 26) |
                ((pgn & 0x3FFFFUL) << 8) | source;
  const uint8_t pduFormat = (uint8_t)((pgn >> 8) & 0xFF);
  if (pduFormat < 0xF0) {
    id = (id & 0x1FFF00FFUL) | ((uint32_t)destination << 8);
  }
  return id;
}

inline uint32_t j1939GetPgn(uint32_t id) {
  uint32_t pgn = (id >> 8) & 0x3FFFFUL;
  const uint8_t pduFormat = (uint8_t)((id >> 16) & 0xFF);
  if (pduFormat < 0xF0) pgn &= 0x3FF00UL;
  return pgn;
}

// Test NAME: unique identity numbers, unassigned manufacturer placeholder,
// and unspecified function/system fields. This is not a globally valid NAME.
inline uint64_t j1939MakeTestName(uint32_t identityNumber) {
  const uint64_t testManufacturerCode = 2047ULL;
  return ((uint64_t)7 << 60) |       // Industry group: global (test profile)
         ((uint64_t)0x7F << 49) |    // Vehicle system: not available
         ((uint64_t)0xFF << 40) |    // Function: not available
         (testManufacturerCode << 21) |
         (identityNumber & 0x1FFFFFUL);
}

class J1939AddressClaim {
 public:
  void begin(uint8_t preferredAddress, uint64_t name) {
    address_ = preferredAddress;
    name_ = name;
    claimStartedMs_ = millis();
    claimPending_ = true;
    claimed_ = false;
    cannotClaim_ = false;
    sendAddressClaim();
  }

  void update() {
    if (claimPending_ && millis() - claimStartedMs_ >= J1939_ADDRESS_CLAIM_WAIT_MS) {
      claimPending_ = false;
      claimed_ = !cannotClaim_;
    }
  }

  bool mayTransmitApplication() {
    update();
    return claimed_ && !cannotClaim_;
  }

  uint8_t address() const { return address_; }
  uint64_t name() const { return name_; }

  // Call for each received extended frame after its payload has been read.
  void handleFrame(uint32_t id, bool extended, const uint8_t* data, int length) {
    if (!extended || length < 0) return;

    const uint32_t pgn = j1939GetPgn(id);
    const uint8_t source = (uint8_t)(id & 0xFF);
    const uint8_t destination = (uint8_t)((id >> 8) & 0xFF);

    if (pgn == J1939_PGN_ADDRESS_CLAIMED && destination == J1939_GLOBAL_ADDRESS &&
        source == address_ && length == 8) {
      uint64_t otherName = 0;
      for (uint8_t i = 0; i < 8; i++) {
        otherName |= (uint64_t)data[i] << (8 * i);
      }
      // Lower numeric NAME wins arbitration for a shared source address.
      // These ECUs have fixed addresses, so a loser reports Cannot Claim.
      if (otherName <= name_) sendCannotClaim();
      return;
    }

    if (pgn == J1939_PGN_REQUEST && length >= 3 &&
        (destination == J1939_GLOBAL_ADDRESS || destination == address_)) {
      const uint32_t requestedPgn = (uint32_t)data[0] |
                                    ((uint32_t)data[1] << 8) |
                                    ((uint32_t)data[2] << 16);
      if (requestedPgn == J1939_PGN_ADDRESS_CLAIMED && !cannotClaim_) {
        sendAddressClaim();
      }
    }
  }

 private:
  uint8_t address_ = J1939_NULL_ADDRESS;
  uint64_t name_ = 0;
  unsigned long claimStartedMs_ = 0;
  bool claimPending_ = false;
  bool claimed_ = false;
  bool cannotClaim_ = false;

  void writeNamePayload() {
    for (uint8_t i = 0; i < 8; i++) {
      CAN.write((uint8_t)(name_ >> (8 * i)));
    }
  }

  void sendAddressClaim() {
    CAN.beginExtendedPacket(j1939MakeId(6, J1939_PGN_ADDRESS_CLAIMED,
                                       J1939_GLOBAL_ADDRESS, address_));
    writeNamePayload();
    CAN.endPacket();
  }

  void sendCannotClaim() {
    if (cannotClaim_) return;
    cannotClaim_ = true;
    claimed_ = false;
    claimPending_ = false;
    CAN.beginExtendedPacket(j1939MakeId(6, J1939_PGN_ADDRESS_CLAIMED,
                                       J1939_GLOBAL_ADDRESS, J1939_NULL_ADDRESS));
    writeNamePayload();
    CAN.endPacket();
  }
};

#endif
