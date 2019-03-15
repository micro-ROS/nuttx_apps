// Copyright (c) 2015, Lab-RoCoCo
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.

// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.

// * Neither the name of thin_drivers nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


#pragma once
#include <stdint.h>
#include <vector>

//! responses
#define BasicSensorDataPayload_HEADER 1
#define DockingIRPayload_HEADER 3
#define InertialSensorDataPayload_HEADER 4
#define CliffSensorDataPayload_HEADER 5
#define CurrentPayload_HEADER 6
#define HardwareVersionPayload_HEADER 10
#define FirmwareVersionPayload_HEADER 11
#define GyroPayload_HEADER 13
#define GPIOPayload_HEADER 16
#define UUIDPayload_HEADER 19
#define ControllerInfoPayload_HEADER 21

//! commands
#define BaseControlPayload_HEADER 1
#define SoundPayload_HEADER 3
#define SoundSequencePayload_HEADER 4
#define RequestExtraPayload_HEADER 9
#define GPIOOutputPayload_HEADER 12
#define SetControllerGain_HEADER 13 // not sure documentation wrong
#define GetControllerGain_HEADER 14 // not sure documentation wrong


// read/write functions
inline uint32_t parseInt(const unsigned char*&buffer) {
  uint32_t value = *(uint32_t*)buffer;
  buffer+=sizeof(uint32_t);
  return value;
}

inline uint16_t parseShort(const unsigned char*&buffer) {
  uint16_t value = *(uint16_t*)buffer;
  buffer+=sizeof(uint16_t);
  return value;
}

inline uint8_t parseChar(const unsigned char*&buffer) {
  uint8_t value = *(uint8_t*)buffer;
  buffer+=sizeof(uint8_t);
  return value;
}


inline int writeInt(unsigned char*&buffer, uint32_t value) {
  *(uint32_t*)buffer = value;
  buffer+=sizeof(uint32_t);
  return sizeof(uint32_t);
}

inline int writeShort(unsigned char*&buffer, uint16_t value) {
  *(uint16_t*)buffer = value;
  buffer+=sizeof(uint16_t);
  return sizeof(uint16_t);
}

inline int writeChar(unsigned char*&buffer, uint8_t value) {
  *(uint8_t*)buffer = value;
  buffer+=sizeof(uint8_t);
  return sizeof(uint8_t);
}

struct SubPayload{
  SubPayload(uint8_t header_, uint8_t length_);

  inline uint8_t header() const { return _header; }
  inline uint8_t length() const {return _length; }
  virtual int parse(const unsigned char*&buffer);
  virtual int write(unsigned char*&buffer);
  virtual ~SubPayload();
  uint8_t _header;
  uint8_t _length;
};

struct BasicSensorDataPayload : public SubPayload {
  BasicSensorDataPayload();
  virtual int parse(const unsigned char*&buffer);
  uint16_t timestamp;
  uint8_t bumper;
  uint8_t wheel_drop;
  uint8_t cliff;
  uint16_t left_encoder;
  uint16_t right_encoder;
  uint8_t left_pwm;
  uint8_t right_pwm;
  uint8_t button;
  uint8_t charger;
  uint8_t battery;
  uint8_t overcurrent_flags;
};

struct DockingIRPayload: public SubPayload {
  DockingIRPayload();
  virtual int parse(const unsigned char*&buffer);
  uint8_t right_signal;
  uint8_t center_signal;
  uint8_t left_signal;
};

struct InertialSensorDataPayload: public SubPayload {
  InertialSensorDataPayload();
  virtual int parse(const unsigned char*&buffer);
  uint16_t angle;
  uint16_t rate;
  uint8_t unused[3];
};

struct CliffSensorDataPayload: public SubPayload {
  CliffSensorDataPayload();
  virtual int parse(const unsigned char*&buffer);
  uint16_t right_signal;
  uint16_t center_signal;
  uint16_t left_signal;
};

struct CurrentPayload: public SubPayload {
  CurrentPayload();
  virtual int parse(const unsigned char*&buffer);
  uint8_t right_motor;
  uint8_t left_motor;
};


struct HardwareVersionPayload: public SubPayload {
  HardwareVersionPayload();
  virtual int parse(const unsigned char*&buffer);
  uint8_t patch;
  uint8_t major;
  uint8_t minor;
  uint8_t unised;
};

struct FirmwareVersionPayload: public SubPayload {
  FirmwareVersionPayload();
  virtual int parse(const unsigned char*&buffer);
  uint8_t patch;
  uint8_t major;
  uint8_t minor;
  uint8_t unised;
};

struct GyroPayload: public SubPayload {
  struct GyroData{
    uint16_t x,y,z;
  };
  GyroPayload();
  virtual int parse(const unsigned char*&buffer);
  uint8_t frame_id;
  uint8_t subdata_length;
  int num_gyro_data;
  GyroData gyro_datas[10];
};

struct GPIOPayload: public SubPayload {
  GPIOPayload();
  virtual int parse(const unsigned char*&buffer);
  uint16_t digital_input;
  uint16_t analog_input[4];
  uint16_t unused[3];
};

struct UUIDPayload: public SubPayload {
  UUIDPayload();
  virtual int parse(const unsigned char*&buffer);
  uint32_t udid[3];
};

struct ControllerInfoPayload: public SubPayload {
  ControllerInfoPayload();
  virtual int parse(const unsigned char*&buffer);
  uint8_t type;
  uint32_t P,I,D;
};


struct BaseControlPayload : public SubPayload {
  BaseControlPayload();
  virtual int write(unsigned char*&buffer);
  uint16_t speed;
  uint16_t radius;
};

struct SoundPayload : public SubPayload {
  SoundPayload();
  virtual int write(unsigned char*&buffer);
  uint16_t note;
  uint8_t duration;
};

struct SoundSequencePayload : public SubPayload {
  SoundSequencePayload();
  virtual int write(unsigned char*&buffer);

  /*0 for ON sound
    1 for OFF sound
    2 for RECHARGE sound
    3 for BUTTON sound
    4 for ERROR sound
    5 for CLEANINGSTART sound
    6 for CLEANINGEND sound 
   */
  uint8_t sequence;
};

struct RequestExtraPayload : public SubPayload {
  RequestExtraPayload();
  virtual int write(unsigned char*&buffer);
  /*
    0x01 for Hardware Version
    0x02 for Firmware Version
    0x08 for Unique Device ID 
  */
  uint16_t request_flags;
};

struct GPIOOutputPayload : public SubPayload {
  GPIOOutputPayload();
  virtual int write(unsigned char*&buffer);
  /*
    0x0001 for digital output ch. 0
    0x0002 for digital output ch. 1
    0x0004 for digital output ch. 2
    0x0008 for digital output ch. 3

    Set the flags to turn on external powers
    0x0010 for external power 3.3V ch.
    0x0020 for external power 5V ch.
    0x0040 for external power 12V/5A ch.
    0x0080 for external power 12V/1.5A ch.

    Set the flags to turn on LEDs
    0x0100 for red colour of LED1
    0x0200 for green colour of LED1
    0x0400 for red colour of LED2
    0x0800 for green colour of LED2
  */
  uint16_t digital_output_flags;
};

//! this is wrong in the documentation, i assigned a command number 13 by default
struct SetControllerGain : public SubPayload {
  SetControllerGain();

  virtual int write(unsigned char*&buffer);
  /*
    0 for factory-default PID gain
    1 for user-configured PID gain 
  */
  uint8_t type;
  uint32_t P;
  uint32_t I;
  uint32_t D;
};

//! this is wrong in the documentation, i assigned a command number 13 by default
struct GetControllerGain : public SubPayload {
  GetControllerGain();
  virtual int write(unsigned char*&buffer);
  /*
    0 for factory-default PID gain
    1 for user-configured PID gain 
  */
  uint8_t unused;
};

struct Packet{
  uint8_t length;
  std::vector<SubPayload*> _payloads;
  Packet();
  ~Packet();
  void clear();
  int write(unsigned char* buffer);
};

class PacketSyncFinder{
public:
  enum State {Unsynced = 0, Sync1 = 1, Length = 2, Payload = 3, Checksum = 4};
  PacketSyncFinder();
  void putChar(unsigned char c);    
  bool packetReady() const {return _packet_ready;}
  const unsigned char* buffer() const { return _payload_buf; }
  uint8_t bufferLength() const { return _length; }
protected:
  bool _packet_ready;
  State _state;
  uint8_t _length;
  uint8_t _payload_buf_idx;
  uint8_t _payload_buf[255];
};

class BasePayloadCreator;

class PacketParser{
public:
  PacketParser();
  Packet* parseBuffer(const unsigned char*& buffer, uint8_t length);
protected:
  SubPayload* createPayload(uint8_t header);
  std::vector<BasePayloadCreator*> _creators;
};

