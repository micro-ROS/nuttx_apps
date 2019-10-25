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



#include "kobuki_protocol.h"
#include <stdexcept>
#include <iostream>
#include <cmath>

using namespace std;


/************************************************************/

SubPayload::SubPayload(uint8_t header_, uint8_t length_) {
  _header=header_;
  _length=length_;
}

int SubPayload::parse(const unsigned char*&buffer) {
  throw std::runtime_error("not implemented");
}
int SubPayload::write(unsigned char*&buffer) {
  throw std::runtime_error("not implemented");
}
  
SubPayload::~SubPayload(){}


/************************************************************/

BasicSensorDataPayload::BasicSensorDataPayload(): SubPayload(BasicSensorDataPayload_HEADER,15){}
int BasicSensorDataPayload::parse(const unsigned char*&buffer) {
  timestamp = parseShort(buffer);
  bumper = parseChar(buffer);
  wheel_drop = parseChar(buffer);
  cliff = parseChar(buffer);
  left_encoder = parseShort(buffer);
  right_encoder = parseShort(buffer);
  left_pwm = parseChar(buffer);
  right_pwm = parseChar(buffer);
  button = parseChar(buffer);
  charger = parseChar(buffer);
  battery = parseChar(buffer);
  overcurrent_flags = parseChar(buffer);
  return _length;
}

/************************************************************/
DockingIRPayload::DockingIRPayload(): SubPayload(DockingIRPayload_HEADER,3){}

int DockingIRPayload::parse(const unsigned char*&buffer) {
  right_signal = parseChar(buffer);
  center_signal = parseChar(buffer);
  left_signal = parseChar(buffer);
  return 3;
}

/************************************************************/
InertialSensorDataPayload::InertialSensorDataPayload(): SubPayload(InertialSensorDataPayload_HEADER,7){}

int InertialSensorDataPayload::parse(const unsigned char*&buffer) {
  angle = parseShort(buffer);
  rate =parseShort(buffer);
  for(int i = 0 ; i<3; i++)
    unused[i]  = parseChar(buffer);
  return 7;
}

/************************************************************/

CliffSensorDataPayload::CliffSensorDataPayload(): SubPayload(CliffSensorDataPayload_HEADER,6){}
int CliffSensorDataPayload::parse(const unsigned char*&buffer) {
  right_signal = parseShort(buffer);
  center_signal =parseShort(buffer);
  left_signal = parseShort(buffer);
  return 6;
}

/************************************************************/

CurrentPayload::CurrentPayload(): SubPayload(CurrentPayload_HEADER,2){}

int CurrentPayload::parse(const unsigned char*&buffer) {
  right_motor = parseChar(buffer);
  left_motor = parseChar(buffer);
  return 2;
}


/************************************************************/

HardwareVersionPayload::HardwareVersionPayload(): SubPayload(HardwareVersionPayload_HEADER,4){}

int HardwareVersionPayload::parse(const unsigned char*&buffer) {
  patch = parseChar(buffer);
  major = parseChar(buffer);
  minor = parseChar(buffer);
  unised = parseChar(buffer);
  return 4;
}


/************************************************************/

FirmwareVersionPayload::FirmwareVersionPayload(): SubPayload(FirmwareVersionPayload_HEADER,4){}

int FirmwareVersionPayload::parse(const unsigned char*&buffer) {
  patch = parseChar(buffer);
  major = parseChar(buffer);
  minor = parseChar(buffer);
  unised = parseChar(buffer);
  return 4;
}


/************************************************************/

GyroPayload::GyroPayload(): SubPayload(GyroPayload_HEADER,-1){}

int GyroPayload::parse(const unsigned char*&buffer) {
  frame_id = parseChar(buffer);
  subdata_length = parseChar(buffer);
  size_t s = subdata_length/3;
  num_gyro_data = s;
  for (size_t i=0; i<s; i++){
    gyro_datas[i].x=parseShort(buffer);
    gyro_datas[i].y=parseShort(buffer);
    gyro_datas[i].z=parseShort(buffer);
  }
  _length = 2+6*s;
  return 2+6*s;
}


/************************************************************/

GPIOPayload::GPIOPayload(): SubPayload(GPIOPayload_HEADER,16){}

int GPIOPayload::parse(const unsigned char*&buffer) {
  digital_input = parseShort(buffer);
  for(int i = 0; i<4; i++){
    analog_input[i] = parseShort(buffer);
  }
  for(int i = 0; i<3; i++){
    unused[i] = parseShort(buffer);
  }
  return 16;
}


/************************************************************/

UUIDPayload::UUIDPayload(): SubPayload(UUIDPayload_HEADER,12){}

int UUIDPayload::parse(const unsigned char*&buffer) {
  for(int i = 0; i<3; i++){
    udid[i] = parseInt(buffer);
  }
  return 12;
}


/************************************************************/
ControllerInfoPayload::ControllerInfoPayload(): SubPayload(ControllerInfoPayload_HEADER,7){}

int ControllerInfoPayload::parse(const unsigned char*&buffer) {
  type=parseChar(buffer);
  P=parseInt(buffer);
  I=parseInt(buffer);
  D=parseInt(buffer);
  return 7;
}


/************************************************************/

BaseControlPayload::BaseControlPayload(): SubPayload(BaseControlPayload_HEADER,4){}

int BaseControlPayload::write(unsigned char*&buffer) {
  int k = 0;
  k+=writeShort(buffer, speed);
  k+=writeShort(buffer, radius);
  return k;
}

/************************************************************/
SoundPayload::SoundPayload(): SubPayload(SoundPayload_HEADER,3){}

int SoundPayload::write(unsigned char*&buffer) {
  int k = 0;
  k+=writeShort(buffer, note);
  k+=writeChar(buffer, duration);
  return k;
}

/************************************************************/

SoundSequencePayload::SoundSequencePayload(): SubPayload(SoundSequencePayload_HEADER,1){}

int SoundSequencePayload::write(unsigned char*&buffer) {
  int k = 0;
  k+=writeShort(buffer, sequence);
  return k;
}

/************************************************************/

RequestExtraPayload::RequestExtraPayload(): SubPayload(RequestExtraPayload_HEADER,1){}

int RequestExtraPayload::write(unsigned char*&buffer) {
    int k = 0;
    k+=writeShort(buffer, request_flags);
    return k;
  }


/************************************************************/

GPIOOutputPayload::GPIOOutputPayload(): SubPayload(GPIOOutputPayload_HEADER,2){}

int GPIOOutputPayload::write(unsigned char*&buffer) {
  int k = 0;
  k+=writeShort(buffer, digital_output_flags);
  return k;
}


/************************************************************/

SetControllerGain::SetControllerGain(): SubPayload(SetControllerGain_HEADER,2){}

int SetControllerGain::write(unsigned char*&buffer) {
  int k = 0;
  k+=writeChar(buffer, type);
  k+=writeInt(buffer, P);
  k+=writeInt(buffer, I);
  k+=writeInt(buffer, D);
  return k;
}


/************************************************************/

GetControllerGain::GetControllerGain(): SubPayload(GetControllerGain_HEADER,1){}
int GetControllerGain::write(unsigned char*&buffer) {
  int k = 0;
  k+=writeChar(buffer, unused);
  return k;
}


/************************************************************/

Packet::Packet(){ length=0;}


Packet::~Packet() {
  clear();
}


void Packet::clear() {
  for (size_t i = 0; i<_payloads.size(); i++){
    delete _payloads[i];
  }
  _payloads.clear();
}

int Packet::write(unsigned char* buffer){
  unsigned char* b = buffer;
  int k = 0;
  k+=writeChar(b,0xAA);
  k+=writeChar(b,0x55);
  k+=writeChar(b,0x00);
  for (size_t i=0; i<_payloads.size(); i++) {
    SubPayload* p=_payloads[i];
    k+=writeChar(b,p->header());
    k+=writeChar(b,p->length());
    k+=p->write(b);
  }
  if (k>255)
    return -1;
  buffer[2] = k-1;
  unsigned char cs=0;
  for (uint8_t i = 2; i<k; i++){
    cs^=buffer[i];
  }
  k+=writeChar(b,cs);
  
  int size = b-buffer;
  return size;
}

/************************************************************/
PacketSyncFinder::PacketSyncFinder() : _length(0), _state(Unsynced), _packet_ready(false){
}

void PacketSyncFinder::putChar(unsigned char c) {
  _packet_ready = false;
  switch(_state){
  case Unsynced:
    if (c==0xAA){
      _state = Sync1;
      //cerr << "sync1" << endl;
    }
    break;
  case Sync1:
    if (c==0x55){
      _state = Length;
      //cerr << "sync2" << endl;
    }
    break;
  case Length:
    _length=c;
    _payload_buf_idx = 0;
    _state=Payload;
    //cerr << "length: " << (int) length << endl;;
    break;
  case Payload:
    //cerr << ".";
    _payload_buf[_payload_buf_idx]=c;
    _payload_buf_idx++;
    if (_payload_buf_idx==_length)
      _state = Checksum;
    break;
  case Checksum:
    {
    unsigned char cs=0;
    cs^=_length;
    cs^=c;
    for (unsigned int i = 0; i < _length; i++) {
      cs ^=  _payload_buf[i];
    }
    if (!cs) {
      //cerr << "packet ready" << endl;
      _packet_ready=true;
    }
    _state = Unsynced;
    }
    break;
  }
  
}


/************************************************************/


// these are only in the c file
struct BasePayloadCreator {
  virtual SubPayload* create() =0;
  virtual ~BasePayloadCreator(){};
  uint8_t header;
};


template <typename T>
struct PayloadCreator: public BasePayloadCreator{
  PayloadCreator(){
    T t;
    header = t.header();
  }
  virtual SubPayload* create() {
    return new T();
  }
};


PacketParser::PacketParser(){
  printf("pp ctor\n");
  _creators.resize(30,0);
  BasePayloadCreator* c;
  c= new PayloadCreator<BasicSensorDataPayload>;
  _creators[c->header] = c;
  
  //c= new PayloadCreator<DockingIRPayload>;
  //_creators[c->header] = c;

  c= new PayloadCreator<InertialSensorDataPayload>;
  _creators[c->header] = c;

  /*c= new PayloadCreator<CliffSensorDataPayload>;
  _creators[c->header] = c;

  c= new PayloadCreator<CurrentPayload>;
  _creators[c->header] = c;

  c= new PayloadCreator<HardwareVersionPayload>;
  _creators[c->header] = c;

  c= new PayloadCreator<FirmwareVersionPayload>;
  _creators[c->header] = c;

  c= new PayloadCreator<GyroPayload>;
  _creators[c->header] = c;
*/
  /*
  c= new PayloadCreator<GPIOPayload>;
  _creators[c->header] = c;
  */

 /* c= new PayloadCreator<UUIDPayload>;
  _creators[c->header] = c;

  c= new PayloadCreator<ControllerInfoPayload>;
  _creators[c->header] = c;*/
}
  
SubPayload* PacketParser::createPayload(uint8_t header) {
  if(header >= 30)
	  return NULL;
  BasePayloadCreator *c = _creators[header];
  if(c == NULL) {
	 return NULL;
  } else {
	  return c->create();
  }
}

Packet* PacketParser::parseBuffer(const unsigned char*& buffer, uint8_t length){
  int i = 0;
  if(length == 0)
	  return NULL;
  Packet* p = new Packet();
  p->length = length;
  while (i<length) {
    uint8_t type = parseChar(buffer);
    i++;
    SubPayload* bp = createPayload(type);
    if ( ! bp) {
      break;
    }
    uint8_t l = parseChar(buffer);
    i++;
    i+=bp->parse(buffer);
    p->_payloads.push_back(bp);
    if (l!=bp->length()){
      throw std::runtime_error("packet length mismatch");
    }
  }
  return p;
}




