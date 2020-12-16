/*
    ModbusSerial.cpp - Source for Modbus Serial Library
    Copyright (C) 2014 André Sarmento Barbosa
*/
#include "ModbusSerial.h"

ModbusSerial::ModbusSerial() {

}

bool ModbusSerial::setSlaveId(byte slaveId){
    _slaveId = slaveId;
    return true;
}

byte ModbusSerial::getSlaveId() {
    return _slaveId;
}

bool ModbusSerial::config(HardwareSerial* port, long baud, u_int format, int txPin) {
    this->_port = port;
    this->_txPin = txPin;
    (*port).begin(baud, format);

    delay(2000);

    if (txPin >= 0) {
        pinMode(txPin, OUTPUT);
        digitalWrite(txPin, LOW);
    }

    if (baud > 19200) {
        _t15 = 750;
        _t35 = 1750;
    } else {
        _t15 = 15000000/baud; // 1T * 1.5 = T1.5
        _t35 = 35000000/baud; // 1T * 3.5 = T3.5
    }

    return true;
}

#ifdef USE_SOFTWARE_SERIAL
bool ModbusSerial::config(SoftwareSerial* port, long baud, int txPin) {
    this->_port = port;
    this->_txPin = txPin;
    (*port).begin(baud);

    delay(2000);

    if (txPin >= 0) {
        pinMode(txPin, OUTPUT);
        digitalWrite(txPin, LOW);
    }

    if (baud > 19200) {
        _t15 = 750;
        _t35 = 1750;
    } else {
        _t15 = 15000000/baud; // 1T * 1.5 = T1.5
        _t35 = 35000000/baud; // 1T * 3.5 = T3.5
    }

    return true;
}
#endif

#ifdef __AVR_ATmega32U4__
bool ModbusSerial::config(Serial_* port, long baud, u_int format, int txPin) {
    this->_port = port;
    this->_txPin = txPin;
    (*port).begin(baud, format);
    while (!(*port));

    if (txPin >= 0) {
        pinMode(txPin, OUTPUT);
        digitalWrite(txPin, LOW);
    }

    if (baud > 19200) {
        _t15 = 750;
        _t35 = 1750;
    } else {
        _t15 = 15000000/baud; // 1T * 1.5 = T1.5
        _t35 = 35000000/baud; // 1T * 3.5 = T3.5
    }

    return true;
}
#endif

bool ModbusSerial::receive(byte *frame)
{
    // very often the first bytes are FF - is line noice
    // try to filter it
    while ((frame[0] == 0xFF) && (_len > 6))
    {
        // posible broadcast packet if CRC OK
        u_int crc = ((frame[_len - 2] << 8) | frame[_len - 1]);
        if (crc != this->calcCrc(_frame[0], _frame + 1, _len - 3))
        {

#ifdef DEBUG_NETWORK
            Serial.print(F("RS485 bad packet: "));
            for (size_t i = 0; i < _len; i++)
            {
                Serial.print(frame[i], HEX);
                Serial.print(" ");
            }
            Serial.println(F(" -> fixed"));
#endif
            // error -> try to exclude first byte
            for (uint8_t i = 0; i < _len - 1; i++)
            {
                frame[i] = frame[i + 1];
            }
            _len = _len - 1;
        }
        else
        {
// CRC Ok
#ifdef DEBUG_NETWORK
            Serial.print(F("RS485 good multicast packet: "));
            for (size_t i = 0; i < _len; i++)
            {
                Serial.print(frame[i], HEX);
                Serial.print(" ");
            }
            Serial.println();
#endif
            //PDU starts after first byte
            //framesize PDU = framesize - address(1) - crc(2)
            this->receivePDU(frame + 1);
            //No reply to Broadcasts
            _reply = MB_REPLY_OFF;
            return true;
        }
    }

    //first byte of frame = address
    byte address = frame[0];
    //Slave Check & minimal lenght chceck
    if ((address != 0xFF && address != this->getSlaveId()) && _len > 6)
    {
        return false;
    }

#ifdef DEBUG_NETWORK
    Serial.print(F("RS485 receive:\t"));
    for (size_t i = 0; i < _len; i++)
    {
        Serial.print(frame[i], HEX);
        Serial.print(F(" "));
    }
    Serial.print(F("\n"));
#endif
    //CRC Check
    //Last two bytes = crc
    u_int crc = ((frame[_len - 2] << 8) | frame[_len - 1]);
    if (crc != this->calcCrc(_frame[0], _frame + 1, _len - 3))
    {
#ifdef DEBUG_NETWORK
        Serial.print(F("\tCRC wrong\n"));
#endif
        return false;
    }

    //PDU starts after first byte
    //framesize PDU = framesize - address(1) - crc(2)
    this->receivePDU(frame+1);
    //No reply to Broadcasts
    if (address == 0xFF) _reply = MB_REPLY_OFF;
    return true;
}

bool ModbusSerial::send(byte* frame) {
    byte i;

    if (this->_txPin >= 0) {
        digitalWrite(this->_txPin, HIGH);
        delay(1);
    }

    for (i = 0 ; i < _len ; i++) {
        (*_port).write(frame[i]);
    }

    (*_port).flush();
    delayMicroseconds(_t35);

    if (this->_txPin >= 0) {
        digitalWrite(this->_txPin, LOW);
    }
#ifdef DEBUG_NETWORK
    Serial.print(F("RS485 send: "));
    for (size_t i = 0; i < _len; i++)
    {
        Serial.print(frame[i], HEX);
        Serial.print(" ");
    }
    Serial.print(F("\n"));
#endif
    return true;
}

bool ModbusSerial::sendPDU(byte* pduframe) {
    if (this->_txPin >= 0) {
        digitalWrite(this->_txPin, HIGH);
        delay(1);
    }

    word crc = calcCrc(_slaveId, _frame, _len);
    //Send slaveId
    (*_port).write(_slaveId);
    //Send PDU
    byte i;
    for (i = 0; i < _len; i++)
    {
        (*_port).write(pduframe[i]);
    }
    //Send CRC
    (*_port).write(crc >> 8);
    (*_port).write(crc & 0xFF);

    (*_port).flush();
    delayMicroseconds(_t35);

    if (this->_txPin >= 0) {
        digitalWrite(this->_txPin, LOW);
    }

#ifdef DEBUG_NETWORK
    Serial.print(F("RS485 sendPDU:\t"));
    Serial.print(_slaveId, HEX);
    Serial.print(F(" "));
    for (size_t i = 0; i < _len; i++)
    {
        Serial.print(pduframe[i], HEX);
        Serial.print(" ");
    }
    Serial.print(crc >> 8, HEX);
    Serial.print(F(" "));
    Serial.print(crc >> 8, HEX);
    Serial.print(F("\n"));
#endif
    return true;
}

void ModbusSerial::task()
{
    _len = 0;

    while ((*_port).available() > _len)
    {
        _len = (*_port).available();
        delayMicroseconds(_t15);
    }
#define MINIMAL_PACKET_LENGHT 8
    if (_len < MINIMAL_PACKET_LENGHT)
        return;

    byte i;
    _frame = (byte*) malloc(_len);
    for (i=0 ; i < _len ; i++) _frame[i] = (*_port).read();

    if (this->receive(_frame)) {
        if (_reply == MB_REPLY_NORMAL)
            this->sendPDU(_frame);
        else
        if (_reply == MB_REPLY_ECHO)
            this->send(_frame);
    }

    free(_frame);
    _len = 0;
}

word ModbusSerial::calcCrc(byte address, byte* pduFrame, byte pduLen) {
	byte CRCHi = 0xFF, CRCLo = 0x0FF, Index;

    Index = CRCHi ^ address;
    CRCHi = CRCLo ^ _auchCRCHi[Index];
    CRCLo = _auchCRCLo[Index];

    while (pduLen--) {
        Index = CRCHi ^ *pduFrame++;
        CRCHi = CRCLo ^ _auchCRCHi[Index];
        CRCLo = _auchCRCLo[Index];
    }

    return (CRCHi << 8) | CRCLo;
}





