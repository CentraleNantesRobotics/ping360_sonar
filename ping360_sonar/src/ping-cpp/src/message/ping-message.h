// C++ implementation of the Blue Robotics 'Ping' binary message protocol

#pragma once
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

class ping_message
{
public:
    ping_message() : msgData { nullptr } {}

    ping_message(const ping_message &msg)
        : _bufferLength { msg.msgDataLength() }
        , msgData { static_cast<uint8_t*>(malloc(sizeof(uint8_t) * _bufferLength)) }
    {
        memcpy(msgData, msg.msgData, _bufferLength);
    }

    ping_message(const uint16_t bufferLength)
        : _bufferLength { bufferLength }
        , msgData { static_cast<uint8_t*>(malloc(sizeof(uint8_t) * _bufferLength)) }
    {
        if (bufferLength >= 2) {
            msgData[0] = 'B';
            msgData[1] = 'R';
        }
    }

    ping_message(const uint8_t* buf, const uint16_t length)
        : _bufferLength { length }
        , msgData { static_cast<uint8_t*>(malloc(sizeof(uint8_t) * _bufferLength)) }
    {
        memcpy(msgData, buf, _bufferLength);
    }

    ping_message& operator = (const ping_message &msg) {
        _bufferLength = msg.msgDataLength();
        if(msgData) free(msgData);
        msgData = static_cast<uint8_t*>(malloc(sizeof(uint8_t) * _bufferLength));
        memcpy(msgData, msg.msgData, _bufferLength);
        return *this;
    }

    ~ping_message() { if(msgData) free(msgData); }

protected:
    uint16_t _bufferLength;

public:
    static const uint8_t headerLength = 8;
    static const uint8_t checksumLength = 2;

    uint8_t* msgData;
    uint16_t bufferLength() const { return _bufferLength; } // size of internal buffer allocation
    uint16_t msgDataLength() const { return static_cast<uint16_t>(headerLength + payload_length() + checksumLength); } // size of entire message buffer (header, payload, and checksum)
    uint8_t* message_data(uint32_t offset=0) const { return msgData + offset; }
    uint8_t* payload_data(uint16_t offset=0) const { return msgData + headerLength + offset; }

    uint16_t payload_length()                const { return reinterpret_cast<uint16_t&>(msgData[2]); }
    void set_payload_length(const uint16_t payload_length) { reinterpret_cast<uint16_t&>(msgData[2]) = payload_length; }
    uint16_t message_id()                    const { return reinterpret_cast<uint16_t&>(msgData[4]); }
    void set_message_id(const uint16_t message_id) { reinterpret_cast<uint16_t&>(msgData[4]) = message_id; }
    uint8_t  source_device_id()              const { return msgData[6]; }
    void set_source_device_id(const uint8_t device_id) { msgData[6] = device_id; }
    uint8_t  destination_device_id()         const { return msgData[7]; }
    void set_destination_device_id(const uint8_t device_id) { msgData[7] = device_id; }
    // to migrate legacy ping1d devices, a new message id should be used with the same fields + padding
    uint16_t checksum()                      const { return static_cast<uint16_t>(msgData[msgDataLength() - checksumLength] + (msgData[msgDataLength() - checksumLength + 1] << 8)); }
    void set_checksum(uint16_t checksum)           { msgData[msgDataLength() - checksumLength] = (uint8_t)checksum; msgData[msgDataLength() - checksumLength + 1] = static_cast<uint8_t>(checksum >> 8); }

    bool verifyChecksum() const {
        if(msgDataLength() > bufferLength()) {
            return false;
        }
        return checksum() == calculateChecksum();
    }

    void updateChecksum() {
        if(msgDataLength() <= bufferLength()) {
            set_checksum(calculateChecksum());
        }
    }

    uint16_t calculateChecksum() const {
        uint16_t calculatedChecksum = 0;
        for(uint32_t i = 0, data_size = msgDataLength() - checksumLength; i < data_size; i++) {
            calculatedChecksum = static_cast<uint16_t>(msgData[i] + calculatedChecksum);
        }
        return calculatedChecksum;
    }

    int getMessageAsString(char* string, size_t size) const {
        return snprintf(string, size,
            "ping_message:\n"
            " payload_length: %d\n"
            " message_id: %d\n"
            " src_device_id: %d\n"
            " dst_device_id: %d\n"
            , payload_length()
            , message_id()
            , source_device_id()
            , destination_device_id()
        );
    }
};
