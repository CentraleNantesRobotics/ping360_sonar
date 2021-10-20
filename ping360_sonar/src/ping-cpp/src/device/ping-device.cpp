#include "ping-device.h"

#include <link/ping-port.h>
#include <time/ping-time.h>

#include <cstdio>

bool PingDevice::initialize() { return request(CommonId::PROTOCOL_VERSION) && request(CommonId::DEVICE_INFORMATION); }

ping_message* PingDevice::read()
{
    uint8_t data;
    int result = _port.read(&data, 1);
    if (result != 0 && _parser.parseByte(data) == PingParser::State::NEW_MESSAGE) {
        return &_parser.rxMessage;
    }
    return nullptr;
}

ping_message PingDevice::message() { return _parser.rxMessage; }

ping_message* PingDevice::request(uint16_t id, int timeoutMs)
{
    _general_request.set_requested_id(id);
    writeMessage(_general_request);

    ping_message* reply = waitMessage(id, timeoutMs);
    if (reply && reply->message_id() == CommonId::NACK) {
        common_nack* m = static_cast<common_nack*>(reply);
        if (m->nacked_id() != CommonId::GENERAL_REQUEST) {
            return nullptr;
        }
    }

    return reply;
}

ping_message* PingDevice::waitMessage(uint16_t id, int timeoutMs)
{
    int timeStart = PingTime::timeMs();
    while (PingTime::timeMs() < timeStart + timeoutMs) {
        // TODO: make this totally nonblocking
        // this will block up to 0.1s for data to be available
        ping_message* message = read();

        if (!message) {
            continue;
        }

        _handleMessage(message);

        if (message->message_id() == id || message->message_id() == CommonId::NACK) {
            return message;
        }
        // Prevent cpu spinlock
        PingTime::yeild();
    }

    return nullptr;
}

int PingDevice::write(const uint8_t* data, int nBytes) { return _port.write(data, nBytes); }

void PingDevice::writeMessage(ping_message& message)
{
    message.updateChecksum();
    write(reinterpret_cast<uint8_t*>(message.msgData), message.msgDataLength());
}

void PingDevice::_handleMessage(const ping_message* message)
{
    device_id = message->source_device_id();

    switch (message->message_id()) {
    case CommonId::NACK:
        break;
    case CommonId::PROTOCOL_VERSION: {
        const common_protocol_version* message_protocol_version = static_cast<const common_protocol_version*>(message);
        protocol_version.version_major = message_protocol_version->version_major();
        protocol_version.version_minor = message_protocol_version->version_minor();
        protocol_version.version_patch = message_protocol_version->version_patch();
        break;
    }
    case CommonId::DEVICE_INFORMATION: {
        const common_device_information* message_device_information = static_cast<const common_device_information*>(message);
        device_information.device_type = message_device_information->device_type();
        device_information.device_revision = message_device_information->device_revision();
        device_information.firmware_version_major = message_device_information->firmware_version_major();
        device_information.firmware_version_minor = message_device_information->firmware_version_minor();
        device_information.firmware_version_patch = message_device_information->firmware_version_patch();
        break;
    }
    default:
        break;
    }
}
