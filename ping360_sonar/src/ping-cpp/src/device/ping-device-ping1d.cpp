#include "ping-device-ping1d.h"
#include <ping-message-common.h>
#include <ping-message-ping1d.h>

Ping1d::~Ping1d()
{
    if (profile_data.profile_data) {
        delete[] profile_data.profile_data;
    }
}

bool Ping1d::initialize(uint16_t pingIntervalMs)
{
    if (!PingDevice::initialize()) {
        return false;
    }

    // Configure ping interval
    if (!set_ping_interval(pingIntervalMs)) {
        return false;
    }

    return true;
}

void Ping1d::_handleMessage(const ping_message* message)
{
    switch (message->message_id()) {
        case Ping1dId::DISTANCE:
        {
            const ping1d_distance* message_distance = static_cast<const ping1d_distance*>(message);
            distance_data.distance = message_distance->distance();
            distance_data.confidence = message_distance->confidence();
            distance_data.transmit_duration = message_distance->transmit_duration();
            distance_data.ping_number = message_distance->ping_number();
            distance_data.scan_start = message_distance->scan_start();
            distance_data.scan_length = message_distance->scan_length();
            distance_data.gain_setting = message_distance->gain_setting();
        }
        break;
        case Ping1dId::DISTANCE_SIMPLE:
        {
            const ping1d_distance_simple* message_distance_simple = static_cast<const ping1d_distance_simple*>(message);
            distance_simple_data.distance = message_distance_simple->distance();
            distance_simple_data.confidence = message_distance_simple->confidence();
        }
        break;
        case Ping1dId::FIRMWARE_VERSION:
        {
            const ping1d_firmware_version* message_firmware_version = static_cast<const ping1d_firmware_version*>(message);
            firmware_version_data.device_type = message_firmware_version->device_type();
            firmware_version_data.device_model = message_firmware_version->device_model();
            firmware_version_data.firmware_version_major = message_firmware_version->firmware_version_major();
            firmware_version_data.firmware_version_minor = message_firmware_version->firmware_version_minor();
        }
        break;
        case Ping1dId::GAIN_SETTING:
        {
            const ping1d_gain_setting* message_gain_setting = static_cast<const ping1d_gain_setting*>(message);
            gain_setting_data.gain_setting = message_gain_setting->gain_setting();
        }
        break;
        case Ping1dId::GENERAL_INFO:
        {
            const ping1d_general_info* message_general_info = static_cast<const ping1d_general_info*>(message);
            general_info_data.firmware_version_major = message_general_info->firmware_version_major();
            general_info_data.firmware_version_minor = message_general_info->firmware_version_minor();
            general_info_data.voltage_5 = message_general_info->voltage_5();
            general_info_data.ping_interval = message_general_info->ping_interval();
            general_info_data.gain_setting = message_general_info->gain_setting();
            general_info_data.mode_auto = message_general_info->mode_auto();
        }
        break;
        case Ping1dId::MODE_AUTO:
        {
            const ping1d_mode_auto* message_mode_auto = static_cast<const ping1d_mode_auto*>(message);
            mode_auto_data.mode_auto = message_mode_auto->mode_auto();
        }
        break;
        case Ping1dId::PCB_TEMPERATURE:
        {
            const ping1d_pcb_temperature* message_pcb_temperature = static_cast<const ping1d_pcb_temperature*>(message);
            pcb_temperature_data.pcb_temperature = message_pcb_temperature->pcb_temperature();
        }
        break;
        case Ping1dId::PING_ENABLE:
        {
            const ping1d_ping_enable* message_ping_enable = static_cast<const ping1d_ping_enable*>(message);
            ping_enable_data.ping_enabled = message_ping_enable->ping_enabled();
        }
        break;
        case Ping1dId::PING_INTERVAL:
        {
            const ping1d_ping_interval* message_ping_interval = static_cast<const ping1d_ping_interval*>(message);
            ping_interval_data.ping_interval = message_ping_interval->ping_interval();
        }
        break;
        case Ping1dId::PROCESSOR_TEMPERATURE:
        {
            const ping1d_processor_temperature* message_processor_temperature = static_cast<const ping1d_processor_temperature*>(message);
            processor_temperature_data.processor_temperature = message_processor_temperature->processor_temperature();
        }
        break;
        case Ping1dId::PROFILE:
        {
            const ping1d_profile* message_profile = static_cast<const ping1d_profile*>(message);
            profile_data.distance = message_profile->distance();
            profile_data.confidence = message_profile->confidence();
            profile_data.transmit_duration = message_profile->transmit_duration();
            profile_data.ping_number = message_profile->ping_number();
            profile_data.scan_start = message_profile->scan_start();
            profile_data.scan_length = message_profile->scan_length();
            profile_data.gain_setting = message_profile->gain_setting();
            if (message_profile->profile_data_length() > profile_data.profile_data_length) {
                if (profile_data.profile_data) {
                    delete[] profile_data.profile_data;
                }
                profile_data.profile_data = new uint8_t[message_profile->profile_data_length()];
            }

            // If pointer is invalid, make sure to abort, there is no more memory!
            if (profile_data.profile_data == nullptr) {
                profile_data.profile_data_length = -1;
                return;
            }

            profile_data.profile_data_length = message_profile->profile_data_length();
            memcpy(profile_data.profile_data, message_profile->profile_data(), message_profile->profile_data_length());
        }
        break;
        case Ping1dId::RANGE:
        {
            const ping1d_range* message_range = static_cast<const ping1d_range*>(message);
            range_data.scan_start = message_range->scan_start();
            range_data.scan_length = message_range->scan_length();
        }
        break;
        case Ping1dId::SPEED_OF_SOUND:
        {
            const ping1d_speed_of_sound* message_speed_of_sound = static_cast<const ping1d_speed_of_sound*>(message);
            speed_of_sound_data.speed_of_sound = message_speed_of_sound->speed_of_sound();
        }
        break;
        case Ping1dId::TRANSMIT_DURATION:
        {
            const ping1d_transmit_duration* message_transmit_duration = static_cast<const ping1d_transmit_duration*>(message);
            transmit_duration_data.transmit_duration = message_transmit_duration->transmit_duration();
        }
        break;
        case Ping1dId::VOLTAGE_5:
        {
            const ping1d_voltage_5* message_voltage_5 = static_cast<const ping1d_voltage_5*>(message);
            voltage_5_data.voltage_5 = message_voltage_5->voltage_5();
        }
        break;

        default:
            break;
    }

    PingDevice::_handleMessage(message);
}

bool Ping1d::set_device_id(uint8_t _device_id, bool verify)
{
    ping1d_set_device_id message;
    message.set_device_id(_device_id);
    writeMessage(message);
    // Check if we have a reply from the device
    if (!request(Ping1dId::DEVICE_ID)) {
        return false;
    }
    // Read back the data and check that changes have been applied
    if (verify
        && (device_id != _device_id)) {
        return false;
    }
    return true;
}
bool Ping1d::set_gain_setting(uint8_t _gain_setting, bool verify)
{
    ping1d_set_gain_setting message;
    message.set_gain_setting(_gain_setting);
    writeMessage(message);
    // Check if we have a reply from the device
    if (!request(Ping1dId::GAIN_SETTING)) {
        return false;
    }
    // Read back the data and check that changes have been applied
    if (verify
        && (gain_setting_data.gain_setting != _gain_setting)) {
        return false;
    }
    return true;
}
bool Ping1d::set_mode_auto(uint8_t _mode_auto, bool verify)
{
    ping1d_set_mode_auto message;
    message.set_mode_auto(_mode_auto);
    writeMessage(message);
    // Check if we have a reply from the device
    if (!request(Ping1dId::MODE_AUTO)) {
        return false;
    }
    // Read back the data and check that changes have been applied
    if (verify
        && (mode_auto_data.mode_auto != _mode_auto)) {
        return false;
    }
    return true;
}
bool Ping1d::set_ping_enable(uint8_t _ping_enabled, bool verify)
{
    ping1d_set_ping_enable message;
    message.set_ping_enabled(_ping_enabled);
    writeMessage(message);
    // Check if we have a reply from the device
    if (!request(Ping1dId::PING_ENABLE)) {
        return false;
    }
    // Read back the data and check that changes have been applied
    if (verify
        && (ping_enable_data.ping_enabled != _ping_enabled)) {
        return false;
    }
    return true;
}
bool Ping1d::set_ping_interval(uint16_t _ping_interval, bool verify)
{
    ping1d_set_ping_interval message;
    message.set_ping_interval(_ping_interval);
    writeMessage(message);
    // Check if we have a reply from the device
    if (!request(Ping1dId::PING_INTERVAL)) {
        return false;
    }
    // Read back the data and check that changes have been applied
    if (verify
        && (ping_interval_data.ping_interval != _ping_interval)) {
        return false;
    }
    return true;
}
bool Ping1d::set_range(uint32_t _scan_start, uint32_t _scan_length, bool verify)
{
    ping1d_set_range message;
    message.set_scan_start(_scan_start);
    message.set_scan_length(_scan_length);
    writeMessage(message);
    // Check if we have a reply from the device
    if (!request(Ping1dId::RANGE)) {
        return false;
    }
    // Read back the data and check that changes have been applied
    if (verify
        && (range_data.scan_start != _scan_start
        || range_data.scan_length != _scan_length)) {
        return false;
    }
    return true;
}
bool Ping1d::set_speed_of_sound(uint32_t _speed_of_sound, bool verify)
{
    ping1d_set_speed_of_sound message;
    message.set_speed_of_sound(_speed_of_sound);
    writeMessage(message);
    // Check if we have a reply from the device
    if (!request(Ping1dId::SPEED_OF_SOUND)) {
        return false;
    }
    // Read back the data and check that changes have been applied
    if (verify
        && (speed_of_sound_data.speed_of_sound != _speed_of_sound)) {
        return false;
    }
    return true;
}
