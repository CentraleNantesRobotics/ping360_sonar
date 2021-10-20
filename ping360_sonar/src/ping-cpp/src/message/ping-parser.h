#pragma once

#include <inttypes.h>
#include "ping-message.h"

/**
 * @brief Parser that digests data and notifies owner when something interesting happens
 *
 */
class PingParser
{
public:
    PingParser(uint16_t bufferLength = 512) : rxMessage(bufferLength), rxBuffer_(rxMessage.msgData) {}
    ~PingParser() = default;

    ping_message rxMessage; // This message is used as the rx buffer

     /**
     * @brief Number of messages/packets successfully parsed
     *
     */
    uint32_t parsed = 0; // number of messages/packets successfully parsed

     /**
     * @brief Number of parse errors
     *
     */
    uint32_t errors = 0; // number of parse errors

    // This enum MUST be contiguous
    enum class State : uint8_t {
        NEW_MESSAGE,   // Just got a complete checksum-verified message
        WAIT_START,    // Waiting for the first character of a message 'B'
        WAIT_HEADER,   // Waiting for the second character in the two-character sequence 'BR'
        WAIT_LENGTH_L, // Waiting for the low byte of the payload length field
        WAIT_LENGTH_H, // Waiting for the high byte of the payload length field
        WAIT_MSG_ID_L, // Waiting for the low byte of the payload id field
        WAIT_MSG_ID_H, // Waiting for the high byte of the payload id field
        WAIT_SRC_ID,   // Waiting for the source device id
        WAIT_DST_ID,   // Waiting for the destination device id
        WAIT_PAYLOAD,  // Waiting for the last byte of the payload to come in
        WAIT_CHECKSUM_L, // Waiting for the checksum low byte
        WAIT_CHECKSUM_H, // Waiting for the checksum high byte
        ERROR, // Checksum didn't check out
    };

    /**
     * @brief Reset parser state
     *
     */
    void reset()
    {
        state_ = State::WAIT_START;
    }

    /**
     * @brief Parse a single byte
     *
     * @param data
     * @return State
     */
    PingParser::State parseByte(const uint8_t data);

// variables used for parsing
private:
    uint8_t* rxBuffer_;
    uint32_t rxBufferLength_;
    uint32_t rxCount_ = 0;
    uint16_t payloadLength_ = 0;
    State state_ = PingParser::State::WAIT_START;
};

static const char* PINGPARSER_STATE_TO_STRING(PingParser::State state) {
    switch(state) {
        case PingParser::State::NEW_MESSAGE:
            return "NEW_MESSAGE";
        case PingParser::State::WAIT_START:
            return "WAIT_START";
        case PingParser::State::WAIT_HEADER:
            return "WAIT_HEADER";
        case PingParser::State::WAIT_LENGTH_L:
            return "WAIT_LENGTH_L";
        case PingParser::State::WAIT_LENGTH_H:
            return "WAIT_LENGTH_H";
        case PingParser::State::WAIT_MSG_ID_L:
            return "WAIT_MSG_ID_L";
        case PingParser::State::WAIT_MSG_ID_H:
            return "WAIT_MSG_ID_H";
        case PingParser::State::WAIT_SRC_ID:
            return "WAIT_SRC_ID";
        case PingParser::State::WAIT_DST_ID:
            return "WAIT_DST_ID";
        case PingParser::State::WAIT_PAYLOAD:
            return "WAIT_PAYLOAD";
        case PingParser::State::WAIT_CHECKSUM_L:
            return "WAIT_CHECKSUM_L";
        case PingParser::State::WAIT_CHECKSUM_H:
            return "WAIT_CHECKSUM_H";
        case PingParser::State::ERROR:
            return "ERROR";
        default:
            return "UNKNOWN";
    };
}

/**
 * @brief Increate state
 *
 * @param state
 */
inline void operator++(PingParser::State& state, int) {
    state = static_cast<PingParser::State>((static_cast<int>(state)) + 1);
}

inline PingParser::State PingParser::parseByte(const uint8_t data)
{
    switch(state_) {
    case PingParser::State::WAIT_START:
        rxCount_ = 0;
        if (data == 'B') {
            rxBuffer_[rxCount_++] = data;
            state_++;
        }
        break;
    case PingParser::State::WAIT_HEADER:
        if (data == 'R') {
            rxBuffer_[rxCount_++] = data;
            state_++;
        } else {
            reset();
        }
        break;
    case PingParser::State::WAIT_LENGTH_L:
        rxBuffer_[rxCount_++] = data;
        payloadLength_ = data;
        state_++;
        break;
    case PingParser::State::WAIT_LENGTH_H:
        rxBuffer_[rxCount_++] = data;
        payloadLength_ = static_cast<uint16_t>((data << 8) | payloadLength_);
        if (payloadLength_ <= rxBufferLength_ - 8 - 2) {
            state_++;
        } else {
            reset();
        }
        break;
    case PingParser::State::WAIT_MSG_ID_L: // fall-through
    case PingParser::State::WAIT_MSG_ID_H:
    case PingParser::State::WAIT_SRC_ID:
    case PingParser::State::WAIT_DST_ID:
        rxBuffer_[rxCount_++] = data;
        state_++;
        if (payloadLength_ == 0) {
            // no payload bytes, so we skip WAIT_PAYLOAD state
            state_++;
        }
        break;
    case PingParser::State::WAIT_PAYLOAD:
        rxBuffer_[rxCount_++] = data;
        if (--payloadLength_ == 0) {
            state_++;
        }
        break;
    case PingParser::State::WAIT_CHECKSUM_L:
        rxBuffer_[rxCount_++] = data;
        state_++;
        break;
    case PingParser::State::WAIT_CHECKSUM_H:
        rxBuffer_[rxCount_++] = data;
        state_ = PingParser::State::WAIT_START;
        if (rxMessage.verifyChecksum()) {
            parsed++;
            return PingParser::State::NEW_MESSAGE;
        } else {
            errors++;
            return PingParser::State::ERROR;
        }
    default:
        return state_;
    }
    return state_;
}
