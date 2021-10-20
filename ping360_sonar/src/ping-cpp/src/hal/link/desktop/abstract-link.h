#pragma once

#include <memory>
#include <vector>

#include <boost/signals2.hpp>

#include "../ping-port.h"

/**
 * @brief AbstractLink for desktop like links
 *
 */
class AbstractLink : public PingPort {
public:
    AbstractLink() = default;
    virtual ~AbstractLink() = default;

    /**
     * @brief Construct and return an owner smart pointer for the link class
     *
     * @param url Should follow TYPE:HOST:CONFIGURATION format.
     *  Eg: udp:0.0.0.0:12345 or serial:/dev/ttyUSB0:115200
     *  Possible types right now are only udp and serial.
     *
     * @return std::shared_ptr<AbstractLink>
     */
    static std::shared_ptr<AbstractLink> openUrl(const std::string& url);

    /**
     * @brief Close link connection
     *
     */
    virtual void close() = 0;

    /**
     * @brief Check if link is open
     *
     * @return true
     * @return false
     */
    virtual bool isOpen() = 0;

    /**
     * @brief Allow connections via functions or binds when data is received from link
     *
     * @param slot
     * @return boost::signals2::connection
     */
    boost::signals2::connection doOnReceived(std::function<void(std::vector<uint8_t>)> slot)
    {
        return _onReceived.connect(slot);
    }

protected:
    boost::signals2::signal<void(std::vector<uint8_t>)> _onReceived;

private:
    // Shame on you, this is and should not be shared!
    // This class should only be an abstract class that provides the construction for any registered link
    AbstractLink(const AbstractLink&) = delete;
    AbstractLink& operator=(const AbstractLink&) = delete;

    static const char* _urlStringRegex;
};
