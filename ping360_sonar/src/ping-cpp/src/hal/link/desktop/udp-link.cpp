#include <memory>
#include <string>

#include <iostream>

#include "udp-link.h"

UdpLink::UdpLink(const std::string& host, const std::string& port)
    : AbstractLink()
    , _context()
    , _socket(_context.eventLoop)
    , _rxBuffer(4096)
{
    using namespace boost;

    // Identify endpoint from host and port
    _endpoint = asio::ip::udp::endpoint(asio::ip::make_address(host), std::stoi(port));

    // Do async connection request, schedule an async read operation if connection was successful
    _socket.async_connect(_endpoint, boost::bind(&UdpLink::connectionHandle, this, asio::placeholders::error));

    runEventLoop();
}

UdpLink::~UdpLink()
{
    close();

    // Ask context to finish and wait for it
    _context.run = false;
    _context.future.get();
}

void UdpLink::runEventLoop()
{
    // Allow event loop to run, for now
    _context.run = true;
    _context.future = std::async(std::launch::async, [this] {
        // Run event loop
        while (_context.run) {
            using namespace std::chrono_literals;
            try {
                _context.eventLoop.run_for(100ms);
            } catch (const std::exception& exception) {
                std::cerr << "Event loop failed to run:" << exception.what() << std::endl;
            }
        }
    });
}

void UdpLink::connectionHandle(boost::system::error_code const& error)
{
    if (error) {
        std::cerr << "Error while connecting with socket:" << error.message() << std::endl;
        return;
    }

    bindRead();

    // Send something to allow UDP server to know that we exist
    write({0, 255}); // Send a full low byte and a full high byte to trigger any hardware event
}

void UdpLink::bindRead()
{
    using namespace boost;

    _socket.async_receive(asio::buffer(_rxBuffer),
        boost::bind(
            &UdpLink::doRead, this, asio::placeholders::error, asio::placeholders::bytes_transferred));
}

void UdpLink::doRead(boost::system::error_code error, size_t bytesReceived)
{
    if (error) {
        // The read can canceled by someone else or when class is deleted
        if (error.value() == boost::system::errc::operation_canceled) {
            return;
        }
        // That's sad, something is really wrong!
        std::cerr << "Error while reading socket: " << error.message() << std::endl;
        // Close the socket and abort
        if (_socket.is_open()) {
            _socket.close();
        }

        return;
    }

    // We got something
    if (bytesReceived) {
        // Notify signal that we got something
        const std::vector<uint8_t> output(std::cbegin(_rxBuffer), std::next(std::cbegin(_rxBuffer), bytesReceived));
        // Emit signal
        _onReceived(output);

        // Update our shared buffer
        _linkBuffer.mutex.lock();
        _linkBuffer.data.insert(
            std::end(_linkBuffer.data), std::begin(_rxBuffer), std::next(std::begin(_rxBuffer), bytesReceived));
        _linkBuffer.mutex.unlock();
    }

    // Schedule a new async read
    bindRead();
}

int UdpLink::read(uint8_t* buffer, int nBytes)
{
    _linkBuffer.mutex.lock();
    // Check for the valid amount of data that we can give
    const int amount = std::min(nBytes, static_cast<int>(_linkBuffer.data.size()));
    std::copy_n(std::begin(_linkBuffer.data), amount, buffer);
    // Erase bytes that was copied and update out shared buffer
    _linkBuffer.data.erase(_linkBuffer.data.begin(), std::next(_linkBuffer.data.begin(), amount));
    _linkBuffer.mutex.unlock();
    return amount;
}

int UdpLink::write(const uint8_t* data, int nBytes)
{
    _socket.async_send(
        boost::asio::buffer(data, nBytes), [this](boost::system::error_code error, size_t /*bytes_transferred*/) {
            if (error) {
                std::cerr << "Error while sending data to socket: " << _endpoint.address().to_string() << ":"
                          << _endpoint.port() << std::endl;
                std::cerr << "Error: " << error.category().name() << " " << error.message() << std::endl;
            }
        });

    return nBytes;
}

void UdpLink::write(const std::vector<uint8_t>& vector) { write(vector.data(), vector.size()); }
