#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "serial-link.h"

using boost::asio::serial_port_base;

SerialLink::SerialLink(const std::string& port, uint32_t baudrate)
    : AbstractLink()
    , _context()
    , _serialPort(_context.eventLoop)
    , _rxBuffer(4096)
{
    using namespace boost;

    // Open and do the necessary configuration for the serial port
    try {
        _serialPort.open(port);
        _serialPort.set_option(serial_port_base::baud_rate(baudrate));
        _serialPort.set_option(serial_port_base::character_size(8));
        _serialPort.set_option(serial_port_base::parity(serial_port_base::parity::none));
        _serialPort.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
        _serialPort.set_option(asio::serial_port_base::character_size(8));

        // We should remove flow control to make sure that will work in any case
        _serialPort.set_option(asio::serial_port::flow_control(asio::serial_port::flow_control::none));
    } catch (const std::exception& exception) {
        std::cerr << "Error while configuring serial port: " << exception.what() << std::endl;
        close();
        return;
    }

    // Bind event loop start with the async read schedulement
    _context.eventLoop.post(boost::bind(&SerialLink::bindRead, this));

    runEventLoop();

    // Start Automatic Baud Rate procedure
    _serialPort.send_break();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    _serialPort.write_some(asio::buffer("UUUUUUUUUU"));
}

SerialLink::~SerialLink()
{
    close();

    // Ask context to finish and wait for it
    _context.run = false;
    _context.future.wait();
}

void SerialLink::runEventLoop()
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
                std::cerr << "Event loop failed to run: " << exception.what() << std::endl;
            }
        }
        _context.eventLoop.stop();
    });
}

void SerialLink::bindRead()
{
    using namespace boost;

    _serialPort.async_read_some(asio::buffer(_rxBuffer),
        boost::bind(
            &SerialLink::doRead, this, asio::placeholders::error, asio::placeholders::bytes_transferred));
}

void SerialLink::doRead(boost::system::error_code error, size_t bytesReceived)
{
    if (error) {
        // The read can canceled by someone else or when class is deleted
        if (error.value() == boost::system::errc::operation_canceled) {
            return;
        }
        // That's sad, something is really wrong!
        std::cerr << "Error while reading from serial port: " << error.message() << std::endl;
        // Close the serial port and abort
        if (_serialPort.is_open()) {
            _serialPort.close();
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

int SerialLink::read(uint8_t* buffer, int nBytes)
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

int SerialLink::write(const uint8_t* data, int nBytes)
{
    _serialPort.async_write_some(boost::asio::buffer(data, nBytes),
        [](const boost::system::error_code& error, std::size_t /*bytes_transferred*/) {
            if (error) {
                std::cout << "Error while writing in serial port: " << error.message() << std::endl;
            }
        });

    return nBytes;
}
