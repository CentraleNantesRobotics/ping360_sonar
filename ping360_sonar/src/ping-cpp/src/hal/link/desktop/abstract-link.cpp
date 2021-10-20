#include "abstract-link.h"
#include "serial-link.h"
#include "udp-link.h"

// TODO: Remove this
#include <iostream>

#include <boost/format.hpp>
#include <boost/xpressive/xpressive.hpp>

/**
 * @brief Accepts inputs with format described in `openUrl`
 *
 */
const char* AbstractLink::_urlStringRegex = R"((?P<type>udp|serial):(?P<host>.*):(?P<config>\d+))";

std::shared_ptr<AbstractLink> AbstractLink::openUrl(const std::string& url)
{
    if (url.empty()) {
        return {};
    }

    const auto regex = boost::xpressive::sregex::compile(_urlStringRegex);
    boost::xpressive::smatch match;

    struct {
        std::string type;
        std::string host;
        std::string config;
    } urlStruct;

    if (!regex_search(url, match, regex)) {
        return {};
    }

#if __cplusplus >= 201703L
#warning "Move match strings to string_view and concatenate it with regex string inside the named regex match"
#endif
    urlStruct.type = match["type"].str();
    urlStruct.host = match["host"].str();
    urlStruct.config = match["config"].str();

    if (urlStruct.type == "serial") {
        return std::make_shared<SerialLink>(urlStruct.host, std::stoi(urlStruct.config));
    } else if (urlStruct.type == "udp") {
        return std::make_shared<UdpLink>(urlStruct.host, urlStruct.config);
    }

    return {};
}
