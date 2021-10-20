#pragma once

/**
 * @brief Abstract namespace to allow usage between different implementations
 *
 */
namespace PingTime {
void microsecondDelay(unsigned int microseconds);
int timeMs();
void yeild();
}
