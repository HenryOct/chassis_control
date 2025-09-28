#ifndef UART_HPP
#define UART_HPP

#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"

inline sp::DBus remote(&huart3);

#endif  // UART_HPP