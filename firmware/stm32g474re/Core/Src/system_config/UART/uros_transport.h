#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <rmw_microros/rmw_microros.h>

bool uros_transport_open(uxrCustomTransport* transport);
bool uros_transport_close(uxrCustomTransport* transport);
size_t uros_transport_write(
        uxrCustomTransport* transport,
        const uint8_t* buffer,
        size_t length,
        uint8_t* errcode);
size_t uros_transport_read(
        uxrCustomTransport* transport,
        uint8_t* buffer,
        size_t length,
        int timeout,
        uint8_t* errcode);


