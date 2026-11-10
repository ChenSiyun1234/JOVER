#include "uros_transport.h"

#include <stddef.h>
#include <stdbool.h>
#include <rmw_microros/rmw_microros.h>

// TODO: implement stubs

bool uros_transport_open(uxrCustomTransport* transport) {
	return false;
}

bool uros_transport_close(uxrCustomTransport* transport) {
	return false;
}

size_t uros_transport_write(
        uxrCustomTransport* transport,
        const uint8_t* buffer,
        size_t length,
        uint8_t* errcode)
{
	return (size_t)0;
}

size_t uros_transport_read(
        uxrCustomTransport* transport,
        uint8_t* buffer,
        size_t length,
        int timeout,
        uint8_t* errcode)
{
	return (size_t)0;
}
