#ifndef __ERROR_HANDLER_H
#define __ERROR_HANDLER_H

enum ENC28j60_Error {
    SPI_ERROR
};

#ifdef __cplusplus
	{
#endif

void __attribute__((weak)) ENC28j60_Error_Handler(enum ENC28j60_Error error);

#ifdef __cplusplus
	}
#endif

#endif
