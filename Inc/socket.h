/**
 ******************************************************************************
 * @file           : socket.h
 * @brief          : Header for socket.c file.
 *                   This file contains socket layer on top of the
 *                   Wiznet W5500 chip
 ******************************************************************************
 */
/**
 ******************************************************************************
 * MIT License
 *
 * Copyright (c) 2020 John Vedder
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this hardware, software, and associated documentation files (the "Product"),
 * to deal in the Product without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Product, and to permit persons to whom the Product is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Product.
 *
 * THE PRODUCT IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE PRODUCT OR THE USE OR OTHER DEALINGS IN THE
 * PRODUCT.
 *
 ******************************************************************************
 */


/* Prevent recursive inclusion */
#ifndef SOCKET_H
#define SOCKET_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  Include files
 */


/**
 *  Public Defines and Macros
 */

/* Socket error codes */
#define SOCK_OK                     0
#define SOCK_ERR                    1
#define SOCK_ERR_BUSY               2
#define SOCK_ERR_CLOSED             3
#define SOCK_ERR_STATUS             4
#define SOCK_ERR_TIMEOUT            5
#define SOCK_ERR_MODE               6
#define SOCK_ERR_PARAM              7

/* Convenience Names for Socket N Mode Register */
#define SOCK_STREAM                 W5500_SnMR_TCP
#define SOCK_DATAGRAM               W5500_SnMR_UDP


/**
 *  Public Function Prototypes
 */
uint8_t SOCK_socket(uint8_t sn, uint8_t protocol, uint16_t port, uint8_t flag);
uint8_t SOCK_close(uint8_t sn);
uint8_t SOCK_listen(uint8_t sn);
uint8_t SOCK_connect(uint8_t sn, uint8_t *addr, uint16_t port);
uint8_t SOCK_disconnect(uint8_t sn);
uint16_t SOCK_send(uint8_t sn, uint8_t *buf, uint16_t len);
uint16_t SOCK_recv(uint8_t sn, uint8_t *buf, uint16_t len);


#ifdef __cplusplus
}
#endif

#endif /* SOCKET_H */
