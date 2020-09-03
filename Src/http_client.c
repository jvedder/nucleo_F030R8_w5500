/**
 ******************************************************************************
 * @file:   http_client.c
 * @author: John Vedder
 * @brief:  Basic HTTP client implementation using W5500 and socket API
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

/**
 *  Include files
 */
#include "main.h"
#include "socket.h"
#include "api_key.h"
#include "http_client.h"
#include <stdio.h>

/**
 *  Private defines and macros
 */
#define DEBUG 1

#define HTTP_SOCKET     0       /* W5500 socket # to use */
#define HTTP_PORT       1000    /* local port number */

/**
 * Private variables
 */

static const char *request = 0;
static uint16_t length = 0;
#define RECV_BUFFER_LENGTH  2048
static uint8_t recv_buf[RECV_BUFFER_LENGTH];

#if 0
/**
 *  Remote IP address and port of server.
 *  (Hard coded here)
 */
static const uint8_t server_ip[] = {192, 168, 1, 33};
static const uint16_t server_port = 6434;

/**
 * Basic HTTP request with headers.
 * Ref: https://www.w3.org/Protocols/rfc2616/rfc2616-sec14.html
 */

#define CONN     "Connection: close\r\n"
#define TYPE     "Content-Type: text/plain;charset=UTF-8\r\n"
#define AGENT    "User-Agent: W5500 (Nucleo-F030R8)\r\n"

static const char GET_ABC[] = "GET /abc HTTP/1.1\r\n" CONN TYPE AGENT "\r\n";
static const char GET_XYZ[] = "GET /xyz HTTP/1.1\r\n" CONN TYPE AGENT "\r\n";

#else

/**
 *  Remote IP address and port of server.
 *  (Hard coded here)
 */
static const uint8_t server_ip[] =
{192, 168, 1, 113};
static const uint16_t server_port = 80;

/**
 * Basic HTTP request with headers.
 * Note: IP address and port are hard-coded here
 * Ref: https://www.w3.org/Protocols/rfc2616/rfc2616-sec14.html
 */
#define TYPE     "Content-Type: text/plain;charset=UTF-8\r\n"
#define HOST     "Host: 192.168.1.113\r\n"
#define CONN     "Connection: close\r\n"
#define AGENT    "User-Agent: W5500 (Nucleo-F030R8)\r\n"

#define LEN_OFF  "Content-Length: 13\r\n"
#define JSON_OFF "{\"on\":false}\r\n"

#define LEN_ON   "Content-Length: 31\r\n"
#define JSON_ON  "{\"on\":true,\"bri\":254,\"ct\":366}\r\n"

static const char LIGHT_ON[] =
        "PUT /api/" API_KEY "/lights/9/state HTTP/1.1\r\n" HOST CONN TYPE AGENT LEN_ON "\r\n" JSON_ON;
static const char LIGHT_OFF[] =
        "PUT /api/" API_KEY "/lights/9/state HTTP/1.1\r\n" HOST CONN TYPE AGENT LEN_OFF "\r\n" JSON_OFF;

#endif

/**
 *  Public Functions
 * */

/**
 * @brief  Makes an HTTP request os a remote server and prints the response
 * @param  state: zero to turn off; otherwise to turn on
 * @retval None
 */
void http_client_request(uint16_t state)
{
    if (DEBUG) printf("http_client_request( %u ) starting\r\n", state);

    uint8_t err = 0;

    /* Initialize TCP connection socket */
    if (DEBUG) printf("Calling SOCK_socket()\r\n");
    err = SOCK_socket(HTTP_SOCKET, SOCK_STREAM, HTTP_PORT);
    if (err != SOCK_OK)
    {
        if (DEBUG) printf("SOCK_ERR: %u\r\n", (uint16_t) err);
        return;
    }

    /* connect to remote server */
    if (DEBUG) printf("Calling SOCK_connect()\r\n");
    err = SOCK_connect(HTTP_SOCKET, server_ip, server_port);
    if (err != SOCK_OK)
    {
        if (DEBUG) printf("SOCK_ERR: %u\r\n", (uint16_t) err);
        return;
    }

    if (state)
    {
        request = LIGHT_ON;
        length = sizeof(LIGHT_ON);
    }
    else
    {
        request = LIGHT_OFF;
        length = sizeof(LIGHT_OFF);
    }

    /* send the request */
    if (DEBUG) printf("Calling SOCK_send(len=%u)\r\n", (uint16_t) length);
    length = SOCK_send(HTTP_SOCKET, (const uint8_t*) request, length);
    if (DEBUG) printf("length sent: %u\r\n", (uint16_t) length);
    if (length == 0) return;

    /* receive the response */
    if (DEBUG) printf("Calling SOCK_recv()\r\n");
    length = SOCK_recv(HTTP_SOCKET, recv_buf, RECV_BUFFER_LENGTH);
    if (DEBUG) printf("length recv: %u\r\n", (uint16_t) length);

    if (length == 0) return;

    if (length > (RECV_BUFFER_LENGTH - 1)) length = (RECV_BUFFER_LENGTH - 1);
    recv_buf[length] = '\0';

    if (DEBUG)
    {
        printf("### RESPONSE BEGIN ###\r\n");
        printf((const char*) recv_buf);
        printf("\r\n### RESPONSE END ###\r\n");
    }

    /* disconnect the socket */
    if (DEBUG) printf("Calling SOCK_disconnect()\r\n");
    err = SOCK_disconnect(HTTP_SOCKET);
    if (err != SOCK_OK)
    {
        if (DEBUG) printf("SOCK_ERR: %u\r\n", (uint16_t) err);
        return;
    }

    /* close the socket */
    if (DEBUG) printf("Calling SOCK_close()\r\n");
    err = SOCK_close(HTTP_SOCKET);
    if (err != SOCK_OK)
    {
        if (DEBUG) printf("SOCK_ERR: %u\r\n", (uint16_t) err);
        return;
    }

    if (DEBUG) printf("http_client_request() exiting\r\n");
}
