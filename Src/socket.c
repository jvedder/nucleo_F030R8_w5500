/**
 ******************************************************************************
 * @file           : socket.h
 * @brief          : This file contains socket layer on top of the
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


/**
 *  Include files
 */
#include "w5500.h"
#include "socket.h"

/**
 *  Private Function Prototypes
 */


/**
 * @brief   Opens the specified socket to the specified port
 * and protocol. Waits for completion.
 *
 * @param  sn: The socket number to open. Must be in range 0 to 7.
 * @param  protocol: The protocol to use on the socket.
 * @param  port: The local port number to use with this socket.
 * @param  flag: Flag at add the Socket N Command Register.
 * @retval Error status.
 */
uint8_t SOCK_socket(uint8_t sn, uint8_t protocol, uint16_t port, uint8_t flag)
{
    /* Validate Socket Range is 0 to 7 */
    if (sn > 7) return SOCK_ERR;

    /* Check that source IP address has been set */
    uint32_t ip_addr;
    W5500_ReadSIPR((uint8_t* ) &ip_addr);
    if (ip_addr == 0) return SOCK_ERR;

    /* Get port if not already assigned */
    if (port == 0)
    {
        port = 10000 + sn;
    }

    /* Check socket is closed before opening */
    if (SOCK_close(sn) != SOCK_OK) return SOCK_ERR;

    /* Set protocol and port */
    W5500_WriteSnMR(sn, protocol);
    W5500_WriteSnPORT(sn, port);

    /* open the socket */
    W5500_ExecuteSnCmd(sn, W5500_SnCR_OPEN);
    return SOCK_OK;
}

/**
 * @brief   Closes a socket and waits for completion.
 *
 * @param  sn: The socket number to close. Must be in range 0 to 7.
 * @retval Error status.
 */
uint8_t SOCK_close(uint8_t sn)
{
    /* Validate Socket Range is 0 to 7 */
    if (sn > 7) return SOCK_ERR;

    /* issue socket close command */
    W5500_ExecuteSnCmd(sn, W5500_SnCR_CLOSE);

    /* clear all interrupts of the socket. */
    W5500_WriteSnIR(sn, 0xFF);

    return SOCK_OK;
}

/**
 * @brief   Configures an open socket in TCP mode to listen for an
 * incoming connection,
 *
 * @param  sn: The socket number to close. Must be in range 0 to 7.
 * @retval Error status.
 */
uint8_t SOCK_listen(uint8_t sn)
{
    /* Validate Socket Range is 0 to 7 */
    if (sn > 7) return SOCK_ERR;

    /* check that socket is in TCP mode */
    if ((W5500_ReadSnMR(sn) & W5500_SnMR_PROTOCOL) != W5500_SnMR_TCP)
    {
        return SOCK_ERR;
    }

    /*check that the socket is initialized */
    if ((W5500_ReadSnSR(sn) != W5500_SnSR_INIT)) return SOCK_ERR;

    /* set the socket to listen mode */
    W5500_ExecuteSnCmd(sn, W5500_SnCR_LISTEN);

    /* confirm that the socket is listening */
    if (W5500_ReadSnSR(sn) != W5500_SnCR_LISTEN)
    {
        SOCK_close(sn);
        return SOCK_ERR;
    }
    return SOCK_OK;
}

/**
 * @brief   Configures an open socket in TCP mode to connect to
 * a remote server.
 *
 * @param  sn: The socket number to close. Must be in range 0 to 7.
 * @param  addr: pointer to the IPV4 address (4 bytes) of the remote server.
 * @param  port: port of remote server
 * @retval Error status.
 */
uint8_t SOCK_connect(uint8_t sn, uint8_t *addr, uint16_t port)
{
    /* Validate Socket Range is 0 to 7 */
    if (sn > 7) return SOCK_ERR;

    /* check that socket is in TCP mode */
    if ((W5500_ReadSnMR(sn) & W5500_SnMR_PROTOCOL) != W5500_SnMR_TCP)
    {
        return SOCK_ERR;
    }

    /* Check IP address is not 255.255.255.255 */
    if ((addr[0] == 0xFF) && (addr[1] == 0xFF) && (addr[2] == 0xFF)
            && (addr[3] == 0xFF))
    {
        return SOCK_ERR;
    }

    /* Check IP address is not 0.0.0.0 */
    if ((addr[0] == 0xFF) && (addr[1] == 0xFF) && (addr[2] == 0xFF)
            && (addr[3] == 0xFF))
    {
        return SOCK_ERR;
    }

    /* Check that the remote port is not zero */
    if (port == 0) return SOCK_ERR;

    /*Check that the socket is initialized */
    if ((W5500_ReadSnSR(sn) != W5500_SnSR_INIT)) return SOCK_ERR;

    /* Set remote address and port for socket */
    W5500_WriteSnDIPR(sn, addr);
    W5500_WriteSnDPORT(sn, port);

    /* Do connection */
    W5500_ExecuteSnCmd(sn, W5500_SnCR_CONNECT);

    /* Wait until connection is established */
    while (W5500_ReadSnSR(sn) != W5500_SnSR_ESTABLISHED)
    {
        /* Check for timeout interrupt flag */
        if (W5500_ReadSnIR(sn) & W5500_SnIR_TIMEOUT)
        {
            /* Clear interrupt flag*/
            W5500_WriteSnIR(sn, W5500_SnIR_TIMEOUT);
            return SOCK_ERR_TIMEOUT;
        }

        /* Check if socket was closed (by remote host?) */
        if (W5500_ReadSnSR(sn) == W5500_SnSR_CLOSED)
        {
            return SOCK_ERR_CLOSED;
        }
    }

    return SOCK_OK;
}

/**
 * @brief   Disconnects a socket from the remote server.
 *
 * @param  sn: The socket number to close. Must be in range 0 to 7.
 * @retval Error status.
 */
uint8_t SOCK_disconnect(uint8_t sn)
{
    /* Validate Socket Range is 0 to 7 */
    if (sn > 7) return SOCK_ERR;

    /* check that socket is in TCP mode */
    if ((W5500_ReadSnMR(sn) & W5500_SnMR_PROTOCOL) != W5500_SnMR_TCP)
    {
        return SOCK_ERR;
    }

    /* do disconnect */
    W5500_ExecuteSnCmd(sn, W5500_SnCR_DISCON);

    /* wait for the socket to close */
    while (W5500_ReadSnSR(sn) != W5500_SnSR_CLOSED)
    {
        /* Check for timeout interrupt flag */
        if (W5500_ReadSnIR(sn) & W5500_SnIR_TIMEOUT)
        {
            /* Close the socket (which clear interrupt flags */
            SOCK_close(sn);
            return SOCK_ERR_TIMEOUT;
        }
    }
    return SOCK_OK;
}

/**
 * @brief   Sends data to a socket open in TCP mode. Blocks until
 * data can be loaded into W5500 chip.
 *
 * @param  sn: The socket number to close. Must be in range 0 to 7.
 * @param  *buf: pointer to the local buffer to write.
 * @param  len: number of the bytes to write.
 * @retval Number of bytes sent or zero on error.
 */
uint16_t SOCK_send(uint8_t sn, uint8_t *buf, uint16_t len)
{
    /* Validate Socket Range is 0 to 7 */
    if (sn > 7) return 0;

    /* check that socket is in TCP mode */
    if ((W5500_ReadSnMR(sn) & W5500_SnMR_PROTOCOL) != W5500_SnMR_TCP)
    {
        return 0;
    }

    /* validate the data length */
    if (len == 0) return 0;
    if (len > W5500_TXBUF_SIZE) len = W5500_TXBUF_SIZE;

    /* check the socket status */
    uint8_t status = W5500_ReadSnSR(sn);
    if (status != W5500_SnSR_ESTABLISHED && status != W5500_SnSR_CLOSE_WAIT)
    {
        return 0;
    }

    /* wait for enough space to free up for the whole message */
    while (len <= W5500_ReadSnTX_FSR(sn))
    {
        status = W5500_ReadSnSR(sn);
        if (status != W5500_SnSR_ESTABLISHED && status != W5500_SnSR_CLOSE_WAIT)
        {
            return 0;
        }
    }
    /* TODO: possibly inline SOCK_send_TX_data() */
    W5500_WriteTxBuffer(sn, buf, len);
    W5500_ExecuteSnCmd(sn, W5500_SnCR_SEND);

    return len;
}

/**
 * @brief   Receives data from a socket open in TCP mode.
 *
 * @param  sn: The socket number to close. Must be in range 0 to 7.
 * @param  *buf: pointer to the local buffer to receive the data.
 * @param  len: length of the receive buffer in bytes.
 * @retval Number of bytes received or zero on error.
 */
uint16_t SOCK_recv(uint8_t sn, uint8_t *buf, uint16_t len)
{
    /* Validate Socket Range is 0 to 7 */
    if (sn > 7) return 0;

    /* check that socket is in TCP mode */
    if ((W5500_ReadSnMR(sn) & W5500_SnMR_PROTOCOL) != W5500_SnMR_TCP)
    {
        return 0;
    }

    /* validate the data length */
    if (len == 0) return 0;

    /* check the socket status */
    uint8_t status = W5500_ReadSnSR(sn);
    if (status != W5500_SnSR_ESTABLISHED && status != W5500_SnSR_CLOSE_WAIT)
    {
        return 0;
    }

    /* wait for receive data to be available */
    uint16_t recvsize = 0;
    while ((recvsize = W5500_ReadSnRX_RSR(sn)) == 0)
    {
        status = W5500_ReadSnSR(sn);
        if (status != W5500_SnSR_ESTABLISHED && status != W5500_SnSR_CLOSE_WAIT)
        {
            return 0;
        }
    }

    if (recvsize < len) len = recvsize;
    W5500_ReadRXBuffer(sn, buf, len);
    W5500_ExecuteSnCmd(sn, W5500_SnCR_RECV);

    return len;
}
