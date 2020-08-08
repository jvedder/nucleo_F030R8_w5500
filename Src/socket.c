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
static uint8_t SOCK_SnCommand(uint8_t s, uint8_t cmd);
#if 0
static void SOCK_write_TX_data(uint8_t sn, uint8_t *buf, uint16_t len);
static void SOCK_read_RX_data(uint8_t sn, uint8_t *buf, uint16_t len);
#endif


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
    return SOCK_SnCommand(sn, W5500_SnCR_OPEN);
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
    SOCK_SnCommand(sn, W5500_SnCR_CLOSE);

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
    SOCK_SnCommand(sn, W5500_SnCR_LISTEN);

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
    SOCK_SnCommand(sn, W5500_SnCR_CONNECT);

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
    SOCK_SnCommand(sn, W5500_SnCR_DISCON);

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
    SOCK_SnCommand(sn, W5500_SnCR_SEND);

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
    SOCK_SnCommand(sn, W5500_SnCR_RECV);

    return len;
}

/**
 * @brief A convenience method to issue the specified command on
 * the specified socket and wait for the W5500 to accept it.
 *
 * @param  sn: The socket number.
 * @param  cmd: The command to execute.
 * @retval Error status
 *
 * TODO: Add a timeout on the wait for completion.
 *
 */
static uint8_t SOCK_SnCommand(uint8_t sn, uint8_t cmd)
{
    /* Send command to socket command register */
    W5500_WriteSnCR(sn, cmd);

    /*  Wait for command to be accepted */
    while (W5500_ReadSnCR(sn))
    {
        /* spin wait */
        /* TODO: add timeout */
    }
    return SOCK_OK;
}

#if 0
/**
 * @brief  Utility function to send data to the W5500 Tx Buffer based on the Socket's Tx_WR register.
 * Also supports the case of data that wraps around past the end of the Tx buffer.
 */
static void SOCK_write_TX_data(uint8_t sn, uint8_t *buf, uint16_t len)
{
    uint16_t tx_ptr;
    uint16_t buf_ptr;

    /* get the transmit pointer */
    tx_ptr = W5500_ReadSnTX_WR(sn);

    /* mask it to be within the physical Tx buffer */
    buf_ptr = tx_ptr & W5500_TXBUF_MASK;

    /* If this data extends past the end of the TX buffer, it has
     * to wrap around to the beginning of the TX buffer. */
    if ((buf_ptr + len) > W5500_TXBUF_SIZE)
    {
        /* compute the size that fits to the end of TX buffer */
        uint16_t size = W5500_TXBUF_SIZE - buf_ptr;
        W5500_WriteSnTXBuf(sn, buf_ptr, buf, size);

        /* what's left goes in the beginning of the TX buffer */
        W5500_WriteSnTXBuf(sn, 0, (buf + size), (len - size));
    }
    else
    {
        /* does not wrap around end of Tx buffer */
        W5500_WriteSnTXBuf(sn, buf_ptr, buf, len);
    }

    /* indicate how much we have written into the Tx buffer */
    W5500_WriteSnTX_WR(sn, tx_ptr + len);
}

/**
 * @brief  Utility function to read data from the W5500 Rx Buffer based on the Socket's Rx_RD register.
 * Also supports the case of data that wraps around past the end of the Rx buffer.
 */
static void SOCK_read_RX_data(uint8_t sn, uint8_t *buf, uint16_t len)
{
    uint16_t rx_ptr;
    uint16_t buf_ptr;

    /* get the receive pointer */
    rx_ptr = W5500_ReadSnRX_RD(sn);

    /* mask it to be within the physical Rx buffer */
    buf_ptr = rx_ptr & W5500_RXBUF_MASK;

    /* If this data extends past the end of the Rx buffer, it is
     * wrapped around to the beginning of the Rx buffer. */
    if ((buf_ptr + len) > W5500_RXBUF_SIZE)
    {
        /* compute the size that fits to the end of Rx buffer */
        uint16_t size = W5500_RXBUF_SIZE - buf_ptr;
        W5500_ReadSnRXBuf(sn, buf_ptr, buf, size);

        /* what's left is at the beginning of he Rx buffer */
        W5500_ReadSnRXBuf(sn, 0, (buf + size), (len - size));
    }
    else
    {
        /* does not wrap around end of Rx buffer */
        W5500_ReadSnRXBuf(sn, buf_ptr, buf, len);
    }

    /* indicate how much we have read from the Rx buffer*/
    W5500_WriteSnRX_RD(sn, rx_ptr + len);
}
#endif
