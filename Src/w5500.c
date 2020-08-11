/**
 ******************************************************************************
 * @file           : w5500.c
 * @brief          : Driver for the Wiznet W5500 Ethernet Chip
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
#include "w5500.h"
#include <stdio.h>

/**
 *  Private Defines and Macros
 */

/* Control of SPI Chip Select to W5500 */
#define W5500_ASSERT_CS()    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET)
#define W5500_NEGATE_CS()    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET)

/* Critical Code Section Markers -- update if using threads to protect SPI transmissions */
#define W5500_CRITICAL_SPI_ENTER()
#define W5500_CRITICAL_SPI_EXIT()

/* Control printing of debug info the to console (UART) */
#define W5500_DEBUG        1

/**
 * Static buffers for building SPI packets to read/write registers.
 *
 * Note: Since these buffers are shared, all the Read and Write methods
 * in this module are NOT thread-safe.
 */
static uint8_t txbuf[8];
static uint8_t rxbuf[8];

/**
 * @brief Initializes the W5500.
 *
 * @param  None
 * @retval None
 */
void W5500_Init()
{
    W5500_NEGATE_CS();
    Delay_ms(10);
    W5500_SoftReset();

    /* set all 8 socket buffers to 2 KB Tx and 2 KB Rx */
    for (uint8_t s=0; s<W5500_MAX_SOCKET_NUM; s++)
    {
        W5500_WriteSnRXBUF_SIZE(s, W5500_RXBUF_SIZE >> 10);
        W5500_WriteSnTXBUF_SIZE(s, W5500_TXBUF_SIZE >> 10);
    }
}

/**
 * @brief Performs a software reset of the W5500.
 *
 * @param  None
 * @retval None
 */
void W5500_SoftReset( )
{
    W5500_WriteMR(W5500_MR_RST);
    Delay_ms(250);
}

/**
 * @brief Writes to a 8-bit register in the W5500.
 *
 * This routine is not thread-safe.
 *
 * @param  bsb: The block select bits to use in the control byte.
 * @param  reg: The address of the register to write. Register addresses are 8-bits.
 * @param  val: The value to write to the register.
 * @retval None
 */
void W5500_WriteReg8(uint8_t bsb, uint8_t reg, uint8_t val)
{
    /* SPI Packet: [0:RegAddrHi][1:RegAddrLo][2:ControlByte][3:Value] */

    /* SPI packet length */
    const uint16_t len = 4;

    /* Register Address (8-bit, so first byte is zero) */
    txbuf[0] = 0x00;
    txbuf[1] = reg;

    /* Control Byte */
    txbuf[2] = bsb | W5500_CB_WRITE;

    /* Register Value (b-bit) */
    txbuf[3] = val;

    if (W5500_DEBUG) printf("WriteReg8(0x%02X, 0x%02X, 0x%02X)\r\n", (uint16_t)bsb, (uint16_t)reg, (uint16_t)val);

    /* Write SPI packet */
    W5500_CRITICAL_SPI_ENTER();
    W5500_ASSERT_CS();
    Delay_ms(1);
    HAL_SPI_Transmit(&hspi2, txbuf, len, TIMEOUT_1_SEC);
    W5500_NEGATE_CS();
    W5500_CRITICAL_SPI_EXIT();
}

/**
 * @brief Writes to a 16-bit Register in the W5500.
 *
 * This method is not thread-safe.
 *
 * @param  bsb: The block select bits to use in the control byte.
 * @param  reg: The address of the register to write. Register addresses are 8-bits.
 * @param  val: The value to write to the register.
 * @retval None
 */
void W5500_WriteReg16(uint8_t bsb, uint8_t reg, uint16_t val)
{
    /* SPI Packet: [0:RegAddrHi][1:RegAddrLo][2:ControlByte][3:ValueHi][4:ValueLo] */

    /* SPI packet length */
    const uint16_t len = 5;

    /* Register Address (8-bit) */
    txbuf[0] = 0x00;
    txbuf[1] = reg;

    /* Control Byte */
    txbuf[2] = bsb | W5500_CB_WRITE;

    /* Register Value (16-bit) */
    txbuf[3] = (uint8_t) (val >> 8);
    txbuf[4] = (uint8_t) (val & 0xFF);

    if (W5500_DEBUG) printf("WriteReg16(0x%02X, 0x%02X, 0x%04X)\r\n", (uint16_t)bsb, (uint16_t)reg, (uint16_t)val);

    /* Write SPI packet */
    W5500_CRITICAL_SPI_ENTER();
    W5500_ASSERT_CS();
    Delay_ms(1);
    HAL_SPI_Transmit(&hspi2, txbuf, len, TIMEOUT_1_SEC);
    W5500_NEGATE_CS();
    W5500_CRITICAL_SPI_EXIT();
}

/**
 * @brief Reads an 8-bit register in the W5500.
 *
 * This method is not thread-safe.
 *
 * @param  bsb: The block select bits to use in the control byte.
 * @param  reg: The address of the register to read. Register addresses are 8-bits.
 * @retval The value read from the register.
 */
uint8_t W5500_ReadReg8(uint8_t bsb, uint8_t reg)
{
    /* SPI Packet: [0:RegAddrHi][1:RegAddrLo][2:ControlByte][3:Value] */

    /* SPI packet length */
    const uint16_t len = 4;

    /* Register Address (8-bit) */
    txbuf[0] = 0x00;
    txbuf[1] = reg;

    /* Control Byte */
    txbuf[2] = bsb | W5500_CB_READ;

    /* Zero the read value (8-bits) */
    rxbuf[3] = 0x00;

    /* Write & Read SPI packets */
    W5500_CRITICAL_SPI_ENTER();
    W5500_ASSERT_CS();
    Delay_ms(1);
    HAL_SPI_TransmitReceive(&hspi2, txbuf, rxbuf, len, TIMEOUT_1_SEC);
    W5500_NEGATE_CS();
    W5500_CRITICAL_SPI_EXIT();

    if (W5500_DEBUG) printf("ReadReg8(0x%02X, 0x%02X)=0x%02X\r\n", (uint16_t)bsb, (uint16_t)reg, (uint16_t)rxbuf[3]);

    /* Read value (8-bits) */
    return rxbuf[3];
}


/**
 * @brief Reads an 16-bit register in the W5500.
 *
 * This method is not thread-safe.
 *
 * @param  bsb: The block select bits to use in the control byte.
 * @param  reg: The address of the register to read. Register addresses are 8-bits.
 * @retval The value read from the register.
 */
uint16_t W5500_ReadReg16(uint8_t bsb, uint8_t reg)
{
    /* SPI Packet: [0:RegAddrHi][1:RegAddrLo][2:ControlByte][3:ValueHi][4:ValueLo] */

    /* SPI packet length */
    const uint16_t len = 5;

    /* Register Address (8-bit) */
    txbuf[0] = 0x00;
    txbuf[1] = reg;

    /* Control Byte */
    txbuf[2] = bsb | W5500_CB_READ;

    /* Zero the read value (16-bits) */
    rxbuf[3] = 0x00;
    rxbuf[4] = 0x00;


    /* Write & Read SPI packets */
    W5500_CRITICAL_SPI_ENTER();
    W5500_ASSERT_CS();
    Delay_ms(1);
    HAL_SPI_TransmitReceive(&hspi2, txbuf, rxbuf, len, TIMEOUT_1_SEC);
    W5500_NEGATE_CS();
    W5500_CRITICAL_SPI_EXIT();

    /* Read value (16-bits) */
    uint16_t ret = (rxbuf[3] << 8) | rxbuf[4];

    if (W5500_DEBUG) printf("ReadReg16(0x%02X, 0x%02X)=0x%04X\r\n", (uint16_t)bsb, (uint16_t)reg, ret);

    return ret;
}

/**
 * @brief Reads and validates a 16-bit register in the W5500. The W5500
 * datasheet notes that reading some 16-bit registers is not atomic. This
 * method implements the recommended process of repeatedly reading the
 * register until 2 successive reads return the same value.
 *
 * This method is not thread-safe.
 *
 * @param  bsb: The block select bits to use in the control byte.
 * @param  reg: The address of the register to read. Register addresses are 8-bits.
 * @retval The validated value read from the register.
 */
uint16_t W5500_ReadReg16Val(uint8_t bsb, uint8_t reg)
{
    uint16_t val1 = 0;
    uint16_t val2 = W5500_ReadReg16(bsb, reg);
    do
    {
        val1 = val2;
        val2 = W5500_ReadReg16(bsb, reg);
    } while (val1 != val2);

    return val1;
}

/**
 * @brief Writes to a multi-byte register or buffer in the W5500.
 *
 * This method is not thread-safe.
 *
 * @param  bsb: The block select bits to use in the control byte.
 * @param  addr: The 16-bit address in the W5500 to begin writing.
 * @param  *buf: pointer to the local buffer containing the data.
 * @param  len: number of the bytes to write.
 * @retval None
 */
void W5500_WriteBuf(uint8_t bsb, uint16_t addr, uint8_t *buf, uint16_t len)
{
    /* SPI Packet: [0:RegAddrHi][1:RegAddrLo][2:ControlByte][3:Data0]...[N+3:DataN] */

    /* header length */
    const uint16_t header_len = 3;

    /* Address */
    txbuf[0] = (uint8_t) (addr >> 8);
    txbuf[1] = (uint8_t) (addr & 0xFF);

    /* Control Byte */
    txbuf[2] = bsb | W5500_CB_WRITE;

    if (W5500_DEBUG) printf("WriteBuf(0x%02X, 0x%04X, &0x%08lX, 0x%04X)\r\n", (uint16_t)bsb, (uint16_t)addr, (uint32_t) buf, (uint16_t)len);

    /* Write 2-part SPI packet */
    W5500_CRITICAL_SPI_ENTER();
    W5500_ASSERT_CS();
    Delay_ms(1);

    /* Write header */
    HAL_SPI_Transmit(&hspi2, txbuf, header_len, TIMEOUT_1_SEC);

    /* Write buffer */
    HAL_SPI_Transmit(&hspi2, buf, len, TIMEOUT_1_SEC);

    W5500_NEGATE_CS();
    W5500_CRITICAL_SPI_EXIT();
}

/**
 * @brief Reads a multi-byte register or buffer from the W5500.
 *
 * Buffer space for storing the read bytes must be allocated by the caller.
 *
 * This method is not thread-safe.
 *
 * @param  bsb: The block select bits to use in the control byte.
 * @param  addr: The 16-bit address in the W5500 to begin reading.
 * @param  *buf: pointer to the local buffer to store read data.
 * @param  len: number of the bytes to read.
 * @retval None
 */
void W5500_ReadBuf(uint8_t bsb, uint16_t addr, uint8_t *buf, uint16_t len)
{
    /* SPI Packet: [0:RegAddrHi][1:RegAddrLo][2:ControlByte][3:Data0]...[N+3:DataN] */

    /* header length */
    const uint16_t header_len = 3;

    /* Address */
    txbuf[0] = (uint8_t) (addr >> 8);
    txbuf[1] = (uint8_t) (addr & 0xFF);

    /* Control Byte */
    txbuf[2] = bsb | W5500_CB_READ;

    if (W5500_DEBUG) printf("ReadBuf(0x%02X, 0x%04X, &0x%08lX, 0x%04X)\r\n", (uint16_t)bsb, (uint16_t)addr, (uint32_t) buf, (uint16_t)len);

    /* Write 2-part SPI packet */
    W5500_CRITICAL_SPI_ENTER();
    W5500_ASSERT_CS();
    Delay_ms(1);

    /* Write header */
    HAL_SPI_Transmit(&hspi2, txbuf, header_len, TIMEOUT_1_SEC);

    /* Write buffer */
    HAL_SPI_Receive(&hspi2, buf, len, TIMEOUT_1_SEC);

    W5500_NEGATE_CS();
    W5500_CRITICAL_SPI_EXIT();
}

/**
 * @brief  Utility function to write data to the W5500 Transmit Buffer for the specified socket
 * based on the pointers in the Socket's registers. Supports the case where data wraps
 * around past the end of the W5500 circular buffer. Caller is responsible for determining
 * the amount of W5500 transmit buffer space available.
 *
 * Note: This method uses the socket number (sn), not block select bits (bsb), as an argument.
 *
 * This method is not thread-safe.
 *
 * @param  sn: The socket number.
 * @param  *buf: pointer to the local buffer containing the data.
 * @param  len: number of the bytes to write.
 * @retval None
 *
 */
void W5500_WriteTxBuffer(uint8_t sn, uint8_t *buf, uint16_t len)
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
 * @brief  Utility function to read data from the W5500 Rx Buffer for the specified socket
 * based on the pointers in the Socket's registers. Also supports the case where data wraps
 * around past the end of the W5500 circular buffer. Caller is responsible for determining
 * the amount of W5500 receive data available.
 *
 * Buffer space for storing the read bytes must be allocated by the caller.
 *
 * Note: This method uses the socket number (sn), not block select bits (bsb), as an argument.
 *
 * This method is not thread-safe.
 *
 * @param  sn: The socket number.
 * @param  *buf: pointer to the local buffer to receive the data.
 * @param  len: number of the bytes to read.
 * @retval None
 *
 */
void W5500_ReadRXBuffer(uint8_t sn, uint8_t *buf, uint16_t len)
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

/**
 * @brief A convenience method to issue a command on the specified socket
 * and wait for the W5500 to accept it.
 *
 * @param  sn: The socket number.
 * @param  cmd: The command to execute.
 * @retval None
 */
void W5500_ExecuteSnCmd(uint8_t sn, uint8_t cmd)
{
    /* Send command to socket command register */
    W5500_WriteReg8(W5500_CB_SnREG(sn), W5500_REG_SnCR, cmd);

    /*  Wait for command to be accepted */
    while (W5500_ReadReg8(W5500_CB_SnREG(sn), W5500_REG_SnCR))
    {
        /* spin wait */
        /* TODO: add timeout */
    }
}
