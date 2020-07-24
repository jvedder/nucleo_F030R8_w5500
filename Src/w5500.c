/**
 ******************************************************************************
 * @file           : w5500.c
 * @brief          : Driver for the Wiznet W5500 Ethernet Chip
 ******************************************************************************
 * @attention
 *
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



#include "main.h"
#include "w5500.h"
#include <stdio.h>

#define ASSERT_SPI_CS    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET)
#define NEGATE_SPI_CS    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET)

typedef enum regsize {REG8 = 0, REG16=1} regsize_t;

uint32_t failures = 0;

/* static buffers for building SPI packets to read/write registers */
static uint8_t txbuf[8];
static uint8_t rxbuf[8];

static uint16_t next_free_port = 1024;

/**
 * @brief Initializes the W5500.
 *
 * @param  None
 * @retval None
 */
void W5500_Init()
{
    NEGATE_SPI_CS;
    Delay_ms(10);
    W5500_SoftReset();

    /* set all 8 socket buffers to 2 KB Tx and 2 KB Rx */
    for (uint8_t s=0; s<W5500_MAX_SOCKET_NUM; s++)
    {
        W5500_WriteSnRXBUF_SIZE(s, 2);
        W5500_WriteSnTXBUF_SIZE(s, 2);
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
 * @brief Execute a command on the specified socket.
 *
 * @param  s: The socket number.
 * @param  cmd: The command to execute.
 * @retval None
 *
 * TODO: Add a timeout on the wait for completion.
 */
void W5500_execSnCmd(uint8_t s, uint8_t cmd)
{
    /* Send command to socket command register */
    W5500_WriteSnCR(s, cmd);

    /*  Wait for command to complete */
    while (W5500_ReadSnCR(s))
    {
        /* spin wait */
        //TODO: add timeout
    }
}


/**
 * @brief Writes to a 8-bit register in the W5500.
 *
 * @param  bsb: The block select bits to use in the control byte.
 * @param  reg: The address of the register to write (assumed to be 8-bits).
 * @param  val: The value to write to the register.
 * @retval None
 */
void W5500_WriteReg8(uint8_t bsb, uint8_t reg, uint8_t val)
{
    /* SPI Packet: [0:RegAddrHi][1:RegAddrLo][2:ControlByte][3:Value] */

    /* SPI packet length */
    const uint16_t len = 4;

    /* Register Address (8-bit) */
    txbuf[0] = 0x00;
    txbuf[1] = reg;

    /* Control Byte */
    txbuf[2] = bsb | W5500_CB_WRITE;

    /* Register Value (b-bit) */
    txbuf[3] = val;

    printf("WriteReg8(0x%02X, 0x%02X, 0x%02X)\r\n", (uint16_t)bsb, (uint16_t)reg, (uint16_t)val);

    /* Write SPI packet */
    W5500_CRITICAL_ENTER();
    ASSERT_SPI_CS;
    Delay_ms(1);
    HAL_SPI_Transmit(&hspi2, txbuf, len, TIMEOUT_1_SEC);
    NEGATE_SPI_CS;
    W5500_CRITICAL_EXIT();
}

/**
 * @brief Writes to a 16-bit Register in the W5500.
 *
 * @param  bsb: The block select bits to use in the control byte.
 * @param  reg: The address of the register to write (assumed to be 8-bits).
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

    printf("WriteReg16(0x%02X, 0x%02X, 0x%04X)\r\n", (uint16_t)bsb, (uint16_t)reg, (uint16_t)val);

    /* Write SPI packet */
    W5500_CRITICAL_ENTER();
    ASSERT_SPI_CS;
    Delay_ms(1);
    HAL_SPI_Transmit(&hspi2, txbuf, len, TIMEOUT_1_SEC);
    NEGATE_SPI_CS;
    W5500_CRITICAL_EXIT();
}

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

    printf("ReadReg8(0x%02X, 0x%02X)\r\n", (uint16_t)bsb, (uint16_t)reg);

    /* Write & Read SPI packets */
    W5500_CRITICAL_ENTER();
    ASSERT_SPI_CS;
    Delay_ms(1);
    HAL_SPI_TransmitReceive(&hspi2, txbuf, rxbuf, len, TIMEOUT_1_SEC);
    NEGATE_SPI_CS;
    W5500_CRITICAL_EXIT();

    /* Read value (8-bits) */
    return rxbuf[3];
}

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

    printf("ReadReg16(0x%02X, 0x%02X)\r\n", (uint16_t)bsb, (uint16_t)reg);

    /* Write & Read SPI packets */
    W5500_CRITICAL_ENTER();
    ASSERT_SPI_CS;
    Delay_ms(1);
    HAL_SPI_TransmitReceive(&hspi2, txbuf, rxbuf, len, TIMEOUT_1_SEC);
    NEGATE_SPI_CS;
    W5500_CRITICAL_EXIT();

    /* Read value (16-bits) */
    return (rxbuf[3] << 8) | rxbuf[4];
}

/**
 * @brief Writes to a multi-byte register or buffer in the W5500.
 *
 * @param  bsb: The block select bits to use in the control byte.
 * @param  addr: The address in the W5500 to write to (16-bit value).
 * @param  *buf: pointer to the local buffer to write.
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

    printf("WriteBuf(0x%02X, 0x%04X, *buf, 0x%04X)\r\n", (uint16_t)bsb, (uint16_t)addr, (uint16_t)len);

    /* Write 2-part SPI packet */
    W5500_CRITICAL_ENTER();
    ASSERT_SPI_CS;
    Delay_ms(1);

    /* Write header */
    HAL_SPI_Transmit(&hspi2, txbuf, header_len, TIMEOUT_1_SEC);

    /* Write buffer */
    HAL_SPI_Transmit(&hspi2, buf, len, TIMEOUT_1_SEC);

    NEGATE_SPI_CS;
    W5500_CRITICAL_EXIT();
}

/**
 * @brief Reads a multi-byte register or buffer from the W5500.
 *
 * @param  bsb: The block select bits to use in the control byte.
 * @param  addr: The address in the W5500 to read from to (16-bit value).
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

    printf("ReadBuf(0x%02X, 0x%04X, *buf, 0x%04X)\r\n", (uint16_t)bsb, (uint16_t)addr, (uint16_t)len);

    /* Write 2-part SPI packet */
    W5500_CRITICAL_ENTER();
    ASSERT_SPI_CS;
    Delay_ms(1);

    /* Write header */
    HAL_SPI_Transmit(&hspi2, txbuf, header_len, TIMEOUT_1_SEC);

    /* Write buffer */
    HAL_SPI_Receive(&hspi2, buf, len, TIMEOUT_1_SEC);

    NEGATE_SPI_CS;
    W5500_CRITICAL_EXIT();
}

