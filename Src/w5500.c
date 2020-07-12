/**
  ******************************************************************************
  * @file           : w5500.c
  * @brief          : low-level support for the Wiznet W5500 Ethernet Chip
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

#include "main.h"
#include "w5500.h"
#include <stdio.h>


uint32_t failures = 0;

void W5500_Init()
{
    /* Negate CS is high */
    HAL_GPIO_WritePin(SPI2_RST_GPIO_Port, SPI2_RST_Pin, GPIO_PIN_RESET);

    Delay_ms(1000);

#if 0
    /* Assert reset low > 1 ms*/
    HAL_GPIO_WritePin(SPI2_RST_GPIO_Port, SPI2_RST_Pin, GPIO_PIN_RESET);
    Delay_ms(2);
    HAL_GPIO_WritePin(SPI2_RST_GPIO_Port, SPI2_RST_Pin, GPIO_PIN_SET);
    Delay_ms(2);
#endif
}

void W5500_WriteMR(uint8_t val)
{
    /**
     * Address16 = 0x0000 (MR)
     * ControlByte = 0x04 as:
     *     0000 0... : BSB = Common Registers
     *     .... .1.. : RWB = Write
     *     .... ..00 : OM =  Variable Length Data Mode (VDM)
     *  Data[0] = val
     */

    uint8_t txbuf[8] = { 0x00, 0x00, 0x04, val};

    printf("Writing MR= 0x%02X\r\n", (uint16_t)val);

    /* Assert CS low */
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
    Delay_ms(1);

    /* Write SPI packet */
    HAL_SPI_Transmit(&hspi2, txbuf, 4, TIMEOUT_1_SEC);

    /* Negate CS high */
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
}

void W5500_SoftReset( )
{
    /**
     * Address16 = 0x0000 (MR)
     * ControlByte = 0x04 as:
     *     0000 0... : BSB = Common Registers
     *     .... .1.. : RWB = Write
     *     .... ..00 : OM =  Variable Length Data Mode (VDM)
     *  Data[0] = val
     */

    uint8_t txbuf[8] = { 0x00, 0x00, 0x04, 0x80};

    printf("Writing MR= 0x80\r\n");

    /* Assert CS low */
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
    Delay_ms(1);

    /* Write SPI packet */
    HAL_SPI_Transmit(&hspi2, txbuf, 4, TIMEOUT_1_SEC);

    /* Negate CS high */
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

    Delay_ms(250);
}


void W5500_Write(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3)
{
    /**
     * Address16 = 0x00 01
     * ControlByte = 0x04 as:
     *     0000 0... : BSB = Common Registers
     *     .... .1.. : RWB = Write
     *     .... ..00 : OM =  Variable Length Data Mode (VDM)
     *  Data[0] = d0
     *  Data[1] = d1
     *  Data[2] = d2
     *  Data[3] = d3
     */

    uint8_t buf[8] = { 0x00, 0x01, 0x04, 0x0A, 0x0B, 0x0C, 0x0D, 0x00 };
    buf[3] = d0;
    buf[4] = d1;
    buf[5] = d2;
    buf[6] = d3;

    printf("Write 0x%02X 0x%02X 0x%02X 0x%02X\r\n", (uint16_t)d0, (uint16_t)d1, (uint16_t)d2, (uint16_t)d3);

    /* Assert CS low */
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

    /* Write SPI packet */
    HAL_SPI_Transmit(&hspi2, buf, 7, TIMEOUT_1_SEC);

    /* Negate CS high */
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
}

void W5500_Read(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3)
{
    /**
     * Address16 = 0x0001
     * ControlByte = 0x04 as:
     *     0000 0... : BSB = Common Registers
     *     .... .0.. : RWB = Read
     *     .... ..00 : OM =  Variable Length Data Mode (VDM)
     *  Data[0] = 0x00
     *  Data[1] = 0x00
     *  Data[2] = 0x00
     *  Data[3] = 0x00
     */

    uint8_t txbuf[8] = { 0x00, 0x01, 0x00, 0x0A, 0x0B, 0x0C, 0x0D, 0x00 };
    uint8_t rxbuf[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    /* Assert CS low */
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

    /* Write SPI packet */
    HAL_SPI_TransmitReceive(&hspi2, txbuf, rxbuf, 7, TIMEOUT_1_SEC);

    /* Negate CS high */
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

    printf("Read  0x%02X 0x%02X 0x%02X 0x%02X ", (uint16_t)rxbuf[3], (uint16_t)rxbuf[4], (uint16_t)rxbuf[5], (uint16_t)rxbuf[6]);
    if (d0 == rxbuf[3] && d1 == rxbuf[4] &&  d2 == rxbuf[5] &&  d3 == rxbuf[6])
    {
        printf("OK %lu\r\n", failures);
    }
    else
    {
        failures++;
        printf("FAIL %lu\r\n", failures);
    }

}
