/**
  ******************************************************************************
  * @file           : w5500.h
  * @brief          : low-level support for the Wiznet W5500 Ethernet Chip
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef W5500_H
#define W5500_H


void W5500_Init();
void W5500_SoftReset();
void W5500_Write(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);
void W5500_Read(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);




#endif
