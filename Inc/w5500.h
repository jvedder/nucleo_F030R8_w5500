/**
 ******************************************************************************
 * @file           : w5500.h
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


/* Prevent recursive inclusion */
#ifndef W5500_H
#define W5500_H

/**
 *  Include files
 */
#include <stdint.h>


/**
 *  Public Defines and Macros
 */

/* The number of sockets in the W5500 */
#define W5500_MAX_SOCKET_NUM            8

/* The size of each socket buffer is 2K byes */
#define W5500_RXBUF_SIZE                0x0800
#define W5500_TXBUF_SIZE                0x0800

/* bit mask for the socket buffer address range */
#define W5500_RXBUF_MASK                0x07FF
#define W5500_TXBUF_MASK                0x07FF


/**
 * Note: all register addresses are 8-bit, so they are defines as such
 */


/* Common Register Address Definitions */
#define W5500_REG_MR                    0x00    /* Mode Reg */
#define W5500_REG_GAR                   0x01    /* Gateway Addr */
#define W5500_REG_SUBR                  0x05    /* Subnet Mask Addr */
#define W5500_REG_SHAR                  0x09    /* Source Hardware Addr */
#define W5500_REG_SIPR                  0x0F    /* Source IP Addr */
#define W5500_REG_INTLEVEL              0x13    /* Interrupt Low Level Timer */
#define W5500_REG_IR                    0x15    /* Interrupt Reg */
#define W5500_REG_IMR                   0x16    /* Interupt Mask */
#define W5500_REG_SIR                   0x17    /* Socket Interrupt */
#define W5500_REG_SIMR                  0x18    /* Socket Interrupt Mask */
#define W5500_REG_RTR                   0x19    /* Retry Time */
#define W5500_REG_RCR                   0x1B    /* Retry Count */
#define W5500_REG_PRIMER                0x1C    /* PPP LCP Request Timer */
#define W5500_REG_PMAGIC                0x1D    /* PPP LCP Magic Number */
#define W5500_REG_PHAR                  0x1E    /* PPP Dest Mac Addr */
#define W5500_REG_PSID                  0x24    /* PPP Session ID */
#define W5500_REG_PMRU                  0x26    /* PPP Max Segment Size */
#define W5500_REG_UIPR                  0x28    /* Unreachable IP Addr */
#define W5500_REG_UPORTR                0x2C    /* Unreachable Port */
#define W5500_REG_PHYCFGR               0x2E    /* Phy Config */
#define W5500_REG_VERSIONR              0x39    /* Chip Version */


/* Socket N Register Address Definitions */
#define W5500_REG_SnMR                  0x00    /* Mode Reg */
#define W5500_REG_SnCR                  0x01    /* Command Reg */
#define W5500_REG_SnIR                  0x02    /* Interrupt Reg */
#define W5500_REG_SnSR                  0x03    /* Status Reg */
#define W5500_REG_SnPORT                0x04    /* Source Port */
#define W5500_REG_SnDHAR                0x06    /* Dest Hardware Addr */
#define W5500_REG_SnDIPR                0x0C    /* Dest IP Addr */
#define W5500_REG_SnDPORT               0x10    /* Dest Port */
#define W5500_REG_SnMSSR                0x12    /* Max Segment Size */
#define W5500_REG_SnTOS                 0x15    /* IP TOS */
#define W5500_REG_SnTTL                 0x16    /* IP TTL */
#define W5500_REG_SnRXBUF_SIZE          0x1E    /* Rx Buffer Size */
#define W5500_REG_SnTXBUF_SIZE          0x1F    /* Tx Buffer Size */
#define W5500_REG_SnTX_FSR              0x20    /* Tx Free Size */
#define W5500_REG_SnTX_RD               0x22    /* Tx Read Pointer */
#define W5500_REG_SnTX_WR               0x24    /* Tx Write Pointer */
#define W5500_REG_SnRX_RSR              0x26    /* Rx Received Size */
#define W5500_REG_SnRX_RD               0x28    /* Rx Read Pointer */
#define W5500_REG_SnRX_WR               0x2A    /* Rx Write Pointer */
#define W5500_REG_SnIMR                 0x2C    /* Interrupt Mask */
#define W5500_REG_SnFEAG                0x2D    /* Fragment Offset in IP Header */
#define W5500_REG_SnKPALVTR             0x2F    /* Keep Alive Timer */


/* Control Byte Definitions and Macros */
#define W5500_CB_REG                    0x00                /* Common Registers Block */
#define W5500_CB_SnREG(n)               (0x01|((n)<<5))     /* Socket N Register Block */
#define W5500_CB_SnTX(n)                (0x02|((n)<<5))     /* Socket N Transmit Block */
#define W5500_CB_SnRX(n)                (0x03|((n)<<5))     /* Socket N Receive Block */
#define W5500_CB_READ                   0x00                /* Control Byte Read Mask */
#define W5500_CB_WRITE                  0x04                /* Control Byte Write Mask */


/* Long Name convenience macros  */
#define W5500_setGatewayIp(ptr)             W5500_SetGAR(ptr)
#define W5500_getGatewayIp(ptr)             W5500_GetGAR(ptr)

#define W5500_setSubnetMask(ptr)            W5500_SetSUBR(ptr)
#define W5500_getSubnetMask(ptr)            W5500_GetSUBR(ptr)

#define W5500_setMACAddress(ptr)            W5500_SetSHAR(ptr)
#define W5500_getMACAddress(ptr)            W5500_GetSHAR(ptr)

#define W5500_setIPAddress(ptr)             W5500_SetSIPR(ptr)
#define W5500_getIPAddress(ptr)             W5500_GetSIPR(ptr)

#define W5500_setRetransmissionTime(val)    W5500_SetRTR(val)
#define W5500_setRetransmissionCount(val)   W5500_SetRCR(val)


/* Common Mode Register (MR) Values */
#define W5500_MR_RST                    0x80
#define W5500_MR_WOL                    0x20
#define W5500_MR_PB                     0x10
#define W5500_MR_PPPOE                  0x08
#define W5500_MR_FARP                   0x02

/* Common Interrupt Register (IR) Masks */
#define W5500_IR_CONFLICT               0x80
#define W5500_IR_UNREACH                0x40
#define W5500_IR_PPPoE                  0x20
#define W5500_IR_MP                     0x10

/* Common Interrupt Mask Register (IM) Values */
#define W5500_IM_IR7                        0x80
#define W5500_IM_IR6                        0x40
#define W5500_IM_IR5                        0x20
#define W5500_IM_IR4                        0x10

/* Common Phy Configuration Register (PHYCFGR) Values */
#define W5500_PHYCFGR_RST                   ~(1<<7)  //< For PHY reset, must operate AND mask.
#define W5500_PHYCFGR_OPMD                  (1<<6)   // Configure PHY with OPMDC value
#define W5500_PHYCFGR_OPMDC_ALLA            (7<<3)
#define W5500_PHYCFGR_OPMDC_PDOWN           (6<<3)
#define W5500_PHYCFGR_OPMDC_NA              (5<<3)
#define W5500_PHYCFGR_OPMDC_100FA           (4<<3)
#define W5500_PHYCFGR_OPMDC_100F            (3<<3)
#define W5500_PHYCFGR_OPMDC_100H            (2<<3)
#define W5500_PHYCFGR_OPMDC_10F             (1<<3)
#define W5500_PHYCFGR_OPMDC_10H             (0<<3)
#define W5500_PHYCFGR_DPX_FULL              (1<<2)
#define W5500_PHYCFGR_DPX_HALF              (0<<2)
#define W5500_PHYCFGR_SPD_100               (1<<1)
#define W5500_PHYCFGR_SPD_10                (0<<1)
#define W5500_PHYCFGR_LNK_ON                (1<<0)
#define W5500_PHYCFGR_LNK_OFF               (0<<0)

/* Socket N Mode Register (SnMR) Values */
#define W5500_SnMR_MULTI                    0x80
#define W5500_SnMR_BCASTB                   0x40
#define W5500_SnMR_ND                       0x20
#define W5500_SnMR_UCASTB                   0x10
#define W5500_SnMR_PROTOCOL                 0x0F    /* Protocol Mask bits */
#define W5500_SnMR_MACRAW                   0x04
#define W5500_SnMR_UDP                      0x02
#define W5500_SnMR_TCP                      0x01
#define W5500_SnMR_CLOSE                    0x00
#define W5500_SnMR_MFEN                     W5500_SnMR_MULTI
#define W5500_SnMR_MMB                      W5500_SnMR_ND
#define W5500_SnMR_MIP6B                    W5500_SnMR_UCASTB
#define W5500_SnMR_MC                       W5500_SnMR_ND

/* Socket N Command Register (SnCR) Values */
#define W5500_SnCR_OPEN                     0x01
#define W5500_SnCR_LISTEN                   0x02
#define W5500_SnCR_CONNECT                  0x04
#define W5500_SnCR_DISCON                   0x08
#define W5500_SnCR_CLOSE                    0x10
#define W5500_SnCR_SEND                     0x20
#define W5500_SnCR_SEND_MAC                 0x21
#define W5500_SnCR_SEND_KEEP                0x22
#define W5500_SnCR_RECV                     0x40

/* Socket N Interrupt Register (SnIR) Values */
#define W5500_SnIR_SENDOK                   0x10
#define W5500_SnIR_TIMEOUT                  0x08
#define W5500_SnIR_RECV                     0x04
#define W5500_SnIR_DISCON                   0x02
#define W5500_SnIR_CON                      0x01

/* Socket N Status Register (SnSR) Values */
#define W5500_SnSR_CLOSED                   0x00
#define W5500_SnSR_INIT                     0x13
#define W5500_SnSR_LISTEN                   0x14
#define W5500_SnSR_SYNSENT                  0x15
#define W5500_SnSR_SYNRECV                  0x16
#define W5500_SnSR_ESTABLISHED              0x17
#define W5500_SnSR_FIN_WAIT                 0x18
#define W5500_SnSR_CLOSING                  0x1A
#define W5500_SnSR_TIME_WAIT                0x1B
#define W5500_SnSR_CLOSE_WAIT               0x1C
#define W5500_SnSR_LAST_ACK                 0x1D
#define W5500_SnSR_UDP                      0x22
#define W5500_SnSR_IPRAW                    0x32     /**< IP raw mode socket */
#define W5500_SnSR_MACRAW                   0x42
#define W5500_SnSR_PPPOE                    0x5F

/* IP Protocols */
#define IPPROTO_IP                          0        //< Dummy for IP
#define IPPROTO_ICMP                        1        //< Control message protocol
#define IPPROTO_IGMP                        2        //< Internet group management protocol
#define IPPROTO_GGP                         3        //< Gateway^2 (deprecated)
#define IPPROTO_TCP                         6        //< TCP
#define IPPROTO_PUP                         12       //< PUP
#define IPPROTO_UDP                         17       //< UDP
#define IPPROTO_IDP                         22       //< XNS idp
#define IPPROTO_ND                          77       //< UNOFFICIAL net disk protocol
#define IPPROTO_RAW                         255      //< Raw IP packet


/**
 *  Public Function Prototypes
 */
void W5500_Init();
void W5500_SoftReset();
void W5500_WriteReg8(uint8_t bsb, uint8_t reg, uint8_t val);
void W5500_WriteReg16(uint8_t bsb, uint8_t reg, uint16_t val);
uint8_t W5500_ReadReg8(uint8_t bsb, uint8_t reg);
uint16_t W5500_ReadReg16(uint8_t bsb, uint8_t reg);
uint16_t W5500_ReadReg16Val(uint8_t bsb, uint8_t reg);
void W5500_WriteBuf(uint8_t bsb, uint16_t addr, uint8_t *buf, uint16_t len);
void W5500_ReadBuf(uint8_t bsb, uint16_t addr, uint8_t *buf, uint16_t len);
void W5500_WriteTxBuffer(uint8_t sn, uint8_t *buf, uint16_t len);
void W5500_ReadRXBuffer(uint8_t sn, uint8_t *buf, uint16_t len);
void W5500_ExecuteSnCmd(uint8_t sn, uint8_t cmd);


/**
 *  Public Inline Function Definitions (instead of macros).
 *
 *  @See: https://gcc.gnu.org/onlinedocs/gcc/Inline.html
 */

/* TODO: does this need a namespace prefix? */
#define INLINE   __attribute__((always_inline)) static inline

/* Receive and Transmit Buffer Write and Read Methods */
INLINE void W5500_WriteSnRXBuf(uint8_t s, uint16_t addr, uint8_t *buf,
        uint16_t len)
{
    W5500_WriteBuf(W5500_CB_SnRX(s), addr, buf, len);
}
INLINE void W5500_WriteSnTXBuf(uint8_t s, uint16_t addr, uint8_t *buf,
        uint16_t len)
{
    W5500_WriteBuf(W5500_CB_SnTX(s), addr, buf, len);
}
INLINE void W5500_ReadSnRXBuf(uint8_t s, uint16_t addr, uint8_t *buf,
        uint16_t len)
{
    W5500_ReadBuf(W5500_CB_SnRX(s), addr, buf, len);
}
INLINE void W5500_ReadSnTXBuf(uint8_t s, uint16_t addr, uint8_t *buf,
        uint16_t len)
{
    W5500_ReadBuf(W5500_CB_SnTX(s), addr, buf, len);
}

/* Common Register Write Methods */
INLINE void W5500_WriteMR(uint8_t val)
{
    W5500_WriteReg8(W5500_CB_REG, W5500_REG_MR, val);
}
INLINE void W5500_WriteGAR(uint8_t *ptr)
{
    W5500_WriteBuf(W5500_CB_REG, W5500_REG_GAR, ptr, 4);
}
INLINE void W5500_WriteSUBR(uint8_t *ptr)
{
    W5500_WriteBuf(W5500_CB_REG, W5500_REG_SUBR, ptr, 4);
}
INLINE void W5500_WriteSHAR(uint8_t *ptr)
{
    W5500_WriteBuf(W5500_CB_REG, W5500_REG_SHAR, ptr, 6);
}
INLINE void W5500_WriteSIPR(uint8_t *ptr)
{
    W5500_WriteBuf(W5500_CB_REG, W5500_REG_SIPR, ptr, 4);
}
INLINE void W5500_WriteINTLEVEL(uint16_t val)
{
    W5500_WriteReg16(W5500_CB_REG, W5500_REG_INTLEVEL, val);
}
INLINE void W5500_WriteIR(uint8_t val)
{
    W5500_WriteReg8(W5500_CB_REG, W5500_REG_IR, val);
}
INLINE void W5500_WriteIMR(uint8_t val)
{
    W5500_WriteReg8(W5500_CB_REG, W5500_REG_IMR, val);
}
INLINE void W5500_WriteSIR(uint8_t val)
{
    W5500_WriteReg8(W5500_CB_REG, W5500_REG_SIR, val);
}
INLINE void W5500_WriteSIMR(uint8_t val)
{
    W5500_WriteReg8(W5500_CB_REG, W5500_REG_SIMR, val);
}
INLINE void W5500_WriteRTR(uint16_t val)
{
    W5500_WriteReg16(W5500_CB_REG, W5500_REG_RTR, val);
}
INLINE void W5500_WriteRCR(uint8_t val)
{
    W5500_WriteReg8(W5500_CB_REG, W5500_REG_RCR, val);
}
INLINE void W5500_WriteUIPR(uint8_t *ptr)
{
    W5500_WriteBuf(W5500_CB_REG, W5500_REG_UIPR, ptr, 4);
}
INLINE void W5500_WriteUPORTR(uint16_t val)
{
    W5500_WriteReg16(W5500_CB_REG, W5500_REG_UPORTR, val);
}
INLINE void W5500_WritePHYCFGR(uint8_t val)
{
    W5500_WriteReg8(W5500_CB_REG, W5500_REG_PHYCFGR, val);
}
INLINE void W5500_WriteVERSIONR(uint8_t val)
{
    W5500_WriteReg8(W5500_CB_REG, W5500_REG_VERSIONR, val);
}

/* Socket N Register Write Methods */
INLINE void W5500_WriteSnMR(uint8_t s, uint8_t val)
{
    W5500_WriteReg8(W5500_CB_SnREG(s), W5500_REG_SnMR, val);
}
INLINE void W5500_WriteSnCR(uint8_t s, uint8_t val)
{
    W5500_WriteReg8(W5500_CB_SnREG(s), W5500_REG_SnCR, val);
}
INLINE void W5500_WriteSnIR(uint8_t s, uint8_t val)
{
    W5500_WriteReg8(W5500_CB_SnREG(s), W5500_REG_SnIR, val);
}
INLINE void W5500_WriteSnSR(uint8_t s, uint8_t val)
{
    W5500_WriteReg8(W5500_CB_SnREG(s), W5500_REG_SnSR, val);
}
INLINE void W5500_WriteSnPORT(uint8_t s, uint16_t val)
{
    W5500_WriteReg16(W5500_CB_SnREG(s), W5500_REG_SnPORT, val);
}
INLINE void W5500_WriteSnDHAR(uint8_t s, uint8_t *ptr)
{
    W5500_WriteBuf(W5500_CB_SnREG(s), W5500_REG_SnDHAR, ptr, 6);
}
INLINE void W5500_WriteSnDIPR(uint8_t s, uint8_t *ptr)
{
    W5500_WriteBuf(W5500_CB_SnREG(s), W5500_REG_SnDIPR, ptr, 4);
}
INLINE void W5500_WriteSnDPORT(uint8_t s, uint16_t val)
{
    W5500_WriteReg16(W5500_CB_SnREG(s), W5500_REG_SnDPORT, val);
}
INLINE void W5500_WriteSnMSSR(uint8_t s, uint16_t val)
{
    W5500_WriteReg16(W5500_CB_SnREG(s), W5500_REG_SnMSSR, val);
}
INLINE void W5500_WriteSnTOS(uint8_t s, uint8_t val)
{
    W5500_WriteReg8(W5500_CB_SnREG(s), W5500_REG_SnTOS, val);
}
INLINE void W5500_WriteSnTTL(uint8_t s, uint8_t val)
{
    W5500_WriteReg8(W5500_CB_SnREG(s), W5500_REG_SnTTL, val);
}
INLINE void W5500_WriteSnRXBUF_SIZE(uint8_t s, uint8_t val)
{
    W5500_WriteReg8(W5500_CB_SnREG(s), W5500_REG_SnRXBUF_SIZE, val);
}
INLINE void W5500_WriteSnTXBUF_SIZE(uint8_t s, uint8_t val)
{
    W5500_WriteReg8(W5500_CB_SnREG(s), W5500_REG_SnTXBUF_SIZE, val);
}
INLINE void W5500_WriteSnTX_FSR(uint8_t s, uint16_t val)
{
    W5500_WriteReg16(W5500_CB_SnREG(s), W5500_REG_SnTX_FSR, val);
}
INLINE void W5500_WriteSnTX_RD(uint8_t s, uint16_t val)
{
    W5500_WriteReg16(W5500_CB_SnREG(s), W5500_REG_SnTX_RD, val);
}
INLINE void W5500_WriteSnTX_WR(uint8_t s, uint16_t val)
{
    W5500_WriteReg16(W5500_CB_SnREG(s), W5500_REG_SnTX_WR, val);
}
INLINE void W5500_WriteSnRX_RSR(uint8_t s, uint16_t val)
{
    W5500_WriteReg16(W5500_CB_SnREG(s), W5500_REG_SnRX_RSR, val);
}
INLINE void W5500_WriteSnRX_RD(uint8_t s, uint16_t val)
{
    W5500_WriteReg16(W5500_CB_SnREG(s), W5500_REG_SnRX_RD, val);
}
INLINE void W5500_WriteSnRX_WR(uint8_t s, uint16_t val)
{
    W5500_WriteReg16(W5500_CB_SnREG(s), W5500_REG_SnRX_WR, val);
}
INLINE void W5500_WriteSnIMR(uint8_t s, uint8_t val)
{
    W5500_WriteReg8(W5500_CB_SnREG(s), W5500_REG_SnIMR, val);
}
INLINE void W5500_WriteSnFEAG(uint8_t s, uint16_t val)
{
    W5500_WriteReg16(W5500_CB_SnREG(s), W5500_REG_SnFEAG, val);
}
INLINE void W5500_WriteSnKPALVTR(uint8_t s, uint8_t val)
{
    W5500_WriteReg8(W5500_CB_SnREG(s), W5500_REG_SnKPALVTR, val);
}

/* Common Register Read Methods */
INLINE uint8_t W5500_ReadMR(uint8_t val)
{
    return W5500_ReadReg8(W5500_CB_REG, W5500_REG_MR);
}
INLINE void W5500_ReadGAR(uint8_t *ptr)
{
    W5500_ReadBuf(W5500_CB_REG, W5500_REG_GAR, ptr, 4);
}
INLINE void W5500_ReadSUBR(uint8_t *ptr)
{
    W5500_ReadBuf(W5500_CB_REG, W5500_REG_SUBR, ptr, 4);
}
INLINE void W5500_ReadSHAR(uint8_t *ptr)
{
    W5500_ReadBuf(W5500_CB_REG, W5500_REG_SHAR, ptr, 6);
}
INLINE void W5500_ReadSIPR(uint8_t *ptr)
{
    W5500_ReadBuf(W5500_CB_REG, W5500_REG_SIPR, ptr, 4);
}
INLINE uint16_t W5500_ReadINTLEVEL(uint16_t val)
{
    return W5500_ReadReg16(W5500_CB_REG, W5500_REG_INTLEVEL);
}
INLINE uint8_t W5500_ReadIR(uint8_t val)
{
    return W5500_ReadReg8(W5500_CB_REG, W5500_REG_IR);
}
INLINE uint8_t W5500_ReadIMR(uint8_t val)
{
    return W5500_ReadReg8(W5500_CB_REG, W5500_REG_IMR);
}
INLINE uint8_t W5500_ReadSIR(uint8_t val)
{
    return W5500_ReadReg8(W5500_CB_REG, W5500_REG_SIR);
}
INLINE uint8_t W5500_ReadSIMR(uint8_t val)
{
    return W5500_ReadReg8(W5500_CB_REG, W5500_REG_SIMR);
}
INLINE uint16_t W5500_ReadRTR(uint16_t val)
{
    return W5500_ReadReg16(W5500_CB_REG, W5500_REG_RTR);
}
INLINE uint8_t W5500_ReadRCR(uint8_t val)
{
    return W5500_ReadReg8(W5500_CB_REG, W5500_REG_RCR);
}
INLINE void W5500_ReadUIPR(uint8_t *ptr)
{
    W5500_ReadBuf(W5500_CB_REG, W5500_REG_UIPR, ptr, 4);
}
INLINE uint16_t W5500_ReadUPORTR(uint16_t val)
{
    return W5500_ReadReg16(W5500_CB_REG, W5500_REG_UPORTR);
}
INLINE uint8_t W5500_ReadPHYCFGR(uint8_t val)
{
    return W5500_ReadReg8(W5500_CB_REG, W5500_REG_PHYCFGR);
}
INLINE uint8_t W5500_ReadVERSIONR(uint8_t val)
{
    return W5500_ReadReg8(W5500_CB_REG, W5500_REG_VERSIONR);
}

/* Socket N Register Read Methods */
INLINE uint8_t W5500_ReadSnMR(uint8_t s)
{
    return W5500_ReadReg8(W5500_CB_SnREG(s), W5500_REG_SnMR);
}
INLINE uint8_t W5500_ReadSnCR(uint8_t s)
{
    return W5500_ReadReg8(W5500_CB_SnREG(s), W5500_REG_SnCR);
}
INLINE uint8_t W5500_ReadSnIR(uint8_t s)
{
    return W5500_ReadReg8(W5500_CB_SnREG(s), W5500_REG_SnIR);
}
INLINE uint8_t W5500_ReadSnSR(uint8_t s)
{
    return W5500_ReadReg8(W5500_CB_SnREG(s), W5500_REG_SnSR);
}
INLINE uint16_t W5500_ReadSnPORT(uint8_t s)
{
    return W5500_ReadReg16(W5500_CB_SnREG(s), W5500_REG_SnPORT);
}
INLINE void W5500_ReadSnDHAR(uint8_t s, uint8_t *ptr)
{
    W5500_ReadBuf(W5500_CB_SnREG(s), W5500_REG_SnDHAR, ptr, 6);
}
INLINE void W5500_ReadSnDIPR(uint8_t s, uint8_t *ptr)
{
    W5500_ReadBuf(W5500_CB_SnREG(s), W5500_REG_SnDIPR, ptr, 4);
}
INLINE uint16_t W5500_ReadSnDPORT(uint8_t s)
{
    return W5500_ReadReg16(W5500_CB_SnREG(s), W5500_REG_SnDPORT);
}
INLINE uint16_t W5500_ReadSnMSSR(uint8_t s)
{
    return W5500_ReadReg16(W5500_CB_SnREG(s), W5500_REG_SnMSSR);
}
INLINE uint8_t W5500_ReadSnTOS(uint8_t s)
{
    return W5500_ReadReg8(W5500_CB_SnREG(s), W5500_REG_SnTOS);
}
INLINE uint8_t W5500_ReadSnTTL(uint8_t s)
{
    return W5500_ReadReg8(W5500_CB_SnREG(s), W5500_REG_SnTTL);
}
INLINE uint8_t W5500_ReadSnRXBUF_SIZE(uint8_t s)
{
    return W5500_ReadReg8(W5500_CB_SnREG(s), W5500_REG_SnRXBUF_SIZE);
}
INLINE uint8_t W5500_ReadSnTXBUF_SIZE(uint8_t s)
{
    return W5500_ReadReg8(W5500_CB_SnREG(s), W5500_REG_SnTXBUF_SIZE);
}
INLINE uint16_t W5500_ReadSnTX_FSR(uint8_t s)
{
    return W5500_ReadReg16Val(W5500_CB_SnREG(s), W5500_REG_SnTX_FSR);
}
INLINE uint16_t W5500_ReadSnTX_RD(uint8_t s)
{
    return W5500_ReadReg16(W5500_CB_SnREG(s), W5500_REG_SnTX_RD);
}
INLINE uint16_t W5500_ReadSnTX_WR(uint8_t s)
{
    return W5500_ReadReg16(W5500_CB_SnREG(s), W5500_REG_SnTX_WR);
}
INLINE uint16_t W5500_ReadSnRX_RSR(uint8_t s)
{
    return W5500_ReadReg16Val(W5500_CB_SnREG(s), W5500_REG_SnRX_RSR);
}
INLINE uint16_t W5500_ReadSnRX_RD(uint8_t s)
{
    return W5500_ReadReg16(W5500_CB_SnREG(s), W5500_REG_SnRX_RD);
}
INLINE uint16_t W5500_ReadSnRX_WR(uint8_t s)
{
    return W5500_ReadReg16(W5500_CB_SnREG(s), W5500_REG_SnRX_WR);
}
INLINE uint8_t W5500_ReadSnIMR(uint8_t s)
{
    return W5500_ReadReg8(W5500_CB_SnREG(s), W5500_REG_SnIMR);
}
INLINE uint16_t W5500_ReadSnFEAG(uint8_t s)
{
    return W5500_ReadReg16(W5500_CB_SnREG(s), W5500_REG_SnFEAG);
}
INLINE uint8_t W5500_ReadSnKPALVTR(uint8_t s)
{
    return W5500_ReadReg8(W5500_CB_SnREG(s), W5500_REG_SnKPALVTR);
}


#endif /* W5500_H */
