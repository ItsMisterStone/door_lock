#include "mfrc522.h"

/* ================= Low level CS / SPI byte helpers ================= */

static void CS_Select(void)
{
    HAL_GPIO_WritePin(RC522_CS_GPIO_Port, RC522_CS_Pin, GPIO_PIN_RESET);
}

static void CS_Deselect(void)
{
    HAL_GPIO_WritePin(RC522_CS_GPIO_Port, RC522_CS_Pin, GPIO_PIN_SET);
}

static uint8_t SPI_TransferByte(uint8_t data)
{
    uint8_t rx = 0;
    HAL_SPI_TransmitReceive(&MFRC522_SPI, &data, &rx, 1, HAL_MAX_DELAY);
    return rx;
}

/* ================= Register level read / write ================= */

static void MFRC522_WriteReg(uint8_t addr, uint8_t val)
{
    CS_Select();
    SPI_TransferByte((addr << 1) & 0x7E);   /* MSB=0 -> write, address bits 6..1 */
    SPI_TransferByte(val);
    CS_Deselect();
}

static uint8_t MFRC522_ReadReg(uint8_t addr)
{
    uint8_t val;
    CS_Select();
    SPI_TransferByte(((addr << 1) & 0x7E) | 0x80); /* MSB=1 -> read */
    val = SPI_TransferByte(0x00);
    CS_Deselect();
    return val;
}

static void MFRC522_SetBitMask(uint8_t reg, uint8_t mask)
{
    uint8_t tmp = MFRC522_ReadReg(reg);
    MFRC522_WriteReg(reg, tmp | mask);
}

static void MFRC522_ClearBitMask(uint8_t reg, uint8_t mask)
{
    uint8_t tmp = MFRC522_ReadReg(reg);
    MFRC522_WriteReg(reg, tmp & (~mask));
}

/* ================= Antenna control ================= */

void MFRC522_AntennaOn(void)
{
    uint8_t temp = MFRC522_ReadReg(TxControlReg);
    if (!(temp & 0x03))
    {
        MFRC522_SetBitMask(TxControlReg, 0x03);
    }
}

void MFRC522_AntennaOff(void)
{
    MFRC522_ClearBitMask(TxControlReg, 0x03);
}

/* ================= Reset / Init ================= */

void MFRC522_Reset(void)
{
    MFRC522_WriteReg(CommandReg, PCD_RESETPHASE);
}

void MFRC522_Init(void)
{
    /* Hardware reset pulse via NRSTPD pin */
    HAL_GPIO_WritePin(RC522_RST_GPIO_Port, RC522_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(RC522_RST_GPIO_Port, RC522_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(RC522_RST_GPIO_Port, RC522_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(50);

    MFRC522_Reset();

    MFRC522_WriteReg(TModeReg, 0x8D);
    MFRC522_WriteReg(TPrescalerReg, 0x3E);
    MFRC522_WriteReg(TReloadRegL, 30);
    MFRC522_WriteReg(TReloadRegH, 0);

    MFRC522_WriteReg(TxAutoReg, 0x40);
    MFRC522_WriteReg(ModeReg, 0x3D);

    MFRC522_AntennaOn();
}

/* ================= Core transceive routine ================= */

static uint8_t MFRC522_ToCard(uint8_t command, uint8_t *sendData, uint8_t sendLen,
                               uint8_t *backData, uint16_t *backLen)
{
    uint8_t status = MI_ERR;
    uint8_t irqEn = 0x00;
    uint8_t waitIRq = 0x00;
    uint8_t lastBits;
    uint8_t n;
    uint16_t i;

    switch (command)
    {
        case PCD_AUTHENT:
            irqEn = 0x12;
            waitIRq = 0x10;
            break;
        case PCD_TRANSCEIVE:
            irqEn = 0x77;
            waitIRq = 0x30;
            break;
        default:
            break;
    }

    MFRC522_WriteReg(CommIEnReg, irqEn | 0x80);
    MFRC522_ClearBitMask(CommIrqReg, 0x80);
    MFRC522_SetBitMask(FIFOLevelReg, 0x80);   /* flush FIFO */

    MFRC522_WriteReg(CommandReg, PCD_IDLE);

    for (i = 0; i < sendLen; i++)
    {
        MFRC522_WriteReg(FIFODataReg, sendData[i]);
    }

    MFRC522_WriteReg(CommandReg, command);
    if (command == PCD_TRANSCEIVE)
    {
        MFRC522_SetBitMask(BitFramingReg, 0x80);   /* StartSend = 1 */
    }

    /* Wait for the command to complete (roughly 25ms timeout, ~2000 poll loops) */
    i = 2000;
    do
    {
        n = MFRC522_ReadReg(CommIrqReg);
        i--;
    } while ((i != 0) && !(n & 0x01) && !(n & waitIRq));

    MFRC522_ClearBitMask(BitFramingReg, 0x80);

    if (i != 0)
    {
        if (!(MFRC522_ReadReg(ErrorReg) & 0x1B))
        {
            status = MI_OK;
            if (n & irqEn & 0x01)
            {
                status = MI_NOTAGERR;
            }

            if (command == PCD_TRANSCEIVE)
            {
                n = MFRC522_ReadReg(FIFOLevelReg);
                lastBits = MFRC522_ReadReg(ControlReg) & 0x07;
                if (lastBits)
                {
                    *backLen = (n - 1) * 8 + lastBits;
                }
                else
                {
                    *backLen = n * 8;
                }

                if (n == 0)
                {
                    n = 1;
                }
                if (n > MAX_LEN)
                {
                    n = MAX_LEN;
                }

                for (i = 0; i < n; i++)
                {
                    backData[i] = MFRC522_ReadReg(FIFODataReg);
                }
            }
        }
        else
        {
            status = MI_ERR;
        }
    }

    return status;
}

static void MFRC522_CalculateCRC(uint8_t *pIndata, uint8_t len, uint8_t *pOutData)
{
    uint8_t i, n;

    MFRC522_ClearBitMask(DivIrqReg, 0x04);
    MFRC522_SetBitMask(FIFOLevelReg, 0x80);

    for (i = 0; i < len; i++)
    {
        MFRC522_WriteReg(FIFODataReg, *(pIndata + i));
    }
    MFRC522_WriteReg(CommandReg, PCD_CALCCRC);

    i = 0xFF;
    do
    {
        n = MFRC522_ReadReg(DivIrqReg);
        i--;
    } while ((i != 0) && !(n & 0x04));

    pOutData[0] = MFRC522_ReadReg(CRCResultRegL);
    pOutData[1] = MFRC522_ReadReg(CRCResultRegM);
}

/* ================= Card commands ================= */

uint8_t MFRC522_Request(uint8_t reqMode, uint8_t *TagType)
{
    uint8_t status;
    uint16_t backBits;

    MFRC522_WriteReg(BitFramingReg, 0x07);   /* TxLastBits = 7 (short frame) */

    TagType[0] = reqMode;
    status = MFRC522_ToCard(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);

    if ((status != MI_OK) || (backBits != 0x10))
    {
        status = MI_ERR;
    }

    return status;
}

uint8_t MFRC522_Anticoll(uint8_t *serNum)
{
    uint8_t status;
    uint8_t i;
    uint8_t serNumCheck = 0;
    uint16_t unLen;

    MFRC522_WriteReg(BitFramingReg, 0x00);

    serNum[0] = PICC_ANTICOLL;
    serNum[1] = 0x20;
    status = MFRC522_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

    if (status == MI_OK)
    {
        for (i = 0; i < 4; i++)
        {
            serNumCheck ^= serNum[i];
        }
        if (serNumCheck != serNum[4])
        {
            status = MI_ERR;
        }
    }

    return status;
}

uint8_t MFRC522_SelectTag(uint8_t *serNum)
{
    uint8_t i;
    uint8_t status;
    uint8_t size;
    uint16_t recvBits;
    uint8_t buffer[9];

    buffer[0] = PICC_SElECTTAG;
    buffer[1] = 0x70;
    for (i = 0; i < 5; i++)
    {
        buffer[i + 2] = serNum[i];
    }
    MFRC522_CalculateCRC(buffer, 7, &buffer[7]);

    status = MFRC522_ToCard(PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits);

    if ((status == MI_OK) && (recvBits == 0x18))
    {
        size = buffer[0];
    }
    else
    {
        size = 0;
    }

    return size;
}

uint8_t MFRC522_Auth(uint8_t authMode, uint8_t BlockAddr, uint8_t *Sectorkey, uint8_t *serNum)
{
    uint8_t status;
    uint16_t recvBits;
    uint8_t i;
    uint8_t buff[12];

    buff[0] = authMode;
    buff[1] = BlockAddr;
    for (i = 0; i < 6; i++)
    {
        buff[i + 2] = *(Sectorkey + i);
    }
    for (i = 0; i < 4; i++)
    {
        buff[i + 8] = *(serNum + i);
    }

    status = MFRC522_ToCard(PCD_AUTHENT, buff, 12, buff, &recvBits);

    if ((status != MI_OK) || (!(MFRC522_ReadReg(Status2Reg) & 0x08)))
    {
        status = MI_ERR;
    }

    return status;
}

uint8_t MFRC522_Read(uint8_t blockAddr, uint8_t *recvData)
{
    uint8_t status;
    uint16_t unLen;

    recvData[0] = PICC_READ;
    recvData[1] = blockAddr;
    MFRC522_CalculateCRC(recvData, 2, &recvData[2]);

    status = MFRC522_ToCard(PCD_TRANSCEIVE, recvData, 4, recvData, &unLen);

    if ((status != MI_OK) || (unLen != 0x90))
    {
        status = MI_ERR;
    }

    return status;
}

uint8_t MFRC522_Write(uint8_t blockAddr, uint8_t *writeData)
{
    uint8_t status;
    uint16_t recvBits;
    uint8_t i;
    uint8_t buff[18];

    buff[0] = PICC_WRITE;
    buff[1] = blockAddr;
    MFRC522_CalculateCRC(buff, 2, &buff[2]);

    status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &recvBits);

    if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
    {
        status = MI_ERR;
    }

    if (status == MI_OK)
    {
        for (i = 0; i < 16; i++)
        {
            buff[i] = *(writeData + i);
        }
        MFRC522_CalculateCRC(buff, 16, &buff[16]);

        status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 18, buff, &recvBits);

        if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
        {
            status = MI_ERR;
        }
    }

    return status;
}

void MFRC522_Halt(void)
{
    uint16_t unLen;
    uint8_t buff[4];

    buff[0] = PICC_HALT;
    buff[1] = 0;
    MFRC522_CalculateCRC(buff, 2, &buff[2]);

    MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &unLen);
}
