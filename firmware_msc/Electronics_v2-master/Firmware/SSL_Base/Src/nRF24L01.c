// /*
//  * nRF24L01.c
//  *
//  *  Created on: 25 de ago de 2022
//  *      Author: leonardo
//  */
#include "nRF24L01.h"
#include "main.h"
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef* microSecTimer;

uint32_t delayCount = 0;

void Delay_us(uint32_t _delay)
{
    uint32_t beginCount = microSecTimer->Instance->CNT;

    if (beginCount + _delay > UINT32_MAX)
    {
        beginCount = 0;
        microSecTimer->Instance->CNT = 0;
    }
    while (microSecTimer->Instance->CNT - beginCount < _delay)
        ;
    // delayCount = 0;
    // while(delayCount < _delay);
}

void nRF24_RadioConfig()
{
//    const uint8_t PC_ADDRESS = 0x0F;
//    const uint8_t PC_ADDRESS = 123;
//    const uint8_t PC_ADDRESS = 0x10; // "12345"

	const uint8_t PC_ADDRESS[5] = {0x31, 0x32, 0x33, 0x34, 0x35};


    // nRF24_RX_OFF();
    // nRF24_CSN_H();
    // HAL_Delay(5);
    // nRF24_ReadReg(nRF24_REG_CONFIG);
    //
    // // POWER UP + CRC ENABLED + Oper Mode
    // nRF24_SetPowerMode(nRF24_PWR_UP);
    // nRF24_SetCRCScheme(nRF24_CRC_2byte);
    // nRF24_SetTXInterrupt(nRF24_FLAG_TX_DS); // Remove interrupção por TX
    // // set payload length
    // nRF24_SetRXPipe(nRF24_PIPE0, nRF24_AA_OFF, RADIO_PACKET_SIZE);
    // // Disable Auto_ACK, Disable ShockBurst
    // nRF24_DisableAA(0x00);
    // // Disable Auto Retry
    // nRF24_SetAutoRetr(nRF24_ARD_NONE, 0);
    // // Set Rx/Tx Address for the standard radio on the PC
    // // nRF24_SetAddrWidth(5);
    // nRF24_WriteReg(nRF24_REG_TX_ADDR, PC_ADDRESS, true);
    // nRF24_WriteReg(nRF24_REG_RX_ADDR_P0, radioData.robotID, true);
    //
    // //  Nordic_writereg(SPIRadioPtr, _NRF24L01P_REG_RF_SETUP,
    // //                  _NRF24L01P_RF_SETUP_RF_PWR_0DBM +
    // //                  _NRF24L01P_RF_SETUP_RF_DR_250KBPS);
    //
    // // Sets TX/RX
    // nRF24_SetOperationalMode(nRF24_CONFIG_PRIM_RX);
    // nRF24_RX_ON();
    // nRF24_SetRFChannel(radioData.rfRXChannel);
    // nRF24_ClearIRQFlags();
    nRF24_RX_OFF();
    uint8_t config = nRF24_ReadReg(nRF24_REG_CONFIG);

    nRF24_WriteReg(nRF24_REG_CONFIG,
                   config | nRF24_PWR_UP | nRF24_FLAG_TX_DS | nRF24_MASK_CRC,
                   true);
    Delay_us(50);
    nRF24_WriteReg(nRF24_REG_RX_PW_P0, RADIO_PACKET_SIZE, true);
    nRF24_WriteReg(nRF24_REG_EN_AA, 0x0, true);
    nRF24_WriteReg(nRF24_REG_SETUP_RETR, 0x0, true);
//    nRF24_WriteReg(nRF24_REG_TX_ADDR, PC_ADDRESS, true);
    nRF24_WriteMultiReg(nRF24_REG_TX_ADDR, PC_ADDRESS, 5);
    nRF24_WriteReg(nRF24_REG_RX_ADDR_P0, radioData.robotID, true);
    nRF24_RX_ON();

    nRF24_SetOperationalMode(nRF24_CONFIG_PRIM_RX);

    nRF24_SetRFChannel(radioData.rfRXChannel);

    nRF24_SetDataRate(nRF24_DR_2Mbps);

    radioData.dataReceived = true; // Deve ser true para o rádio inicializar
    radioData.bytesReceived = 0;
    radioData.lastPacketTimestamp = HAL_GetTick();
    radioData.packetFrequency = 0;
}

void nRF24_WriteMultiReg(uint8_t reg, const uint8_t* data, uint8_t length)
{
    nRF24_CSN_L();  // Seleciona o rádio
    nRF24_LL_RW(nRF24_CMD_W_REGISTER | (reg & nRF24_MASK_REG_MAP)); // Escreve no registrador

    for (uint8_t i = 0; i < length; i++)
    {
        nRF24_LL_RW(data[i]);  // Escreve os bytes do endereço
    }

    nRF24_CSN_H();  // Desseleciona o rádio
}

void nRF24_Transmit(uint8_t* _input)
{
    int n = 0;
    unsigned char checksum = 0;

//    nRF24_SetRFChannel(radioData.rfTXChannel);
    nRF24_SetRFChannel(75);
    nRF24_RX_OFF();
    nRF24_SetOperationalMode(nRF24_MODE_TX);
//    nRF24_RX_ON();
    Delay_us(20 + 130);

    // Preamble
    radioData.txBuffer[0] = 0x7E; // delimiter
    radioData.txBuffer[1] = TX_PACKET_SIZE / 256;
    radioData.txBuffer[2] = TX_PACKET_SIZE - 4;
    radioData.txBuffer[3] = 0x90; // tx packet
    radioData.txBuffer[4]++;      // Frame Counter
    radioData.txBuffer[PREAMBLE_SZ] =
        PACKET_START; // byte inicial, parte do protocolo
    radioData.txBuffer[TX_PACKET_SIZE - 2] =
        PACKET_END; // byte final, parte do protocolo

    for (n = PREAMBLE_SZ + 1; n < TX_PACKET_SIZE - 2; n++)
    {
        radioData.txBuffer[n] = _input[n - (PREAMBLE_SZ + 1)];
    }

    for (n = 3; n < TX_PACKET_SIZE; n++) // Checksum calculation
    {
        checksum += radioData.txBuffer[n];
    }

    checksum = ~checksum;
    radioData.txBuffer[TX_PACKET_SIZE - 1] = checksum;

    nRF24_WritePayload(radioData.txBuffer, RADIO_PACKET_SIZE);

    nRF24_RX_OFF();
    nRF24_SetOperationalMode(nRF24_MODE_RX);
    nRF24_RX_ON();
    Delay_us(20 + 130);
    nRF24_SetRFChannel(radioData.rfRXChannel);
}

bool nRF24_ReadData()
{
    radioData.dataReceived = false;
    nRF24_ClearIRQFlags();
    nRF24_RXResult pipe =
        nRF24_ReadPayload(radioData.rxBuffer, &radioData.bytesReceived);
    if (radioData.bytesReceived > 0 && pipe != nRF24_RX_EMPTY)
    {
        uint8_t newIndex = 0;
        bool delimiterFound = false;
        for (uint8_t i = 0; i < radioData.bytesReceived; ++i)
        {
            // Checa a integridade do protocolo
            if (radioData.rxBuffer[i] == 0x7E &&
                radioData.rxBuffer[i + PREAMBLE_SZ] == PACKET_START &&
                radioData.rxBuffer[i + ROBOFEI_PACKET_SIZE] == PACKET_END)
            {
                delimiterFound = true;
            }
            if (delimiterFound)
            {
                radioData.rxBuffer[newIndex++] = radioData.rxBuffer[i];
            }
        }

        if (delimiterFound)
        {
            nRF24_ProcessPacket();
        }
        if (delimiterFound == false)
        {
            HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
            return false;
        }
    }
    return true;
}

void nRF24_ProcessPacket()
{
    // Checa se o tipo de msg e diferente de RX
    if (radioData.rxBuffer[3] != ROBOFEI_IDENTIFIER)
    {
        return;
    }

    radioData.packetFrequency = 1000.0 / (HAL_GetTick() - radioData.lastPacketTimestamp);
    radioData.lastPacketTimestamp = HAL_GetTick();

    // Checa se o pacote é destinado ao robô
    if (radioData.rxBuffer[ID_P] == radioData.robotID)
    {
        receivedPacket.velocityX = radioData.rxBuffer[VX_P];
        receivedPacket.velocityY = radioData.rxBuffer[VY_P];
        receivedPacket.velocityW = radioData.rxBuffer[VW_P];

        bool kickType = (radioData.rxBuffer[KICK_CFG_P] & KICK_TYPE_MASK) >> 7; // command byte1 - kick e roller e flags
        if (kickType == true)
            receivedPacket.kickType = DIRECT_CUSTOM;
        else
            receivedPacket.kickType = CHIP_CUSTOM;

        receivedPacket.kickStrength = radioData.rxBuffer[KICK_CFG_P] & KICK_STRENGTH_MASK;
        if (receivedPacket.kickStrength == 0)
            receivedPacket.kickType = KICK_NONE;

        receivedPacket.sendFeedback =
            radioData.rxBuffer[FEEDBACK_P] == SEND_FEEDBACK;

        receivedPacket.rollerVelocity = radioData.rxBuffer[ROLLER_V_P];
        receivedPacket.roller = false;
        if (receivedPacket.rollerVelocity > 0)
            receivedPacket.roller = true;

        receivedPacket.x.lsb = radioData.rxBuffer[X_LSB_P]; 
        receivedPacket.x.msb = radioData.rxBuffer[X_MSB_P]; 
        receivedPacket.y.lsb = radioData.rxBuffer[Y_LSB_P]; 
        receivedPacket.y.msb = radioData.rxBuffer[Y_MSB_P]; 
        receivedPacket.theta.lsb = radioData.rxBuffer[THETA_LSB_P];
        receivedPacket.theta.msb = radioData.rxBuffer[THETA_MSB_P];
    }

    uint8_t n = 0;
    // reseta os bytes do protocolo
    for (n = 0; n < MAX_SPI_BUF_SIZE; n++)
    {
        radioData.rxBuffer[n] = 0;
    }
    radioData.bytesReceived = 0;
}

//============================================================================
// Read a register
// input:
//   reg - number of register to read
// return: value of register
uint8_t nRF24_ReadReg(uint8_t reg)
{
    uint8_t value;

    nRF24_CSN_L();
    nRF24_LL_RW(reg & nRF24_MASK_REG_MAP);
    value = nRF24_LL_RW(nRF24_CMD_NOP);
    nRF24_CSN_H();

    return value;
}

// Write a new value to register
// input:
//   reg - number of register to write
//   value - value to write
void nRF24_WriteReg(uint8_t reg, uint8_t value, bool check)
{
    nRF24_CSN_L();
    if (reg < nRF24_CMD_W_REGISTER)
    {
        // This is a register access
        nRF24_LL_RW(nRF24_CMD_W_REGISTER | (reg & nRF24_MASK_REG_MAP));
        nRF24_LL_RW(value);
    }
    else
    {
        // This is a single byte command or future command/register
        nRF24_LL_RW(reg);
        if ((reg != nRF24_CMD_FLUSH_TX) && (reg != nRF24_CMD_FLUSH_RX) &&
            (reg != nRF24_CMD_REUSE_TX_PL) && (reg != nRF24_CMD_NOP))
        {
            // Send register value
            nRF24_LL_RW(value);
        }
    }
    nRF24_CSN_H();

    // Falha ao escrever no registrador
    volatile uint8_t trueValue = nRF24_ReadReg(reg);
    int attempts = 0;

    while (check && trueValue != value && attempts++ < 100)
    {
        HAL_Delay(5);
        trueValue = nRF24_ReadReg(reg);
    }
    if (check && trueValue != value)
    {
        Error_Handler();
    }
}

// Read a multi-byte register
// input:
//   reg - number of register to read
//   pBuf - pointer to the buffer for register data
//   count - number of bytes to read
static void nRF24_ReadMBReg(uint8_t reg, uint8_t* pBuf, uint8_t count)
{
    nRF24_CSN_L();
    nRF24_LL_RW(reg);
    while (count--)
    {
        *pBuf++ = nRF24_LL_RW(nRF24_CMD_NOP);
    }
    nRF24_CSN_H();
}

// Write a multi-byte register
// input:
//   reg - number of register to write
//   pBuf - pointer to the buffer with data to write
//   count - number of bytes to write
static void nRF24_WriteMBReg(uint8_t reg, uint8_t* pBuf, uint8_t count)
{
    nRF24_CSN_L();
    nRF24_LL_RW(reg);
    while (count--)
    {
        nRF24_LL_RW(*pBuf++);
    }
    nRF24_CSN_H();
}

// Set transceiver to it's initial state
// note: RX/TX pipe addresses remains untouched
void nRF24_Init(void)
{
    // Write to registers their initial values
    bool checkConfig = false;
    nRF24_WriteReg(nRF24_REG_CONFIG, 0x08, checkConfig);
    nRF24_WriteReg(nRF24_REG_EN_AA, 0x3F, checkConfig);
    nRF24_WriteReg(nRF24_REG_EN_RXADDR, 0x03, checkConfig);
    nRF24_WriteReg(nRF24_REG_SETUP_AW, 0x03, checkConfig);
    nRF24_WriteReg(nRF24_REG_SETUP_RETR, 0x03, checkConfig);
    nRF24_WriteReg(nRF24_REG_RF_CH, 0x02, checkConfig);
    nRF24_WriteReg(nRF24_REG_RF_SETUP, 0x0E, checkConfig);
    nRF24_WriteReg(nRF24_REG_STATUS, 0x00, checkConfig);
    nRF24_WriteReg(nRF24_REG_RX_PW_P0, 0x00, checkConfig);
    nRF24_WriteReg(nRF24_REG_RX_PW_P1, 0x00, checkConfig);
    nRF24_WriteReg(nRF24_REG_RX_PW_P2, 0x00, checkConfig);
    nRF24_WriteReg(nRF24_REG_RX_PW_P3, 0x00, checkConfig);
    nRF24_WriteReg(nRF24_REG_RX_PW_P4, 0x00, checkConfig);
    nRF24_WriteReg(nRF24_REG_RX_PW_P5, 0x00, checkConfig);
    nRF24_WriteReg(nRF24_REG_DYNPD, 0x00, checkConfig);
    nRF24_WriteReg(nRF24_REG_FEATURE, 0x00, checkConfig);
    // Clear the FIFO's
    nRF24_FlushRX();
    nRF24_FlushTX();

    // Clear any pending interrupt flags
    nRF24_ClearIRQFlags();

    // Deassert CSN pin (chip release)
    nRF24_CSN_H();
}

// Check if the nRF24L01 present
// return:
//   1 - nRF24L01 is online and responding
//   0 - received sequence differs from original
uint8_t nRF24_Check(void)
{
    uint8_t rxbuf[5];
    uint8_t i;
    uint8_t* ptr = (uint8_t*)nRF24_TEST_ADDR;

    // Write test TX address and read TX_ADDR register
    nRF24_WriteMBReg(nRF24_CMD_W_REGISTER | nRF24_REG_TX_ADDR, ptr, 5);
    nRF24_ReadMBReg(nRF24_CMD_R_REGISTER | nRF24_REG_TX_ADDR, rxbuf, 5);

    // Compare buffers, return error on first mismatch
    for (i = 0; i < 5; i++)
    {
        if (rxbuf[i] != *ptr++)
            return 0;
    }

    return 1;
}

// Control transceiver power mode
// input:
//   mode - new state of power mode, one of nRF24_PWR_xx values
void nRF24_SetPowerMode(uint8_t mode)
{
    uint8_t reg;

    reg = nRF24_ReadReg(nRF24_REG_CONFIG);
    if (mode == nRF24_PWR_UP)
    {
        // Set the PWR_UP bit of CONFIG register to wake the transceiver
        // It goes into Stanby-I mode with consumption about 26uA
        reg |= nRF24_CONFIG_PWR_UP;
    }
    else
    {
        // Clear the PWR_UP bit of CONFIG register to put the transceiver
        // into power down mode with consumption about 900nA
        reg &= ~nRF24_CONFIG_PWR_UP;
    }
    nRF24_WriteReg(nRF24_REG_CONFIG, reg, true);
}

// Configurer the TX_DS interrupt flag
// input:
// mode - nRF24_FLAG_TX_DS
void nRF24_SetTXInterrupt(uint8_t mode)
{
    uint8_t reg;

    reg = nRF24_ReadReg(nRF24_REG_CONFIG);
    if (mode == nRF24_FLAG_TX_DS)
    {
        // Set the FLAG_TX_DS bit of CONFIG register to disable IRQ on TX
        // transmission
        reg |= nRF24_FLAG_TX_DS;
    }
    else
    {
        // Clear the FLAG_TX_DS bit of CONFIG register to enable IRQ on TX
        // transmission
        reg &= ~nRF24_FLAG_TX_DS;
    }
    nRF24_WriteReg(nRF24_REG_CONFIG, reg, true);
}

// Set transceiver operational mode
// input:
//   mode - operational mode, one of nRF24_MODE_xx values
void nRF24_SetOperationalMode(uint8_t mode)
{
    uint8_t reg;
    nRF24_RX_OFF();

    // Configure PRIM_RX bit of the CONFIG register
    reg = nRF24_ReadReg(nRF24_REG_CONFIG);
    reg &= ~nRF24_CONFIG_PRIM_RX;
    reg |= (mode & nRF24_CONFIG_PRIM_RX);
    nRF24_WriteReg(nRF24_REG_CONFIG, reg, true);
    Delay_us(130);
    nRF24_RX_ON();
}

// Set transceiver DynamicPayloadLength feature for all the pipes
// input:
//   mode - status, one of nRF24_DPL_xx values
void nRF24_SetDynamicPayloadLength(uint8_t mode)
{
    uint8_t reg;
    reg = nRF24_ReadReg(nRF24_REG_FEATURE);
    if (mode)
    {
        nRF24_WriteReg(nRF24_REG_FEATURE, reg | nRF24_FEATURE_EN_DPL, true);
        nRF24_WriteReg(nRF24_REG_DYNPD, 0x1F, true);
    }
    else
    {
        nRF24_WriteReg(nRF24_REG_FEATURE, reg & ~nRF24_FEATURE_EN_DPL, true);
        nRF24_WriteReg(nRF24_REG_DYNPD, 0x0, true);
    }
}

// Enables Payload With Ack. NB Refer to the datasheet for proper retransmit
// timing. input:
//   mode - status, 1 or 0
void nRF24_SetPayloadWithAck(uint8_t mode)
{
    uint8_t reg;
    reg = nRF24_ReadReg(nRF24_REG_FEATURE);
    if (mode)
    {
        nRF24_WriteReg(nRF24_REG_FEATURE, reg | nRF24_FEATURE_EN_ACK_PAY, true);
    }
    else
    {
        nRF24_WriteReg(nRF24_REG_FEATURE, reg & ~nRF24_FEATURE_EN_ACK_PAY,
                       true);
    }
}

// Configure transceiver CRC scheme
// input:
//   scheme - CRC scheme, one of nRF24_CRC_xx values
// note: transceiver will forcibly turn on the CRC in case if auto
// acknowledgment
//       enabled for at least one RX pipe
void nRF24_SetCRCScheme(uint8_t scheme)
{
    uint8_t reg;

    // Configure EN_CRC[3] and CRCO[2] bits of the CONFIG register
    reg = nRF24_ReadReg(nRF24_REG_CONFIG);
    reg &= ~nRF24_MASK_CRC;
    reg |= (scheme & nRF24_MASK_CRC);
    nRF24_WriteReg(nRF24_REG_CONFIG, reg, true);
}

// Set frequency channel
// input:
//   channel - radio frequency channel, value from 0 to 127
// note: frequency will be (2400 + channel)MHz
// note: PLOS_CNT[7:4] bits of the OBSERVER_TX register will be reset
void nRF24_SetRFChannel(uint8_t channel)
{
    nRF24_RX_OFF();
    nRF24_WriteReg(nRF24_REG_RF_CH, channel, true);
    nRF24_RX_ON();
}

// Set automatic retransmission parameters
// input:
//   ard - auto retransmit delay, one of nRF24_ARD_xx values
//   arc - count of auto retransmits, value form 0 to 15
// note: zero arc value means that the automatic retransmission disabled
void nRF24_SetAutoRetr(uint8_t ard, uint8_t arc)
{
    // Set auto retransmit settings (SETUP_RETR register)
    nRF24_WriteReg(nRF24_REG_SETUP_RETR,
                   (uint8_t)((ard << 4) | (arc & nRF24_MASK_RETR_ARC)), true);
}

// Set of address widths
// input:
//   addr_width - RX/TX address field width, value from 3 to 5
// note: this setting is common for all pipes
void nRF24_SetAddrWidth(uint8_t addr_width)
{
    nRF24_WriteReg(nRF24_REG_SETUP_AW, addr_width - 2, true);
}

// Set static RX address for a specified pipe
// input:
//   pipe - pipe to configure address, one of nRF24_PIPEx values
//   addr - pointer to the buffer with address
// note: pipe can be a number from 0 to 5 (RX pipes) and 6 (TX pipe)
// note: buffer length must be equal to current address width of transceiver
// note: for pipes[2..5] only first byte of address will be written because
//       other bytes of address equals to pipe1
// note: for pipes[2..5] only first byte of address will be written because
//       pipes 1-5 share the four most significant address bytes
void nRF24_SetAddr(uint8_t pipe, const uint8_t* addr)
{
    // Por algum motivo esta função não funciona
    //
    // NAO UTILIZE
    return;
    //     uint8_t addr_width;
    //
    //     // RX_ADDR_Px register
    //     switch(pipe)
    //     {
    //         case nRF24_PIPETX:
    //         case nRF24_PIPE0:
    //         case nRF24_PIPE1:
    //             // Get address width
    //             addr_width = nRF24_ReadReg(nRF24_REG_SETUP_AW) + 1;
    //             // Write address in reverse order (LSByte first)
    //             addr += addr_width;
    //             nRF24_CSN_L();
    //             nRF24_LL_RW(nRF24_CMD_W_REGISTER | nRF24_ADDR_REGS[pipe]);
    //             do
    //             {
    //                 nRF24_LL_RW(*addr--);
    //             }while (addr_width--);
    //             nRF24_CSN_H();
    //             break;
    //         case nRF24_PIPE2:
    //         case nRF24_PIPE3:
    //         case nRF24_PIPE4:
    //         case nRF24_PIPE5:
    //             // Write address LSBbyte (only first byte from the addr
    //             buffer) nRF24_WriteReg(nRF24_ADDR_REGS[pipe], *addr); break;
    //         default:
    //             // Incorrect pipe number -> do nothing
    //             break;
    //     }
}

// Configure RF output power in TX mode
// input:
//   tx_pwr - RF output power, one of nRF24_TXPWR_xx values
void nRF24_SetTXPower(uint8_t tx_pwr)
{
    uint8_t reg;

    // Configure RF_PWR[2:1] bits of the RF_SETUP register
    reg = nRF24_ReadReg(nRF24_REG_RF_SETUP);
    reg &= ~nRF24_MASK_RF_PWR;
    reg |= tx_pwr;
    nRF24_WriteReg(nRF24_REG_RF_SETUP, reg, true);
}

// Configure transceiver data rate
// input:
//   data_rate - data rate, one of nRF24_DR_xx values
void nRF24_SetDataRate(uint8_t data_rate)
{
    uint8_t reg;

    // Configure RF_DR_LOW[5] and RF_DR_HIGH[3] bits of the RF_SETUP register
    reg = nRF24_ReadReg(nRF24_REG_RF_SETUP);
    reg &= ~nRF24_MASK_DATARATE;
    reg |= data_rate;
    nRF24_WriteReg(nRF24_REG_RF_SETUP, reg, true);
}

// Configure a specified RX pipe
// input:
//   pipe - number of the RX pipe, value from 0 to 5
//   aa_state - state of auto acknowledgment, one of nRF24_AA_xx values
//   payload_len - payload length in bytes
void nRF24_SetRXPipe(uint8_t pipe, uint8_t aa_state, uint8_t payload_len)
{
    uint8_t reg;

    // Enable the specified pipe (EN_RXADDR register)
    reg = (nRF24_ReadReg(nRF24_REG_EN_RXADDR) | (1 << pipe)) & nRF24_MASK_EN_RX;
    nRF24_WriteReg(nRF24_REG_EN_RXADDR, reg, true);

    // Set RX payload length (RX_PW_Px register)
    nRF24_WriteReg(nRF24_RX_PW_PIPE[pipe], payload_len & nRF24_MASK_RX_PW,
                   true);

    // Set auto acknowledgment for a specified pipe (EN_AA register)
    reg = nRF24_ReadReg(nRF24_REG_EN_AA);
    if (aa_state == nRF24_AA_ON)
    {
        reg |= (1 << pipe);
    }
    else
    {
        reg &= ~(1 << pipe);
    }
    nRF24_WriteReg(nRF24_REG_EN_AA, reg, true);
}

// Disable specified RX pipe
// input:
//   PIPE - number of RX pipe, value from 0 to 5
void nRF24_ClosePipe(uint8_t pipe)
{
    uint8_t reg;

    reg = nRF24_ReadReg(nRF24_REG_EN_RXADDR);
    reg &= ~(1 << pipe);
    reg &= nRF24_MASK_EN_RX;
    nRF24_WriteReg(nRF24_REG_EN_RXADDR, reg, true);
}

// Enable the auto retransmit (a.k.a. enhanced ShockBurst) for the specified RX
// pipe input:
//   pipe - number of the RX pipe, value from 0 to 5
void nRF24_EnableAA(uint8_t pipe)
{
    uint8_t reg;

    // Set bit in EN_AA register
    reg = nRF24_ReadReg(nRF24_REG_EN_AA);
    reg |= (1 << pipe);
    nRF24_WriteReg(nRF24_REG_EN_AA, reg, true);
}

// Disable the auto retransmit (a.k.a. enhanced ShockBurst) for one or all RX
// pipes input:
//   pipe - number of the RX pipe, value from 0 to 5, any other value will
//   disable AA for all RX pipes
void nRF24_DisableAA(uint8_t pipe)
{
    uint8_t reg;

    if (pipe > 5)
    {
        // Disable Auto-ACK for ALL pipes
        nRF24_WriteReg(nRF24_REG_EN_AA, 0x00, true);
    }
    else
    {
        // Clear bit in the EN_AA register
        reg = nRF24_ReadReg(nRF24_REG_EN_AA);
        reg &= ~(1 << pipe);
        nRF24_WriteReg(nRF24_REG_EN_AA, reg, true);
    }
}

// Get value of the STATUS register
// return: value of STATUS register
uint8_t nRF24_GetStatus(void)
{
    return nRF24_ReadReg(nRF24_REG_STATUS);
}

// Get pending IRQ flags
// return: current status of RX_DR, TX_DS and MAX_RT bits of the STATUS register
uint8_t nRF24_GetIRQFlags(void)
{
    return (nRF24_ReadReg(nRF24_REG_STATUS) & nRF24_MASK_STATUS_IRQ);
}

// Get status of the RX FIFO
// return: one of the nRF24_STATUS_RXFIFO_xx values
uint8_t nRF24_GetStatus_RXFIFO(void)
{
    return (nRF24_ReadReg(nRF24_REG_FIFO_STATUS) & nRF24_MASK_RXFIFO);
}

// Get status of the TX FIFO
// return: one of the nRF24_STATUS_TXFIFO_xx values
// note: the TX_REUSE bit ignored
uint8_t nRF24_GetStatus_TXFIFO(void)
{
    return ((nRF24_ReadReg(nRF24_REG_FIFO_STATUS) & nRF24_MASK_TXFIFO) >> 4);
}

// Get pipe number for the payload available for reading from RX FIFO
// return: pipe number or 0x07 if the RX FIFO is empty
uint8_t nRF24_GetRXSource(void)
{
    return ((nRF24_ReadReg(nRF24_REG_STATUS) & nRF24_MASK_RX_P_NO) >> 1);
}

// Get auto retransmit statistic
// return: value of OBSERVE_TX register which contains two counters encoded in
// nibbles:
//   high - lost packets count (max value 15, can be reseted by write to RF_CH
//   register) low  - retransmitted packets count (max value 15, reseted when
//   new transmission starts)
uint8_t nRF24_GetRetransmitCounters(void)
{
    return (nRF24_ReadReg(nRF24_REG_OBSERVE_TX));
}

// Reset packet lost counter (PLOS_CNT bits in OBSERVER_TX register)
void nRF24_ResetPLOS(void)
{
    uint8_t reg;

    // The PLOS counter is reset after write to RF_CH register
    reg = nRF24_ReadReg(nRF24_REG_RF_CH);
    nRF24_WriteReg(nRF24_REG_RF_CH, reg, true);
}

// Flush the TX FIFO
void nRF24_FlushTX(void)
{
    nRF24_WriteReg(nRF24_CMD_FLUSH_TX, nRF24_CMD_NOP, false);
}

// Flush the RX FIFO
void nRF24_FlushRX(void)
{
    nRF24_WriteReg(nRF24_CMD_FLUSH_RX, nRF24_CMD_NOP, false);
}

// Clear any pending IRQ flags
void nRF24_ClearIRQFlags(void)
{
    uint8_t reg;

    // Clear RX_DR, TX_DS and MAX_RT bits of the STATUS register
    reg = nRF24_ReadReg(nRF24_REG_STATUS);
    reg |= nRF24_MASK_STATUS_IRQ;
    nRF24_WriteReg(nRF24_REG_STATUS, reg, false);
}

// Write TX payload
// input:
//   pBuf - pointer to the buffer with payload data
//   length - payload length in bytes
void nRF24_WritePayload(uint8_t* pBuf, uint8_t length)
{
    nRF24_WriteMBReg(nRF24_CMD_W_TX_PAYLOAD, pBuf, length);
}

static uint8_t nRF24_GetRxDplPayloadWidth()
{
    uint8_t value;

    nRF24_CSN_L();
    nRF24_LL_RW(nRF24_CMD_R_RX_PL_WID);
    value = nRF24_LL_RW(nRF24_CMD_NOP);
    nRF24_CSN_H();

    return value;
}

static nRF24_RXResult nRF24_ReadPayloadGeneric(uint8_t* pBuf, uint8_t* length,
                                               uint8_t dpl)
{
    uint8_t pipe;
    if (nRF24_GetStatus_RXFIFO() == nRF24_STATUS_RXFIFO_EMPTY)
    {
        return nRF24_RX_EMPTY;
    }
    nRF24_ClearIRQFlags();

    // Extract a payload pipe number from the STATUS register
    pipe = (nRF24_ReadReg(nRF24_REG_STATUS) & nRF24_MASK_RX_P_NO) >> 1;

    // RX FIFO empty?
    if (pipe < 6)
    {
        // Get payload length
        if (dpl)
        {
            *length = nRF24_GetRxDplPayloadWidth();
            if (*length > 32) // broken packet
            {
                *length = 0;
                nRF24_FlushRX();
            }
        }
        else
        {
            *length = nRF24_ReadReg(nRF24_RX_PW_PIPE[pipe]);
        }

        // Read a payload from the RX FIFO
        if (*length)
        {
            nRF24_ReadMBReg(nRF24_CMD_R_RX_PAYLOAD, pBuf, *length);
        }

        nRF24_FlushRX();
        return ((nRF24_RXResult)pipe);
    }

    // The RX FIFO is empty
    *length = 0;
    return nRF24_RX_EMPTY;
}

// Read top level payload available in the RX FIFO
// input:
//   pBuf - pointer to the buffer to store a payload data
//   length - pointer to variable to store a payload length
// return: one of nRF24_RX_xx values
//   nRF24_RX_PIPEX - packet has been received from the pipe number X
//   nRF24_RX_EMPTY - the RX FIFO is empty
nRF24_RXResult nRF24_ReadPayload(uint8_t* pBuf, uint8_t* length)
{
    return nRF24_ReadPayloadGeneric(pBuf, length, 0);
}

nRF24_RXResult nRF24_ReadPayloadDpl(uint8_t* pBuf, uint8_t* length)
{
    return nRF24_ReadPayloadGeneric(pBuf, length, 1);
}

uint8_t nRF24_GetFeatures()
{
    return nRF24_ReadReg(nRF24_REG_FEATURE);
}
void nRF24_ActivateFeatures()
{
    nRF24_CSN_L();
    nRF24_LL_RW(nRF24_CMD_ACTIVATE);
    nRF24_LL_RW(0x73);
    nRF24_CSN_H();
}
void nRF24_WriteAckPayload(nRF24_RXResult pipe, char* payload, uint8_t length)
{
    nRF24_CSN_L();
    nRF24_LL_RW(nRF24_CMD_W_ACK_PAYLOAD | pipe);
    while (length--)
    {
        nRF24_LL_RW((uint8_t)*payload++);
    }
    nRF24_CSN_H();
}
//============================================================================
