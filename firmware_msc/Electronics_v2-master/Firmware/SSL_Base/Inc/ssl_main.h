/*
 * ssl_main.h
 *
 *  Created on: 26 de ago de 2022
 *      Author: leonardo
 */

#ifndef INC_SSL_MAIN_H_
#define INC_SSL_MAIN_H_

#include "kickConfig.h"
#include "main.h"
#include "motorControl.h"
#include "nRF24L01.h"

#define LOOP_uS 5000           // Tempo do loop principal [us]
#define BATTERY_READ_TIMEOUT 2 // Timeout da leitura da bateria [ms]
#define MIN_BATTERY 11000      // Bateria mínima para operação [mV]
#define DEBUG_SIZE 32          // Quantidade de bytes de debug enviados
#define BALL_SENSOR_FILTER_ON                                                  \
    5 // Espera o sensor ficar ligado por A ciclos
       // (A x LOOP_uS) para considerar como ativado
#define BALL_SENSOR_FILTER_OFF                                                 \
    30 // Espera o sensor ficar desligado por A ciclos
       // (A x LOOP_uS) para considerar como ativado
#define KICK_CYCLES                                                            \
    40 // Mantém o comando de chute por 40 ciclos (200 ms)
       // antes de desligar

#define __GET_HIGH_BYTE(__VALUE__) ((__VALUE__ & 0xFF00) >> 8)
#define __GET_LOW_BYTE(__VALUE__) (__VALUE__ & 0x00FF)

#define CAPACITOR_SAMPLES 20
static uint8_t capacitorVoltages[CAPACITOR_SAMPLES] = {0};
static uint16_t capacitorIndex = 0;

typedef struct _RobotSensors
{
	struct
	{
		uint32_t updated;
		uint32_t time;
		float rotVel[3];	// around X, Y, Z [rad/s]
	} gyr;

	struct
	{
		uint32_t updated;
		uint32_t time;
		int16_t linAcc[3];	// X, Y, Z [m^2/s]
	} acc;

	struct PACKED
	{
		uint32_t updated;
		uint32_t time;		// local arrival time (systime)
		uint32_t delay;		// t [us]
		float pos[3];		// X, Y, Z [m]
		uint32_t camId;
		uint32_t noVision;
	} vision;
} RobotSensors;

typedef struct _RobotState
{
	float pos[3];	// [m]
	float vel[3];	// [m/s]
	float acc[3];
	float magZ;     // [rad]

	uint32_t posUpdated;
	uint32_t velUpdated;
	uint32_t accUpdated;
} RobotState;

typedef struct RobotData_t
{
    uint32_t battery;
    int16_t mVbattery;

    RobotSensors sensors;

    RobotState state;

    uint8_t capacitorVoltage;
    int16_t wheelSpeed[MOTOR_SZ];
    int16_t encoderCount[MOTOR_SZ];
    bool ballSensor;
    uint32_t ballCycles;

    uint32_t radioCycles;

    uint32_t cycles;

    KICK_TYPE_t kickType;
    int8_t kickStrength;
    uint16_t kickControl;
    uint32_t kickCyles;
    bool debugKick;
    struct
    {
        int16_t x;
        int16_t y;
        int16_t theta;
    } pose;
    bool rollerEnable;
    int8_t rollerSpeed; // Velocidade do roller em rad/s
    uint32_t rollerTimer;

} RobotData_t;

typedef union DebugData_t {
    struct
    {
        //      informação ---- [indice]
        uint8_t kickSensor;      // [0]
        uint8_t robotID;         // [1]
        uint8_t battery;         // [2]
        uint8_t sensorType;      // [3]
        uint8_t wheelSpeed1High; // [4]
        uint8_t wheelSpeed1Low;  // [5]
        uint8_t wheelSpeed2High; // [6]
        uint8_t wheelSpeed2Low;  // [7]
        uint8_t wheelSpeed3High; // [8]
        uint8_t wheelSpeed3Low;  // [9]
        uint8_t wheelSpeed4High; // [10]
        uint8_t wheelSpeed4Low;  // [11]
        uint8_t count1High;      // [12]
        uint8_t count1Low;       // [13]
        uint8_t count2High;      // [14]
        uint8_t count2Low;       // [15]
        uint8_t count3High;      // [16]
        uint8_t count3Low;       // [17]
        uint8_t count4High;      // [18]
        uint8_t count4Low;       // [19]
        uint8_t capacitorVoltage;// [20]
        uint8_t packetFrequency; // [21]
        uint8_t posXHigh;        // [22]
        uint8_t posXLow;         // [23]
        uint8_t posYHigh;        // [24]
        uint8_t posYLow;         // [25]
        uint8_t posThetaHigh;    // [26]
        uint8_t posThetaLow;     // [27]
        uint8_t data28;          // [28]
        uint8_t data29;          // [29]
        uint8_t data30;          // [30]
        uint8_t data31;          // [31]
    };
    uint8_t data[DEBUG_SIZE];
} DebugData_t;

static RobotData_t robotData = {.battery = 0,
                                .mVbattery = 0,
                                .capacitorVoltage = 0,
                                .wheelSpeed = {0, 0, 0, 0, 0},
                                .encoderCount = {0, 0, 0, 0, 0},
                                .ballSensor = false,
                                .ballCycles = 0,
								.sensors.gyr.rotVel = {0.0, 0.0, 0.0},
								.sensors.acc.linAcc = {0, 0, 0},
                                .radioCycles = 0,
                                .cycles = 0,
                                .kickType = KICK_NONE,
                                .kickStrength = 0,
                                .kickControl = CYCLES_BETWEEN_KICKS,
                                .kickCyles = KICK_CYCLES,
                                .debugKick = false,
                                .pose = {.x = 0, .y = 0, .theta = 0},
                                .rollerEnable = false,
                                .rollerTimer = 0};

static DebugData_t debugData;

/**
 * @brief Faz a configuração inicial e inicialização do firmware
 */
void setup();

/**
 * @brief Roda o controle dos motores
 */
void runMotors();

void HAL_IncTick(void);

/**
 * @brief Retorna o ID atual do robô [0, 15]
 *
 * @return
 */
uint8_t getRobotID();

/**
 * @brief Mostra o ID do robô
 *
 * Atualmente isso é feito piscando um led na quantidade de vezes igual ao ID
 */
void showRobotID();

/**
 * @brief Lê o canal atual do rádio
 *
 * @param _rx - Valor do canal de recebimento de dados. f = 2400 + _rx MHz.
 * @param _tx - Valor do canal de envio de dados. f = 2400 + _tx MHz.
 */
void getRobotChannel(uint8_t* _rx, uint8_t* _tx);

/**
 * @brief A cada 200 ciclos (1s) checa se o ID ou canal mudou
 */
void checkRobotIDChannel();

/**
 * @brief Processa um pacote do rádio caso algum tenha chegado
 *
 * @param _sensorData
 */
void processRadio();

/**
 * @brief A cada 200 ciclos (1s) lê o valor da bateria
 */
void readBattery();

/**
 * @brief Atualiza os dados enviados de volta para a estratégia
 */
void updateDebugInfo();

void convertDebugSpeed(uint8_t* _high, uint8_t* _low, int16_t _speed);

/**
 * @brief Se estiver ativado, faz o robô chutar
 */
void setKickCommand();

/**
 * @brief Lê o sensor da bola
 */
void readBallSensor();

/**
 * @brief Faz a estimativa de posição e velocidade do robô
 */
void estimateState();

#endif /* INC_SSL_MAIN_H_ */
