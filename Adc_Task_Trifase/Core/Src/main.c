/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queue.h"
#include "semphr.h"
#include <math.h>
QueueHandle_t energyqueueHandle;
QueueHandle_t rxuartqueueHandle;
SemaphoreHandle_t uartBinSemaHandle;
QueueHandle_t adchalfselectQueueHandle;
extern UART_HandleTypeDef huart2;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


#define MAX_DATA_LEN 	122
typedef struct __attribute__((packed)) _ENERGY_DATA
{
	uint32_t rms_current;
	uint32_t rms_voltage;
	uint32_t pot_aparente;
	uint32_t pot_reativa;
	uint32_t  pot_ativa;
	uint16_t pf;

}ENERGY_DATA; // 176 bit e 22 bytes

//
typedef struct __attribute__((packed)) thee_phase_energy_data_t_ {
	ENERGY_DATA phaseA;
	ENERGY_DATA phaseB;
	ENERGY_DATA phaseC;
	uint64_t consumption;
	uint32_t TotalM_Voltag;
	uint32_t Total_AC_current;
	uint16_t Total_PowerAparent;
	uint32_t Total_PowerReativa;
	uint32_t Total_PowerAtiva;
	uint32_t Total_PowerFactor;
}thee_phase_energy_data_t; // 768 bits e 96 Bytes

typedef union type_ {
	uint32_t bits32;
	uint16_t bits16[2];
}uint32_16_t;

typedef enum {
  UMS_RECEIVING,
  UMS_PROCESSING_RESPONSE_PACKAGE,
  UMS_SENDING_RESPONSE,
  UMS_TIMEOUT
} UART_MACHINE_STATES;

typedef enum ERRORS_LISTtag
{
    EL_NO_ERROR = 0x00,
    EL_INVALID_OPCODE = 0x01,
    EL_INVALID_DATA = 0x02,
    EL_MEMORY_WRITE_ERROR = 0x03,

    EL_NUM_ERRORS
}ERRORS_LIST;

typedef enum {
  UPP_STX,
  UPP_DEVICE_ADDRESS,
  UPP_OPCODE,
  UPP_DATA_LEN,
  UPP_DATA,
  UPP_CHECKSUM,
  UPP_ETX
} UART_PACKAGE_PARTS;
typedef struct {
  unsigned char uc_Stx;
  unsigned char uc_DeviceAddress;
  unsigned char uc_OpCode;
  unsigned char uc_Datalen;
  unsigned char uc_Data[MAX_DATA_LEN];
  unsigned char uc_Checksum;
  unsigned char uc_Etx;
} UART_PACKAGE_PROTOCOL;

// ---- VARIÁVEIS DA UART ----
UART_MACHINE_STATES m_udtUartmachineStates;
UART_PACKAGE_PARTS m_udtUartPackageParts;
UART_PACKAGE_PROTOCOL m_udtReceptionPackage;
UART_PACKAGE_PROTOCOL m_udtTransmitionPackage;

uint8_t m_ucCorrentDataPos = 0;
uint8_t m_ucCalculatedChecksum = 0;
uint8_t m_blnProcessingScapeChar = 0;
uint8_t m_blnReply = 0;

uint8_t rx_buffer; // Buffer de 1 byte para o Callback RX
uint8_t tx_buffer[MAX_DATA_LEN + 10]; // Buffer final de envio
uint8_t m_ucTXBufferCorrentDataPos = 0;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_RESOLUTION	0.0008056641//0.000837696335f//0.996551//Encontra a tensão da Saída do Sensor x100
#define V_SENSIBILITY 	135.0//789.1033381f//808.1496160//63.37024//Encounter a tensão de entrada do Sensor  x100
#define V_GAIN			ADC_RESOLUTION * V_SENSIBILITY

//DEFINE UART


#define STX 0x02
#define ETX 0x03
#define ESC 0x1B
#define ESC_INC 0x40
#define DEVICE_ADDR 0x01
#define RESPONSE_OPCODE_MASK 0x80

// OPCODES
#define UO_KEEPALIVE 0x01
#define UO_GET_DATA 0x02
#define UO_SETCONFIG 0x03

// ERROS
#define EL_NO_ERROR 0x00
#define EL_INVALID_DATA 0x01
#define EL_MEMORY_WRITE_ERROR 0x02
#define EL_INVALID_OPCODE 0x03



//#define C2_SENSOR_MULT	0.0008056641//0.000878232758f//0.362903//Encontra a tensão da Saída do Sensor x100
#define A_SENSIBILITY	7.575f//5.4691//Encontra a Corrente de entrada do Sensor  x100
#define A_GAIN			ADC_RESOLUTION * A_SENSIBILITY

#define F_BUFFER_SIZE	(256*3)
#define H_BUFFER_SIZE	(128*3)
/*
Isso vai causar um bug de memória grave (seus dados vão virar uma bagunça ou continuar zerados), e eu vou te explicar o porquê de forma simples:
O F_BUFFER_SIZE atual é 768. O DMA vai jogar 768 dados de 16 bits na memória e avisar que chegou na metade (em 384). Mas como o seu array adcBuffer é d
e 32 bits, quando você manda ele ler a posição H_BUFFER_SIZE (384), ele está na verdade indo buscar o dado no endereço 768 da memória... onde não tem nada do ADC! Ele "passa direto" de onde o DMA escreveu.

*/
/*
#define F_BUFFER_SIZE	(128*3) // 384 índices (comportam 768 dados de 16 bits do DMA)
#define H_BUFFER_SIZE	(64*3)  // 192 índices
*/

#define Amostragens 128

#define MIN_RMS_VOLTAGE 300 // pRAque que serve?


uint32_16_t adcBuffer[F_BUFFER_SIZE];
float 	adc_voltageA[Amostragens];
float 	adc_currentA[Amostragens];
float 	adc_voltageB[Amostragens];
float 	adc_currentB[Amostragens];
float 	adc_voltageC[Amostragens];
float 	adc_currentC[Amostragens];



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* Definitions for AdcTaask */
osThreadId_t AdcTaaskHandle;
const osThreadAttr_t AdcTaask_attributes = {
  .name = "AdcTaask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UartTask */
osThreadId_t UartTaskHandle;
const osThreadAttr_t UartTask_attributes = {
  .name = "UartTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* USER CODE BEGIN PV */

thee_phase_energy_data_t EnergyData;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
void StartAdcTask(void *argument);
void StartUartTask(void *argument);

/* USER CODE BEGIN PFP */
void Serialize16Bits(uint16_t value, uint8_t *buffer, uint8_t pos)
{
    buffer[pos] = (uint8_t)((value >> 8) & 0xFF);
    pos++;
    buffer[pos] = (uint8_t)(value & 0xFF);

}

void Serialize32Bits(uint32_t value, uint8_t *buffer, uint8_t pos)
{
    buffer[(pos)] = (uint8_t)((value >> 24) & 0xFF);
    pos++;
    buffer[pos] = (uint8_t)((value >> 16) & 0xFF);
    pos++;
    buffer[pos] = (uint8_t)((value >> 8) & 0xFF);
    pos++;
    buffer[pos] = (uint8_t)(value & 0xFF);
}


void Serialize64Bits(uint64_t value, uint8_t *buffer, uint8_t pos)
{
    buffer[pos] = (uint8_t)((value >> 56) & 0xFF);
    pos++;
    buffer[pos] = (uint8_t)((value >> 48) & 0xFF);
    pos++;
    buffer[(pos)] = (uint8_t)((value >> 40) & 0xFF);
    pos++;
    buffer[(pos)] = (uint8_t)((value >> 32) & 0xFF);
    pos++;
    buffer[(pos)] = (uint8_t)((value >> 24) & 0xFF);
    pos++;
    buffer[(pos)] = (uint8_t)((value >> 16) & 0xFF);
    pos++;
    buffer[(pos)] = (uint8_t)((value >> 8) & 0xFF);
    pos++;
    buffer[(pos)] = (uint8_t)(value & 0xFF);

}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void ResetSerial()
{
  // Inicialmente estamos aguardando a recepção dos dados do dispositivo remoto.
  m_udtUartmachineStates = UMS_RECEIVING;
  // Inicialmente estamos aguardando o STX
  m_udtUartPackageParts = UPP_STX;
  // Cria os ponteiros para os pacotes
  unsigned char* pucReceptionpackage = &m_udtReceptionPackage.uc_Stx;
  unsigned char* pucTranmitionpackage = &m_udtTransmitionPackage.uc_Stx;

  unsigned char ucPosition = 0;

  // Prepara a varredura
  while(ucPosition <  sizeof(UART_PACKAGE_PROTOCOL))
  {
      *pucReceptionpackage = 0x00;
      *pucTranmitionpackage = 0x00;
      pucReceptionpackage++;
      pucTranmitionpackage++;

     ucPosition++;
  }

  // Inicializa a posição a ser processada como 0.
  m_ucCorrentDataPos = 0x00;

  // Inicializa o Checksum calculado com 0
  m_ucCalculatedChecksum = 0;

}


unsigned char CalculateChecksum(unsigned char* udtpackage, unsigned char ucLen);
uint8_t SendData(unsigned char ucDataTosend, uint8_t blnIsSpecialChar);

void UartMainProcess(unsigned char ucData)
{
  switch (m_udtUartmachineStates)
  {
    case UMS_RECEIVING:
    {

//      // Verifica se existem dados para ler
      /*if(Serial2.available() == 0)
//      {
        // Sem dados para ler.
        ///////////////////////

        // Cai fora
         break;
       }*/

      // recebe o dado da queue


      /////////////////////////////////////////////////
      // Processamento dos caracteres especiais
      /////////////////////////////////////////////////

      // Verifica se é um inicializador de pacotes
      if( (ucData == STX) && (m_udtUartPackageParts == UPP_STX) )
      {
          // Inicializador de pacotes.
          //////////////////////////////

          // Reseta a serial;
          ResetSerial();

          // Guarda o dado recebido
          m_udtReceptionPackage.uc_Stx = STX;

          // Vai para o próximo estado
          m_udtUartPackageParts = (UART_PACKAGE_PARTS)UPP_DEVICE_ADDRESS;

          // Cai fora.
          break;
      }

      // Verifica se é um terminador de pacotes
      if( ucData == ETX )
      {
        // Terminador de pacotes.
        //////////////////////////////

        // Verifica se está na hora de receber esse dado.
        if( m_udtUartPackageParts != (UART_PACKAGE_PARTS)UPP_ETX )
        {
          // Dado incorreto.
          //////////////////

          // Reseta a serial;
          ResetSerial();

          // Cai fora.
          break;
        }

        m_udtReceptionPackage.uc_Etx = ETX;

        // Vai para o próximo estado
        m_udtUartmachineStates = UMS_PROCESSING_RESPONSE_PACKAGE;
        m_udtUartPackageParts = (UART_PACKAGE_PARTS)UPP_STX;

        // Cai fora
        break;
      }

      // Verifica se é um scape char e se esse deve ser tratado
      if(ucData == ESC)
      {
        m_blnProcessingScapeChar = 1;

        break;
      }

      // Verifica se está no dado pós scape char
      if(m_blnProcessingScapeChar == 1)
      {
        // Dado pós scape char
        ///////////////////////

        // Processa o dado
        ucData = ucData & ~ESC_INC;

        // Indica que já tratou
        m_blnProcessingScapeChar = 0;
      }

      switch (m_udtUartPackageParts)
      {
        case UPP_DEVICE_ADDRESS:
        {
          // Verifica se o dado recebido é o correto.
          if( ucData == DEVICE_ADDR )
          {
              // Dado correto.
              //////////////////

              // Guarda o dado recebido
              m_udtReceptionPackage.uc_DeviceAddress = ucData;

              // Vai para o próximo estado
              m_udtUartPackageParts = (UART_PACKAGE_PARTS)UPP_OPCODE;
          }
        }
        break;
        case UPP_OPCODE:
        {
          // Guarda o dado recebido
          m_udtReceptionPackage.uc_OpCode = ucData;

          // Vai para o próximo estado
          m_udtUartPackageParts = (UART_PACKAGE_PARTS)UPP_DATA_LEN;

        }
        break;
        case UPP_DATA_LEN:
        {
          // Verifica se o dado recebido é o correto.
          if( ucData >= MAX_DATA_LEN )
          {
              // OpCode inválido.
              //////////////////

              // Reseta a serial;
              ResetSerial();

              // Cai fora.
              break;
          }

          // Guarda o dado recebido
          m_udtReceptionPackage.uc_Datalen = ucData;

          // Verifica se existirão dados
          if (m_udtReceptionPackage.uc_Datalen > 0)
          {
              // Existirão dados.
              // /////////////////

              // Vai para o próximo estado
              m_udtUartPackageParts = (UART_PACKAGE_PARTS)UPP_DATA;
          }
          else
          {
              // Não existirão dados.
              // /////////////////////

              // Vai para o próximo estado
              m_udtUartPackageParts = (UART_PACKAGE_PARTS)UPP_CHECKSUM;
          }


        }
        break;
        case UPP_DATA:
        {
          // Guarda o dado recebido
          m_udtReceptionPackage.uc_Data[m_ucCorrentDataPos] = ucData;

          // Incrementa a posição
          m_ucCorrentDataPos++;

          // Verifica se atingiu o número de dados
          if(m_ucCorrentDataPos >= m_udtReceptionPackage.uc_Datalen)
          {
            // Chegou ao fim dos dados
            /////////////////////////////

            // Vai para o próximo estado
            m_udtUartPackageParts = (UART_PACKAGE_PARTS)UPP_CHECKSUM;
          }
        }
        break;
        case UPP_CHECKSUM:
        {
          // Guarda o dado recebido
          m_udtReceptionPackage.uc_Checksum = ucData;

          // Calcula o checksum do pacote
          m_ucCalculatedChecksum = CalculateChecksum(&m_udtReceptionPackage.uc_Stx, (1 + 1 + 1 + 1 + m_udtReceptionPackage.uc_Datalen +1));

          // Verifica se o checksum bateu
          if(m_udtReceptionPackage.uc_Checksum != m_ucCalculatedChecksum)
          {
            // Não bateu
            /////////////

            // Reseta a serial;
            ResetSerial();

            // Cai fora
            break;
          }

          // Vai para o próximo estado
          m_udtUartPackageParts = (UART_PACKAGE_PARTS)UPP_ETX;
        }
        break;
        case UPP_STX:
        case UPP_ETX:
        	break;
      }
    }
    break;
    case UMS_PROCESSING_RESPONSE_PACKAGE:
    {
      // Prepara os dados fixos do pacote
      m_udtTransmitionPackage.uc_Stx = STX;
      m_udtTransmitionPackage.uc_Etx = ETX;
      m_udtTransmitionPackage.uc_DeviceAddress = 0x01;
      m_udtTransmitionPackage.uc_OpCode = RESPONSE_OPCODE_MASK | m_udtReceptionPackage.uc_OpCode;


      switch( m_udtReceptionPackage.uc_OpCode )
      {
         case UO_KEEPALIVE:
         {
            // Prepara o pacote de resposta
            m_udtTransmitionPackage.uc_Datalen = 0x00;

         }
         break;
         case UO_GET_DATA:
                  {
                      thee_phase_energy_data_t m_udtEnergyDataPacket;

                      // 1. Lê os dados reais vindos da Task do ADC
                      xQueueReceive(energyqueueHandle, &m_udtEnergyDataPacket, portMAX_DELAY);

                      // 2. Serializa os dados recebidos da fila (82 bytes no total)
                      m_udtTransmitionPackage.uc_Datalen = 82;

                      // PHASE A (0 a 17)
                      Serialize16Bits(m_udtEnergyDataPacket.phaseA.rms_voltage, m_udtTransmitionPackage.uc_Data, 0);
                      Serialize16Bits(m_udtEnergyDataPacket.phaseA.rms_current, m_udtTransmitionPackage.uc_Data, 2);
                      Serialize16Bits(m_udtEnergyDataPacket.phaseA.pf,          m_udtTransmitionPackage.uc_Data, 4);
                      Serialize32Bits(m_udtEnergyDataPacket.phaseA.pot_aparente,m_udtTransmitionPackage.uc_Data, 6);
                      Serialize32Bits(m_udtEnergyDataPacket.phaseA.pot_ativa,   m_udtTransmitionPackage.uc_Data, 10);
                      Serialize32Bits(m_udtEnergyDataPacket.phaseA.pot_reativa, m_udtTransmitionPackage.uc_Data, 14);

                      // PHASE B (18 a 35)
                      Serialize16Bits(m_udtEnergyDataPacket.phaseB.rms_voltage, m_udtTransmitionPackage.uc_Data, 18);
                      Serialize16Bits(m_udtEnergyDataPacket.phaseB.rms_current, m_udtTransmitionPackage.uc_Data, 20);
                      Serialize16Bits(m_udtEnergyDataPacket.phaseB.pf,          m_udtTransmitionPackage.uc_Data, 22);
                      Serialize32Bits(m_udtEnergyDataPacket.phaseB.pot_aparente,m_udtTransmitionPackage.uc_Data, 24);
                      Serialize32Bits(m_udtEnergyDataPacket.phaseB.pot_ativa,   m_udtTransmitionPackage.uc_Data, 28);
                      Serialize32Bits(m_udtEnergyDataPacket.phaseB.pot_reativa, m_udtTransmitionPackage.uc_Data, 32);

                      // PHASE C (36 a 53)
                      Serialize16Bits(m_udtEnergyDataPacket.phaseC.rms_voltage, m_udtTransmitionPackage.uc_Data, 36);
                      Serialize16Bits(m_udtEnergyDataPacket.phaseC.rms_current, m_udtTransmitionPackage.uc_Data, 38);
                      Serialize16Bits(m_udtEnergyDataPacket.phaseC.pf,          m_udtTransmitionPackage.uc_Data, 40);
                      Serialize32Bits(m_udtEnergyDataPacket.phaseC.pot_aparente,m_udtTransmitionPackage.uc_Data, 42);
                      Serialize32Bits(m_udtEnergyDataPacket.phaseC.pot_ativa,   m_udtTransmitionPackage.uc_Data, 46);
                      Serialize32Bits(m_udtEnergyDataPacket.phaseC.pot_reativa, m_udtTransmitionPackage.uc_Data, 50);

                      // DADOS TOTAIS E CONSUMO (54 a 83)
                      Serialize16Bits(m_udtEnergyDataPacket.Total_PowerAparent, m_udtTransmitionPackage.uc_Data, 54);
                      Serialize32Bits(m_udtEnergyDataPacket.TotalM_Voltag,      m_udtTransmitionPackage.uc_Data, 56);
                      Serialize32Bits(m_udtEnergyDataPacket.Total_AC_current,   m_udtTransmitionPackage.uc_Data, 60);
                      Serialize32Bits(m_udtEnergyDataPacket.Total_PowerAtiva,   m_udtTransmitionPackage.uc_Data, 64);
                      Serialize32Bits(m_udtEnergyDataPacket.Total_PowerFactor,  m_udtTransmitionPackage.uc_Data, 68);
                      Serialize32Bits(m_udtEnergyDataPacket.Total_PowerReativa, m_udtTransmitionPackage.uc_Data, 72);
                      Serialize64Bits(m_udtEnergyDataPacket.consumption,        m_udtTransmitionPackage.uc_Data, 76);
                  }
                  break;
         case UO_SETCONFIG:
         		 {
         			 ERRORS_LIST udtError = EL_NO_ERROR;
         			 uint8_t blnStatus = 0;

         			 // Verifica se o tamanho do pacote condiz com o esperado
         			 if (m_udtReceptionPackage.uc_Datalen == 4)
         			 {
         				 // Era a opcao 1
         				 //////////////////////////////////


         			 }
         			 else if (m_udtReceptionPackage.uc_Datalen == 0)
         			 {
         				 // Ou a opcao 2
         				 ///////////////////////////////

         				// Executa alguma funcao que retorna booleano de validação
         				//blnStatus = SaveParameters(bytLocation);
         			 }
         			 else
         			 {
         				 //Erro, dados inválidos
         				 /////////////////////////

         				 // Indica o erro
         				 udtError = EL_INVALID_DATA;
         			 }

         			 //Verifica se já veio com erro
         			 if (udtError == EL_NO_ERROR)
         			 {
         				 // Chegou sem erros
         				 ////////////////////

         				 // Verifica se deu algum erro de escrita
         				 if (blnStatus == 0)
         				 {
         					 // Deu erro de escrita
         					 /////////////////////////

         					 // Guarda o erro.
         					 udtError = EL_MEMORY_WRITE_ERROR;
         				 }
         			 }
         			// Prepara os dados que serão enviados (resposta de valdiação)
         			m_udtTransmitionPackage.uc_Datalen = 0x01;	//tamanho do pacote de resposta
         			m_udtTransmitionPackage.uc_Data[0] = (uint8_t)udtError & 0xFF;
         		 }
         		 break;



		 default:
		 {
			 // Retorna um erro
			 ///////////////////

			 // Força o OPCODE de erro

			 m_udtTransmitionPackage.uc_OpCode = RESPONSE_OPCODE_MASK | UO_SETCONFIG;
			m_udtTransmitionPackage.uc_Datalen = 0x01;
			m_udtTransmitionPackage.uc_Data[0] = EL_INVALID_OPCODE;
		 }
      }

      // Calcula o Checksum do pacote a ser enviado
      m_udtTransmitionPackage.uc_Checksum = CalculateChecksum(&m_udtTransmitionPackage.uc_Stx, (1 + 1 + 1 + 1 + m_udtTransmitionPackage.uc_Datalen + 1));

      // Vai para o próximo estado
      m_udtUartmachineStates = UMS_SENDING_RESPONSE;
      m_udtUartPackageParts = (UART_PACKAGE_PARTS)UPP_STX;
    }
    break;
    case UMS_SENDING_RESPONSE:
    {
      //Aqui poderia verificar se a serial está disponivel para responder
//      if( Serial2.availableForWrite() == 0 )
//      {
//        // Não pode transmitir.
//        /////////////////////////
//
//        // Cai fora.
//        break;
//      }

      switch (m_udtUartPackageParts)
      {
        case UPP_STX:
        {
          // Escreve
          if(SendData(m_udtTransmitionPackage.uc_Stx, 1) == 1)
          {
            // Vai para o próximo estado
            m_udtUartPackageParts = (UART_PACKAGE_PARTS)UPP_DEVICE_ADDRESS;
          }
        }
        break;
        case UPP_DEVICE_ADDRESS:
        {
          // Escreve
          if(SendData(m_udtTransmitionPackage.uc_DeviceAddress, 0) == 1)
          {
            // Vai para o próximo estado
            m_udtUartPackageParts = (UART_PACKAGE_PARTS)UPP_OPCODE;
          }
        }
        break;
        case UPP_OPCODE:
        {
          // Escreve
          if(SendData(m_udtTransmitionPackage.uc_OpCode, 0) == 1)
          {
            // Vai para o próximo estado
            m_udtUartPackageParts = (UART_PACKAGE_PARTS)UPP_DATA_LEN;
          }
        }
        break;
        case UPP_DATA_LEN:
        {
          // Escreve
          if(SendData(m_udtTransmitionPackage.uc_Datalen, 0) == 1)
          {
            // Zera a posição
            m_ucCorrentDataPos = 0x00;

            // Verifica se existirão dados
            if (m_udtTransmitionPackage.uc_Datalen > 0)
            {
                // Existirão dados.
                // /////////////////

                // Vai para o próximo estado
                m_udtUartPackageParts = (UART_PACKAGE_PARTS)UPP_DATA;
            }
            else
            {
                // Não existirão dados.
                ////////////////////////

                // Vai para o próximo estado
                m_udtUartPackageParts = (UART_PACKAGE_PARTS)UPP_CHECKSUM;
            }
          }
        }
        break;
        case UPP_DATA:
        {
          // Escreve
          if(SendData(m_udtTransmitionPackage.uc_Data[m_ucCorrentDataPos], 0) == 1)
          {
            // Incrementa a posição
            m_ucCorrentDataPos++;

            // Verifica se atingiu o número de dados
            if(m_ucCorrentDataPos >= m_udtTransmitionPackage.uc_Datalen)
            {
              // Chegou ao fim dos dados
              /////////////////////////////

              // Vai para o próximo estado
              m_udtUartPackageParts = (UART_PACKAGE_PARTS)UPP_CHECKSUM;
            }
          }

        }
        break;
        case UPP_CHECKSUM:
        {
          // Escreve
          if(SendData(m_udtTransmitionPackage.uc_Checksum, 0) == 1)
          {
            // Vai para o próximo estado
            m_udtUartPackageParts = (UART_PACKAGE_PARTS)UPP_ETX;
          }
        }
        break;
        case UPP_ETX:
        {
          // Escreve
          if(SendData(m_udtTransmitionPackage.uc_Etx, 1) == 1)
          {
        	 //Indica para a task que pode enviar a resposta
        	 m_blnReply = 1;

        	 ResetSerial();
          }
        }
        break;
      }
    }
    break;
    case UMS_TIMEOUT:
    {
      // Timeout
      ////////////

      // Reseta a serial;
      ResetSerial();
    }
    break;
  }

}
uint8_t SendData(unsigned char ucDataTosend, uint8_t blnIsSpecialChar)
{

  // Cria a variável de retorno indicando que foi um dado normal
  // false indica um scape char e não deve ir para o próximo
  uint8_t blnReturnValue = 1;

  // Verifica se é um caractere especial.
  if( blnIsSpecialChar == 0 )
  {
    // Não é um caractere especial,
    // devemos tratar
    ///////////////////////////////

    // Verifica se processou um caractere especial na última passada.
    if( m_blnProcessingScapeChar == 1 )
    {
      // Sinalizou um caractere igual a um
      // especial na última passada.
      ///////////////////////////////////////

      // Altera o dado
      ucDataTosend = ucDataTosend | ESC_INC;

      // Indica que já processou.
      m_blnProcessingScapeChar = 0;
    }
    else
    {
      // Não foi um igual a especial na última passada.
      ////////////////////////////////////////////////////

      // Verifica se é item igual a um especial
      if(
          (ucDataTosend == STX)
          ||(ucDataTosend == ETX)
          ||(ucDataTosend == ESC))
      {
        // É um especial
        /////////////////

        // Eviou um scape char, não deve avançar
        blnReturnValue = 0;

        // Indica que enviou um especial
        m_blnProcessingScapeChar = 1;

        // altera o dado
        ucDataTosend = ESC;
      }
    }
  }

  tx_buffer[m_ucTXBufferCorrentDataPos] = ucDataTosend;
  m_ucTXBufferCorrentDataPos++;

  //Serial2.write(ucDataTosend);

  return blnReturnValue;
}

unsigned char CalculateChecksum(unsigned char* udtpackage, unsigned char ucLen)
{
  // Cria a inicializa a variável de retorno.
  unsigned char ucChecksum = 0;

  // Cria a variável do controle de posição
  unsigned char ucPosition = 0;

  // Prepara a varredura
  while(ucPosition < ucLen)
  {
    //Varre os dados.
    //////////////////

    // Soma  o valor da vez.
    ucChecksum = ucChecksum + *udtpackage;

    // Atualiza os indices
    udtpackage++;
    ucPosition++;

  }

  // Retorna a informação
  return ucChecksum;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

 	    /* USER CODE END StartAdcTask */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  // Fila para mandar os dados do ADC para a UART (Tamanho 1, pois só queremos o dado mais recente)
    energyqueueHandle = xQueueCreate(1, sizeof(thee_phase_energy_data_t));

    // Fila para receber bytes da porta serial
    rxuartqueueHandle = xQueueCreate(64, sizeof(uint8_t));

    // Semáforo para avisar o fim da transmissão TX
    uartBinSemaHandle = xSemaphoreCreateBinary();

    // Fila para o sinal do DMA (tamanho 1, guarda apenas um uint8_t)
    adchalfselectQueueHandle = xQueueCreate(1, sizeof(uint8_t));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of AdcTaask */
  AdcTaaskHandle = osThreadNew(StartAdcTask, NULL, &AdcTaask_attributes);

  /* creation of UartTask */
  UartTaskHandle = osThreadNew(StartUartTask, NULL, &UartTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the analog watchdog
  */
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.HighThreshold = 0;
  AnalogWDGConfig.LowThreshold = 0;
  AnalogWDGConfig.Channel = ADC_CHANNEL_6;
  AnalogWDGConfig.ITMode = DISABLE;
  if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 13020;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        // Libera o semáforo para destravar a StartUartTask
        xSemaphoreGiveFromISR(uartBinSemaHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    // Verifica se a interrupção veio do ADC1
    if(hadc->Instance == ADC1)
    {
        uint8_t side = 1; // Avisa a Task que a Metade 1 está pronta
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        // Envia o número 1 para a fila de dentro de uma interrupção (ISR)
        xQueueSendFromISR(adchalfselectQueueHandle, &side, &xHigherPriorityTaskWoken);

        // Pede ao FreeRTOS para acordar a Task imediatamente se ela for prioritária
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    // Verifica se a interrupção veio do ADC1
    if(hadc->Instance == ADC1)
    {
        uint8_t side = 2; // Avisa a Task que a Metade 2 está pronta
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        // Envia o número 2 para a fila de dentro de uma interrupção (ISR)
        xQueueSendFromISR(adchalfselectQueueHandle, &side, &xHigherPriorityTaskWoken);

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}




void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        // Envia o byte recebido para a fila da UART
        xQueueSendFromISR(rxuartqueueHandle, &rx_buffer, &xHigherPriorityTaskWoken);

        // Arma a interrupção de novo para o próximo byte
        HAL_UART_Receive_IT(&huart2, &rx_buffer, 1);

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

//Funções para empacotar as variaveis de dados da Energya em 1 byte para envio da Uart

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartAdcTask */
/**
  * @brief  Function implementing the AdcTaask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartAdcTask */
void StartAdcTask(void *argument)
{
  /* USER CODE BEGIN 5 */

		 	  	uint32_t accumulated_active_power = 0;

		 	  	float cc_voltageA = 0;
		 	  	float cc_voltageB = 0;
		 	  	float cc_voltageC = 0;
		 	  	float cc_currentA = 0;
		 	  	float cc_currentB = 0;
		 	  	float cc_currentC = 0;
		 	  	uint16_t i = 0;
		 	  	uint16_t j = 0;
		 	  	uint16_t cycle_count = 0;
		 	  	uint8_t sidebuffer_choice = 0;
		 	  	uint64_t accumulated_consumption = 0;
		 	  	thee_phase_energy_data_t EnergyData;
		 	  	/*EnergyData.consumption = 0;
		 	  	EnergyData.TotalM_Voltag = 0;
		 	  	EnergyData.Total_AC_current = 0;
		 	  	EnergyData.Total_PowerAparent = 0;
		 	  	EnergyData.Total_PowerAtiva = 0;
		 	  	EnergyData.Total_PowerFactor = 0;
		 	  	EnergyData.Total_PowerReativa = 0;*/


		 	  	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuffer, F_BUFFER_SIZE*2);
		 	  	HAL_TIM_Base_Start(&htim2);


		 	    /* Infinite loop */
		 	    while(1)
		 	    {
		 	  		xQueueReceive(adchalfselectQueueHandle, &sidebuffer_choice, portMAX_DELAY);

		 	  		cc_voltageA = 0;
		 	  		cc_currentA = 0;
		 	  		//////////////////
		 	  		cc_voltageB = 0;
		 			cc_currentB = 0;
		 			////////////////
		 			cc_voltageC = 0;
		 			cc_currentC = 0;

		 			//Inicializando Variaveis dos dados de Eenergia
		 			EnergyData.phaseB.pot_ativa = 0;
		 			EnergyData.phaseA.pot_aparente = 0;
		 			EnergyData.phaseA.pot_reativa = 0;
		 			EnergyData.phaseA.rms_voltage = 0;
		 			EnergyData.phaseA.rms_current = 0;
		 			EnergyData.phaseA.pf = 0;



		 			EnergyData.phaseB.pot_ativa = 0;
					EnergyData.phaseB.pot_aparente = 0;
					EnergyData.phaseB.pot_reativa = 0;
					EnergyData.phaseB.rms_voltage = 0;
					EnergyData.phaseB.rms_current = 0;
					EnergyData.phaseB.pf = 0;


					EnergyData.phaseC.pot_ativa = 0;
					EnergyData.phaseC.pot_aparente = 0;
					EnergyData.phaseC.pot_reativa = 0;
					EnergyData.phaseC.rms_voltage = 0;
					EnergyData.phaseC.rms_current = 0;
					EnergyData.phaseC.pf = 0;


		 	  		if (sidebuffer_choice == 1){
		 	  			i = 0;
		 	  		}
		 	  		if (sidebuffer_choice == 2){
		 	  			i = H_BUFFER_SIZE;
		 	  		}

		 	  		j = 0;
		 	  		for(uint16_t c = i; c < (H_BUFFER_SIZE + i); c += 3){
		 	  				// Extrai os 16 primeiros bits, referentes ao canal 0 do ADC(V primeiro sinal)
		 	  				adc_voltageA[j] = (float)adcBuffer[c].bits16[0];
		 	  				cc_voltageA += adc_voltageA[j];
		 	  				// Extrai o segundo bloco de 16 bits, que vem do canal 1 do ADC, que é referente a corrente do primeir sinal
		 	  				adc_currentA[j] = (float)adcBuffer[c].bits16[1];
		 	  				cc_currentA += adc_currentA[j];
		 	  				//Repete o ciclo para pegar o 2º sinais:
		 	  				adc_voltageB[j] = (float)adcBuffer[c+1].bits16[0];
		 	  				cc_voltageB += adc_voltageB[j];
		 	  				adc_currentB[j] = (float)adcBuffer[c+1].bits16[1];
		 	  				cc_currentB += adc_currentB[j];
		 	  				//REPETE O CICLO PARA PEGAR O 3º SINAL
		 	  				adc_voltageC[j] = (float)adcBuffer[c+2].bits16[0];
		 					cc_voltageC += adc_voltageC[j];
		 					adc_currentC[j] = (float)adcBuffer[c+2].bits16[1];
		 	  			    cc_currentC += adc_currentC[j];
		 	  				j++;
		 	  		}
		 	  		//Calculando o OffSet da aquisição dos dados do ADC
		 	  		cc_voltageA /= (float)Amostragens;
		 	  		cc_currentA /= (float)Amostragens;
		 	  		cc_voltageB /= (float)Amostragens;
		 			cc_currentB /= (float)Amostragens;
		 			cc_voltageC /= (float)Amostragens;
		 			cc_currentC /= (float)Amostragens;



		 	  		float rms_voltageA = 0.0;
		 	  		float rms_currentA = 0.0;
		 	  		float pot_ativaA = 0.0;
		 	  		///////////////////////
		 	  		float rms_voltageB = 0.0;
		 			float rms_currentB = 0.0;
		 			float pot_ativaB = 0.0;
		 			////////////////////////
		 			float rms_voltageC = 0.0;
		 			float rms_currentC = 0.0;
		 			float pot_ativaC = 0.0;
		 	  		for(uint16_t c = 0; c < Amostragens; c++){
		 	  			// retirando o offSet da aquisição de dados do ADC e somando o quadrado
		 	  			rms_voltageA += (adc_voltageA[c] - cc_voltageA) * (adc_voltageA[c] - cc_voltageA);
		 	  			rms_currentA += (adc_currentA[c] - cc_currentA) * (adc_currentA[c] - cc_currentA);
		 	  			pot_ativaA += (adc_currentA[c] - cc_currentA) * (adc_voltageA[c] - cc_voltageA);
		 	  			///Retirando offSet da aquisição do ADC e somando as POTENCIAS
		 	  			rms_voltageB += (adc_voltageB[c] - cc_voltageB) * (adc_voltageB[c] - cc_voltageB);
		 				rms_currentB += (adc_currentB[c] - cc_currentB) * (adc_currentB[c] - cc_currentB);
		 				pot_ativaB += (adc_currentB[c] - cc_currentB) * (adc_voltageB[c] - cc_voltageB);
		 				///RETIRANDO OFFSET DA AQUISIÇÃO DO ADC E SOMANDO AS POTENCIAS
		 				rms_voltageC += (adc_voltageC[c] - cc_voltageC) * (adc_voltageC[c] - cc_voltageC);
		 				rms_currentC += (adc_currentC[c] - cc_currentC) * (adc_currentC[c] - cc_currentC);
		 				pot_ativaC += (adc_currentC[c] - cc_currentC) * (adc_voltageC[c] - cc_voltageC);


		 	  		}
		 	  		// Terminando o calcul do valor RMS: Fazendo a media e retirando sua raiz dos valores já elevados ao quadrado
		 	  		rms_voltageA /= (float)Amostragens;
		 	  		rms_voltageA = sqrtf(rms_voltageA) * V_GAIN;      // 	Como foi calculado o ganho do ADC?

		 	  		rms_currentA = sqrtf(rms_currentA) * A_GAIN;
		 	  		rms_currentA /= (float)Amostragens;
		 	  		//////
		 	  		rms_voltageB /= (float)Amostragens;
		 	  		rms_voltageB = sqrtf(rms_voltageB) * V_GAIN;     // 	Como foi calculado o ganho do ADC?

		 	  		rms_currentB /= (float)Amostragens;
		 			rms_currentB = sqrtf(rms_currentB) * A_GAIN;
		 			////
		 			rms_voltageC /= (float)Amostragens;
		 			rms_voltageC = sqrtf(rms_voltageC) * V_GAIN;     // 	Como foi calculado o ganho do ADC?

		 			rms_currentC /= (float)Amostragens;
		 			rms_currentC = sqrtf(rms_currentC) * A_GAIN;
		 			//Calculo da potencia Ativa de cada fase e passando para variavel de dados:
		 			pot_ativaA /= (float)Amostragens;
		 			//Passando o valor da potencia ativa para seu modulo e passando o valor limitado do ADC para o valor real
		 			pot_ativaA = fabsf(pot_ativaA) * V_GAIN * A_GAIN;
		 			EnergyData.phaseA.pot_ativa = (uint32_t)(pot_ativaA*100.0);

		 			pot_ativaB /= (float)Amostragens;
		 			pot_ativaB = fabsf(pot_ativaB) * V_GAIN * A_GAIN;
		 			EnergyData.phaseB.pot_ativa = (uint32_t)(pot_ativaB*100.0);

		 			pot_ativaC /= (float)Amostragens;
		 			pot_ativaC = fabsf(pot_ativaC) * V_GAIN * A_GAIN;
		 			EnergyData.phaseC.pot_ativa = (uint32_t)(pot_ativaC*100.0);
		 			//Passando os valores RMS para a variavel de dados
		 			EnergyData.phaseA.rms_voltage = (uint32_t)(rms_voltageA*10.0);
		 			EnergyData.phaseA.rms_current = (uint32_t)(rms_currentA*100.0);

		 			EnergyData.phaseB.rms_voltage = (uint32_t)(rms_voltageB*10.0);
		 			EnergyData.phaseB.rms_current = (uint32_t)(rms_currentB*100.0);

		 			EnergyData.phaseC.rms_voltage = (uint32_t)(rms_voltageC*10.0);
		 			EnergyData.phaseC.rms_current = (uint32_t)(rms_currentC*100.0);



		 			//Validação Primeiro sinal
		 	  		float pot_aparenteA = 0.0;
		 	  		if ((EnergyData.phaseA.rms_voltage * EnergyData.phaseA.rms_current) > 0)
		 	  	    {
		 	  			pot_aparenteA = rms_voltageA * rms_currentA;
		 	  			EnergyData.phaseA.pot_aparente = (uint32_t)(pot_aparenteA * 100.0);
		 	  	    }
		 	  		float pot_reativaA = 0.0;
		 	  		if ((EnergyData.phaseA.pot_aparente > 0) && (EnergyData.phaseA.pot_ativa > 0))
		 	  		{
		 	  			pot_reativaA = sqrtf((pot_aparenteA * pot_aparenteA) - (pot_ativaA * pot_ativaA));
		 	  			EnergyData.phaseA.pot_reativa = (uint32_t)(pot_reativaA * 100.0);
		 	  		}

		 	  		if ((EnergyData.phaseA.pot_ativa > 0) && (EnergyData.phaseA.pot_aparente > 0))
		 	  	    {
		 	  			EnergyData.phaseA.pf = (EnergyData.phaseA.pot_ativa*1000)/EnergyData.phaseA.pot_aparente;
		 	  	    }
		 	  		//////segundo sinal///////////
		 	  		float pot_aparenteB = 0.0;
		 			if ((EnergyData.phaseB.rms_voltage * EnergyData.phaseB.rms_current) > 0)
		 			{
		 				pot_aparenteB= rms_voltageB * rms_currentB;
		 				EnergyData.phaseB.pot_aparente = (uint32_t)(pot_aparenteB * 100.0);
		 			}
		 			float pot_reativaB = 0.0;
		 			if ((EnergyData.phaseB.pot_aparente > 0) && (EnergyData.phaseB.pot_ativa > 0))
		 			{
		 				pot_reativaB = sqrtf((pot_aparenteB * pot_aparenteB) - (pot_ativaB * pot_ativaB));
		 				EnergyData.phaseB.pot_reativa = (uint32_t)(pot_reativaB * 100.0);
		 			}

		 			if ((EnergyData.phaseB.pot_ativa > 0) && (EnergyData.phaseB.pot_aparente > 0))
		 			{
		 				EnergyData.phaseB.pf = (EnergyData.phaseB.pot_ativa*1000)/EnergyData.phaseB.pot_aparente;
		 			} //ESSE IF É A MESMA COISA DO DE CIMA, NÃO DARIA PARA POR ELE DENTRO DO OUTRO IF?


		 			/////Terceira senoide////
		 			float pot_aparenteC = 0.0;
		 			if ((EnergyData.phaseC.rms_voltage * EnergyData.phaseC.rms_current) > 0)
		 			{
		 				pot_aparenteC = rms_voltageC * rms_currentC;
		 				EnergyData.phaseC.pot_aparente = (uint32_t)(pot_aparenteC * 100.0);
		 			}
		 			float pot_reativaC = 0.0;
		 			if ((EnergyData.phaseC.pot_aparente > 0) && (EnergyData.phaseC.pot_ativa > 0))
		 			{
		 				pot_reativaC = sqrtf((pot_aparenteC * pot_aparenteC) - (pot_ativaC * pot_ativaC));
		 				EnergyData.phaseC.pot_reativa = (uint32_t)(pot_reativaC * 100.0);
		 			}

		 			if ((EnergyData.phaseC.pot_ativa > 0) && (EnergyData.phaseC.pot_aparente > 0))
		 			{
		 				EnergyData.phaseC.pf = (EnergyData.phaseC.pot_ativa*1000)/EnergyData.phaseC.pot_aparente;
		 			}
		 	  		accumulated_active_power += ((EnergyData.phaseA.pot_ativa + EnergyData.phaseB.pot_ativa + EnergyData.phaseC.pot_ativa));

		 	  		cycle_count++;
		 	  		//Cria
		 	  		if (cycle_count >= 60)
		 	  		{
		 	  			accumulated_consumption += (uint64_t)(accumulated_active_power/216000);
		 	  			EnergyData.consumption = (accumulated_consumption / 100);  // k\h com uma casa depois da virgula
		 	  			EnergyData.TotalM_Voltag = (EnergyData.phaseA.rms_voltage + EnergyData.phaseB.rms_voltage + EnergyData.phaseC.rms_voltage);
						EnergyData.Total_AC_current = (EnergyData.phaseA.rms_current + EnergyData.phaseB.rms_current + EnergyData.phaseC.rms_current);
						EnergyData.Total_PowerAparent = (EnergyData.phaseA.pot_aparente + EnergyData.phaseB.pot_aparente + EnergyData.phaseC.pot_aparente);
						EnergyData.Total_PowerAtiva = (EnergyData.phaseA.pot_ativa + EnergyData.phaseB.pot_ativa + EnergyData.phaseC.pot_ativa);
						EnergyData.Total_PowerFactor = (EnergyData.phaseA.pf + EnergyData.phaseB.pf + EnergyData.phaseC.pf);
						EnergyData.Total_PowerReativa = (EnergyData.phaseA.pot_reativa + EnergyData.phaseB.pot_reativa + EnergyData.phaseC.pot_reativa);
		 	  			accumulated_active_power = 0;
		 	  			cycle_count = 0;
		 	  		}






		 	  		if (uxQueueMessagesWaiting(energyqueueHandle) == 1) // Se a
		 	  		{
		 	  			xQueueOverwrite(energyqueueHandle, &EnergyData);
		 	  		}
		 	  		else
		 	  		{
		 	  			//alimenta queue
		 	  			xQueueSend(energyqueueHandle, &EnergyData, 0);
		 	  		}


		 	    }


  /* USER CODE END 5 */
}
  /* USER CODE END 5 */


/* USER CODE BEGIN Header_StartUartTask */
/**
* @brief Function implementing the UartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartTask */
void StartUartTask(void *argument)
{
  /* USER CODE BEGIN StartUartTask */
	uint8_t receivedByte;
		uint8_t null = 0;

		HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_buffer, 1);
	  /* Infinite loop */
		while(1)
		{
		  if(m_blnReply == 0)
		  {
			  if((m_udtUartmachineStates == UMS_RECEIVING)){
				// Se houver dados recebidos na fila
				if (xQueueReceive(rxuartqueueHandle, &receivedByte, portMAX_DELAY)) {
					// Processa o byte recebido
					UartMainProcess(receivedByte);
				}
			  }
			  else if ((m_udtUartmachineStates == UMS_PROCESSING_RESPONSE_PACKAGE))
			  {
				  UartMainProcess(null);
			  }
			  else if ((m_udtUartmachineStates == UMS_SENDING_RESPONSE))
			  {
				UartMainProcess(null);
			  }
		  }
		  else
		  {
			  HAL_UART_Transmit_IT(&huart2, (uint8_t *)&tx_buffer, m_ucTXBufferCorrentDataPos);
			  xSemaphoreTake(uartBinSemaHandle, portMAX_DELAY);
			  m_ucTXBufferCorrentDataPos = 0;

			  m_blnReply = 0;
		  }

		}
  /* USER CODE END StartUartTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
