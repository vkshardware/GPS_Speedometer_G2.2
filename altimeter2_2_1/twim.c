//***************************************************************************
//
//  Author(s)...: Павел Бобков  http://ChipEnable.Ru   
//
//  Target(s)...: mega16
//
//  Compiler....: IAR
//
//  Description.: Драйвер ведущего TWI устройства. 
//                Код основан на Atmel`овских доках - AVR315.
//
//  Data........: 13.11.13
//
//***************************************************************************
#include "twim.h"

#define TWSR_MASK     0xFC  

volatile static uint8_t twiBuf[TWI_BUFFER_SIZE];
volatile static uint8_t twiState = TWI_NO_STATE;      
volatile static uint8_t twiMsgSize;       

/*предделители для установки скорости обмена twi модуля*/
uint8_t pre[4] = {2, 8, 32, 128};

/****************************************************************************
 Инициализация и установка частоты SCL сигнала
****************************************************************************/
uint8_t TWI_MasterInit(uint16_t fr)
{
  uint8_t i;
  uint16_t twbrValue;
  
  for(i = 0; i<4; i++){
    twbrValue = ((((F_CPU)/1000UL)/fr)-16)/pre[i];
    if ((twbrValue > 0)&& (twbrValue < 256)){
       TWBR = (uint8_t)twbrValue;
       TWSR = i;
       TWDR = 0xFF;
       TWCR = (1<<TWEN);
       return TWI_SUCCESS;
    }
  }
  return 0;  
}    

/****************************************************************************
 Проверка - не занят ли TWI модуль. Используется внутри модуля
****************************************************************************/
static uint8_t TWI_TransceiverBusy(void)
{
  return (TWCR & (1<<TWIE));                 
}

/****************************************************************************
 Взять статус TWI модуля
****************************************************************************/
uint8_t TWI_GetState(void)
{
  while (TWI_TransceiverBusy());             
  return twiState;                        
}

/****************************************************************************
 Передать сообщение msg из msgSize байтов на TWI шину
****************************************************************************/
void TWI_SendData(uint8_t *msg, uint8_t msgSize)
{
  uint8_t i;

  while(TWI_TransceiverBusy());   //ждем, когда TWI модуль освободится             

  twiMsgSize = msgSize;           //сохряняем кол. байт для передачи             
  twiBuf[0]  = msg[0];            //и первый байт сообщения 
  
  if (!(msg[0] & (TRUE<<TWI_READ_BIT))){   //если первый байт типа SLA+W
    for (i = 1; i < msgSize; i++){         //то сохряняем остальную часть сообщения
      twiBuf[i] = msg[i];
    }
  }
                       
  twiState = TWI_NO_STATE ;
  TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWSTA); //разрешаем прерывание и формируем состояние старт                            
}

/****************************************************************************
 Переписать полученные данные в буфер msg в количестве msgSize байт. 
****************************************************************************/
uint8_t TWI_GetData(uint8_t *msg, uint8_t msgSize)
{
  uint8_t i;

  while(TWI_TransceiverBusy());    //ждем, когда TWI модуль освободится 

  if(twiState == TWI_SUCCESS){     //если сообщение успешно принято,                         
    for(i = 0; i < msgSize; i++){  //то переписываем его из внутреннего буфера в переданный
      msg[i] = twiBuf[i];
    }
  }
  
  return twiState;                                   
}

/****************************************************************************
 Обработчик прерывания TWI модуля
****************************************************************************/
void TWI_Process()
{
  static uint8_t ptr;
  uint8_t stat = TWSR & TWSR_MASK;
  
  switch (stat){
    
    case TWI_START:                   // состояние START сформировано 
    case TWI_REP_START:               // состояние повторный START сформировано        
       ptr = 0;      

    case TWI_MTX_ADR_ACK:             // был передан пакет SLA+W и получено подтверждение
    case TWI_MTX_DATA_ACK:            // был передан байт данных и получено подтверждение  
       if (ptr < twiMsgSize){
          TWDR = twiBuf[ptr];                    //загружаем в регистр данных следующий байт
          TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT); //сбрасываем флаг TWINT    
          ptr++;
       }
       else{
          twiState = TWI_SUCCESS;  
          TWCR = (1<<TWEN)|(1<<TWINT)|(1<<TWSTO)|(0<<TWIE); //формируем состояние СТОП, сбрасываем флаг, запрещаем прерывания
       }
       break;
     
    case TWI_MRX_DATA_ACK:          //байт данных принят и передано подтверждение  
       twiBuf[ptr] = TWDR;
       ptr++;
    
    case TWI_MRX_ADR_ACK:           //был передан пакет SLA+R и получено подтвеждение  
      if (ptr < (twiMsgSize-1)){
        TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA);  //если это не предпоследний принятый байт, формируем подтверждение                             
      }
      else {
        TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT);            //если приняли предпоследний байт, подтверждение не формируем
      }    
      break; 
      
    case TWI_MRX_DATA_NACK:       //был принят байт данных без подтверждения      
      twiBuf[ptr] = TWDR;
      twiState = TWI_SUCCESS;  
      TWCR = (1<<TWEN)|(1<<TWINT)|(1<<TWSTO); //формируем состояние стоп
      break; 
     
    case TWI_ARB_LOST:          //был потерян приоритет 
      TWCR = (1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWSTA); // сбрасываем флаг TWINT, формируем повторный СТАРТ
      break;
      
    case TWI_MTX_ADR_NACK:      // был передан пает SLA+W и не получено подтверждение
    case TWI_MRX_ADR_NACK:      // был передан пакет SLA+R и не получено подтверждение    
    case TWI_MTX_DATA_NACK:     // был передан байт данных и не получено подтверждение
    case TWI_BUS_ERROR:         // ошибка на шине из-за некоректных состояний СТАРТ или СТОП
    default:     
      twiState = stat;                                                                                    
      TWCR = (1<<TWEN)|(0<<TWIE)|(0<<TWINT)|(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|(0<<TWWC); //запретить прерывание                              
  }
}
