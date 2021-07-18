#include "driver/pcnt.h"                                  // Biblioteca de pulse count
#include <Wire.h>

#define PCNT_FREQ_UNIT      PCNT_UNIT_0                   // Unidade de Pulse Count 0 
#define PCNT_H_LIM_VAL      10000                        // Limite superior de contagem 32767 
#define PCNT_INPUT_SIG_IO   25                             // Pulse Input GPIO 4 

int16_t contador = 0;                                     // Contador de pulsos - valor max 65536
int contadorOverflow;                                     // Contador de overflow do Contador de Pulsos
float frequencia = 0;                                     // Frequencia medida
String unidade;                                           // Unidade de medida da escala
unsigned long tempo;                                      // base de tempo da medida dos pulsos
int prescaler;                                            // Divisor de frequencia do Timer
bool contadorOK = false;

pcnt_isr_handle_t user_isr_handle = NULL;                 // handler da interrupção - não usado
hw_timer_t * timer = NULL;                                // Instancia do timer

//------------------------------------------------------------------------------------
void IRAM_ATTR overflowContador(void *arg)                // Rotina de interrupção de overflow do Contador
{
  contadorOverflow = contadorOverflow + 1;                // soma contador de overflow
  PCNT.int_clr.val = BIT(PCNT_FREQ_UNIT);                 // Limpa flag de overflow
  pcnt_counter_clear(PCNT_FREQ_UNIT);                     // Zera e reseta o Contador de Pulsos
}

//------------------------------------------------------------
void iniciaContadorPulsos ()
{
  pcnt_config_t pcntFreqConfig = { };                        // Instancia do Contador de Pulsos
  pcntFreqConfig.pulse_gpio_num = PCNT_INPUT_SIG_IO;         // Port de entrada dos pulsos  = GPIO 4
  pcntFreqConfig.pos_mode = PCNT_COUNT_INC;                  // Conta na subida do pulso
  pcntFreqConfig.counter_h_lim = PCNT_H_LIM_VAL;             // Limite maximo de contagem
  pcntFreqConfig.unit = PCNT_FREQ_UNIT;                      // Unidade 0 do Contador de Pulsos
  pcntFreqConfig.channel = PCNT_CHANNEL_0;                   // Canal 0 da Unidade 0 Contador de Pulsos
  pcnt_unit_config(&pcntFreqConfig);                         // configura os registradores do Contador de Pulsos

  pcnt_counter_pause(PCNT_FREQ_UNIT);                        // pausa o Contador de Pulsos
  pcnt_counter_clear(PCNT_FREQ_UNIT);                        // Zera e reseta o Contador de Pulsos

  pcnt_event_enable(PCNT_FREQ_UNIT, PCNT_EVT_H_LIM);         // Ativa evento - interrupção no limite maximo de contagem
  pcnt_isr_register(overflowContador, NULL, 0, &user_isr_handle);  // configura registrador da interrupção
  pcnt_intr_enable(PCNT_FREQ_UNIT);                          // habilita as interrupções do Contador de Pulsos

  pcnt_counter_resume(PCNT_FREQ_UNIT);                       // reinicia o Contador de Pulsos
}

//------------------------------------------------------------
void baseTempo()                                            // Rotina de leitura do contador de pulsos (Base de tempo)
{
  pcnt_get_counter_value(PCNT_FREQ_UNIT, &contador);        // obtem o valor do contador de pulsos - valor max 16 bits
  contadorOverflow = 0;                                     // zera o contador de overflow
  pcnt_counter_clear(PCNT_FREQ_UNIT);                       // Zera e reseta o Contador de Pulsos
  contadorOK = true;
}
