/*!
    \file    main.c
    \brief   led spark with systick

    \version 2024-01-15, V3.2.0, firmware for GD32F4xx
*/

/*
    Copyright (c) 2024, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "gd32f4xx.h"
#include "systick.h"
#include <stdio.h>
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tcp_client.h"
#include "udp_echo.h"
#include "hello_gigadevice.h"

#define START_TASK_PRIO		1
#define START_STK_SIZE 		128 

/* MAC address: MAC_ADDR0:MAC_ADDR1:MAC_ADDR2:MAC_ADDR3:MAC_ADDR4:MAC_ADDR5 */
uint32_t adc_value[2];


TaskHandle_t StartTask_Handler;

void start_task(void *pvParameters);


#define LED1_TASK_PRIO		3

#define LED1_STK_SIZE 		50  

TaskHandle_t LED1Task_Handler;

void LED_Thread1(void *pvParameters);


#define LED2_TASK_PRIO		4

#define LED2_STK_SIZE 		50  

TaskHandle_t LED2Task_Handler;

void LED_Thread2(void *pvParameters);


void rcu_config(void)
{
    /* 初始化GPIOA时钟 */
    rcu_periph_clock_enable(RCU_GPIOA);
    /* 初始化DMA时钟 */
    rcu_periph_clock_enable(RCU_DMA1);
    /* 初始化定时器时钟 */
    rcu_periph_clock_enable(RCU_TIMER1);
    rcu_timer_clock_prescaler_config(RCU_TIMER_PSC_MUL4);
    /* 初始化ADC0时钟 */
    rcu_periph_clock_enable(RCU_ADC0);
    /* 初始化ADC1时钟 */
    //rcu_periph_clock_enable(RCU_ADC1);
    /* 设置ADC分频系数 */
    adc_clock_config(ADC_ADCCK_PCLK2_DIV4);
}

/*初始化ADC A0 PA4*/
void gd_adc_init(void){
    adc_channel_length_config(ADC0,ADC_ROUTINE_CHANNEL,2);
    //adc_channel_length_config(ADC1,ADC_ROUTINE_CHANNEL,2);

    adc_routine_channel_config(ADC0,0,ADC_CHANNEL_4,ADC_SAMPLETIME_144);
    adc_external_trigger_config(ADC0,ADC_ROUTINE_CHANNEL,EXTERNAL_TRIGGER_RISING);

    adc_data_alignment_config(ADC0,ADC_DATAALIGN_RIGHT);
    
    adc_sync_mode_config(ADC_DAUL_ROUTINE_PARALLEL);
    adc_sync_dma_config(ADC_SYNC_DMA_MODE1);
    adc_sync_dma_request_after_last_enable();

    adc_special_function_config(ADC0,ADC_SCAN_MODE,ENABLE);
    adc_enable(ADC0);
    adc_calibration_enable(ADC0);

    
}
/*ADC设置为周期触发，定时器配置*/
void timer_config(void)
{
    timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;

    /* TIMER1 相关配置 */
    timer_initpara.prescaler         = 19999;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 9999;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER1,&timer_initpara);

    /* CH1 配置为 PWM 模式0 */
    timer_ocintpara.ocpolarity  = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.ocnpolarity = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocintpara.outputstate = TIMER_CCX_ENABLE;
    timer_ocintpara.ocidlestate = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
    timer_channel_output_config(TIMER1,TIMER_CH_1,&timer_ocintpara);

    timer_channel_output_pulse_value_config(TIMER1,TIMER_CH_1,3999);
    timer_channel_output_mode_config(TIMER1,TIMER_CH_1,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1,TIMER_CH_1,TIMER_OC_SHADOW_DISABLE);
    
    /* enable */
    timer_enable(TIMER1);
}
/*DMA配置*/
void dma_config(void)
{
    /* ADC的DMA配置 */
    dma_single_data_parameter_struct dma_single_data_parameter;
    
    /* ADC DMA deinit */
    dma_deinit(DMA1,DMA_CH0);
    
    /*DMA单次传输模式 */
    dma_single_data_parameter.periph_addr = (uint32_t)(&ADC_SYNCDATA);
    dma_single_data_parameter.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_single_data_parameter.memory0_addr = (uint32_t)(adc_value);
    dma_single_data_parameter.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_single_data_parameter.periph_memory_width = DMA_PERIPH_WIDTH_32BIT;
    dma_single_data_parameter.circular_mode = DMA_CIRCULAR_MODE_ENABLE;
    dma_single_data_parameter.direction = DMA_PERIPH_TO_MEMORY;
    dma_single_data_parameter.number = 2;
    dma_single_data_parameter.priority = DMA_PRIORITY_HIGH;
    dma_single_data_mode_init(DMA1,DMA_CH0, &dma_single_data_parameter);
  
    dma_channel_subperipheral_select(DMA1,DMA_CH0,DMA_SUBPERI0);

    dma_channel_enable(DMA1,DMA_CH0);
}

/*初始化USART1*/
void gd_usart_init(uint32_t usart_periph){

	//初始化GPIOD时钟
	rcu_periph_clock_enable(RCU_GPIOD);
	//初始化USART时钟
	rcu_periph_clock_enable(RCU_USART1);
	//USART_TX
	gpio_af_set(GPIOD,GPIO_AF_7,GPIO_PIN_5);
	//USART_RX
	gpio_af_set(GPIOD,GPIO_AF_7,GPIO_PIN_6);
	
	gpio_mode_set(GPIOD,GPIO_MODE_AF,GPIO_PUPD_PULLUP,GPIO_PIN_5);
	gpio_output_options_set(GPIOD,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_5);
	
	
	gpio_mode_set(GPIOD,GPIO_MODE_AF,GPIO_PUPD_PULLUP,GPIO_PIN_6);
	gpio_output_options_set(GPIOD,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_6);
	
	usart_deinit(usart_periph);
	usart_baudrate_set(usart_periph,115200);
    usart_word_length_set(usart_periph,USART_WL_8BIT);
    usart_parity_config(usart_periph,USART_PM_NONE);
    usart_stop_bit_set(usart_periph, USART_STB_1BIT);
    usart_hardware_flow_rts_config(usart_periph, USART_RTS_DISABLE); /* 禁用rts */
    usart_hardware_flow_cts_config(usart_periph, USART_CTS_DISABLE); /* 无硬件数据流控制 */
	usart_receive_config(usart_periph,USART_RECEIVE_ENABLE);
	usart_transmit_config(usart_periph,USART_TRANSMIT_ENABLE);
	usart_enable(usart_periph);



}

void lwip_netif_status_callback(struct netif *netif)
{
    if(((netif->flags & NETIF_FLAG_UP) != 0) && (0 != netif->ip_addr.addr)){
        /* initilaize the tcp server: telnet 8000 */
        hello_gigadevice_init();
        /* initilaize the tcp client: echo 10260 */
        tcp_client_init();
        /* initilaize the udp: echo 1025 */
        udp_echo_init();
    }
}

int main(void)
{

    systick_config();
    rcu_periph_clock_enable(RCU_GPIOC);
    gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_0);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
    gd_usart_init(USART1);
    printf("\r\n USART printf example \r\n");
    /*ADC涉及到的外设时钟配置*/
    rcu_config();
    /*配置GPIOA_4为模拟输入模式*/
    gpio_mode_set(GPIOA,GPIO_MODE_ANALOG,GPIO_PUPD_NONE,GPIO_PIN_4);
    /*开启定时器*/
    timer_config();
    /*开启ADC用的DMA*/
    dma_config();
    /*开启ADC*/
    gd_adc_init();
    printf(" the data adc_value[0] is %08X \r\n",adc_value[0]);
		xTaskCreate((TaskFunction_t )start_task,           
								(const char*    )"start_task",        
								(uint16_t       )START_STK_SIZE,       
								(void*          )NULL,                  
								(UBaseType_t    )START_TASK_PRIO,      
								(TaskHandle_t*  )&StartTask_Handler);  
								
		vTaskStartScheduler();         
    while(1){
        
    }
}

void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();          

    xTaskCreate((TaskFunction_t )LED_Thread1,     	
                (const char*    )"led1_task",   	
                (uint16_t       )LED1_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )LED1_TASK_PRIO,	
                (TaskHandle_t*  )&LED1Task_Handler);   


	    xTaskCreate((TaskFunction_t )LED_Thread2,     	
                (const char*    )"led2_task",   	
                (uint16_t       )LED2_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )LED2_TASK_PRIO,	
                (TaskHandle_t*  )&LED2Task_Handler);   
													
							
    vTaskDelete(StartTask_Handler); 
    taskEXIT_CRITICAL();          

}


void LED_Thread1(void *pvParameters)
{
    while(1)
    {
      gpio_bit_reset(GPIOC, GPIO_PIN_0);
			vTaskDelay(10);
			gpio_bit_set(GPIOC, GPIO_PIN_0);
			vTaskDelay(10);
    }
} 


void LED_Thread2(void *pvParameters)
{
    while(1)
    {
			gpio_bit_set(GPIOC, GPIO_PIN_0);
      vTaskDelay(30);
    }
}  



/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(USART1, (uint8_t)ch);
    while(RESET == usart_flag_get(USART1, USART_FLAG_TBE));
    return ch;
}

