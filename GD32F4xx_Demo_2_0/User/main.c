/*!
    \file    main.c
    \brief   adc in freertos

    \version 2024-10-24, V1.1.1, firmware for GD32F4xx
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
#include "gd32f4xx_enet_eval.h"
#include "gd32f4xx_dp83640.h"
#include "netconf.h"

#define START_TASK_PRIO		1
#define START_STK_SIZE 		128 

/* MAC address: MAC_ADDR0:MAC_ADDR1:MAC_ADDR2:MAC_ADDR3:MAC_ADDR4:MAC_ADDR5 */
uint16_t adc_value;
float temperature;
float vref_value;
float battery_value;
static __IO uint32_t enet_init_status = 0;

TaskHandle_t StartTask_Handler;

void start_task(void *pvParameters);


#define LED1_TASK_PRIO		5

#define LED1_STK_SIZE 		128  

TaskHandle_t LED1Task_Handler;

void LED_Thread1(void *pvParameters);


#define LED2_TASK_PRIO		5

#define LED2_STK_SIZE 		128  

TaskHandle_t LED2Task_Handler;

void LED_Thread3(void *pvParameters);

#define LED3_TASK_PRIO		5

#define LED3_STK_SIZE 		128  

TaskHandle_t LED3Task_Handler;

void LED_Thread2(void *pvParameters);
// DMA中断服务函数
void DMA1_Channel0_IRQHandler(void) 
{
    if(dma_flag_get(DMA1, DMA_CH0, DMA_FLAG_FTF) != RESET) {
        // 清除传输完成标志
        dma_flag_clear(DMA1, DMA_CH0, DMA_FLAG_FTF);

        // 处理 adc_value，例如将其传递给其他任务或进一步处理
       gpio_bit_write(GPIOC, GPIO_PIN_0, (bit_status)(1 - gpio_input_bit_get(GPIOC, GPIO_PIN_0)));
        
        // 在此可以增加标志位，通知其他任务数据已更新
    }
}

// 定时器中断服务函数
void TIMER1_UP_TIMER10_IRQHandler(void)
{
    if (timer_flag_get(TIMER1, TIMER_FLAG_UP) != RESET) {
        // 清除定时器更新中断标志
        timer_flag_clear(TIMER1, TIMER_FLAG_UP);

        // 在此处执行定时触发的操作，例如触发ADC转换或通知任务
        //printf("Timer Interrupt Triggered\r\n");
    }
}

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

    /* 初始化ADC1时钟 */
    //rcu_periph_clock_enable(RCU_ADC1);
    /* 设置ADC分频系数 */
    adc_clock_config(ADC_ADCCK_PCLK2_DIV4);

}


/*初始化ADC0_CH4 PA4*/
void gd_adc_init(void){
	
		rcu_periph_clock_enable(RCU_ADC0);
		
	  adc_clock_config(ADC_ADCCK_PCLK2_DIV8);
    adc_channel_length_config(ADC0,ADC_ROUTINE_CHANNEL,1);;
   
    adc_routine_channel_config(ADC0,0,ADC_CHANNEL_4,ADC_SAMPLETIME_56);
    adc_external_trigger_config(ADC0,ADC_ROUTINE_CHANNEL,EXTERNAL_TRIGGER_RISING);
		
		adc_external_trigger_source_config(ADC0,ADC_ROUTINE_CHANNEL,ADC_EXTTRIG_ROUTINE_T1_CH3);
		
		adc_resolution_config(ADC0,ADC_RESOLUTION_12B);

    adc_data_alignment_config(ADC0,ADC_DATAALIGN_RIGHT);
	
		adc_special_function_config(ADC0,ADC_CONTINUOUS_MODE,DISABLE);
    adc_dma_mode_enable(ADC0);
    adc_dma_request_after_last_enable(ADC0);
//    adc_sync_dma_config(ADC_SYNC_DMA_MODE1);
//    adc_sync_dma_request_after_last_enable();
		
//		adc_interrupt_enable(ADC0, ADC_INT_EOC);  // 开启ADC转换完成中断
//    nvic_irq_enable(ADC_IRQn, 0, 0);          // 使能ADC中断
		
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
  	/* 预分频值设置为199，主频200MHz / (199 + 1) = 1MHz */
    timer_initpara.prescaler         = 199;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
		/*计数周期设置为9，1MHz / (9 + 1) = 100kHz*/
    timer_initpara.period            = 9;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER1,&timer_initpara);
	
		timer_master_output_trigger_source_select(TIMER1, TIMER_TRI_OUT_SRC_O1CPRE );
    /* CH1 配置为 PWM 模式0 */
    timer_ocintpara.ocpolarity  = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.ocnpolarity = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocintpara.outputstate = TIMER_CCX_ENABLE;
    timer_ocintpara.ocidlestate = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
    timer_channel_output_config(TIMER1,TIMER_CH_3,&timer_ocintpara);

    timer_channel_output_pulse_value_config(TIMER1,TIMER_CH_3,4);
    timer_channel_output_mode_config(TIMER1,TIMER_CH_3,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER1,TIMER_CH_3,TIMER_OC_SHADOW_DISABLE);
    
    /* enable */
    timer_enable(TIMER1);
}
/*DMA配置，DMA1通道0可以用于ADC0*/
void dma_config(void)
{
    /* ADC的DMA配置 */
    dma_single_data_parameter_struct dma_single_data_parameter;
    
    /* ADC DMA deinit */
    dma_deinit(DMA1,DMA_CH0);
    
    /*DMA单次传输模式 */
    dma_single_data_parameter.periph_addr = (uint32_t)(&ADC_RDATA(ADC0) );
    dma_single_data_parameter.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_single_data_parameter.memory0_addr = (uint32_t)(&adc_value);
    dma_single_data_parameter.memory_inc = DMA_MEMORY_INCREASE_DISABLE;
    dma_single_data_parameter.periph_memory_width = DMA_PERIPH_WIDTH_16BIT;
    dma_single_data_parameter.circular_mode = DMA_CIRCULAR_MODE_ENABLE;
    dma_single_data_parameter.direction = DMA_PERIPH_TO_MEMORY;
    dma_single_data_parameter.number = 1;
    dma_single_data_parameter.priority = DMA_PRIORITY_HIGH;
    dma_single_data_mode_init(DMA1,DMA_CH0, &dma_single_data_parameter);
		
		dma_interrupt_enable(DMA1, DMA_CH0, DMA_INT_FTF);
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

void enable_interrupts(void)
{
    // 使能DMA1通道0中断
    nvic_irq_enable(DMA1_Channel0_IRQn, 6, 0);
    
    // 使能TIMER1中断
    nvic_irq_enable(TIMER1_UP_TIMER10_IRQHandler, 6, 1);
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
		adc_software_trigger_enable(ADC0, ADC_ROUTINE_CHANNEL);
    /*初始化mac和phy*/
    enet_system_setup();
		/*初始化lwip*/
		lwip_stack_init();
		
		enable_interrupts();
    
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
								
		xTaskCreate((TaskFunction_t )LED_Thread3,     	
                (const char*    )"led3_task",   	
                (uint16_t       )LED3_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )LED3_TASK_PRIO,	
                (TaskHandle_t*  )&LED3Task_Handler); 

         
																			
    vTaskDelete(StartTask_Handler); 
    taskEXIT_CRITICAL();          

}


void LED_Thread1(void *pvParameters)
{
    while(1)
    {
			
			printf("task1 running\r\n");
      gpio_bit_reset(GPIOC, GPIO_PIN_0);
			vTaskDelay(10);

			
    }
} 


void LED_Thread2(void *pvParameters)
{
    while(1)
    {
			printf("task2 running\r\n");
			gpio_bit_set(GPIOC,GPIO_PIN_0);
			vTaskDelay(10);

    }
}  

void LED_Thread3(void *pvParameters)
{
    while(1)
    {
			printf("task3 running\r\n");
			gpio_bit_set(GPIOC,GPIO_PIN_0);
			vTaskDelay(10);

    }
}  


/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(USART1, (uint8_t)ch);
    while(RESET == usart_flag_get(USART1, USART_FLAG_TBE));
    return ch;
}

