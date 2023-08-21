#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#include <sys/__assert.h>
#include <string.h>
#include <timing/timing.h>
#include <stdlib.h>
#include <stdio.h>
#include <drivers/gpio.h>
#include <math.h>
#include <drivers/adc.h>

#define SLEEP_TIME_MS   10

/* Size of stack area used by each thread (can be thread specific, if necessary)*/
#define STACK_SIZE 1024

/* Thread scheduling priority */
#define thread_A_prio 2
#define thread_B_prio 2
#define thread_C_prio 2
#define thread_D_prio 2


/* Therad periodicity (in ms)*/
#define thread_A_period 1000


#define LED0_NODE DT_NODELABEL(led0)
#define LED1_NODE DT_NODELABEL(led1)
#define LED2_NODE DT_NODELABEL(led2)
#define LED3_NODE DT_NODELABEL(led3)
#define SW0_NODE DT_NODELABEL(button0)




#define ADC_NODE DT_NODELABEL(adc)  


#include <hal/nrf_saadc.h>
#define ADC_RESOLUTION 10
#define ADC_GAIN ADC_GAIN_1_4
#define ADC_REFERENCE ADC_REF_VDD_1_4
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40)
#define ADC_CHANNEL_ID 1  

/* This is the actual nRF ANx input to use. Note that a channel can be assigned to any ANx. In fact a channel can */
/*    be assigned to two ANx, when differential reading is set (one ANx for the positive signal and the other one for the negative signal) */  
/* Note also that the configuration of differnt channels is completely independent (gain, resolution, ref voltage, ...) */
#define ADC_CHANNEL_INPUT NRF_SAADC_INPUT_AIN1 

#define BUFFER_SIZE 1

#define ADC_NODE DT_NODELABEL(adc)  
const struct device *adc_dev = NULL;

/* Other defines */
#define TIMER_INTERVAL_MSEC 1000 /* Interval between ADC samples */

/* ADC channel configuration */
static const struct adc_channel_cfg my_channel_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id = ADC_CHANNEL_ID,
	.input_positive = ADC_CHANNEL_INPUT
};

/* Global vars */
struct k_timer my_timer;
static uint16_t adc_sample_buffer[BUFFER_SIZE];






static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);
static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(LED3_NODE, gpios);
static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET(SW0_NODE, gpios);

/* Create thread stack space */
K_THREAD_STACK_DEFINE(thread_A_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_B_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_C_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_D_stack, STACK_SIZE);

  
/* Create variables for thread data */
struct k_thread thread_A_data;
struct k_thread thread_B_data;
struct k_thread thread_C_data;
struct k_thread thread_D_data;


/* Create task IDs */
k_tid_t thread_A_tid;
k_tid_t thread_B_tid;
k_tid_t thread_C_tid;
k_tid_t thread_D_tid;




int valor_lido =0;
int media = 0;

/* Semaphores for task synch */
struct k_sem sem_ab;
struct k_sem sem_bc;
// struct k_sem sem_;

/* Thread code prototypes */
void thread_A_code(void *argA, void *argB, void *argC);
void thread_B_code(void *argA, void *argB, void *argC);
void thread_C_code(void *argA, void *argB, void *argC);
void thread_D_code(void *argA, void *argB, void *argC);





static int adc_sample(void)
{
	int ret;
	const struct adc_sequence sequence = {
		.channels = BIT(ADC_CHANNEL_ID),
		.buffer = adc_sample_buffer,
		.buffer_size = sizeof(adc_sample_buffer),
		.resolution = ADC_RESOLUTION,
	};

	if (adc_dev == NULL) {
            printk("adc_sample(): error, must bind to adc first \n\r");
            return -1;
	}

	ret = adc_read(adc_dev, &sequence);
	if (ret) {
            printk("adc_read() failed with code %d\n", ret);
	}	

	return ret;
}


static struct gpio_callback button_cb_data;

static gpio_callback_handler_t button_pressed(struct device *dev, struct gpio_callback *cb, uint32_t pins){
   
    
    k_thread_suspend(thread_A_tid);
    k_thread_suspend(thread_B_tid);
    k_thread_suspend(thread_C_tid);
    int ret =0;
    ret = gpio_pin_set_dt(&led0,0);
    if (ret < 0) {
        return;
    }
    ret = gpio_pin_set_dt(&led1,0);
    if (ret < 0) {
        return;
    }
    ret = gpio_pin_set_dt(&led2,0);
    if (ret < 0) {
        return;
    }
        ret = gpio_pin_set_dt(&led3,0);
    if (ret < 0) {
        return;
    }
    k_thread_resume(thread_D_tid);
    
 
   
}

/* Main function */
void main(void) {
    
   
    /* Create and init semaphores */
    k_sem_init(&sem_ab, 0, 1);
    k_sem_init(&sem_bc, 0, 1);
    int ret;
        /* Check if device is ready */
    if (!device_is_ready(led0.port)) {
        return;
    }

    /* Configure the GPIO pin for output */
    
    ret = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        printk("Error: gpio_pin_configure_dt failed for led0, error:%d", ret);
        return;
    }


    if (!device_is_ready(led1.port)) {
        return;
    }

    
    ret = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        printk("Error: gpio_pin_configure_dt failed for led1, error:%d", ret);
        return;
    }

        if (!device_is_ready(led2.port)) {
        return;
    }

    /* Configure the GPIO pin for output */
    

    ret = gpio_pin_configure_dt(&led2, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        printk("Error: gpio_pin_configure_dt failed for led1, error:%d", ret);
        return;
    }

        if (!device_is_ready(led3.port)) {
        return;
    }

    /* Configure the GPIO pin for output */
    

    ret = gpio_pin_configure_dt(&led3, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        printk("Error: gpio_pin_configure_dt failed for led1, error:%d", ret);
        return;
    }

    if (!device_is_ready(button0.port)) {
    printk("Error: button device %s is not ready\n", button0.port->name);
    return;
	}

    ret = gpio_pin_configure_dt(&button0, GPIO_INPUT );
	if (ret < 0) {
		printk("Error: gpio_pin_configure_dt failed for button, error:%d", ret);
		return;
	}

	/* Configure the interrupt on the button's pin */
	ret = gpio_pin_interrupt_configure_dt(&button0, GPIO_INT_EDGE_RISING );
	if (ret < 0) {
		printk("Error: gpio_pin_interrupt_configure_dt failed for button, error:%d", ret);
		return;
	}

    adc_dev = device_get_binding(DT_LABEL(ADC_NODE));
	if (!adc_dev) {
        printk("ADC device_get_binding() failed\n");
    } 
    int err = adc_channel_setup(adc_dev, &my_channel_cfg);
    if (err) {
        printk("adc_channel_setup() failed with error code %d\n", err);
    }
    NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;


    /* Create tasks */
    thread_A_tid = k_thread_create(&thread_A_data, thread_A_stack,
        K_THREAD_STACK_SIZEOF(thread_A_stack), thread_A_code,
        NULL, NULL, NULL, thread_A_prio, 0, K_NO_WAIT);

    thread_B_tid = k_thread_create(&thread_B_data, thread_B_stack,
        K_THREAD_STACK_SIZEOF(thread_B_stack), thread_B_code,
        NULL, NULL, NULL, thread_B_prio, 0, K_NO_WAIT);

    thread_C_tid = k_thread_create(&thread_C_data, thread_C_stack,
        K_THREAD_STACK_SIZEOF(thread_C_stack), thread_C_code,
        NULL, NULL, NULL, thread_C_prio, 0, K_NO_WAIT);
    
    thread_D_tid = k_thread_create(&thread_D_data, thread_D_stack,
        K_THREAD_STACK_SIZEOF(thread_D_stack), thread_D_code,
        NULL, NULL, NULL, thread_D_prio, 0, K_NO_WAIT);
   
    
       
    gpio_init_callback(&button_cb_data, button_pressed, BIT(button0.pin));

    // attach callback function to button interrupt
    gpio_add_callback(button0.port, &button_cb_data);
    
    

   
    
    
    return;

} 


void thread_A_code(void *argA , void *argB, void *argC)
{
   

 
    while(1) {
        
             
        printk("\nThread A instance released at time: %lld (ms). \n", k_uptime_get());  
        
        
       
        int err;
       
        err =adc_sample();
        if(err) {
            printk("adc_sample() failed with error code %d\n\r",err);
        }
        else {
            if(adc_sample_buffer[0] > 1023) {
                printk("adc reading out of range\n\r");
            }
            else {
                /* ADC is set to use gain of 1/4 and reference VDD/4, so input range is 0...VDD (3 V), with 10 bit resolution */
                printk("adc reading: raw:%4u / %4u mV: \n\r",adc_sample_buffer[0],(uint16_t)(1000*adc_sample_buffer[0]*((float)3/1023)));
            }
            valor_lido = (uint16_t)(1000*adc_sample_buffer[0]*((float)3/1023))/100;
        }


        k_sem_give(&sem_ab);

        k_msleep(1000);
        
    }

}

void thread_B_code(void *argA , void *argB, void *argC)
{
    
    
    int numeros[10];
    int pos;
    int aux=0;
   
    while(1) {
        k_sem_take(&sem_ab,  K_FOREVER);
      
        printk("\nThread B instance released at time: %lld (ms). \n", k_uptime_get());  

        printk("Valor lido =  %d\n",valor_lido);

        if (aux > 9){
                    
            for(int i=0;i<9;i++){
                numeros[i] = numeros[i+1];
            }
            numeros[9] = valor_lido;
        }
        else{
        
            numeros[pos] = valor_lido;
            pos++;
        }
        
        if (aux >= 9 && pos >= 9){
            media =0;
            for (int i=0;i<=9;i++){
                
                media = media + numeros[i];
            }
            
            media = media/10;
            printk("mediaaaaaaaa=  %d\n",media);
           
        }
        
        k_sem_give(&sem_bc);

       
        printk("AUXXXXXXXX=  %d\n",aux);
    
        aux++;
        
        
        
        
  }
}

void thread_C_code(void *argA , void *argB, void *argC)
{
    

    
    while(1) {
        k_sem_take(&sem_bc, K_FOREVER);
         
          
        printk("\nThread C instance released at time: %lld (ms). \n", k_uptime_get());  
        printk("media =  %d\n",media);
        int ret;


        if (media>= 30){
            
            ret = gpio_pin_set_dt(&led0,1);
            if (ret < 0) {
                return;
            }
            ret = gpio_pin_set_dt(&led1,0);
            if (ret < 0) {
                return;
            }
            ret = gpio_pin_set_dt(&led2,0);
            if (ret < 0) {
                return;
            }
             ret = gpio_pin_set_dt(&led3,0);
            if (ret < 0) {
                return;
            }
            
        }
        else if (media >=20 && media <30){
            ret = gpio_pin_set_dt(&led0,1);
            if (ret < 0) {
                return;
            }
            ret = gpio_pin_set_dt(&led1,1);
            if (ret < 0) {
                return;
            }
            ret = gpio_pin_set_dt(&led2,0);
            if (ret < 0) {
                return;
            }
             ret = gpio_pin_set_dt(&led3,0);
            if (ret < 0) {
                return;
            }
        }
        else if(media < 20 && media >= 10){
            ret = gpio_pin_set_dt(&led0,1);
            if (ret < 0) {
                return;
            }
            ret = gpio_pin_set_dt(&led1,1);
            if (ret < 0) {
                return;
            }
            ret = gpio_pin_set_dt(&led2,1);
            if (ret < 0) {
                return;
            }
             ret = gpio_pin_set_dt(&led3,0);
            if (ret < 0) {
                return;
            }
        }
        else if(media >= 0 && media < 10){
            ret = gpio_pin_set_dt(&led0,1);
            if (ret < 0) {
                return;
            }
            ret = gpio_pin_set_dt(&led1,1);
            if (ret < 0) {
                return;
            }
            ret = gpio_pin_set_dt(&led2,1);
            if (ret < 0) {
                return;
            }
             ret = gpio_pin_set_dt(&led3,1);
            if (ret < 0) {
                return;
            }
        }

    }
}
 

void thread_D_code(void *argA , void *argB, void *argC){
   
    while(1){
        k_thread_suspend(thread_D_tid);
        int64_t release = k_uptime_get();
          
        printk("\nThread D instance released at time: %lld (ms). \n", k_uptime_get());  

        while(k_uptime_get() - release < 5000 ){
            int ret;
            ret = gpio_pin_toggle_dt(&led0);
            if (ret < 0) {
                return;
            }
            ret = gpio_pin_toggle_dt(&led1);
            if (ret < 0) {
                return;
            }
            ret = gpio_pin_toggle_dt(&led2);
            if (ret < 0) {
                return;
            }
            ret = gpio_pin_toggle_dt(&led3);
            if (ret < 0) {
                return;
            }
            k_msleep(100);  
        }
        k_thread_resume(thread_A_tid);
        k_thread_resume(thread_B_tid);
        k_thread_resume(thread_C_tid);
        
        
    }
    
}
 


