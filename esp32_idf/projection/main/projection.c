#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/dac.h"
#include "esp_system.h"
#include "esp_intr_alloc.h"
#include "esp_attr.h"
#include "driver/timer.h"

#define GPIO_PIN_ENABLE	  CONFIG_ENABLE_GPIO_Z
#define DAC_CHANNEL_X     CONFIG_EXAMPLE_DAC_CHANNEL_X
#define DAC_CHANNEL_Y     CONFIG_EXAMPLE_DAC_CHANNEL_Y


//Definition of a Laser frame
#define MaxFrameSize 500
typedef struct Frame {
  int size;
  int16_t x[MaxFrameSize];
  int16_t y[MaxFrameSize];
  //byte laseronoff[MaxFrameSize];
}Frame;

Frame g_origFrame;               //Original frame of a horizontal arrow
Frame g_arrFrames[2];            //ISR shows one frame, the main loop updates the other

volatile Frame* g_ptrCurFrame = &g_origFrame;  //frame that should be drawn by the ISR 
int g_isr_cur_frame_idx = 0; //idx of a point as used in the isr routine
int g_angle = 0;
int g_curFrameIdx = 0;

//Look-up table of Cos and Sin for speed
int tblCos[360];
int tblSin[360];

//void setDAC(const uint16_t x, const uint16_t y);
void setupTrigTable(void);
void rotatePoint(const int16_t x, const int16_t y, int16_t* new_x, int16_t* new_y, const int16_t x0, const int16_t y0, int16_t angle);
void setupOrigFrame(Frame *curFrame);

static intr_handle_t s_timer_handle;

static void timer_isr(void* arg)
{
    int16_t x_pos;
    int16_t y_pos;	 
    TIMERG0.int_clr_timers.t0 = 1;
    TIMERG0.hw_timer[0].config.alarm_en = 1;

    // your code, runs in the interrupt
    if(++g_isr_cur_frame_idx >= g_ptrCurFrame->size) 
    {
	g_isr_cur_frame_idx = 0;
    }    
    x_pos = g_ptrCurFrame->x[g_isr_cur_frame_idx];
    y_pos = g_ptrCurFrame->y[g_isr_cur_frame_idx];	 
    dac_output_voltage(DAC_CHANNEL_X, x_pos);
    dac_output_voltage(DAC_CHANNEL_Y, y_pos);
}

void init_timer(int timer_period_us)
{
    timer_config_t config = {
            .alarm_en = true,
            .counter_en = false,
            .intr_type = TIMER_INTR_LEVEL,
            .counter_dir = TIMER_COUNT_UP,
            .auto_reload = true,
            .divider = 80   /* 1 us per tick */
    };
    
    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, timer_period_us);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, &timer_isr, NULL, 0, &s_timer_handle);

    timer_start(TIMER_GROUP_0, TIMER_0);
}

void app_main()
{
    init_timer(100);
    
    esp_err_t r;

    gpio_num_t dac_gpio_num_x, dac_gpio_num_y;

    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    gpio_pad_select_gpio(GPIO_PIN_ENABLE);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_PIN_ENABLE, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_PIN_ENABLE, 1); // led off

    r = dac_pad_get_io_num( DAC_CHANNEL_X, &dac_gpio_num_x );
    assert( r == ESP_OK );
    r = dac_pad_get_io_num( DAC_CHANNEL_Y, &dac_gpio_num_y );
    assert( r == ESP_OK );

    printf("ENABLE GPIO output %d, DAC channel %d @ GPIO %d, DAC channel %d @ GPIO %d.\n", 
		GPIO_PIN_ENABLE,
                DAC_CHANNEL_X, dac_gpio_num_x,
                DAC_CHANNEL_Y, dac_gpio_num_y );

    dac_output_enable( DAC_CHANNEL_X );
    dac_output_enable( DAC_CHANNEL_Y );

    //set up look-up tables
    setupTrigTable();

    //create a drawing
    setupOrigFrame(&g_origFrame);

    vTaskDelay(20 * portTICK_PERIOD_MS);

    printf("start conversion.\n");
    while(1) {
	  //draw the new frame
	  int newFrameIdx = 1 - g_curFrameIdx;
	  Frame* newFrame = &g_arrFrames[newFrameIdx];
	  newFrame->size = g_origFrame.size;
	  
	  for(int i=0; i<g_origFrame.size; ++i) {
	    rotatePoint(g_origFrame.x[i], g_origFrame.y[i], 
			&newFrame->x[i], &newFrame->y[i], 
			128, 128, g_angle); 
	  }

	  //switch the frames
	  g_curFrameIdx = newFrameIdx;
	  g_ptrCurFrame = g_arrFrames + g_curFrameIdx;

	  //update the angle
	  g_angle += 5;
	  if(g_angle >= 360) g_angle -= 360;
	  
          vTaskDelay(20 / portTICK_PERIOD_MS);

    }
}




void setupTrigTable(void) {
  for(int16_t angle=0; angle<360; ++angle) {
    double dAngle = angle * 3.14159 / 180;
    tblCos[angle] = 100*cos(dAngle);
    tblSin[angle] = 100*sin(dAngle);
  }
}


void rotatePoint(const int16_t x, const int16_t y, int16_t* new_x, int16_t* new_y, const int16_t x0, const int16_t y0, int16_t angle) {
  *new_x = x0 + (x-x0)*tblCos[angle]/100 + (y-y0)*tblSin[angle]/100;
  *new_y = y0 - (x-x0)*tblSin[angle]/100 + (y-y0)*tblCos[angle]/100;
}


void setupOrigFrame(Frame *curFrame) {

  int counter = 0;
  
  for(int i=0; i<10; ++i) { curFrame->x[counter] = 64; curFrame->y[counter] = 128 + i; ++counter; }  
  for(int i=0; i<100; ++i) { curFrame->x[counter] = 64+i; curFrame->y[counter] = 128+10; ++counter; }
  for(int i=0; i<15; ++i) { curFrame->x[counter] = 64+99-i; curFrame->y[counter] = 128+10+i; ++counter; }  
  for(int i=0; i<44; ++i) { curFrame->x[counter] = 64+99-14+i; curFrame->y[counter] = 128+10+14-24*i/43; ++counter; }  
  for(int i=0; i<44; ++i) { curFrame->x[counter] = 64+99+29-i; curFrame->y[counter] = 128-24*i/43; ++counter; }  
  for(int i=0; i<15; ++i) { curFrame->x[counter] = 64+99-14+i; curFrame->y[counter] = 128-24+i; ++counter; }  
  for(int i=0; i<100; ++i) { curFrame->x[counter] = 64+99-i; curFrame->y[counter] = 128-10; ++counter; }  
  for(int i=0; i<10; ++i) { curFrame->x[counter] = 64; curFrame->y[counter] = 128-10 + i; ++counter; }
  curFrame->size = counter;

  printf("curFrame.size = %d\n", curFrame->size);
  if(curFrame->size > MaxFrameSize) {
    printf("curFrame.size > MaxFrameSize\n"); 
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  
}

#if 0
void setDAC(const uint16_t x, const uint16_t y) 
{
      dac_output_voltage(DAC_EXAMPLE_CHANNEL_X, x);
      dac_output_voltage(DAC_EXAMPLE_CHANNEL_Y, y);
}
#endif

