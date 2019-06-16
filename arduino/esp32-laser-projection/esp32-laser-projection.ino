#include "Arduino.h"
#include <driver/dac.h>

/*Connections:
Lolin32 Lite ESP32
*/

#define PIN_LASER 22 //LED_BUILTIN

/* create a hardware timer */
hw_timer_t * timer = NULL;

//Definition of a Laser frame
const int MaxFrameSize = 500;

struct Frame {
  int size;
  int16_t x[MaxFrameSize];
  int16_t y[MaxFrameSize];
  //byte laseronoff[MaxFrameSize];
};

Frame g_origFrame;               //Original frame of a horizontal arrow
Frame g_arrFrames[2];            //ISR shows one frame, the main loop updates the other
volatile Frame* g_ptrCurFrame = &g_origFrame;  //frame that should be drawn by the ISR 
int g_isr_cur_frame_idx = 0; //idx of a point as used in the isr routine
////////////////////////////

//Look-up table of Cos and Sin for speed
int tblCos[360];
int tblSin[360];

void IRAM_ATTR my_timer_isr() {

 if(++g_isr_cur_frame_idx >= g_ptrCurFrame->size) g_isr_cur_frame_idx = 0;
    
 const int16_t x_pos = g_ptrCurFrame->x[g_isr_cur_frame_idx];
 const int16_t y_pos = g_ptrCurFrame->y[g_isr_cur_frame_idx];
 setDAC(x_pos, y_pos);
  
}

void setup() {
  // put your setup code here, to run once:

  //set up pins
  pinMode(PIN_LASER, OUTPUT);
  digitalWrite(PIN_LASER, 1);

  dac_output_enable(DAC_CHANNEL_1);
  dac_output_enable(DAC_CHANNEL_2);
  Serial.begin(9600);

  //set up look-up tables
  setupTrigTable();

  //create a drawing
  setupOrigFrame(g_origFrame);

  /* Use 1st timer of 4 */
  /* 1 tick take 1/(80MHZ/80) = 1us so we set divider 80 and count up */
  timer = timerBegin(0, 80, true);
  /* Attach onTimer function to our timer */
  timerAttachInterrupt(timer, &my_timer_isr, true);
  timerAlarmWrite(timer, 20, true);
  timerAlarmEnable(timer);

  digitalWrite(PIN_LASER, 0); //for now, the laser is always on
}


int g_angle = 0;

int g_curFrameIdx = 0;

void loop() {

  //Use this for testing the corners
  //setDAC(0,0); delay(5000); setDAC(4095, 0); delay(5000); setDAC(0, 4095); delay(5000); setDAC(4095, 4095); delay(5000); return;

  //draw the new frame
  int newFrameIdx = 1 - g_curFrameIdx;
  Frame& newFrame = g_arrFrames[newFrameIdx];
  newFrame.size = g_origFrame.size;
  for(int i=0; i<g_origFrame.size; ++i) {
    rotatePoint(g_origFrame.x[i], g_origFrame.y[i], 
                newFrame.x[i], newFrame.y[i], 
                128, 128, g_angle); yield();
  }

  //switch the frames
  g_curFrameIdx = newFrameIdx;
  g_ptrCurFrame = g_arrFrames + g_curFrameIdx;

  //update the angle
  g_angle += 5;
  if(g_angle >= 360) g_angle -= 360;
  delay(100);
  }

void setupTrigTable() {
  for(int16_t angle=0; angle<360; ++angle) {
    double dAngle = angle * 3.14159 / 180;
    tblCos[angle] = 100*cos(dAngle);
    tblSin[angle] = 100*sin(dAngle);
  }
}

inline 
void rotatePoint(const int16_t x, const int16_t y, int16_t& new_x, int16_t& new_y, const int16_t x0, const int16_t y0, int16_t angle) {
  new_x = x0 + (x-x0)*tblCos[angle]/100 + (y-y0)*tblSin[angle]/100;
  new_y = y0 - (x-x0)*tblSin[angle]/100 + (y-y0)*tblCos[angle]/100;
}


void setupOrigFrame(Frame& curFrame) {

  int counter = 0;
  
  for(int i=0; i<10; ++i) { curFrame.x[counter] = 64; curFrame.y[counter] = 128 + i; ++counter; }
  
  for(int i=0; i<100; ++i) { curFrame.x[counter] = 64+i; curFrame.y[counter] = 128+10; ++counter; }
  
  for(int i=0; i<15; ++i) { curFrame.x[counter] = 64+99-i; curFrame.y[counter] = 128+10+i; ++counter; }
  
  for(int i=0; i<44; ++i) { curFrame.x[counter] = 64+99-14+i; curFrame.y[counter] = 128+10+14-24*i/43; ++counter; }
  
  for(int i=0; i<44; ++i) { curFrame.x[counter] = 64+99+29-i; curFrame.y[counter] = 128-24*i/43; ++counter; }
  
  for(int i=0; i<15; ++i) { curFrame.x[counter] = 64+99-14+i; curFrame.y[counter] = 128-24+i; ++counter; }
  
  for(int i=0; i<100; ++i) { curFrame.x[counter] = 64+99-i; curFrame.y[counter] = 128-10; ++counter; }
  
  for(int i=0; i<10; ++i) { curFrame.x[counter] = 64; curFrame.y[counter] = 128-10 + i; ++counter; }

  curFrame.size = counter;

  Serial.print("curFrame.size ="); Serial.println(curFrame.size);
  if(curFrame.size > MaxFrameSize) {
    Serial.println("curFrame.size > MaxFrameSize"); delay(10000);
  }
  
}


inline void setDAC(const uint16_t x, const uint16_t y) {
      dac_output_voltage(DAC_CHANNEL_1, x);
      dac_output_voltage(DAC_CHANNEL_2, y);
}


