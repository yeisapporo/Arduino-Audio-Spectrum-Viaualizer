
/*
ezMTS(Easy Multitasking System) is a library for Arduino with ATMEGA 
microcontrollers. This provides simple multitasking-like system to your 
Arduino. You can use multiple loop()-like functions on your Arduino 
sketch. This is not an operating system but an extension of an interrupt 
handler for Timer2. You should not use tone() in your sketch when using 
this library.

Copyright (c) 2021 Kazuteru Yamada(yeisapporo).  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _HPP_EZMTS_HPP_
#define _HPP_EZMTS_HPP_

#include <Arduino.h>

// specify the timer module that ezMTS occupies. (N) : TimerN(0, 1 or 2)
// when using timer0 all Arduino time-related functions(such as delay())
// may not work correctly. when using timer2 tone() must not be used. 
#ifndef EZMTS_USE_TIMER
//#define EZMTS_USE_TIMER (0)
//#define EZMTS_USE_TIMER (1)
#define EZMTS_USE_TIMER (2) // default
#endif

// specify the unit of timer ivterval value.
#define EZMTS_MILLISEC  (0) // default
#define EZMTS_MICROSEC  (1)

#define EZMTS_TASK_UNUSED   (0)
#define EZMTS_TASK_STOPPED  (1)
#define EZMTS_TASK_RUNNING  (2)

// specify when FIRST executes the callback function registered by .start().
#define EZMTS_TIMEDOUT  (0) // default
#define EZMTS_AT_ONCE   (1)

// task handling information
class taskInfo {
    public:
    int _task_id;
    int _task_state;
    long _timeout_val;
    long _time_rest;
    int (*_cb_func)(void *);
};

int g_task_num = 0;
taskInfo *g_taskInfo = NULL;

class ezMTS {
    private:
    unsigned char _ocrna = 249; // OCRnA for millisec.
    float _k = 1.0;             // coefficient for millisec.

    public:
    ezMTS(int task_num, int unit = EZMTS_MILLISEC) {
        noInterrupts();
        // keep the number of tasks available.
        g_task_num = task_num;
        // prepare OCRnA and the coefficient according to the unit specified.
        if(unit != EZMTS_MILLISEC) {
            // for microsec.
            _ocrna = 9;
            _k = 0.02538071066;
        }
        // get task management area.
        g_taskInfo = new taskInfo[task_num];
        // initialize task management variables.
        for(int i = 0; i < g_task_num; i++) {
            g_taskInfo[i]._task_id = -1;
            g_taskInfo[i]._task_state = EZMTS_TASK_UNUSED;
            g_taskInfo[i]._timeout_val = 0;
            g_taskInfo[i]._time_rest = 0;
            g_taskInfo[i]._cb_func = NULL;
        }
// settings for ATMEGA microcontroller timer interruption.
// specify the timer module that ezMTS uses.
#if (EZMTS_USE_TIMER == 0)
        // ezMTS occupies Timer0.
        TCCR0A = TCCR0B = 0;
        // set mode to CTC.
        TCCR0A |= (1 << WGM01);
        // set top to OCRA0 and prescaler to clk/64.
        TCCR0B |= (1 << WGM02) | (1 << CS01) | (1 << CS00);
        // set top value.
        OCR0A = _ocrna;
        TIMSK0 |= (1 << OCIE0A);
#elif (EZMTS_USE_TIMER == 1)
        // ezMTS occupies Timer1.
        TCCR1A = TCCR1B = 0;
        // set mode to CTC.
        //TCCR1A |= (0 << WGM10) | (0 << WGM11);
        // set top to OCR1A and prescaler to clk/64
        TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10);
        // set top value.
        OCR1A = _ocrna;
        TIMSK1 |= (1 << OCIE1A);

#elif (EZMTS_USE_TIMER == 2)
        // ezMTS occupies Timer2.
        TCCR2A = TCCR2B = 0;
        // set mode to CTC.
        TCCR2A |= (1 << WGM21);
        // set prescaler to clk/64.
        TCCR2B |= (1 << CS22);
        // set to value.
        OCR2A = _ocrna;
        TIMSK2 |= (1 << OCIE2A);
#endif
        interrupts();
    }
    int create(int (*cb_func)(void *)) {
        int ret = -1;
        noInterrupts();
        // search unused task info area
        for(int i = 0; i < g_task_num; i++) {
            if(g_taskInfo[i]._task_state == EZMTS_TASK_UNUSED) {
                g_taskInfo[i]._task_id = i;
                g_taskInfo[i]._task_state = EZMTS_TASK_STOPPED;
                g_taskInfo[i]._timeout_val = 0;
                g_taskInfo[i]._time_rest = 0;
                g_taskInfo[i]._cb_func = cb_func;
                ret = i;
                break;
            }
        }
        interrupts();
        return ret;
    }
    int start(int task_id, long timeout_val, int when_exec = EZMTS_TIMEDOUT) {
        int ret = -1;
        noInterrupts();
        if((task_id < 0) || (task_id > g_task_num - 1) || timeout_val < 0) {
            interrupts();
            return ret;
        }
        g_taskInfo[task_id]._task_id = task_id;
        g_taskInfo[task_id]._task_state = EZMTS_TASK_RUNNING;
        g_taskInfo[task_id]._timeout_val = timeout_val * _k;
        g_taskInfo[task_id]._time_rest = g_taskInfo[task_id]._timeout_val;
        ret = 0;
        interrupts();
        if(when_exec == EZMTS_AT_ONCE) {
            g_taskInfo[task_id]._cb_func(NULL);
        }

        return ret;
    }
    int stop(int task_id) {
        noInterrupts();
        g_taskInfo[task_id]._task_state = EZMTS_TASK_STOPPED;
        g_taskInfo[task_id]._time_rest = 0;
        g_taskInfo[task_id]._timeout_val = 0;
        interrupts();
        return 0;
    }
    int del(int task_id) {
        noInterrupts();
        g_taskInfo[task_id]._task_id = -1;
        g_taskInfo[task_id]._task_state = EZMTS_TASK_UNUSED;
        g_taskInfo[task_id]._time_rest = 0;
        g_taskInfo[task_id]._timeout_val = 0;
        g_taskInfo[task_id]._cb_func = NULL;
        interrupts();
        return 0;
    }
    int handle() {
        return 0;
    }
    // <provisional function>
    // wait (duration) milli-seconds. works well up to 5 tasks created.
    void delay(unsigned int duration) {
        //fastDigitalWrite(10, HIGH);
        for(unsigned int i = 0; i < duration; i++) {
            long dur = 700 - g_task_num * 70;
            if(dur > 0) {
                delayMicroseconds(dur);
            }
        }
        //fastDigitalWrite(10, LOW);
    }
};

// timer interruption handler.
#if (EZMTS_USE_TIMER == 0)
ISR (TIMER0_COMPA_vect) {
    TCNT0 = 0;
#elif (EZMTS_USE_TIMER == 1)
ISR (TIMER1_COMPA_vect) {
    TCNT1 = 0;
#elif (EZMTS_USE_TIMER == 2)
ISR (TIMER2_COMPA_vect) {
    TCNT2 = 0;
#endif
    noInterrupts();
    //fastDigitalWrite(10, !digitalRead(10));
    for(int i = 0; i < g_task_num; i++) {
        if(g_taskInfo[i]._task_state == EZMTS_TASK_RUNNING) {
            if(--g_taskInfo[i]._time_rest == 0) {
                //_taskInfo[i]._task_state = EZMTS_TASK_STOPPED;
                g_taskInfo[i]._time_rest = g_taskInfo[i]._timeout_val;
                interrupts();
                g_taskInfo[i]._cb_func(NULL);
                noInterrupts();
            }
        }
    }
    interrupts();
}

#endif  /* _HPP_EZMTS_HPP_ */
