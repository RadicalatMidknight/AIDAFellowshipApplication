/**************************************************
* File:  hw6briggs
* Homework 6
* By Bailey Briggs
* CMPEN 473, Spring 2025, Penn State University
* 
* Revision V1.0 2/27/2025
* 
* For Raspberry Pi 4 Computer - with Raspberry Pi OS 32bit
* Raspberry Pi 4 Computer (RPi4) GPIO pin connections:
*   Red    LED on GPIO 22 - with 220 Ohm resistor in series
*   Green  LED on GPIO 22 - with 220 Ohm resistor in series
*   Blue   LED on GPIO 23 - with 220 Ohm resistor in series
*   Orange LED on GPIO 23 - with 220 Ohm resistor in series
* 
***************************************************/

// header files - at /usr/include and ../include and .
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <termios.h>
#include <fcntl.h>
#include <pthread.h>
#include <linux/videodev2.h>
#include <time.h>
#include "../include/import_registers.h"
#include "../include/cm.h"
#include "../include/gpio.h"
#include "../include/uart.h"
#include "../include/spi.h"
#include "../include/bsc.h"
#include "../include/pwm.h"
#include "../include/enable_pwm_clock.h"
#include "../include/io_peripherals.h"
#include "../include/wait_period.h"
#include "../include/FIFO.h"
#include "../include/MPU6050.h"
#include "../include/MPU9250.h"
#include "../include/wait_key.h"
#include "../include/raspicam_wrapper.h"
#include "keypress.h"
#include "pixel_format_RGB.h"
#include "video_interface.h"
#include "wait_key.h"
#include "scale_image_data.h"
#include "draw_bitmap.h"

#define PWM_RANGE 100
#define REPORT_TIMES              0   /* != 0 to print some timing statistics */
#define GET_FRAMES                10  /* the number of frame times to average when determining the FPS */
#define SCALE_REDUCTION_PER_AXIS  4   /* the image size reduction ratio (note that 640*480*3*8*FPS = your bandwidth usage, at 24FPS, that is 177MPBS) */

int PWM_RANGE1 = 90;
int PWM_RANGE2 = 90;
int mode = 1;
char turn = 's';
int tiem = 100000;
int degrees = 5;
struct pixel_format_RGB *         scaled_RGB_data;
struct pixel_format_RGB *         grayscale_RGB_data;
struct pixel_format_RGB *         bw_RGB_data;

struct pause_flag
{
  pthread_mutex_t lock;
  bool pause;
};

struct done_flag
{
  pthread_mutex_t lock;
  bool done;
};

struct thread_parameter
{
  volatile struct gpio_register * gpio;
  volatile struct pwm_register * pwm;
  int pin;
  struct pause_flag * pause;
  struct done_flag * done;
};

struct key_thread_parameter
{
  struct done_flag * done;
  struct pause_flag * pause1;
  struct pause_flag * pause2;
  struct pause_flag * pause3;
  struct pause_flag * pause4;
  struct pause_flag * pause5;
  struct pause_flag * pause6;
};

int Tstep = 50;
int Tlevel = 5;


void *thread_control( void * arg )
{
  //int iterations;
  //int timeu;
  //int dlevel;
  
  struct thread_parameter * parameter = (struct thread_parameter *)arg;
  volatile struct io_peripherals *io;
  io = import_registers();
  io->gpio->GPFSEL2.field.FSEL4 = GPFSEL_INPUT;
  io->gpio->GPFSEL2.field.FSEL5 = GPFSEL_INPUT;
  
   while (!(parameter->done->done))
  {

    usleep( 10000 );
    if (mode == 1) {
    
      if (parameter->pin == 12)
      {
        parameter->pwm->DAT1 = PWM_RANGE1;
      }
      else if (parameter->pin == 13)
      {
        parameter->pwm->DAT2 = PWM_RANGE2;
      }
    }
    else if (mode == 2){  //uses automatic control of car engine
      if (parameter->pin == 12)
      {
        if (GPIO_READ( io->gpio, 24) == 0 && GPIO_READ( io->gpio, 25) != 0) { //turn left onto black line
          pthread_mutex_lock( &(parameter->pause->lock) );
          PWM_RANGE1 = 80;
          PWM_RANGE2 = 0;
          parameter->pwm->DAT1 = PWM_RANGE1;
          usleep(400000);
          PWM_RANGE1 = 50;
          PWM_RANGE2 = 50;
          pthread_mutex_unlock( &(parameter->pause->lock) );
        }
        parameter->pwm->DAT1 = PWM_RANGE1;
      }
      else if (parameter->pin == 13)
      {
        if (GPIO_READ( io->gpio, 24) != 0 && GPIO_READ( io->gpio, 25) == 0) { //turn right onto black line
          pthread_mutex_lock( &(parameter->pause->lock) );
          PWM_RANGE1 = 0;
          PWM_RANGE2 = 80;
          parameter->pwm->DAT2 = PWM_RANGE2;
          usleep(400000);
          PWM_RANGE1 = 50;
          PWM_RANGE2 = 50;
          pthread_mutex_unlock( &(parameter->pause->lock) );
        }
        parameter->pwm->DAT2 = PWM_RANGE2;
      }
    }
  } 

  return 0;
}



void *ThreadKey( void * arg )
{
  volatile struct io_peripherals *io;
  struct key_thread_parameter *thread_key_parameter = (struct key_thread_parameter *)arg;
  bool done;
  pthread_t thread05_handle;
  pthread_t thread06_handle;
  pthread_t thread22_handle;
  pthread_t thread23_handle;
  struct done_flag done0   = {PTHREAD_MUTEX_INITIALIZER, false};
  struct pause_flag pause3 = {PTHREAD_MUTEX_INITIALIZER, false};
  struct pause_flag pause4 = {PTHREAD_MUTEX_INITIALIZER, false};
  struct pause_flag pause5 = {PTHREAD_MUTEX_INITIALIZER, false};
  struct pause_flag pause6 = {PTHREAD_MUTEX_INITIALIZER, false};
  struct thread_parameter thread05_parameter;
  struct thread_parameter thread06_parameter;
  struct thread_parameter thread22_parameter;
  struct thread_parameter thread23_parameter; 

  io = import_registers();
  if (io != NULL)
  {
    enable_pwm_clock(io->cm, io->pwm);

    io->gpio->GPFSEL0.field.FSEL5 = GPFSEL_OUTPUT;
    io->gpio->GPFSEL0.field.FSEL6 = GPFSEL_OUTPUT;
    io->gpio->GPFSEL2.field.FSEL2 = GPFSEL_OUTPUT;
    io->gpio->GPFSEL2.field.FSEL3 = GPFSEL_OUTPUT;
    
    io->pwm->RNG1 = PWM_RANGE;     //the default value
    io->pwm->RNG2 = PWM_RANGE;
    
    io->pwm->CTL.field.MODE1 = 0;  //PWM mode
    io->pwm->CTL.field.MODE2 = 0;
    
    io->pwm->CTL.field.RPTL1 = 1;  //not using FIFO, but repeat the last byte anyway
    io->pwm->CTL.field.RPTL2 = 1;
    
    io->pwm->CTL.field.SBIT1 = 0;  //idle low
    io->pwm->CTL.field.SBIT2 = 0;
    
    io->pwm->CTL.field.POLA1 = 0;  //non-inverted polarity
    io->pwm->CTL.field.POLA2 = 0;
    
    io->pwm->CTL.field.USEF1 = 0;  //do not use FIFO
    io->pwm->CTL.field.USEF2 = 0;
    
    io->pwm->CTL.field.MSEN1 = 1;  //use M/S algorithm
    io->pwm->CTL.field.MSEN2 = 1;
    
    io->pwm->CTL.field.CLRF1 = 1;  //clear the FIFO, even though it is not used
    
    io->pwm->CTL.field.PWEN1 = 1;  //enable the PWM channel
    io->pwm->CTL.field.PWEN2 = 1;
    
    thread05_parameter.pin = 05;
    thread05_parameter.pwm = io->pwm;
    thread05_parameter.gpio = io->gpio;
    thread05_parameter.done = &done0;
    thread05_parameter.pause = &pause3;
    thread06_parameter.pin = 06;
    thread06_parameter.pwm = io->pwm;
    thread06_parameter.gpio = io->gpio;
    thread06_parameter.done = &done0;
    thread06_parameter.pause = &pause4;
    thread22_parameter.pin = 22;
    thread22_parameter.pwm = io->pwm;
    thread22_parameter.gpio = io->gpio;
    thread22_parameter.done = &done0;
    thread22_parameter.pause = &pause5;
    thread23_parameter.pin = 23;
    thread23_parameter.pwm = io->pwm;
    thread23_parameter.gpio = io->gpio;
    thread23_parameter.done = &done0;
    thread23_parameter.pause = &pause6;

    GPIO_CLR( io->gpio, 05);
    GPIO_CLR( io->gpio, 06);
    GPIO_CLR( io->gpio, 22);
    GPIO_CLR( io->gpio, 23);

    int pwm1;
    int pwm2;

    int dir = 0;

    do
    {
      switch (get_pressed_key())
      {
        case 'q':
          printf("q\n");
          done = true;
          //unpause everything
          pthread_mutex_lock( &(thread_key_parameter->pause1->lock) );
          thread_key_parameter->pause1->pause = false;
          pthread_mutex_unlock( &(thread_key_parameter->pause1->lock) );
          pthread_mutex_lock( &(thread_key_parameter->pause2->lock) );
          thread_key_parameter->pause2->pause = false;
          pthread_mutex_unlock( &(thread_key_parameter->pause2->lock) );
          pthread_mutex_lock( &(thread_key_parameter->pause3->lock) );
          thread_key_parameter->pause3->pause = false;
          pthread_mutex_unlock( &(thread_key_parameter->pause3->lock) );
          pthread_mutex_lock( &(thread_key_parameter->pause4->lock) );
          thread_key_parameter->pause4->pause = false;
          pthread_mutex_unlock( &(thread_key_parameter->pause4->lock) );

          //shut down
          pthread_mutex_lock( &(thread_key_parameter->done->lock) );
          thread_key_parameter->done->done = true;
          pthread_mutex_unlock( &(thread_key_parameter->done->lock) );
          GPIO_CLR( io->gpio, 05);
          GPIO_CLR( io->gpio, 06);
          GPIO_CLR( io->gpio, 22);
          GPIO_CLR( io->gpio, 23);
          break;
        case 's':
          pthread_mutex_lock( &(thread_key_parameter->pause1->lock) );
          if (mode == 1) { printf("s m1\n"); }
          else if (mode == 2) { printf("s m2\n"); }
          GPIO_CLR( io->gpio, 05);
          GPIO_CLR( io->gpio, 06);
          GPIO_CLR( io->gpio, 22);
          GPIO_CLR( io->gpio, 23);
          pthread_mutex_unlock( &(thread_key_parameter->pause1->lock) );
          break;
        case 'w':
          pthread_mutex_lock( &(thread_key_parameter->pause1->lock) );
          if (mode == 1) { printf("w m1\n"); }
          else if (mode == 2) { printf("w m2\n"); }
          if (dir == -1) {
            GPIO_CLR( io->gpio, 05);
            GPIO_CLR( io->gpio, 06);
            GPIO_CLR( io->gpio, 22);
            GPIO_CLR( io->gpio, 23);
            usleep( 10000 );
          }
          GPIO_SET( io->gpio, 05);
          GPIO_CLR( io->gpio, 06);
          GPIO_SET( io->gpio, 22);
          GPIO_CLR( io->gpio, 23);
          dir = 1;
          pthread_mutex_unlock( &(thread_key_parameter->pause1->lock) );
          break;
        case 'x':
          pthread_mutex_lock( &(thread_key_parameter->pause1->lock) );
          if (mode == 1) { printf("x m1\n"); 
            if (dir == 1) {
              GPIO_CLR( io->gpio, 05);
              GPIO_CLR( io->gpio, 06);
              GPIO_CLR( io->gpio, 22);
              GPIO_CLR( io->gpio, 23);
              usleep( 10000 );
            }
            GPIO_CLR( io->gpio, 05);
            GPIO_SET( io->gpio, 06);
            GPIO_CLR( io->gpio, 22);
            GPIO_SET( io->gpio, 23);
            dir = -1;
          }
          pthread_mutex_unlock( &(thread_key_parameter->pause1->lock) );
          break;
        case 'r':
          pthread_mutex_lock( &(thread_key_parameter->pause1->lock) );
          if (mode == 1) { printf("r m1\n"); 
            GPIO_CLR( io->gpio, 05);
            GPIO_CLR( io->gpio, 06);
            GPIO_CLR( io->gpio, 22);
            GPIO_CLR( io->gpio, 23);
            usleep( 10000 );
            GPIO_CLR( io->gpio, 05);
            GPIO_SET( io->gpio, 06);
            GPIO_SET( io->gpio, 22);
            GPIO_CLR( io->gpio, 23);
          }
          pthread_mutex_unlock( &(thread_key_parameter->pause1->lock) );
          break;
        case 'i':
          pthread_mutex_lock( &(thread_key_parameter->pause1->lock) );
          if (PWM_RANGE1 < 90 && PWM_RANGE2 < 90) {
            PWM_RANGE1 += 5;
            PWM_RANGE2 += 5;
          }
          pthread_mutex_unlock( &(thread_key_parameter->pause1->lock) );
          printf("i -> %d\n", PWM_RANGE1);
          break;
        case 'j':
          pthread_mutex_lock( &(thread_key_parameter->pause1->lock) );
          if (PWM_RANGE1 > 10 && PWM_RANGE2 > 10) {
             PWM_RANGE1 -= 5;
             PWM_RANGE2 -= 5;
          }
          pthread_mutex_unlock( &(thread_key_parameter->pause1->lock) );
          printf("j -> %d\n", PWM_RANGE1);
          break;
        case 'a':
          pthread_mutex_lock( &(thread_key_parameter->pause1->lock) );
          if (mode == 1) {
            printf("a\n");
            pwm1 = PWM_RANGE1;
            pwm2 = PWM_RANGE2;
            PWM_RANGE1 = 0;
            PWM_RANGE2 = 80;
            usleep(tiem);
            PWM_RANGE1 = pwm1;
            PWM_RANGE2 = pwm2;
          }
          pthread_mutex_unlock( &(thread_key_parameter->pause1->lock) );
          break;
        case 'd':
          pthread_mutex_lock( &(thread_key_parameter->pause1->lock) );
          if (mode == 1) {
            printf("d\n");
            pwm1 = PWM_RANGE1;
            pwm2 = PWM_RANGE2;
            PWM_RANGE1 = 80;
            PWM_RANGE2 = 0;
            usleep(tiem);
            PWM_RANGE1 = pwm1;
            PWM_RANGE2 = pwm2;
          }
          pthread_mutex_unlock( &(thread_key_parameter->pause1->lock) );
          break;
          
        case 'o':
          pthread_mutex_lock( &(thread_key_parameter->pause1->lock) );
          if (tiem < 500000) {
            tiem += 50000;
            degrees += 5;
          }
          pthread_mutex_unlock( &(thread_key_parameter->pause1->lock) );
          printf("o -> %d\n", degrees);
          break;
        case 'k':
          pthread_mutex_lock( &(thread_key_parameter->pause1->lock) );
          if (tiem > 50000) {
             tiem -= 50000;
             degrees -= 5;
          }
          pthread_mutex_unlock( &(thread_key_parameter->pause1->lock) );
          printf("k -> %d\n", degrees);
          break;
          
        case 'm': //choose mode
          printf("m");
          break;
            case '1': //switch to mode 1
              pthread_mutex_lock( &(thread_key_parameter->pause1->lock) );
              printf("1\n");
              mode = 1;
              GPIO_CLR( io->gpio, 05);
              GPIO_CLR( io->gpio, 06);
              GPIO_CLR( io->gpio, 22);
              GPIO_CLR( io->gpio, 23);
              PWM_RANGE1 = 90;
              PWM_RANGE2 = 90;
              pthread_mutex_unlock( &(thread_key_parameter->pause1->lock) );
              break;
            case '2': //switch to mode 2
              pthread_mutex_lock( &(thread_key_parameter->pause1->lock) );
              printf("2\n");
              mode = 2;
              GPIO_CLR( io->gpio, 05);
              GPIO_CLR( io->gpio, 06);
              GPIO_CLR( io->gpio, 22);
              GPIO_CLR( io->gpio, 23);
              PWM_RANGE1 = 50;
              PWM_RANGE2 = 50;
              pthread_mutex_unlock( &(thread_key_parameter->pause1->lock) );
              break;
        default:
          break;
      }
    } while (!done);
  }
  else
  {
    ; /* warning message already issued */
  }
  printf( "exit\n" );

  return (void *)0;
}

void convert_to_grayscale(struct pixel_format_RGB *color_data, size_t width, size_t height, struct pixel_format_RGB *gray_data)
{
    for (size_t i = 0; i < width * height; i++) {
        unsigned char gray_value = (unsigned char)(0.299 * color_data[i].R + 0.587 * color_data[i].G + 0.114 * color_data[i].B);
        gray_data[i].R = gray_data[i].G = gray_data[i].B = gray_value;
    }
}

void convert_to_black_and_white(struct pixel_format_RGB *color_data, size_t width, size_t height, struct pixel_format_RGB *bw_data)
{
    for (size_t i = 0; i < width * height; i++) {
        unsigned char bw_value = (unsigned char)((0.299 * color_data[i].R + 0.587 * color_data[i].G + 0.114 * color_data[i].B) > 128 ? 255 : 0);
        bw_data[i].R = bw_data[i].G = bw_data[i].B = bw_value;
    }
}

int main( int argc, char * argv[] )
{
  volatile struct io_peripherals *io;
  pthread_t thread12_handle;
  pthread_t thread13_handle;
  pthread_t camera_tid;
  pthread_t thread_key_handle;
  
  
  struct done_flag done   = {PTHREAD_MUTEX_INITIALIZER, false};
  struct pause_flag pause1 = {PTHREAD_MUTEX_INITIALIZER, false};
  struct pause_flag pause2 = {PTHREAD_MUTEX_INITIALIZER, false};
  struct pause_flag pause3 = {PTHREAD_MUTEX_INITIALIZER, false};
  struct pause_flag pause4 = {PTHREAD_MUTEX_INITIALIZER, false};
  struct pause_flag pause5 = {PTHREAD_MUTEX_INITIALIZER, false};
  struct pause_flag pause6 = {PTHREAD_MUTEX_INITIALIZER, false};
  struct thread_parameter thread12_parameter;
  struct thread_parameter thread13_parameter;
  struct key_thread_parameter thread_key_parameter;
  
  struct video_interface_handle_t * handle;
  static struct image_t             image;
  unsigned char *                   scaled_data;
  unsigned int                      scaled_height;
  unsigned int                      scaled_width;
  
#if REPORT_TIMES
  struct timespec                   start_time              = {0};
  struct timespec                   open_time               = {0};
  struct timespec                   print_time              = {0};
  struct timespec                   set_mode_time           = {0};
  struct timespec                   close_time              = {0};
  unsigned int                      frame_index             = 0;
  struct timespec                   frame_times[GET_FRAMES] = {0};

  clock_gettime( CLOCK_REALTIME, &start_time );
#endif

  handle = video_interface_open( "/dev/video0" );
#if REPORT_TIMES
  clock_gettime( CLOCK_REALTIME, &open_time );
#endif

  video_interface_print_modes( handle );
#if REPORT_TIMES
  clock_gettime( CLOCK_REALTIME, &print_time );
#endif

#if 0
  // just an example of how to manually select a video format
  if (video_interface_set_mode_manual( handle, 0 ))
#else
  if (video_interface_set_mode_auto( handle ))
#endif
  {
#if REPORT_TIMES
    clock_gettime( CLOCK_REALTIME, &set_mode_time );
#endif

     printf( "configured resolution: %zux%zu\n", handle->configured_width, handle->configured_height );
    // set up the buffer for scaled data
    scaled_width  = handle->configured_width/SCALE_REDUCTION_PER_AXIS;
    scaled_height = handle->configured_height/SCALE_REDUCTION_PER_AXIS;
    scaled_data     = (unsigned char *)malloc( sizeof(image)/(SCALE_REDUCTION_PER_AXIS*SCALE_REDUCTION_PER_AXIS) );
    scaled_RGB_data = (struct pixel_format_RGB *)scaled_data;

    // create separate buffers for color, grayscale, and black & white
    grayscale_RGB_data = (struct pixel_format_RGB *)malloc(sizeof(struct pixel_format_RGB) * scaled_width * scaled_height);
    bw_RGB_data = (struct pixel_format_RGB *)malloc(sizeof(struct pixel_format_RGB) * scaled_width * scaled_height);

    // create the window to show the bitmap
    draw_bitmap_create_window( argc, argv, scaled_width, scaled_height );

    while (!wait_key(0, NULL) &&
           !draw_bitmap_window_closed())
    {
      // capture an image
      if (video_interface_get_image( handle, &image ))
      {
        // scale the image to a more agreeable size
        scale_image_data(
            (struct pixel_format_RGB *)&image,
            handle->configured_height,
            handle->configured_width,
            scaled_RGB_data,
            SCALE_REDUCTION_PER_AXIS,
            SCALE_REDUCTION_PER_AXIS );

        // convert to grayscale
        convert_to_grayscale(scaled_RGB_data, scaled_width, scaled_height, grayscale_RGB_data);
        
        // convert to black and white
        convert_to_black_and_white(scaled_RGB_data, scaled_width, scaled_height, bw_RGB_data);

              draw_bitmap_display(scaled_RGB_data);        // color
              draw_bitmap_display(bw_RGB_data);            // black & white
              draw_bitmap_display(grayscale_RGB_data);     // grayscale
#if REPORT_TIMES
        clock_gettime( CLOCK_REALTIME, &(frame_times[frame_index]) );
        frame_index = (frame_index + 1) % GET_FRAMES;
#endif
      }
      else
      {
        printf( "did not get an image\n" );
      }
    }
  }
  else
  {
    printf( "failed to configure\n" );
  }
  
  

  // clean up
  draw_bitmap_close_window();
  video_interface_close( handle );
  free( scaled_data );
  free(grayscale_RGB_data);
  free(bw_RGB_data);

#if REPORT_TIMES
  clock_gettime( CLOCK_REALTIME, &close_time );

  printf( "\n" );
  printf(   "run duration:\n" );
  printf(   "open:      %lu\n", calc_time_diff_in_us( &start_time, &open_time ) );
  printf(   "print:     %lu\n", calc_time_diff_in_us( &start_time, &print_time ) );
  printf(   "set mode:  %lu\n", calc_time_diff_in_us( &start_time, &set_mode_time ) );
  printf(   "close:     %lu\n", calc_time_diff_in_us( &start_time, &close_time ) );
  printf( "approximate frames/sec = %.2f\n", 1.0 / ((calc_time_diff_in_us( &start_time, &(frame_times[GET_FRAMES-1]) ) - calc_time_diff_in_us( &start_time, &(frame_times[GET_FRAMES-2]) )) / 1000000.0) );
#endif

  io = import_registers();
  if (io != NULL)
  {
    /* print where the I/O memory was actually mapped to */
    printf( "mem at 0x%8.8X\n", (unsigned long)io );

    enable_pwm_clock(io->cm, io->pwm);


    //set the pin function to alternate function 0 for GPIO12 and 13
    io->gpio->GPFSEL1.field.FSEL2 = GPFSEL_ALTERNATE_FUNCTION0;
    io->gpio->GPFSEL1.field.FSEL3 = GPFSEL_ALTERNATE_FUNCTION0;

    //configure the PWM channels
    io->pwm->RNG1 = PWM_RANGE;     //the default value
    io->pwm->RNG2 = PWM_RANGE;
    
    io->pwm->CTL.field.MODE1 = 0;  //PWM mode
    io->pwm->CTL.field.MODE2 = 0;
    
    io->pwm->CTL.field.RPTL1 = 1;  //not using FIFO, but repeat the last byte anyway
    io->pwm->CTL.field.RPTL2 = 1;
    
    io->pwm->CTL.field.SBIT1 = 0;  //idle low
    io->pwm->CTL.field.SBIT2 = 0;
    
    io->pwm->CTL.field.POLA1 = 0;  //non-inverted polarity
    io->pwm->CTL.field.POLA2 = 0;
    
    io->pwm->CTL.field.USEF1 = 0;  //do not use FIFO
    io->pwm->CTL.field.USEF2 = 0;
    
    io->pwm->CTL.field.MSEN1 = 1;  //use M/S algorithm
    io->pwm->CTL.field.MSEN2 = 1;
    
    io->pwm->CTL.field.CLRF1 = 1;  //clear the FIFO, even though it is not used
    
    io->pwm->CTL.field.PWEN1 = 1;  //enable the PWM channel
    io->pwm->CTL.field.PWEN2 = 1;

    thread12_parameter.pin = 12;
    thread12_parameter.gpio = io->gpio;
    thread12_parameter.pwm = io->pwm;
    thread12_parameter.done = &done;
    thread12_parameter.pause = &pause1;
    
    thread13_parameter.pin = 13;
    thread13_parameter.pwm = io->pwm;
    thread13_parameter.gpio = io->gpio;
    thread13_parameter.done = &done;
    thread13_parameter.pause = &pause2;
    thread_key_parameter.done = &done;
    
    thread_key_parameter.pause1 = &pause1;
    thread_key_parameter.pause2 = &pause2;
    thread_key_parameter.pause3 = &pause3;
    thread_key_parameter.pause4 = &pause4;
    thread_key_parameter.pause5 = &pause5;
    thread_key_parameter.pause6 = &pause6;

    pthread_create( &thread12_handle, 0, thread_control, (void *)&thread12_parameter );
    pthread_create( &thread13_handle, 0, thread_control, (void *)&thread13_parameter );
    pthread_create( &thread_key_handle, 0, ThreadKey, (void *)&thread_key_parameter);
    pthread_join( thread12_handle, 0 );
    pthread_join( thread13_handle, 0 );
    pthread_join( thread_key_handle, 0 );
  }
  else
  {
    ; /* warning message already issued */
  }

  return 0;
}

