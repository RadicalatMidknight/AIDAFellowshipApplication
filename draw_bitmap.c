#include <stdlib.h>
#include <stdbool.h>
#include <cairo/cairo.h>
#include <gtk/gtk.h>
#include <pthread.h>
#include <string.h>
#include "pixel_format_RGB.h"
#include "draw_bitmap.h"

#define ARRAYSIZE(A) (sizeof(A)/sizeof(A[0]))

#define REPORT_TIME_DIFFERENCE  0
#define REPORT_PIXEL_VALUES     0

#if REPORT_TIME_DIFFERENCE
static long time_difference_ms( struct timespec *start_time, struct timespec *end_time )
{
  return (end_time->tv_sec - start_time->tv_sec)*1000 + (end_time->tv_nsec - start_time->tv_nsec)/(1000*1000);
}
#endif

/*
 * What is the following interface?
 * Normally, I prefer inter-thread communication to come in one of two forms:
 * [1] a "flag" where no history is maintained, every write stomps on whatever was there
 * [2] a "queue" where all history is maintained, there is a backlog of previous writes to be read
 *
 * The following structure is somewhat similar to the queue option in that there is some history.
 * In this case, a reader active in the communication channel switches the mode of operation of the interface to that of a flag; the writer stomps on the data not being read.
 * When no reader is in the channel, a writer will ping-pong between the two bitmaps.
 * The reader only ever cares about the most recent bitmap.
 * The writer tries to update the oldest bitmap. Sometimes, the writer does not have that opportunity because the reader is using the oldest bitmap.
 *
 * Note:
 * If the GUI thread cannot display fast enough to keep up with the timer, the display will not update.
 * Neither the GUI thread nor the main thread will block, but the display will not make progress.
 */
static struct data_interface_t
{
    /* dynamic data */
    pthread_mutex_t           lock;                   /* a lock for the "most_recently_updated" and "GUI_reading" data */
    unsigned int              most_recently_updated;  /* the index of the most recent bitmap */
    bool                      GUI_reading;            /* set to true when a reader is active in the channel */
    struct pixel_format_RGB * bitmap[2];              /* the data within the bitmaps are read/written with no lock held, the index above is read while locked to figure out which data to read/write */
    bool                      time_to_exit;           /* set to true when it is time for the GUI to exit */
    bool                      window_closed;          /* set to true when the window was prematurely closed */

    /* static configuration */
    int                       argc;
    char **                   argv;
    unsigned int              width;
    unsigned int              height;
    pthread_t                 thread_id;
}                           data_interface =
{
    PTHREAD_MUTEX_INITIALIZER,
    0,
    false,
    {
        NULL,
        NULL
    },
    false,
    false,
    0,
    NULL,
    0,
    0,
    0
};

/* Backing pixmap for drawing area */
static GdkPixmap *pixmap = NULL;
struct cairo_RGB24_t
{
    unsigned char B;
    unsigned char G;
    unsigned char R;
    unsigned char unused;
};
static struct cairo_RGB24_t * cairo_bitmap;
static cairo_format_t         cairo_format;
static int                    cairo_stride;

/* Create a new backing pixmap of the appropriate size */
static gboolean configure_event(GtkWidget *widget, GdkEventConfigure *event)
{
  if (pixmap)
  {
    g_object_unref( pixmap );
  }
  else
  {
    ; /* no pixmap to delete */
  }

  pixmap = gdk_pixmap_new(
      widget->window,
      widget->allocation.width, widget->allocation.height,
      -1 );

  return TRUE;
}

/* Redraw the screen from the backing pixmap */
static gboolean expose_event(GtkWidget *widget, GdkEventExpose *event)
{
  gdk_draw_drawable(
      widget->window,
      widget->style->fg_gc[gtk_widget_get_state( widget )],
      pixmap,
      event->area.x,      event->area.y,
      event->area.x,      event->area.y,
      event->area.width,  event->area.height );

  return FALSE;
}

/* timeout event handler */
static gint timeout_event( gpointer data )
{
  GtkWidget *       widget = (GtkWidget *)data;
  cairo_t *         cairo_context;
  unsigned int      most_recently_updated;
  bool              time_to_exit;
  unsigned int      index;
  cairo_surface_t * image_surface;
#if REPORT_TIME_DIFFERENCE
  struct timespec   start_time;
  struct timespec   stop_time;

  clock_gettime( CLOCK_REALTIME, &start_time );
#endif

  /* determine what is going on and what data is being used when updating the window */
  pthread_mutex_lock( &data_interface.lock );
  time_to_exit = data_interface.time_to_exit;
  data_interface.GUI_reading = true;
  most_recently_updated = data_interface.most_recently_updated;
  pthread_mutex_unlock( &data_interface.lock );

  if (!time_to_exit)
  {
    /* update the window */
    cairo_context = gdk_cairo_create( widget->window );

#if REPORT_PIXEL_VALUES
    printf( "R[%d]: [%3.0d, %3.0d, %3.0d] [%3.0d, %3.0d, %3.0d]\n",
        most_recently_updated,
        data_interface.bitmap[most_recently_updated][0].R,
        data_interface.bitmap[most_recently_updated][0].G,
        data_interface.bitmap[most_recently_updated][0].B,
        data_interface.bitmap[most_recently_updated][1].R,
        data_interface.bitmap[most_recently_updated][1].G,
        data_interface.bitmap[most_recently_updated][1].B );
#endif
    for (index = 0; index < data_interface.width * data_interface.height; index++)
    {
      cairo_bitmap[index].R = data_interface.bitmap[most_recently_updated][index].R;
      cairo_bitmap[index].G = data_interface.bitmap[most_recently_updated][index].G;
      cairo_bitmap[index].B = data_interface.bitmap[most_recently_updated][index].B;
      cairo_bitmap[index].unused = 0;
    }
    image_surface = cairo_image_surface_create_for_data(
        (unsigned char *)cairo_bitmap,
        cairo_format,
        data_interface.width,
        data_interface.height,
        cairo_stride );
    cairo_set_source_surface( cairo_context, image_surface, 0, 0 );
    cairo_paint( cairo_context );
    cairo_show_page( cairo_context );
    cairo_surface_destroy( image_surface );
#if REPORT_PIXEL_VALUES
    printf( "R[%d]: done\n", most_recently_updated );
#endif

    cairo_destroy( cairo_context );
  }
  else
  {
    ; /* no point in wasting time updating the window if it is time to shut down */
  }

  /* release the bitmap for future updates */
  pthread_mutex_lock( &data_interface.lock );
  data_interface.GUI_reading = false;
  pthread_mutex_unlock( &data_interface.lock );

  if (time_to_exit)
  {
    gtk_main_quit();
  }
  else
  {
    ; /* keep updating the window */
  }

#if REPORT_TIME_DIFFERENCE
  clock_gettime( CLOCK_REALTIME, &stop_time );
  printf( "%6.6ld\n", time_difference_ms( &start_time, &stop_time ) );
#endif

  return TRUE;
}

/* the function that shuts down the event loop when the window is destroyed */
static void destroy( GtkWidget *widget, gpointer data )
{
  gtk_main_quit();

  return;
}

/* the GUI thread, creates the window and waits for events */
static void *GUI_thread( void *unused )
{
  GtkWidget * window;
  GtkWidget * drawing_area;
  GtkWidget * vbox;
  gint        timeout_handle;

  gtk_init( &data_interface.argc, &data_interface.argv );

  window = gtk_window_new( GTK_WINDOW_TOPLEVEL );
  gtk_widget_set_name( window, "Draw Bitmap" );

  vbox = gtk_vbox_new( FALSE, 0 );
  gtk_container_add( GTK_CONTAINER( window ), vbox );
  gtk_widget_show( vbox );

  g_signal_connect( window, "destroy", G_CALLBACK( destroy ), NULL );

  /* Create the drawing area */
  drawing_area = gtk_drawing_area_new();
  gtk_widget_set_size_request( GTK_WIDGET( drawing_area ), data_interface.width, data_interface.height );
  gtk_box_pack_start( GTK_BOX( vbox ), drawing_area, TRUE, TRUE, 0 );
  gtk_widget_show( drawing_area );

  /* Signals and events */
  g_signal_connect( drawing_area, "expose_event",         G_CALLBACK( expose_event ),         NULL );
  g_signal_connect( drawing_area, "configure_event",      G_CALLBACK( configure_event ),      NULL );
  timeout_handle = g_timeout_add( 30, timeout_event, drawing_area );

  /* display the window and handle events */
  gtk_widget_show( window );
  gtk_main();

  /* clean up */
  g_source_remove( timeout_handle );

  data_interface.window_closed    = true;

  return NULL;
}

void draw_bitmap_create_window(  int argc, char **argv, unsigned int width, unsigned int height )
{
  unsigned int  index;

  data_interface.argc             = argc;
  data_interface.argv             = argv;
  data_interface.width            = width;
  data_interface.height           = height;
  for (index = 0; index < ARRAYSIZE(data_interface.bitmap); index++)
  {
    data_interface.bitmap[index]  = malloc( width*height*sizeof(*data_interface.bitmap[index]) );
    memset( data_interface.bitmap[index], 0, width*height*sizeof(*data_interface.bitmap[index]) );
  }
  cairo_format = CAIRO_FORMAT_RGB24;
  cairo_stride = cairo_format_stride_for_width( cairo_format, data_interface.width );
  cairo_bitmap = (struct cairo_RGB24_t *)malloc( cairo_stride * data_interface.height );

  pthread_create( &data_interface.thread_id, NULL, GUI_thread, NULL );

  return;
}

void draw_bitmap_display( struct pixel_format_RGB * bitmap )
{
  unsigned int  index;
  unsigned int  bitmap_to_update;

  /* the writer always updates the oldest data, the reader only ever reads the newest data */
  pthread_mutex_lock( &data_interface.lock );
  bitmap_to_update = (data_interface.most_recently_updated + 1) % ARRAYSIZE(data_interface.bitmap);
  pthread_mutex_unlock( &data_interface.lock );

  /* update the stale bitmap */
  for (index = 0; index < data_interface.width*data_interface.height; index++)
  {
    data_interface.bitmap[bitmap_to_update][index] = bitmap[index];
  }
#if REPORT_PIXEL_VALUES
  printf( "W[%d]: -%3.0d, %3.0d, %3.0d- -%3.0d, %3.0d, %3.0d-\n",
      bitmap_to_update,
      data_interface.bitmap[bitmap_to_update][0].R,
      data_interface.bitmap[bitmap_to_update][0].G,
      data_interface.bitmap[bitmap_to_update][0].B,
      data_interface.bitmap[bitmap_to_update][1].R,
      data_interface.bitmap[bitmap_to_update][1].G,
      data_interface.bitmap[bitmap_to_update][1].B );
#endif

  /* if there is no reader present, let the reader know that there is updated data */
  pthread_mutex_lock( &data_interface.lock );
  if (data_interface.GUI_reading)
  {
    ; /* there is a reader using the newest data, do not try to touch it */
  }
  else
  {
    data_interface.most_recently_updated = bitmap_to_update;
  }
  pthread_mutex_unlock( &data_interface.lock );

  return;
}

void draw_bitmap_close_window( void )
{
  /* this flag is an atomic data element (written in a single clock cycle), so there is no possibility for a race condition */
  data_interface.time_to_exit = true;

  pthread_join( data_interface.thread_id, NULL );

  return;
}

bool draw_bitmap_window_closed( void )
{
  return data_interface.window_closed;
}
