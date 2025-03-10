TARGET=hw6briggs

SOURCES=../include/import_registers.c \
        ../include/enable_pwm_clock.c \
        ../include/wait_period.c \
        ../include/wait_key.c \
        keypress.c \
        video_interface.c \
        scale_image_data.c \
        draw_bitmap.c \
        hw6briggs.c

OBJECTS=$(patsubst %.cpp,%.o,$(patsubst %.c,%.o,$(notdir $(SOURCES))))

# Get the correct compiler flags for GTK, Cairo, and FFmpeg
CFLAGS = `pkg-config --cflags gtk+-2.0 cairo` \
         -Wno-deprecated-declarations \
         -g

# Get the correct linker flags for GTK, Cairo, and FFmpeg
LDFLAGS = `pkg-config --libs gtk+-2.0 cairo` -lswscale -lm -lpthread

all: $(OBJECTS)
	# Link with the proper libraries using g++
	g++ $(OBJECTS) -o $(TARGET) $(LDFLAGS)

clean:
	rm -f $(OBJECTS) $(TARGET)

$(TARGET): $(OBJECTS)
	# Link the objects with necessary libraries
	gcc $^ $(CFLAGS) -o $@ $(LDFLAGS)

%.o:%.c
	# Compile C files
	gcc $(CFLAGS) -c $< -o $@

%.o:../include/%.c
	# Compile C files from include folder
	gcc -c $< -o $@

%.o:%.cpp
	# Compile C++ files
	g++ -c $< -o $@

%.o:../include/%.cpp
	# Compile C++ files from include folder
	g++ -c $< -o $@

