CC = g++
CCFLAGS = -Wall -I/export/teach/1BRobot
LDFLAGS = -L/export/teach/1BRobot
LIBS = -lrobot

ARM_CC = arm-unknown-linux-gnueabi-g++
ARM_CCFLAGS = -Wall -I/usr/arm-unknown-linux-gnueabi/include -I/export/teach/1BRobot
ARM_LDFLAGS = -L/usr/arm-unknown-linux-gnueabi/lib
LIBS_ARM = -lrobot


EXECUTABLE=robot
SOURCES = main.cpp navigation.cpp robot_state.cpp line_following.cpp sensors_actuators.cpp endpoint_tasks.cpp
OBJECTS = $(SOURCES:.cpp=.o)

.DEFAULT_GOAL = all

all: x86 arm

x86: $(EXECUTABLE) $(SOURCES)

arm: $(EXECUTABLE)-arm $(SOURCES)

$(EXECUTABLE): $(SOURCES)
	$(CC) $(CCFLAGS) $(LDFLAGS) $(SOURCES) -o $@ $(LIBS)

$(EXECUTABLE)-arm: $(SOURCES)
	$(ARM_CC) $(ARM_CCFLAGS) $(ARM_LDFLAGS) $(SOURCES) -o $@ $(LIBS_ARM)

clean:
	rm -f $(EXECUTABLE) $(EXECUTABLE)-arm $(OBJECTS)
