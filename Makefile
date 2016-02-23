CC = g++
CCFLAGS = -Wall -I/export/teach/1BRobot
LDFLAGS = -L/export/teach/1BRobot
LIBS = -lrobot

EXECUTABLE=robot
SOURCES = main.cpp navigation.cpp robot_state.cpp line_following.cpp sensors_actuators.cpp
OBJECTS = $(SOURCES:.cpp=.o)

.DEFAULT_GOAL = all

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(SOURCES)
	$(CC) $(CCFLAGS) $(LDFLAGS) $(SOURCES) -o $@ $(LIBS)
clean:
	rm -f $(EXECUTABLE) $(OBJECTS)
