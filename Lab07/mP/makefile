CC= gcc
CC2= g++

CFLAGS=`pkg-config --cflags opencv`
LDFLAGS=`pkg-config --libs opencv`

TARGET= lab07
OBJECTS= closedLoopControl.o lab07.o

all: clean compile link

#NEW RULES MORE VERSATILE (CAN CHANGE FILES NAMES)
compile: $(OBJECTS)

link: $(TARGET).out

clean:
	rm -f *.o
	rm -f *.out

.PHONY: clean all compile link

# IMPLICIT RULE FOR COMPILING OBJECTS
%.o: %.c
	$(CC2) -c $^ -o $@ -Wall $(CFLAGS)
	
%.out:
	$(CC2) -o $(TARGET).out $(OBJECTS) -Wall $(LDFLAGS)

#OLD RULES
#compile: lab07_functions.h
#	$(CC2) -c lab07_function.c -o lab07_function.o -Wall $(CFLAGS)
#	$(CC2) -c lab07.c -o lab07.o -Wall $(CFLAGS)

#link: pid.h
#	$(CC2) -o lab07.out lab07_function.o lab07.o -Wall $(LDFLAGS)

