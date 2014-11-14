
CC=gcc
CFLAGS=-c 
LDFLAGS=
SOURCES=leso1prog.c 
OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE=leso1prog


#$(warning $(SOURCES))
#$(warning $(OBJECTS))

all: $(SOURCES) $(EXECUTABLE)
	
$(EXECUTABLE): $(OBJECTS) 
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

.c.o:
	$(CC) $(CFLAGS) $< -o $@

clean	:
	rm *.o $(EXECUTABLE)