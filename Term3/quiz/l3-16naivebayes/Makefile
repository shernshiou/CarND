CC = clang++
CFLAGS  = -g -Wall -std=c++11
LFLAGS =
SOURCES = $(wildcard *.cpp)
EXECUTABLE = naivebayes

all: $(EXECUTABLE)

clean:
	rm $(EXECUTABLE)

$(EXECUTABLE):
	$(CC) $(CFLAGS) $(SOURCES) -o $(EXECUTABLE)

.PHONY: all clean
