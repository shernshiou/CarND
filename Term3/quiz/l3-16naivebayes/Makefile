CC = clang++
CFLAGS  = -g -Wall -std=c++11
LFLAGS =
SOURCES = $(wildcard *.cpp)
EXECUTABLE = naivebayes

all: $(EXECUTABLE)

rebuild: clean $(EXECUTABLE)

clean:
	rm -rf $(EXECUTABLE)

$(EXECUTABLE):
	$(CC) $(CFLAGS) $(SOURCES) -o $(EXECUTABLE)

.PHONY: all clean rebuild
