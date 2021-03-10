CXX = g++
CXXFLAGS = -Wall -O3
LIBS = -lwiringPi
TARGET = liblorapi.a
OBJS = lora.o packet.o
LIB = ../lib

all: $(TARGET)

clean:
	rm -f $(TARGET) *.o

$(TARGET): $(OBJS)
	ar rcs $(LIB)/$@ $^

lora.o: lora.cpp lora.h
	g++ -c $(CXXFLAGS) $(INC) $<

packet.o: packet.cpp packet.h
	g++ -c $(CXXFLAGS) $(INC) $<
