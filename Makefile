## Files for building main ARWAIN program.
arwain_c_src = src/bmi2.c src/bmi270.c src/bmm150.c
arwain_cpp_src = src/main.cpp src/quaternions.cpp src/stance.cpp \
	src/imu_utils.cpp src/utils.cpp src/madgwick.cpp src/efaroe.cpp \
	src/math_util.cpp src/bin_log.cpp src/input_parser.cpp \
	src/indoor_positioning_wrapper.cpp
arwain_obj = $(arwain_cpp_src:.cpp=.o) $(arwain_c_src:.c=.o)

## Files for building calibration program.
calib_c_src = src/bmi2.c src/bmi270.c src/bmm150.c
calib_cpp_src = src/imu_utils.cpp src/calibrate_bmi270.cpp src/utils.cpp src/input_parser.cpp
calib_obj = $(calib_cpp_src:.cpp=.o) $(calib_c_src:.c=.o)

## Files to remove when cleaning.
clean_obj = $(wildcard src/*.o)

## Compiler and linker flags.
CFLAGS = -Wall -O3
CPPFLAGS = -Wall -Wno-psabi -O3
LDFLAGS = -pthread -lstdc++fs -ldl -li2c -larwain_torch

## Rule for compiling C++ files.
.cpp.o:
	$(CXX) -c $(INCL) $(CPPFLAGS) -o $@ $< $(LDFLAGS)

## Rule for compiling C files.
.c.o:
	$(CXX) -c $(INCL) $(CFLAGS) -o $@ $< $(LDFLAGS) 

all: arwain calib

## Build main ARWAIN program.
arwain: $(arwain_obj)
	$(CXX) -o build/$@ $^ $(INCL) $(LIBS) $(CPPFLAGS) $(LDFLAGS) -O3

## Build calibration tool.
calib: $(calib_obj)
	$(CXX) -o build/$@ $^ $(CPPFLAGS) $(LDFLAGS)

## Remove all objects and binaries.
clean:
	-@rm $(clean_obj) 2> /dev/null || true
	-@rm build/arwain 2> /dev/null || true
	-@rm build/calib 2> /dev/null || true
