arwain_src = src/main.cpp \
	src/quaternions.cpp \
  	src/stance.cpp \
   	src/bmi2.c \
    src/bmi270.c \
	src/bmm150.c \
	src/imu_utils.cpp \
	src/utils.cpp \
	src/madgwick.cpp \
	src/efaroe.cpp \
	src/math_util.cpp \
	src/bin_log.cpp \
	src/input_parser.cpp
arwain_obj = $(arwain_src:.cpp=.o)

calib_src = src/calibrate_bmi270.cpp \
	src/bmi2.c \
	src/bmi270.c \
	src/imu_utils.cpp \
	src/bmm150.c
calib_obj = $(calib_src:.cpp=.o)

clean_obj = $(wildcard src/*.o)

CFLAGS = -std=c++17 -Wall -Wno-psabi
LDFLAGS = -pthread -lstdc++fs -ldl -li2c

# %.o: %.cpp
.cpp.o:
	$(CXX) -c $(CFLAGS) $(LDFLAGS) -o $@ $<

## Build main ARWAIN program.
arwain: $(arwain_obj)
	$(CXX) -o build/$@ $^ $(CFLAGS) $(LDFLAGS)

## Build calibration tool.
calib: $(calib_obj)
	$(CXX) -o build/$@ $^ $(CFLAGS) $(LDFLAGS)

## Remove all objects and binaries.
clean:
	-@rm $(clean_obj) 2> /dev/null || true
	-@rm build/arwain 2> /dev/null || true
	-@rm build/calib 2> /dev/null || true
