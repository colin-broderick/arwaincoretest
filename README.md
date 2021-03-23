## Purpose

This repository is for development and testing of the ARWAIN C++ rewrite.

It has been written using only the C++ standard library so should be portable to various hardware with very little work.

It is almost feature-complete but no features have been formally tested. Missing feautres are

1. NPU inference, though NCS2 and RKNN inference is partially written. Inference can also be done by Python NCS2 with data communicated between this software and the Python script via socket.

2. LoRa communication. Tranmission is implemented using an SX127x but not fully developed and not fully tested.

## Build

### Requirements
1. Need pytorch installed. At time of writing the following wheel will work, but you may have to source your own.
```
wget https://wintics-opensource.s3.eu-west-3.amazonaws.com/torch-1.3.0a0%2Bdeadc27-cp37-cp37m-linux_armv7l.whl
pip3 install torch-1.3.0a0+deadc27-cp37-cp37m-linux_armv7l.whl
```
Other python requirements can be installed by
```
pip3 install -r ./python_utils/requirements.txt
```

2. Some apt packages:
```
sudo apt install i2c-tools libzmq3-dev libi2c-dev cmake
```

3. Enable I2C and SPI via `raspi-config`. You may need to restart to enable complete enablement of SPI.

4. Clone repository
```
git clone https://bitbucket.org/arwain/arwain_inference_core
```


### Building
To build for the Raspberry Pi or similar ARM architectures:
```
cd ~/arwain_inference_core
mkdir build lib
cmake .
make -j4
```

## Run
To run with just console output
```
./build/arwain -lstd
```
For other options run
```
./build/arwain -h
```
Code currently assumes running from the repository's root directory, so you'll need to specify `-conf` if you run from elsewhere. While testing I'd recommend running with `-noinf` for now since it will use a lot of CPU time.

## Visualisation
A simple Python visualisation script is included, to aid in testing and development of orientation filters. VPython is required. This will track orientation, and also position if inference is running. It fails half the time but I'm 99.9% sure it's a vpython bug, and nothing to do with arwain.
```
pip3 install vpython
```
To run it, pipe the output from the main program into this python script, e.g.
```
./build/arwain -lstd | ./visualisation.py
```
Other arguments can optionally be passed by `-lstd` is required.
