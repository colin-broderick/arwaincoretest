## Purpose

This repository is for development and testing of the ARWAIN C++ rewrite.

It has been written using only the C++ standard library so should be portable to various hardware with very little work.

It is almost feature-complete but no features have been formally tested. Missing feautres are
1. NPU inference, though NCS2 inference is partially written
2. LoRa communication

## Build
To build for the Raspberry Pi or similar ARM architectures:
```
mkdir build
./buildall pi
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
Code currently assumes running from the repository's root directory, so you'll need to specify `-conf` and `-calib` if you run from elsewhere.

## Visualisation
A simply Python visualisation script is included, to aid in testing and development of orientation filters. VPython is required.
```
pip3 install vpython
```
To run it, pipe the output from the main program into this python script, e.g.
```
./build/arwain -lstd | ./visualisation.py
```
Other arguments can optionally be passed by `-lstd` is required.
