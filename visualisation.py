#!/usr/bin/env python3

"""
Pipe the output from arwain -lstd into this process to launch a vpython
visualization of DCM orientation, e.g.

    ./arwain -lstd | ./visualisation.py

For testing visualization only. Requires vpython:
    
    pip3 install vpython

"""

import sys

from vpython import *

## Build box.
bx = box(
    color=vector(1,0,0),
    opacity=0.5,
    shininess=1,
    emissive=False,
    height=1,
    width=6,
    length=3
)

## Define rotation axes.
X = vector(1, 0, 0)
Y = vector(0, 1, 0)
Z = vector(0, 0, 1)

angles = [0, 0, 0]

for line in sys.stdin:
    if "Orientation (E)" in line:
        bx.rotate(angle=-angles[2], axis=Y)
        bx.rotate(angle=-angles[1], axis=Z)
        bx.rotate(angle=-angles[0], axis=X)

        ## Parse angles from the orientation string.
        angles = [float(angle)*pi/180 for angle in line[18:-2].split(",")]

        ## Rotate by the change in angle.
        bx.rotate(angle=angles[0], axis=X)
        bx.rotate(angle=angles[1], axis=Z)
        bx.rotate(angle=angles[2], axis=Y)

        print(angles)
