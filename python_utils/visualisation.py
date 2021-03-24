#!/usr/bin/env python3

"""
Pipe the output from arwain -lstd into this process to launch a vpython
visualization of DCM orientation and/or position, e.g.

    ./arwain -lstd | ./visualisation.py
    ./arwain -lstd -noinf | ./visualisation.py

For testing purposes only. Requires vpython:
    
    pip3 install vpython

"""

import sys
import time
import math
from vpython import *


if __name__ == "__main__":
    """
    This randomly stopped working one day and I don't know why. The script freezes when vpython tries to create a box.
    """

    ## Create canvas with size:
    scene = canvas(width=600, height=400)

    ## Set camera at oblique angle for perspective.
    scene.camera.pos = vector(0, -50, 30)
    scene.camera.axis = vector(0, 50, -30)

    ## Draw the floor
    fl = box(
        pos=vector(0, 0, -10),
        color=vector(0.5, 0.5, 0.5),
        height=200,
        width=1,
        length=200,
    )

    ## Build box.
    bx = box(
        color=vector(1,0,0),
        opacity=0.5,
        shininess=1,
        emissive=False,
        height=1,
        width=6,
        length=3,
        make_trail=True,
        retain=500
    )

    ## Define rotation axes.
    X = vector(1, 0, 0)
    Y = vector(0, 1, 0)
    Z = vector(0, 0, 1)

    rad_angles = [0, 0, 0]
    first_angles = None

    ## Initial offset angle.
    bx.rotate(angle=math.pi/2, axis=X)

    for line in sys.stdin:
        if "Orientation (E)" in line:
            print("sdf")
            bx.rotate(angle=-rad_angles[2], axis=Z)
            bx.rotate(angle=-rad_angles[1], axis=Y)
            bx.rotate(angle=-rad_angles[0], axis=X)

            ## Parse angles from the orientation string.
            deg_angles = [float(angle) for angle in line[18:-2].split(",")]
            rad_angles = [angle*pi/180 for angle in deg_angles]
            
            if first_angles is None:
                first_angles = deg_angles[:]

            ## Rotate by the change in angle.
            bx.rotate(angle=rad_angles[0], axis=X)
            bx.rotate(angle=rad_angles[1], axis=Y)
            bx.rotate(angle=rad_angles[2], axis=Z)

        if "Position:        " in line:
            x, y, z = line[18:-2].split(",")
            bx.pos = vector(float(x)*10, float(y)*10, float(z)*10)
