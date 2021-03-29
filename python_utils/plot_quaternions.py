#!/usr/bin/python3

import io
import sys
import math
import numpy as np
import matplotlib.pyplot as plt
import quaternion


def q2e(w, x, y, z):
    roll = math.atan2(w*x + y*z, 0.5 - x*x - y*y)
    pitch = math.asin(-2.0 * (x*z - w*y))
    yaw = math.atan2(x*y + w*z, 0.5 - y*y - z*z)
    return roll, pitch, yaw


def q2eWithProjection(w, x, y, z, axis=quaternion.quaternion(0,0,0,1)):
    q = quaternion.quaternion(w, x, y, z)

    ## This creates a quaternion-form 3-vector representing the way the z-axis of the device is pointing.
    v = q * axis * q.conjugate()

    ## If, say, the device z-axis is mostly now in the world xy plane, this finds the of the vector in that plane.
    return math.atan2(v.x, v.y)


def createHeadingArray(quaternionArray, headingAxis=quaternion.quaternion(0, 0, 0, 1)):
    headings = list()
    i = startingIndex
    for d in quaternionArray:
        heading = q2eWithProjection(d[0], d[1], d[2], d[3], axis=headingAxis)
        headings.append(heading)
    return np.array(headings)


def disagreement(array1, array2, startingIndex=0):
    quantity = min(array1.shape[0], array2.shape[0])
    dotproduct = [0]*quantity

    rot1 = quaternion.from_float_array(array1) * quaternion.quaternion(0, 0, 1, 0) * quaternion.from_float_array(array1)
    rot2 = quaternion.from_float_array(array2) * quaternion.quaternion(0, 0, 1, 0) * quaternion.from_float_array(array2)

    min_size = min(rot1.shape[0], rot2.shape[0])

    dots = quaternion_dot(rot1[:min_size], rot2[:min_size])

    return np.arccos(dots)*180.0/math.pi


def quaternion_dot(array1, array2):
    dots = list()
    for a, b in zip(array1, array2):
        dots.append(a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z)
    return np.array(dots)


def smooth(data, weight=0.99):
    new = list()
    new.append(data[0])
    for i in range(1, data.shape[0]):
        new.append(
            weight*new[-1] + (1.0-weight)*data[i]
        )
    return np.array(new)


def process(files, startingIndex, axis):
    datasets = list()
    for f in files:
        with open(f, "rb") as ff:
            s = io.BytesIO(ff.read().replace(b' ', b','))
        data = np.genfromtxt(
                f,
                delimiter=" ",
                skip_footer=10,
                skip_header=10,
                dtype=float
            )[:, startingIndex:startingIndex+4]
        datasets.append(
            data
        )
    
    headings = list()
    for data in datasets:
        headings.append( smooth(np.unwrap(createHeadingArray(data, axis))) )

    fig, (a0, a1) = plt.subplots(2, 1, gridspec_kw={"height_ratios":[3,1]})

    for i in range(len(headings)):
        a0.plot(180/math.pi*(headings[i] - headings[i][0]), linewidth=1, label=files[i])
    
    for i in range(1, len(datasets)):
        dis = smooth(disagreement(datasets[0], datasets[i], startingIndex))
        a1.plot(dis - dis[0], label=f"Disagreement between files 1 and {i+1}")

    plt.tight_layout()

    a0.set_ylabel("Heading /degrees")
    a1.set_ylabel("Disagreement /degrees")

    a1.set_xlabel("time")

    a0.legend()
    a1.legend()
    # plt.savefig("plot.png")
    plt.show()


if __name__ == "__main__":
    """
    First command line argument is starting index of quaternion in csv
    Subsequent commands are file names
    """
    ## Check suitable arguments are passed.
    if len(sys.argv) < 3:
        print("Usage:")
        print("  ./plotquaternions.py <starting_index, e.g. 0> <axis_to_rotate, e.g. y> <filenames...>")
        sys.exit(1)

    ## Index of column where quaternion starts.
    startingIndex = int(sys.argv[1])

    ## Which axis to rotate with the quaternion.
    axis = {
        "x": quaternion.quaternion(0, 1, 0, 0),
        "y": quaternion.quaternion(0, 0, 1, 0),
        "z": quaternion.quaternion(0, 0, 0, 1)
    }[sys.argv[2]]

    ## List of files to process.
    files = sys.argv[3:]

    ## Go.
    process(files, startingIndex, axis)
