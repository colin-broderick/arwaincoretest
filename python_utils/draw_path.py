import matplotlib.pyplot as plt
import numpy as np
import quaternion
import math
import sys

import plot_quaternions

if __name__ == "__main__":
    """
    Draw a path using an arwain-style position log file.
    Pass in file as command line argument.
    """
    path = np.genfromtxt(sys.argv[1], delimiter=" ", skip_header=1, skip_footer=1)
    
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2)

    ax1.scatter(path[:,1], path[:,2], s=1, label="(x, y)")
    ax2.scatter(path[:,1], path[:,3], s=1, label="(y, z)")
    ax3.scatter(path[:,2], path[:,3], s=1, label="(x, z)")
    ax4.plot(path[:,3], linewidth=1, label="z(t)")

    # headings, _ = plot_quaternions.process([sys.argv[1][:-12]+"game_rv.txt"], 1, "z")

    # for i in range(len(headings)):
    #     ax4.plot(180/math.pi*(headings[i]-headings[i][0]), linewidth=1, label="heading")

    for each in [ax1, ax2, ax3, ax4]:
        # each.set_aspect("equal")
        each.legend()
    fig.tight_layout()
    
    plt.savefig("drawnpath.png")
