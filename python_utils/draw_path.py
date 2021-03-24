import matplotlib.pyplot as plt
import numpy as np
import sys

if __name__ == "__main__":
    """
    Draw a path using an arwain-style position log file.
    Pass in file as command line argument.
    """
    path = np.genfromtxt(sys.argv[1], delimiter=" ", skip_header=1, skip_footer=1)
    plt.scatter(path[:,1], path[:,2])
    plt.savefig("drawnpath.png")
