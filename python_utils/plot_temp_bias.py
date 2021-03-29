import sys
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    data = np.genfromtxt(
        sys.argv[1],
        delimiter=" ",
        skip_header=1,
        skip_footer=1
    )
    plt.plot((data[:,1]-39)*0.01, label="temp")
    plt.plot(data[:,2], label="gyro-x")
    plt.plot(data[:,3], label="gyro-y")
    plt.plot(data[:,4], label="gyro-z")
    plt.legend()
    plt.savefig("temp-gyro-bias.png")
    