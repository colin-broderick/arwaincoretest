import sys
import numpy as np
import matplotlib.pyplot as plt

"""
Plots the file produced by running ./build/arwain_calibration -temp
"""

def alt(folder):
    temperature = np.genfromtxt(folder + "imu_temperature.txt")
    gyro = np.genfromtxt(folder + "gyro.txt", skip_footer=1, skip_header=1, delimiter=" ")

    gx_avgs = list()
    for i in range(0, gyro.shape[0]-100, 200):
        gx_avgs.append(np.mean(gyro[i:i+100,1]))
    # plt.plot(gx_avgs, label="gyro")
    # plt.plot((temperature[:,1]/40-1)/100, label="temp")
    min_length = min(np.array(gx_avgs).shape[0], ((temperature[:,1]/40-1)/100).shape[0])
    plt.scatter(gx_avgs[:min_length], (temperature[:min_length,1]), s=1)
    plt.scatter(0, 39, s=1)
    plt.scatter(0, 42.5, s=1)
    plt.legend()
    plt.savefig("temp-gyro-bias.png")

if __name__ == "__main__":
    alt(sys.argv[1])

    # data = np.genfromtxt(
    #     sys.argv[1],
    #     delimiter=" ",
    #     skip_header=1,
    #     skip_footer=1
    # )
    # plt.plot((data[:,1]-39)*0.01, label="temp")
    # plt.plot(data[:,2], label="gyro-x")
    # plt.plot(data[:,3], label="gyro-y")
    # plt.plot(data[:,4], label="gyro-z")
    # plt.legend()
    # plt.savefig("temp-gyro-bias.png")
    