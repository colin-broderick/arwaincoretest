import math
import sys
import numpy as np
import matplotlib.pyplot as plt


def AllanDeviation(data_array, sample_frequency, maxNumM=100):
    """
    Compute the Allan deviation (sigma) of time-series data.

    Algorithm obtained from Mathworks:
    https://www.mathworks.com/help/fusion/ug/inertial-sensor-noise-analysis-using-allan-variance.html

    Args
    ----
        data_array (numpy.ndarray): 1D data array
        sample_frequency (int, float): Data sample frequency in Hz
        maxNumM (int): Number of output points
    
    Returns
    -------
        (taus, allanDev): Tuple of results
        taus (numpy.ndarray): Array of tau values
        allanDev (numpy.ndarray): Array of computed Allan deviations
    """
    dt = 1.0 / sample_frequency
    N = len(data_array)
    Mmax = 2**np.floor(np.log2(N / 2))
    M = np.logspace(np.log10(1), np.log10(Mmax), num=maxNumM)
    M = np.ceil(M)  # Round up to integer
    M = np.unique(M)  # Remove duplicates
    taus = M * dt  # Compute 'cluster durations' tau

    # Compute Allan variance
    allanVar = np.zeros(len(M))
    for i, mi in enumerate(M):
        twoMi = int(2 * mi)
        mi = int(mi)
        allanVar[i] = np.sum(
            (data_array[twoMi:N] - (2.0 * data_array[mi:N-mi]) + data_array[0:N-twoMi])**2
        )
    
    allanVar /= (2.0 * taus**2) * (N - (2.0 * M))
    return (taus, np.sqrt(allanVar))  # Return deviation (dev = sqrt(var))


if __name__ == "__main__":
    try:
        DATA_FILE = sys.argv[1]
    except IndexError:
        print("Supply a data file name")
        sys.exit(1)

    sample_frequency = 200

    data = np.genfromtxt(DATA_FILE, skip_footer=4)
    dt = 1.0 / sample_frequency

    gx = data[:, 1] * 180.0 / np.pi
    gy = data[:, 2] * 180.0 / np.pi
    gz = data[:, 3] * 180.0 / np.pi

    thetax = np.cumsum(gx) * dt
    thetay = np.cumsum(gy) * dt
    thetaz = np.cumsum(gz) * dt

    taux, adx = AllanDeviation(thetax, sample_frequency, maxNumM=200)
    tauy, ady = AllanDeviation(thetay, sample_frequency, maxNumM=200)
    tauz, adz = AllanDeviation(thetaz, sample_frequency, maxNumM=200)

    plt.figure()
    plt.title("Allan Deviations")
    plt.plot(taux, adx, label='gx')
    plt.plot(tauy, ady, label='gy')
    plt.plot(tauz, adz, label='gz')
    plt.xlabel(r'$\tau$ [sec]')
    plt.ylabel('Deviation [deg/sec]')
    plt.grid(True, which="both", ls="-", color='0.65')
    plt.legend()
    plt.xscale('log')
    plt.yscale('log')
    plt.savefig("allan.png")
