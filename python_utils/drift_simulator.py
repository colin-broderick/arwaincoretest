# import matplotlib.pyplot as plt
import cmath as math


def into_complex_single(n):
    n = n[0] + n[1] * 1j
    return n


def into_complex(n):
    for i, val in enumerate(n):
        n[i] = into_complex_single(val)
    return n


# adds dift to an array of positions in - needs to be given the angle of drift in degrees
def add_drift_one_array(angle, element):
    e = math.exp(1j * (angle * math.pi/180))
    for i, val in enumerate(element):
        element[i] = element[i] * e
    return element


def add_drift(angle, x):
    for i, val in enumerate(x):
        x[i:] = add_drift_one_array(angle, x[i:])
    return x


def into_real(x):
    for i, val in enumerate(x):
        x[i] = [val.real, val.imag]
    return x

# simulates drift - requires an array of points and the angle of drift to be applied
def simulate_drift(x, angle):
    x = into_complex(x)
    x = add_drift(angle, x)
    x = into_real(x)
    return x


# simple example
#points = [[1, 1], [2, 2], [3, 3], [4, 4], [5, 5]]
# print(points)
#points = simulate_drift(points, 1)
# print(points)
# plt.scatter(
#    [pt[0] for pt in points],
#    [pt[1] for pt in points]
# )
# plt.show()
