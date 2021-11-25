# import time
# import math as mth
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
def add_drift(angle, element):
    e = math.exp(1j * (angle * math.pi/180))
    element = element * e
    return element

def into_real(x):
    for i, val in enumerate(x):
        x[i] = [val.real, val.imag]
    return x

# simulates drift - requires an array of points and the angle of drift to be applied
def simulate_drift(x, angle):
    x = into_complex(x)
    new_points = [x[0]]
    for i in range(1, len(x)):
        pt = x[i] - x[i-1]
        pt = add_drift(i*angle, pt)
        pt = pt + new_points[i-1]
        new_points.append(pt)

    new_points = into_real(new_points)
    return new_points

# # simple example
# # points = [[1, 1], [2, 2], [3, 3], [4, 4], [5, 5]]
# # points = [[i, i] for i in range(50)]
# points = [
#     [mth.sin(i/100.0), mth.cos(i/100.0)] for i in range(1, 36000)
# ]

# plt.scatter(
#    [pt[0] for pt in points],
#    [pt[1] for pt in points],
#    label="Original path"
# )

# start = time.time()
# drifted_points = simulate_drift(points, 0.1)
# end = time.time()
# print(end - start)


# plt.scatter(
#    [pt[0] for pt in drifted_points],
#    [pt[1] for pt in drifted_points],
#    label="Drifted path"
# )
# plt.axis("equal")
# plt.legend()
# plt.savefig("asdfasdf.png")
