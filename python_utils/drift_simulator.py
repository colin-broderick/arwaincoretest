import math
import cmath

def closed_path_error(points):
    return math.sqrt((points[0][0] - points[-1][0])**2 + (points[0][1] - points[-1][1])**2)

def sign(number):
    return number >= 0

def find_best_drift_correction(points):
    """
    Assuming a closed path, find the artificial drift which best closes the path.
    """
    error = closed_path_error(points)
    switches = 5
    drift = 1.0/60.0/20.0
    drift_delta = 1.0/60.0/20.0
    print("Start error:", error)
    first_try = True
    print("Drift:", drift, "    New error:", "N/A", "     delta:", drift_delta)

    while switches > 0:
        new_points = simulate_drift(points, drift)
        new_error = closed_path_error(new_points)
        if new_error > error:
            if first_try:
                drift *= -1
                drift_delta *= -1
            else:
                drift_delta /= -2.0
                drift += drift_delta
                switches -= 1
        elif new_error < error:
            error = new_error
            drift += drift_delta
        first_try = False
        print("Drift:", drift, "    New error:", new_error, "     delta:", drift_delta)

    return new_points, drift*60*20

def into_complex_single(n):
    n = n[0] + n[1] * 1j
    return n

def into_complex(n):
    new_list = []
    for i, val in enumerate(n):
        new_list.append(into_complex_single(val))
    return new_list

# adds dift to an array of positions in - needs to be given the angle of drift in degrees
def add_drift(angle, element):
    e = cmath.exp(1j * (angle * cmath.pi/180))
    element = element * e
    return element

def into_real(x):
    for i, val in enumerate(x):
        x[i] = [val.real, val.imag]
    return x

# simulates drift - requires an array of points and the angle of drift to be applied
def simulate_drift(x, angle):
    y = into_complex(x)
    new_points = [y[0]]
    for i in range(1, len(y)):
        pt = y[i] - y[i-1]
        pt = add_drift(i*angle, pt)
        pt = pt + new_points[i-1]
        new_points.append(pt)

    new_points = into_real(new_points)
    return new_points
