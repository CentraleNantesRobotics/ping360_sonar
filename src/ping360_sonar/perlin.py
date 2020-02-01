import math
import random


def rand():
    M = 4294967296.0
    # a-1 should be divisible by m's prime factors
    A = 1664525.0
    # c and m should be co-prime
    C = 1.0
    Z = math.floor((M * random.random()))
    Z = (A * Z + C) % M
    return Z / M - 0.5


def interpolate(pa, pb, px):
    ft = px * math.pi
    f = (1 - math.cos(ft)) * 0.5
    return pa * (1 - f) + pb * f


def noise(w, amplitude, wavelength):
    x = 0
    amp = amplitude * 1.0
    wl = wavelength * 1.0  # wavelength
    a = random.random()
    b = random.random()
    noise_list = []

    while(x < w):
        if(x % wl == 0):
            a = b
            b = random.random()
            y = a * amp
        else:
            y = interpolate(a, b, (x % wl) / wl) * amp
        noise_list.append(y)
        x += 1

    return noise_list
