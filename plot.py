#!/usr/bin/python3

from unicodedata import name
import numpy as np
import matplotlib.pyplot as plt

data = []
with open("output.csv", encoding="utf-8-sig") as f:
    lines = f.readlines()[1:]

    for line in lines:
        data.append( line.strip().split(", ") )

data = np.array(data, dtype=float)
plt.plot(data[:, 1], data[:, 2], 'r', label="Deviation")
plt.plot(data[:, 1], data[:, 3], 'y', label="Controller Output")
plt.plot(data[:, 1], data[:, 4], 'b', label="Plant Output")
plt.legend()

plt.savefig("output.png")
