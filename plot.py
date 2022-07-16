#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

data = []
with open("output.csv", encoding="utf-8-sig") as f:
    lines = f.readlines()[1:]

    for line in lines:
        data.append( line.strip().split(", ") )

data = np.array(data, dtype=float)
plt.plot(data[:, 1], data[:, 2])
plt.savefig("output.png")
