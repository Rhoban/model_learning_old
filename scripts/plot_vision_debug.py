#! /usr/bin/env python
import csv
import matplotlib.pyplot as plt
from scipy.linalg import norm
import numpy as np

import sys

argv = sys.argv
if len(argv) < 1:
    print("Usage : %s path_to_results.csv" % argv[0])
    sys.exit(1)
else:
    path_to_results = argv[1]

res = {}

with open(path_to_results) as f:
    reader = csv.reader(f, delimiter=",")
    for row in reader:
        if "#" not in row[0] and "m" not in row[0]:
            id_tag = int(row[0])
            if id_tag not in res:
                res[id_tag] = {}
                res[id_tag]["observations"] = []
                res[id_tag]["predictions"] = []
                res[id_tag]["diffs"] = []
                res[id_tag]["errors"] = []

            res[id_tag]["observations"].append(
                np.array([float(row[1]), float(row[2])]))
            res[id_tag]["predictions"].append(
                np.array([float(row[3]), float(row[4])]))

            observation = res[id_tag]["observations"][-1]
            prediction = res[id_tag]["predictions"][-1]
            res[id_tag]["diffs"].append(prediction - observation)
            res[id_tag]["errors"].append(norm(observation - prediction))
    f.close()

fig, ax = plt.subplots(nrows=len(res), ncols=2, figsize=(13, 10*len(res)))

i = -1
for id_tag in res:
    i += 1
    observations = res[id_tag]["observations"]
    predictions = res[id_tag]["predictions"]
    diffs = res[id_tag]["diffs"]
    errors = res[id_tag]["errors"]

    tmp = ax[i][0]
    test = tmp.scatter([diff[0] for diff in diffs],
                       [diff[1] for diff in diffs],
                       c=errors)
    tmp.axis('equal')
    tmp.set_title("Tag %d" % id_tag)
    tmp.set_aspect('equal', adjustable='box')
    fig.colorbar(test, ax=tmp)

    tmp = ax[i][1]
    test = tmp.quiver([obs[0] for obs in observations],
                      [obs[1] for obs in observations],
                      [diff[0] for diff in diffs], [diff[1] for diff in diffs],
                      errors,
                      angles='xy',
                      scale=0.1,
                      scale_units='xy',
                      width=0.002,
                      headwidth=4,
                      headlength=1.3)
    tmp.axis([0, 644, 0, 482])
    tmp.set_aspect('equal', adjustable='box')
    tmp.set_title("Tag %d" % id_tag)
    fig.colorbar(test, ax=tmp)

fig.suptitle('Observation : center\nPrediction : points\nScale of arrow = 10')
plt.show()
