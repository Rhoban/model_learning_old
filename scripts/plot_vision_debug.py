import csv
import matplotlib
import matplotlib.pyplot as plt
from scipy.linalg import norm
import numpy as np

import sys

argv=sys.argv
if len(argv)<1:
    print("Usage : %s path_to_results.csv"%argv[0])
    sys.exit(1)
else:
    path_to_results=argv[1]

observations = []
predictions = []
errors = []
id_tags = []
with open(path_to_results) as f:
    reader = csv.reader(f, delimiter=",")
    for row in reader:
        if not "#" in row[0] and not "m" in row[0]:
            id_tags.append(int(row[0]))
            observations.append(np.array([float(row[1]),float(row[2])]))
            predictions.append(np.array([float(row[3]),float(row[4])]))
            errors.append(norm(observations[-1]-predictions[-1]))
    f.close()

X = [obs[0] for obs in observations]
Y = [obs[1] for obs in observations]

sc = plt.scatter(X,Y,c=errors)
plt.colorbar(sc)
plt.show()
