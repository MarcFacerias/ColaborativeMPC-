import numpy as np
import pandas as pd
import numpy

it=10
N=10
data = pd.read_csv("trajectory_line.csv")
x = data.loc[:, "x"].to_numpy(copy=True)[:,numpy.newaxis]
y = data.loc[:, "y"].to_numpy(copy=True)[:,numpy.newaxis]
th = data.loc[:, "th"].to_numpy(copy=True)[:,numpy.newaxis]

refs = numpy.hstack((x,y,th))

print(refs[it:it + N,:])
print(refs.shape)

a = []

print(np.hstack([a,[3,3,3]]))