#!/usr/bin/env python3

""" depth of field measurements
preliminary CSLICS DoF measurements using remote_focus.py and visual inspection of a focus target
"""

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# focus,far,near
# also do mid

data = pd.read_csv('focus_observations.csv')
data['focus'] = pd.to_numeric(data['focus'])
data['far'] = pd.to_numeric(data['far'])

data['near'] = pd.to_numeric(data['near'])
data['mean'] = (data['far'] + data['near'])/2

# expect that the lowest value is "far" (given that CSLICS was pointing down)
# so subtract smallest value of 'far' to zero the plot
min_far = min(data['far'])
data['far'] = data['far'] - min_far
data['near'] = data['near'] - min_far
data['mean'] = data['mean'] - min_far

plt.figure()
plt.plot(data['focus'], data['far'], label='Far', marker='o')
plt.plot(data['focus'], data['near'], label='Near', marker='*')
plt.plot(data['focus'], data['mean'], label='Mean', marker='^')

plt.xlabel('Focus Setting')
plt.ylabel('Near, Far Values [mm]')
plt.title('Focus vs Near and Far')
plt.legend()
plt.grid()


# and actually, what we want is the difference between near and far:
data['dif'] = data['near'] - data['far']

plt.figure()
plt.plot(data['focus'], data['dif'], label='dif', marker='o')
plt.xlabel('Focus Setting')
plt.ylabel('Dif of Near, Far DOF [mm]')
plt.title('Diff of DoF vs Focus setting')
plt.legend()
plt.grid()

# we typically want to be operating at the point of maximum depth of field range
# so 0 focus setting (assuming the 200 spike was a bit of a misread)
# assumes 0 focus setting allows us to get more into the water column




plt.show()