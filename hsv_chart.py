import numpy as np
import pylab as pl
from matplotlib.colors import hsv_to_rgb

S, H = np.mgrid[0:1:30j, 0:1:90j]
V = np.ones_like(S)
HSV = np.dstack((H,S,V))
RGB = hsv_to_rgb(HSV)
pl.imshow(RGB, origin="lower", extent=[0, 360, 0, 1], aspect=180)
pl.xlabel("$H$")
pl.ylabel("$S$")
pl.title("$V_{HSV}=1$")
pl.show()
