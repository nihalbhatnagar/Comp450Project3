import numpy
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from mpl_toolkits.mplot3d import Axes3D
data = numpy.loadtxt('project3/src/boxPath.txt')
fig = plt.figure()
ax = fig.gca()
ax.plot(data[:,0],data[:,1],'.-')
#For box plot
for dataArr in data:
    plt.annotate(dataArr[2], # this is the text
                 (dataArr[0],dataArr[1]), # these are the coordinates to position the label
                 textcoords="offset points", # how to position the text
                 xytext=(0,.02), # distance from text to points (x,y)
                 ha='center')
ax.add_patch(Rectangle((-1, -1), .5, .5))
ax.add_patch(Rectangle((-.25, -.25), .2, .2))
ax.add_patch(Rectangle((0, 0), .3, .3))
ax.add_patch(Rectangle((0.4,.4), .35, .35))
ax.add_patch(Rectangle((.9, .9), 1, 1))
ax.add_patch(Rectangle((2.5, 2.3), .5, .2))
# ax.add_patch(Rectangle((16, 8), 6, 7))

plt.show()