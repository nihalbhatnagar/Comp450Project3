import numpy
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from mpl_toolkits.mplot3d import Axes3D
data = numpy.loadtxt('project3/src/boxPath.txt')
fig = plt.figure()
ax = fig.gca()
print(data)
ax.plot(data[:,0],data[:,1],'.-')
#For box plot
for dataArr in data:
    plt.annotate(dataArr[2], # this is the text
                 (dataArr[0],dataArr[1]), # these are the coordinates to position the label
                 textcoords="offset points", # how to position the text
                 xytext=(0,.02), # distance from text to points (x,y)
                 ha='center')
ax.add_patch(Rectangle((1.0, 2.0), 1.5, 1))
ax.add_patch(Rectangle((0, 1), .5, .5))
ax.add_patch(Rectangle((1, .5), .3, .5))
ax.add_patch(Rectangle((-1.5, 1), 1, .75))
ax.add_patch(Rectangle((.8, -1.5), 1, .5))
plt.show()