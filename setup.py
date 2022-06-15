import os
import sys

if not os.name == 'nt' :
    raise ImportError("This script is only meant for Windows")

pylib = os.path.join(os.path.dirname(sys.executable), os.path.join('Lib', 'site-packages'))
realpth = os.path.dirname(os.path.realpath(__file__))

print("'Installing' Fake_ROS in : {}".format(pylib))

print(os.path.join(pylib, 'fake_ros.pth'))


p = os.path.join(pylib, 'fake_ros.pth')
f = open(p, "w")
f.write("import sys; sys.path.append('{}')".format(realpth).replace(os.sep, 2*os.sep))
f.close()
