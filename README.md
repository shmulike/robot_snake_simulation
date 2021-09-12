
# Hyper redundant articulated robot for NDT of close complex structures


by Edelman Shmulik

PhD student in Robotics & Control (ROS) at Ben-Gurion University, Israel.

Email: Shmulike@post.bgu.ac.il

## Abstract
Composite aircraft structures have become increasingly widespread in civil and
military aircrafts and drones. It will also be used in the vehicles industry in the
future as well. Composite materials are so named because they consist of two or
more materials. The composites used in aircraft consist of fibers suspended in
a matrix of epoxy resin. For example, the Boeing 787 Dreamliner was the first
commercial airplane to be constructed from 50% composite materials. We can
say that even though the composite materials have revolutionized the aviation
industry, but their use does present some engineering, manufacturing and main-
tenance challenges. One of the most common problems in the use of composite
material is the air bubbles between the layers that can form during the manu-
facture or assembly of the structures. These bubbles can cause structural failure
and endanger the aircraft in the long run. The current methods for detecting the
bubbles are done manually or by using robotic systems. The first method is done
by the workers who hold the sensor with their hands. With this method, it’s
very difficult to inspect large structures with limited access panels. The second
method; using robotic systems, is used only on the outer structure of the compo-
nents or before the assembly operation, when the robotic manipulators can still
reach the required areas.
The purpose of this research proposal is to design a novel and unique robotic
system which will help with NDT (Non-Destructive Tests) of closed and complex
structures, like aircraft’s wings. The robotic system will consist of several links
and joints that can be controlled separately for maximum functionality. Since we
need it to be able to pass through narrow passages, the links have to be as small
as possible in diameter and still be able to carry the sensors and other equipment
at the end. As a result, we will move the actuators to the system’s base and
make it tendon driven. Each joint will be controlled with two strings, one for
the position, and the second for stiffness and keeping tension. The actuators are
lead-screw mechanisms that can pull or release the strings. The main challenges
of this research are the required system accuracy for such tests and the motion
planning that requires the sensor to be in full contact with the inspected surface
during the scan.
In the last two years, we have developed the mechanical system and developed
a simple control system for a 4-link prototype. The current control system showed
good results for controlling the joints and end-effector position. Further research
will focus on upgrading the control system for reducing error, increasing system
accuracy, and developing a macro and micro motion planning algorithm for two-
step scanning. The macro-motion planning to be designed for the main robotic
system will help to navigate between the required areas, while the micro-motion
planning designed will be for optimizing the end-effector scanning path.

## Installation
This package was developed and tested in ROS-noetic + Python3

This package requires the next ROS packages to be installed:

```sh
sudo apt-get install rosserial
$ sudo apt-get install ros-noetic-joy


```

This package requires Python3 and the the next Python packages to be installed:

- [numpy] - numpy
- [scipy] - scipy

## How to run

Dillinger is currently extended with the following plugins.

## License

Shmulik Edelman


[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)

   [dill]: <https://github.com/joemccann/dillinger>
   [git-repo-url]: <https://github.com/joemccann/dillinger.git>
   [john gruber]: <http://daringfireball.net>
   [df1]: <http://daringfireball.net/projects/markdown/>
   [markdown-it]: <https://github.com/markdown-it/markdown-it>
   [Ace Editor]: <http://ace.ajax.org>
   [node.js]: <http://nodejs.org>
   [Twitter Bootstrap]: <http://twitter.github.com/bootstrap/>
   [jQuery]: <http://jquery.com>
   [@tjholowaychuk]: <http://twitter.com/tjholowaychuk>
   [express]: <http://expressjs.com>
   [AngularJS]: <http://angularjs.org>
   [Gulp]: <http://gulpjs.com>

   [PlDb]: <https://github.com/joemccann/dillinger/tree/master/plugins/dropbox/README.md>
   [PlGh]: <https://github.com/joemccann/dillinger/tree/master/plugins/github/README.md>
   [PlGd]: <https://github.com/joemccann/dillinger/tree/master/plugins/googledrive/README.md>
   [PlOd]: <https://github.com/joemccann/dillinger/tree/master/plugins/onedrive/README.md>
   [PlMe]: <https://github.com/joemccann/dillinger/tree/master/plugins/medium/README.md>
   [PlGa]: <https://github.com/RahulHP/dillinger/blob/master/plugins/googleanalytics/README.md>
   [numpy]:<https://numpy.org/>
   [scipy]:<>

