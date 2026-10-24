# ConveyorSorter
Arduino IoT Task - 1 for prodex - automatic sorting of objects according to their lengths.

I have used a mechanism where a conveyor belt is constantly run at a constant slow speed by a DC motor. A ping ultrasonic sensor detetcs if an object appears in front of it on the belt and measures for how much time it is appearing. With the time and the speed of the conveyor belt (to be calibrated in real life), it calculates the length of the object.

Then a sero with a one sided long tail directs the object to the respective bins.

[if (object_length > 8 cm) >> keeps the belt unblocked >> falls where the belt ends];
[if (4 < object length < 8) >> spins the servo instantaneously so that theta becomes 30 degree]
[if (object length < 4) >> spins the servo instantaneously so that theta becomes 150 degree]


Applications: sorting similar partf of different length like screws, plastic parts like legos etc. in factory production
              sorting any kind of similar objects of differnt sizes.
