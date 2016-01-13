# drone-project
Real world person tracking application using Parrot ARDrone 2.0 
DON'T FORGET about the site https://sites.google.com/site/iugacristi/:maybe you want a step by step timesheet of what I did, the problems I faced and the solutions and workarounds.
#1
There exists now a part of the code. This runs on Ubuntu and you also need ROS to make it work.
For the ones acustomed to ROS, the source file hogdetect.cpp and the CMakeFile should be in the agitr file(i am saying this just for my case,other may change the names).The complete project directory is: workspace(give it whatever name you want)->src->agitr.
You create a workspace with catkin,this will build a set of files in your workspace.Then you simply create a file wih your desired name to hold your source files(in my case it is agitr).
Compile your code with "catkin_make"(command from your workspace).Then type in "source devel/setup.bash".Finally type in the terminal "run agitr hogdetect" and the program will run. The general way to run it is "rosrun yourPackage yourExecutable".
#2
The method with the histogram of gradients works fine if the tracked person is always at a considerable distance to the camera, otherwise the drawn rectangle resulted from the detection will be a "degenerated" one and it won't work fine with the method to compute the distance. See the hogdetect.cpp file for source code.
#3
Tried something new and different even if there were good results with the default HOG detector
There is a clean version of a fine working program that detects faces and computes the distance between the camera and the person who is tracked.It uses haar classifiers and it behaves pretty neatly. It is the facedetect.cpp file.
I have modified the CMakeList to work with both souce files so there won't be any problems there.
You cant put all files in a directory,compile both source files and see the difference between methods and the problems faced.
