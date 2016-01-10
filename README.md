# drone-project
Real world person tracking application using Parrot ARDrone 2.0 
#1
There exists now a part of the code. This runs on Ubuntu and you also need ROS to make it work.
For the ones acustomed to ROS, the source file hogdetect.cpp and the CMakeFile should be in the agitr file(i am saying this just for my case,other may change the names).The complete project directory is: workspace(give it whatever name you want)->src->agitr.
You create a workspace with catkin,this will build a set of files in your workspace.Then you simply create a file wih your desired name to hold your source files(in my case it is agitr).
Compile your code with "catkin_make"(command from your workspace).Then type in "source devel/setup.bash".Finally type in the terminal "run agitr hogdetect" and the program will run. The general way to run it is "rosrun yourPackage yourExecutable".
#2
