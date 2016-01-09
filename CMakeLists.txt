# What version of CMake i s needed?
cmake_minimum_required(VERSION 2.8.3)
# Name of this package .
project(agitr)

# Find the catkin build system , and any other packages on
# which we depend .
# find_package( catkin REQUIRED COMPONENTS roscpp )
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs ardrone_autonomy std_msgs std_srvs cv_bridge image_transport)

# Declare our catkin package .
catkin_package()

# Specify locations of header files .
include_directories( include ${catkin_INCLUDE_DIRS})

# Declare the executable , along with its source files.If
# there are multiple executables , use multiple copies of
# this line .

add_executable(hogdetect hogdetect.cpp)#pentru detectie oameni
target_link_libraries(hog2 ${catkin_LIBRARIES})

# Specify libraries against which to link . Again , this
# line should be copied for each distinct executable in
# the package .
