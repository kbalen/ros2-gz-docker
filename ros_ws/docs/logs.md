ros2 pkg create --build-type ament_cmake rb-car-sim


So far we have created a package with a launch script that can open gazebo and spawn the robot. 
We ported the xacro urdf robot from the old ros noetic repository to the current ros2 repo. Along with the xacro files are stl meshes that these files reference. Initially we had a problem where the robot did not load in gazebo properly and gazebo complained about not being able to find the stl meshes and began search them online. This was resolved by pointing the GAZEBO_MODELS_PATH to ros_ws/src. We added the following line to our dockerfile
echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:\$HOME/ros_ws/src" >> ~/.bashrc which resolved the issue. 

Spawning the robot requires us to first convert the xacro file to urdf and providing the output as parameter to robot state publisher. 