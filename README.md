# REMOTE ROBOT CONTROL MECHANISM USING A PHYSIOLOGICAL SENSOR - ROS Implementations
ROS - Shimmer3 Project

This project has been implemented for University of Bristol MSc Robotics Dissertation module under 
the supervision of Alex Smith and Anouk Van Maris. 

There are a couple of dependencies, and you will require them to run 
this project.

Please install the following dependencies:

- https://github.com/dpkoch/async_comm


**How to install the project**

  After installing related dependecities, get the "my_example" directory and source the ROS files by typing:
  
  *source devel/setup.bash*
  
  Then, the ROS files need to be built by using the following command:
  
  *catkin_make*
  
  After successfully making the project, we can run the ROS nodes using the *rosrun* command. However, the com port needs to be configured in Linux environment for the Bluetooth setup before running the **publisher** node. The following steps will set the com port on linux environment.
  - At first, we need to find the shimmer3 MAC address by searching via the Blueooth (Please be ensure the Shimmer3 is open). So, the following command can be used on terminal to find the possible Bluetooth devices.
  
    *hcitool scan*
    
    This command will search all the Bluetooth devices with their MAC address around. After finding the shimmer3, copy the corresponding MAC address to open a new port.
    
  - After coying the MAC address, the following code will open a new port between Shimmer3 and Linux environment. The **port_numer** can be selected any, but if there is no port opened, we can set it simply 0. 
  
    *sudo rfcomm bind **port_number** **MAC_address***
    
  - To check whether the intended port has been opened or not, please simply type *rfcomm* command on the terminal. This should show you the related port with the settled parameters.
  
  - After opening the new port between Shimmer3 and Linux environment, we need to give permission to the opened port by using the command follow(The same **port_number** needs to be put as we did above, for example, **rfcomm0**):
    
    *sudo chmod a+rw /dev/rfcomm(**port_number**)*
    
  - Now we can run the **publisher** node as follows:
    
    *rosrun my_example_pkg publisher /dev/rfcomm(**port_number**)*
    
  - The **robot_controller** node can also be run using the same command:
  
    *rosrun my_example_pkg robot_controller*
    
  - The **stress_detection.py** python script can be found under the *dissertation_project/my_example/src/my_example_pkg/src/*, and it runs separately on the VScode environment (you can use it in your favourite IDE). Please be aware that the python script has its library dependencies. You should install all of them before running the node.
  

**Example images from the project**

![plot](./images/gazebo_ros_exp.png)  ![plot](./images/whole_path.png)

The Gazebo simulation utilized during the experiment can be found following the link:

*https://github.com/Frkncm/Gazebo_design*


**Regarding WESAD_analysis_and_ML.ipynb file**

This file is created to analyse the WESAD dataset and train the SVM and KNN models. You can find all the code steps, but to run it, you will need to download the WESAD dataset that can be found from the following link below (Since the WESAD file is too large, it is left to the user who wants to run the codes).

*https://archive.ics.uci.edu/ml/datasets/WESAD+%28Wearable+Stress+and+Affect+Detection%29*

Alternatively, to directly download it, please follow the link below:

*https://uni-siegen.sciebo.de/s/HGdUkoNlW1Ub0Gx*

