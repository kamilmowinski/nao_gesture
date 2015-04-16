# nao_kinnect
Kinnect and Nao

Authors: Kamil Mówiński, Patryk Bogdański

 Freekinect:

      $ sudo apt-get install freenect
      $ sudo modprobe -r gscpa_kinect
      $ sudo modprobe -r gspca_main
      $ echo "blacklist gspca_kinect" |sudo tee -a /etc/modprobe.d/blacklist.conf

Add user to plugdev group

      $ sudo adduser <username> plugdev
      

and check 

      $ freenect-glview

ROS Enviroment

       $ source /opt/ros/hydro/setup.bash
       $ mkdir -p ~/catkin_ws/src
       $ cd ~/catkin_ws/src
       $ catkin_init_workspace
       $ cd ~/catkin_ws/
       $ catkin_make
       $ source devel/setup.bash
       $ sudo apt-get install ros-hydro-openni-camera
       $ sudo apt-get install ros-hydro-openni-launch
       $ sudo apt-get install ros-hydro-openni-tracker

Download NITE from OpeNI: www.openni.ru/openni-sdk/openni-sdk-history-2/

Download file and unzip
NiTE v1.5.2.23

as a adminstrator, run the install script:

        # ./install.sh

Preview 
In catkin enviroment (source ~/catkin_ws/devel/setup.bash), put this command in new terminal

        $ roslaunch my_kinnect kinnect.launch

To display skeletno coordinate system in rviz, its neccessery (or use coords.rviz file):
- Press "Add"
- In window choice "TF" and apply
- In "Global Options" in "Fixed Frame" put "openni_depth_frame".


Next steps:
Information about users joint is available as transform head_1, torso_1 etc...
Only 3 users can be detected (http://wiki.ros.org/openni_tracker).

Script, which publish information about head:

       $ rosrun my_kinnect tracker.py 

Other body part

       $ rosrun my_kinnect tracker.py _target_joint:=/left_hand

Naoqi SDK
1. Unzip archive in destination directory
2. Add this line to ~/.bashrc

      export PYTHONPATH=${PYTHONPATH}:/path/to/python-sdk

3. Restart shell.


Move nao by ROS

      rostopic pub -1 /nao my_kinnect/NaoCoords  '{header: auto, Part: ['Head'], Angles1: [100], Angles2: [20]}'
