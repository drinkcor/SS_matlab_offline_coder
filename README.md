```
online RCBHT (real time RCBHT)
=======================

Overview:

The online RCBHT enables semantic encoding of low-level real time wrench data. It takes real time ROS message (includeing 
wrench data) as input, do semantic encoding, and finally publishs stream of semantic labels using ROS message. The taxonomy 
is built on the premise that low-level relative-change patterns can be classified through a small set of categoric labels 
in an increasingly abstract manner. The RCBHT is a multi-layer behavior aggregating scheme. It is composed of three 
bottom-to-top increasingly abstract layers. Starting from the bottom layer and going up we have the Primitive layer, the 
Motion Composition (MC) layer, and the Low-Level Behavior layer (LLB).


Prerequisite：

1. Install matlab ROS custom message support. 
      If your matlab's version is R2015b, call roboticsSupportPackages and follow the instructions for installation. 
      If your matlab's version is R2016a and later versions, call roboticsAddons and follow the instructions for installation.
      
2. Install ROS package "publish_files". This package can transform file data to be real time data, and publish it to RCBHT. 
   Besides, this package contains the required msg, srv, and package.xml files for RCBHT's use. 
      2.1. Put folder: publish_files/ to the src folder of your ros workspace
      2.2. Run catkin_make outside src folder
      2.3. To use it, run the command: $ rosrun publish_Files force_publisher [path_of_force_file]
      
3. Generate custom message for RCBHT's use.
      3.1 Specify a folder path where your want to place your custom messages in. Such as: 
                  folderpath = '/home/drinkcor/MATLAB/custom_msg'
      3.2 Put folder: publish_files/ under the folder path that you defined above.
      3.3 Generate custom message: rosgenmsg(folderpath)

4. Be sure that you have added all needed codes to the Path:
      i)   the folder of online RCBHT
      ii)  the folder of custom message support
      iii) the folder of generated custom message

To use online RCBHT, there are several steps listed below,

1. Start ROS, 
      $ roscore

2. Open parpool in MATLAB command window.
      % Delete any possible existing pools running previously
      >> delete(gcp);
      >> parpool(2);
  Then, wait for the parpool start. If the prompt, which is ">>"， available again, that means the parpool is ready to work. 
  Now we can move on to the next step.

3. Run RCBHT in MATLAB command window.
      >> rt_snapVerification('HSA','any_path_here_is_ok');
   "HSA" means a kind of strategy, the second parameter is a path, some files containing predefined theshold are placed here. 
   If there aren't any predefined threshold files, you can text any string to fill this parameter. 
   After you see message like: “The value of the ROS_MASTER_URI environment variable, http://localhost:11311, will be used to 
   connect to the ROS master.Initializing global node /matlab_global_node_88464 with NodeURI http://localhost:60491/” It mean 
   that the RCBHT is ready to work. Move on to the next step.
   
4. Publish real time force/torque data to RCBHT.
   The RCBHT receive real time force/torque data. To publish real time data to it, we have several approach: 1)connect to a 
   baxter robot, 2)run a rosbag, 3)publish a force/torque file to be real time data. 
   4.1 If use approach 2), just run,
       $ rosbag play -r 0.4 [path/xxx.bag]
       Here, -r 0.4 means multiply rate with 0.4. (If you want to know the rate of a topic, you can use 
            $ rostopic hz /topic_name.)
       Then, the online RCBHT will receive data from rosbag and process it.
   4.2 If use approach 3), you need to have publishFiles package in your ros workspace, so that you can run force_publisher node.
       After you installing publishFiles, run,
       $ rosrun publishFiles force_publisher [path_of_force_file]
       Then, the online RCBHT will receive data from a real time data(originnally is a file) and process it.
```

