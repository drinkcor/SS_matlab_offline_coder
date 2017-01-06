```
online RCBHT (real time RCBHT)
=======================

To use online RCBHT, there are several steps listed below,

1. Start ROS, 
      $ roscore

2. Open parpool in MATLAB command window.
      % Delete any possible existing pools running previously
      delete(gcp);
      parpool(2);
  Then, wait for the parpool start. If the prompt, which is ">>"， available again, that means the parpool is ready to work. Now we can move on to the next step.

3. Run RCBHT in MATLAB command window.
      rt_snapVerification('HSA','any_path_here_is_ok');
   "HSA" means a kind of strategy, the second parameter is a path, some files containing predefined theshold are placed here. If there aren't any predefined threshold files, you can text any string to fill this parameter. 
   After you see message like: “The value of the ROS_MASTER_URI environment variable, http://localhost:11311, will be used to connect to the ROS master.Initializing global node /matlab_global_node_88464 with NodeURI http://localhost:60491/” It mean that the RCBHT is ready to work. Move on to the next step.
   
4. Publish real time force/torque data to RCBHT.
   The RCBHT receive real time force/torque data. To publish real time data to it, we have several approach: 1)connect to a baxter robot, 2)run a rosbag, 3)publish a force/torque file to be real time data. 
   4.1 If use approach 2), just run,
       $ rosbag play -r 0.4 [path/xxx.bag]
       Here, -r 0.4 means multiply rate with 0.4. (If you want to know the rate of a topic, you can use $ rostopic hz /topic_name.)
       Then, the online RCBHT will receive data from rosbag and process it.
   4.2 If use approach 3), you need to have publishFiles package in your ros workspace, so that you can run force_publisher node.
       To use the force publisher, you should 
          1. Put folder: publishFiles/ to the src folder of your ros workspace
          2. Run catkin_make outside src folder
          3. To use it, run the command: $ rosrun publishFiles force_publisher [path_of_force_file]
       After you installing publishFiles, run,
       $ rosrun publishFiles force_publisher [path_of_force_file]
       Then, the online RCBHT will receive data from a real time data(originnally is a file) and process it.
```


