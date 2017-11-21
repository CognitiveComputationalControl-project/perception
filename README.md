# perception

# To launch the handle detection server : 

roslaunch handle_detector localization_sensor.launch

# To launch the tracking server who keeps track of the best detection :

rosrun handle_tracking scanner

# To call the series of services : 

rosservice call track_handle "entry: false"
