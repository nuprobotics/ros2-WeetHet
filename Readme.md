[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/kVBSh0lw)
# ROS2 practise tasks

In this practise you will work with ROS2. 

For all tasks you should create launch file in directory `launch` of your package with name `task<number>.launch`. 
Don't forget to include `('share/' + package_name + "/launch", ['launch/task<number>.launch']),` in your `setup.py`.

All packages should be created in the `src` directory of your workspace.

Tasks marked with `*` have no autotests.

## Task 1
Create a ROS2 node named `receiver` in package `task01` that receive a message of type `std_msgs/msg/String` from topic `/spgc/sender` 
and print the message in the console using logger. Log level should be `INFO`.

## Task 2
Create a ROS2 node in package `task02` that publish a message of type `std_msgs/msg/String` to topic `/spgc/receiver`, name 
of the topic should be specified in the config file. The text of message should be passed via param in the 
command line. Default value of the text should be `Hello, ROS2!`.

**Autotests specs:**
+ Config file should be called `task02.yaml` and placed in the `config` directory of your package
+ Topic name parameter should be called `topic_name`
+ Param name should be `text`

## Task 3
Create a ROS2 node in package `task03` that call a service of type `std_srvs/srv/Trigger` with name `/spgc/trigger` 
and store the returned string value. This node also should provide a service of type `std_srvs/srv/Trigger`
with name specified in the launch file. The response of the service should be the stored string value. 
If the `/spgc/trigger` service wasn't available during calling, the service created by the node should return
string specified in the config file.

**Autotests specs:**
+ Config file should be called `task03.yaml` and placed in the `config` directory of your package
+ Service name parameter should be called `service_name`, default value should be `/trigger_service`
+ Default string value should be `No service available` and should be named `default_string`

## Task 4 *
Create a ROS2 node that receive a message of type `sensor_msgs/msg/CompressedImage` from topic 
specified in the command line and save the image to the disk. Also this node should provide a service
of type `std_srvs/srv/Trigger` with name `/stop` in `YOUR_NAME` namespace. If the service is called, the node should stop saving images.

_Hint for task 4: if you use Docker don't forget to mount directory where you want to store outputs_
