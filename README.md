# pythonapi-test

This repository is created to learn to use Carla's PythonAPI

#### Scripts

##### GlobalPathCarla2ROS

This is a ROS node that publishes global path on request. 


###### Topics

**Publisher:**  ```/<name_space>/global_path```  
 - Message Type : Path (nav_msgs/path)
 
**Subscriber:** ```/<name_space>/get_global_path```  
 - Message Type : String (publish empty string on this topic to get global path)  

###### Use
```python
def main(argv):

	client = carla.Client('localhost',2000)
	client.set_timeout(2.0)

	world = client.get_world()

	
	# namespace - 'carla'
	node = globalPathServer(world,'carla')

	while not rospy.is_shutdown():
		rospy.spin()
```