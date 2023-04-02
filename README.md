# Scorpion_ROS_Gazebo
随便乱做的蝎子机器人，6足+5关节尾巴+2个3自由度手爪


# Command to Start Simulation

	roslaunch sc_gazebo sc_world.launch


	roslaunch sc_control sc_control.launch


	rosrun sc_control main.py (or directly run on IDE)

# 一些废话+笔记：

Some nonsense + notes:

sc_control, sc_gazebo, 以及sc_description所组成。其中:


sc_control中有由config, launch,以及scripts 文件。config文件所存放的.yaml文件是用于对ROS Gazebo之间的通信节点进行配置；
    
	launch文件中的sc_control.launch则是将sc_control.yaml所配置的文件对sc_world.launch 的关节控制器进行通信连接；
    
	scripts == 所编写的控制代码。控制代码是基于Python进行编写。

sc_description是通过Solidworks导出URDF文件并将其导入Moveit!!中对其传感器进行配置当.
    
	meshes文件所存放的是URDF所导出的STL文件。
    
	urdf文件夹中则是机器人的模型文件，通过调用meshes文件所存放STL文件，并通过坐标对其进行配置装配。

sc_gazebo/launch文件中的sc_world.launch为加载机器人、机器人关节控制、Gazebo世界等。
    
	worlds/sc.world则是对Gazebo世界进行配置。
