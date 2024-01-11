from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution, Command, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, GroupAction
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage
from launch.events import Shutdown


#use Xacro files to make life easier
def generate_launch_description():
   
   return LaunchDescription([   
      GroupAction(
         actions=[
            DeclareLaunchArgument(name = "use_jsp",
                        default_value = "true",
                        description="use_jsp: if true, then joint state publisher is used to publish joint states"),

            DeclareLaunchArgument(name = "use_rviz",
                              default_value = "true",
                              description = "control whether rviz is launched"),
            DeclareLaunchArgument(name = "color",
                              default_value = "purple",
                              description = "the color of the baselink",
                              choices=["purple","red","green","blue"]),
            SetLaunchConfiguration(name = "rviz_config", value = ["basic_",  LaunchConfiguration("color"), ".rviz"]),
            
            Node(
               package='robot_state_publisher',
               executable='robot_state_publisher',
               name='rsp',
               parameters=[
                  {"robot_description" :
                  Command([ExecutableInPackage("xacro", "xacro"), " ",
                           PathJoinSubstitution(
                           [FindPackageShare("nuturtle_description"), "turtlebot3_burger.urdf.xacro"]),
                           " ",
                           "color:=",
                           LaunchConfiguration("color"),
                           ])}, 
                  {"frame_prefix" : [LaunchConfiguration("color"), TextSubstitution(text="/")]},
               ]
            ),
            Node(
               package='joint_state_publisher',
               executable='joint_state_publisher',
               name='jsp',
               condition=IfCondition(EqualsSubstitution(LaunchConfiguration("use_jsp"), "true"))
            ),
            Node(
               package='rviz2',
               executable='rviz2',
               name='rviz',
               arguments=[
                  '-d',
                  PathJoinSubstitution([FindPackageShare("nuturtle_description"),LaunchConfiguration("rviz_config")]),
               " ",
               "-f",
               [LaunchConfiguration("color"), "/base_footprint"],
               ],
               condition=IfCondition(EqualsSubstitution(LaunchConfiguration("use_rviz"), "true")),
               on_exit=Shutdown(),
            ),
                     
         ]
      ),

      
    
    
   ])