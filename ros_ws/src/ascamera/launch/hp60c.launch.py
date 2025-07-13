from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()
    ascamera_node = Node(
        namespace= "ascamera_hp60c",
        package='ascamera',
        executable='ascamera_node',
        respawn=True,
        output='both',
        parameters=[
            {"usb_bus_no": -1},
            {"usb_path": "null"},
            {"confiPath": "/carpi/ros_ws/src/ascamera/configurationfiles"},
            {"color_pcl": True},
            {"pub_tfTree": True},
            {"depth_width": 640},
            {"depth_height": 480},
            {"rgb_width": 640},
            {"rgb_height": 480},
            {"fps": 15},
        ],
        remappings=[]
    )
    
    cam_link_0_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        #arguments = ['0', '0', '0', '-1.57', '0', '-1.57', 'depth_cam', 'ascamera_camera_link_0']
        arguments = ['0', '0', '0', '-1.57', '0', '-1.57', 'depth_cam', 'ascamera_hp60c_camera_link_0']
        )
        
    camera_0_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        #arguments = ['0', '0', '0', '-1.57', '0', '-1.57', 'depth_cam', 'ascamera_camera_link_0']
        arguments = ['0', '0', '0', '-1.57', '0', '-1.57', 'depth_cam', 'ascamera_hp60c_color_0']
        )


    # ascamera_node2 = Node(
    #     namespace= "ascamera_hp60c_2",
    #     package='ascamera',
    #     executable='ascamera_node',
    #     respawn=True,
    #     output='both',
    #     parameters=[
    #         {"usb_bus_no": 3},    # set your usb_bus_no
    #         {"usb_path": "2.2"},  # set your usb_path
    #         {"confiPath": "./ascamera/configurationfiles"},
    #         {"color_pcl": False},
    #         {"depth_width": 640},
    #         {"depth_height": 480},
    #         {"rgb_width": 640},
    #         {"rgb_height": 480},
    #         {"fps": 15},
    #     ],
    #     remappings=[]
    # )

    ld.add_action(ascamera_node)
    ld.add_action(cam_link_0_transform)
    ld.add_action(camera_0_transform)
    # ld.add_action(ascamera_node2)

    return ld

