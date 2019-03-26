import argparse

import numpy as np

from lxml import etree
from lxml.etree import Element


class Creator(object):
    def __init__(self, file_name, robot_num, env_size=5.0):
        parser = etree.XMLParser(remove_blank_text=True)
        self.tree = etree.parse(file_name, parser)
        self.robot_num = robot_num
        self.env_size = env_size

    def run(self):
        root = self.tree.getroot()
        
        for i in range(self.robot_num):
            angle = i * 2 * np.pi / self.robot_num
            sx = self.env_size * np.cos(angle)
            sy = self.env_size * np.sin(angle)
            group = Element("group", ns="$(arg mav_name)"+str(i))
            include = Element("include", file="$(find rotors_gazebo)/launch/spawn_mav.launch")
            x = Element("arg", name="x", value=str(sx))
            y = Element("arg", name="y", value=str(sy))
            z = Element("arg", name="z", value="0.1")
            namespace = Element("arg", name="namespace", value="$(arg mav_name)"+str(i))
            mav_name = Element("arg", name="mav_name", value="$(arg mav_name)")
            model = Element("arg", name="model", value="$(find rotors_description)/urdf/mav_generic_odometry_sensor.gazebo")
            enable_logging = Element("arg", name="enable_logging", value="$(arg enable_logging)")
            enable_ground_truth = Element("arg", name="enable_ground_truth", value="$(arg enable_ground_truth)")
            log_file = Element("arg", name="log_file", value="$(arg log_file)")
            include.append(x)
            include.append(y)
            include.append(z)
            include.append(namespace)
            include.append(mav_name)
            include.append(model)
            include.append(enable_logging)
            include.append(enable_ground_truth)
            include.append(log_file)
            rotors_joy_interface = Element("node", name="rotors_joy_interface", pkg = "rotors_joy_interface", type="rotors_joy_interface")
            rotorno = Element("param", name = "rotorno", type="double", value=str(i))
            destx  = Element("param", name = "destx", type="double", value=str(sx+1))
            desty = Element("param", name = "desty", type="double", value=str(sy+1))
            destz = Element("param", name = "destz", type="double", value="10.0")
            modelname = Element("param", name = "modelname", value="/firefly"+str(i))
            rotors_joy_interface.append(rotorno)
            rotors_joy_interface.append(destx)
            rotors_joy_interface.append(desty)
            rotors_joy_interface.append(destz)
            rotors_joy_interface.append(modelname)
            lee_position_controller_node = Element("node", name="lee_position_controller_node", pkg="rotors_control", type="lee_position_controller_node", output="screen")
            rosparam1 =  Element("rosparam", command="load", file="$(find rotors_gazebo)/resource/lee_controller_$(arg mav_name).yaml")
            rosparam2 =  Element("rosparam", command="load", file="$(find rotors_gazebo)/resource/$(arg mav_name).yaml")
            #remap = Element("remap", from = "odometry", to="odometry_sensor1/odometry")
            lee_position_controller_node.append(rosparam1)
            lee_position_controller_node.append(rosparam2)
            hovering = Element("node", name="hovering_example", pkg="rotors_gazebo", type="hovering_example", output="screen")
            #roll_pitch_yawrate_thrust_controller_node.append(remap)
            robot_state_publisher = Element("node", name="robot_state_publisher", pkg="robot_state_publisher", type="robot_state_publisher" )
            joint_state_publisher = Element("node", name="joint_state_publisher", pkg="joint_state_publisher", type="joint_state_publisher" )
            group.append(include)
            group.append(rotors_joy_interface)
            group.append(lee_position_controller_node)
            group.append(hovering)
            group.append(robot_state_publisher)
            group.append(joint_state_publisher)
            root.append(group)
##generate target obj
            





        self.tree.write("multidrones.launch", pretty_print=True, xml_declaration=True, encoding="utf-8")
        
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Create multi-robot launch file for Gazebo")
    parser.add_argument(
        "--launch_file", default="empty.launch", type=str
    )
    parser.add_argument(
        "--robot_num", default=10, type=int
    )
    parser.add_argument(
        "--env_size", default=10.0, type=float
    )

    args = parser.parse_args()

    creator = Creator(args.launch_file, args.robot_num, args.env_size)
    creator.run()
    
