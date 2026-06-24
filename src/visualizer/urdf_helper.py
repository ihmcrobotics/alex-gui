"""
This file provides helpful information and methods for using the URDF. As ihmc-alex-sdk has each robot split into
separate parts (upper arm, forearm, left, right, etc), we provide the list of parts for each configuration and
a method to merge into a single temporary urdf file.
"""

import sys
from pathlib import Path
from typing import List, Literal

from lxml import etree

# Lists of urdf parts for Alex 001 and 002
ALEX_NUBFOREARMS_PARTS = ["head", "leftUpperArm", "leftFixedForearm", "rightUpperArm", "rightFixedForearm"]
ALEX_CYCLOIDFOREARMS_PARTS = ["head", "leftUpperArm", "leftForearm", "rightUpperArm", "rightForearm"]
ALEX_FULLBODY_PARTS = ["head", "leftUpperArm", "leftForearm", "leftAbilityHandAdapter", "ability_hand_left_large",
                       "rightUpperArm", "rightForearm", "rightAbilityHandAdapter", "ability_hand_right_large"]
ALEX_NOARMS_PARTS = ["head"]

# List of urdf parts for Alex 003 (Purdue)
PURDUE_CYCLOIDFOREARMS_PARTS = ["leftUpperArm", "leftForearm", "rightUpperArm", "rightForearm"]
PURDUE_FULL_PARTS = ["leftUpperArm", "leftForearm", "leftEZGripperAdapter", "left_ezgripper_gen2", "rightUpperArm",
                     "rightForearm", "rightEZGripperAdapter", "right_ezgripper_gen2"]

# Directory location for urdfs. Assumes ihmc-alex-sdk is located in the same workspace as alex-gui
SCRIPT_DIR = str(Path(__file__).resolve().parent.parent.parent.parent)
if sys.platform == "win32":
    ALEX_URDF_PATH = SCRIPT_DIR + "\\ihmc-alex-sdk\\alex-models\\alex_purdue_description\\urdf\\"
    HANDS_PATH = SCRIPT_DIR + "\\ihmc-alex-sdk\\alex-ros2\\ihmc_hands_ros2\\"
else:
    ALEX_URDF_PATH = SCRIPT_DIR + "/ihmc-alex-sdk/alex-models/alex_purdue_description/urdf/"
    HANDS_PATH = SCRIPT_DIR + "/ihmc-alex-sdk/alex-ros2/ihmc_hands_ros2/"


def merge_urdfs(robot_version: Literal['001', '002', 'purdue'], wanted_parts: List[str], fixed_joints: List[str] = [""],
                allowed_collisions: List[str] | None = [""], save_name: str = "temp") -> str:
    """
    Helper method to merge URDFs together into a single temporary URDF and save it.

    :param robot_version: Version of the robot model. Options available are 001, 002, and purdue
    :param wanted_parts: List of URDF parts wanted to be merged. Do not include the lower body for 001 or 002, as they
    are assumed as the base URDF. For Purdue, the torso URDF is assumed to be the base URDF.
    :param fixed_joints: List of joints that you want converted to fixed. The previous joint type does not matter
    :param allowed_collisions: List of collisions you want to have kept in the URDF.
    :param save_name: Name of the save file for the new URDF.
    :return: The filepath of the merged URDF in string format.
    """
    prefix = "alex_"
    excluded_name = "IMU"

    # Define the directory locations
    main_directory = ALEX_URDF_PATH
    if sys.platform == "win32":
        hand_mesh_directory = HANDS_PATH + "meshes\\"
    else:
        hand_mesh_directory = HANDS_PATH + "meshes/"

    # Define the prefix for each urdf based on the robot version
    robot_prefix = prefix + "v1"
    if robot_version == "002":
        robot_prefix = prefix + "v2"
    base_urdf_file = main_directory + robot_prefix + ".lowerBody.urdf"
    if robot_version == "purdue":
        robot_prefix = prefix + robot_version
        base_urdf_file = main_directory + robot_prefix + ".headTorso.urdf"
    # base_urdf_file = main_directory + robot_prefix + ".lowerBody.urdf"
    base_urdf = etree.parse(base_urdf_file)
    base_robot = base_urdf.getroot()

    # Remove any excluded joints and links from the urdf that are excluded (IMU joints) and fix any joints in the fixed
    # joints list
    for joint in base_robot.findall("joint"):
        if excluded_name in joint.get("name", ""):
            base_robot.remove(joint)
        if joint.get("name", "") in fixed_joints:
            joint.set("type", "fixed")
    for link in base_robot.findall("link"):
        if (excluded_name in link.get("name", "")):
            base_robot.remove(link)
        for visual in link.findall("visual"):
            mesh_location = visual.find("geometry/mesh").get("filename")
            # Sometimes the package name causes issues, so we replace with the actual location
            if sys.platform == "win32":
                new_location = mesh_location.replace("package://",
                                                     "..\\ihmc-alex-sdk\\alex-models\\")
            else:
                new_location = mesh_location.replace("package://",
                                                     "../ihmc-alex-sdk/alex-models/")
            visual.find("geometry/mesh").set("filename", new_location)

    # Remove gazebo references as that can break some urdf loaders
    for gazebo in base_robot.findall("gazebo"):
        if (excluded_name in gazebo.get("reference", "")):
            base_robot.remove(gazebo)

    # Do the same thing we did for the base urdf on all the wanted parts, then merge them in
    for wanted_part in wanted_parts:
        # Each hand has a different directory, so they need to be defined
        if "gen2" in wanted_part:
            if sys.platform == "win32":
                directory = HANDS_PATH + "urdf\\ezGripper\\"
            else:
                directory = HANDS_PATH + "urdf/ezGripper/"
            filename = wanted_part + ".urdf"
        elif "ability_hand" in wanted_part:
            if sys.platform == "win32":
                directory = HANDS_PATH + "urdf\\abilityHand\\"
            else:
                directory = HANDS_PATH + "urdf/abilityHand/"
            filename = wanted_part + ".urdf"
        else:
            directory = main_directory
            filename = robot_prefix + "." + wanted_part + ".urdf"
        curr_urdf = etree.parse(directory + filename).getroot()

        for joint in curr_urdf.findall("joint"):
            if excluded_name not in joint.get("name", ""):
                if joint.get("name", "") in fixed_joints or "hand" in wanted_part:
                    joint.set("type", "fixed")
                base_robot.append(joint)

        for link in curr_urdf.findall("link"):
            if excluded_name not in link.get("name", ""):
                if "hand" in wanted_part or "gen2" in wanted_part:
                    for visual in link.findall("visual"):
                        mesh_location = visual.find("geometry/mesh").get("filename")
                        new_location = mesh_location.replace("package://", hand_mesh_directory)
                        if sys.platform == "win32":
                            new_location = new_location.replace("/", "\\")
                        visual.find("geometry/mesh").set("filename", new_location)
                else:
                    for visual in link.findall("visual"):
                        mesh_location = visual.find("geometry/mesh").get("filename")
                        if sys.platform == "win32":
                            new_location = mesh_location.replace("package://",
                                                                 "..\\ihmc-alex-sdk\\alex-models\\")
                            new_location = new_location.replace("/", "\\")
                        else:
                            new_location = mesh_location.replace("package://",
                                                                 "../ihmc-alex-sdk/alex-models/")
                        visual.find("geometry/mesh").set("filename", new_location)
                base_robot.append(link)
        for gazebo in curr_urdf.findall("gazebo"):
            if (excluded_name not in gazebo.get("reference", "")):
                base_robot.append(gazebo)

    if allowed_collisions is not None:
        for link in base_robot.findall(".//link"):
            for collision in link.findall("collision"):
                # print(collision.get("name"))
                if collision.get("name") not in allowed_collisions:
                    link.remove(collision)

    save_filepath = main_directory + save_name + ".urdf"
    base_urdf.write(save_filepath, pretty_print=True, xml_declaration=True, encoding="UTF-8")
    return save_filepath


if __name__ == "__main__":
    urdf_path = merge_urdfs("purdue", PURDUE_FULL_PARTS, fixed_joints=["PEDESTAL_F"], save_name="hehe_full",
                            allowed_collisions=[""])
    print(urdf_path)
