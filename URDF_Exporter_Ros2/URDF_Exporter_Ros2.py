#Author-syuntoku14
#Description-Generate URDF file from Fusion 360

import adsk, adsk.core, adsk.fusion, traceback
import os
import sys
from .utils import utils
from .core import Link, Joint, Write

"""
# length unit is 'cm' and inertial unit is 'kg/cm^2'
# If there is no 'body' in the root component, maybe the corrdinates are wrong.
"""

# joint effort: 100
# joint velocity: 100
# supports "Revolute", "Rigid" and "Slider" joint types

# I'm not sure how prismatic joint acts if there is no limit in fusion model

# Override the rotation/slide axis for specific joints after automatic computation.
# Use this for mirrored joints (e.g. wheels on opposite sides) where Fusion reports
# the same axis direction but you need one side negated.
# Values are expressed in the joint frame.
# Per-joint axis overrides.
# The axis is expressed in the joint frame (after rpy is applied).
# Use this when Fusion reports the same axis for mirrored joints that should
# spin in opposite directions — e.g. wheels on opposite sides of a robot.
#
# Example:
#   'wheel_nw': [0, 1, 0],   # flip from [0,-1,0] to [0,1,0]
#   'wheel_sw': [0, 1, 0],
JOINT_AXIS_OVERRIDES = {
    # 'your_joint_name': [x, y, z],
    'wheel_se': [0.0, 1.0, 0.0],
    'wheel_ne': [0.0, 1.0, 0.0]
}

def run(context):
    ui = None
    success_msg = 'Successfully create URDF file'
    msg = success_msg

    try:
        # --------------------
        # initialize
        app = adsk.core.Application.get()
        ui = app.userInterface
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        title = 'Fusion2URDF'
        if not design:
            ui.messageBox('No active Fusion design', title)
            return

        root = design.rootComponent  # root component
        components = design.allComponents

        # set the names
        default_robot = root.name.split()[0]
        robot_name, cancelled = ui.inputBox('Robot name', title, default_robot)
        if cancelled:
            ui.messageBox('Fusion2URDF was canceled', title)
            return 0
        robot_name = robot_name.strip()

        package_name, cancelled = ui.inputBox('Package name', title, robot_name + '_description')
        if cancelled:
            ui.messageBox('Fusion2URDF was canceled', title)
            return 0
        package_name = package_name.strip()

        save_dir = utils.file_dialog(ui)
        if save_dir == False:
            ui.messageBox('Fusion2URDF was canceled', title)
            return 0

        save_dir = save_dir + '/' + package_name
        try: os.mkdir(save_dir)
        except: pass

        package_dir = os.path.abspath(os.path.dirname(__file__)) + '/package/'

        # --------------------
        # set dictionaries

        # Generate joints_dict. All joints are related to root.
        joints_dict, msg = Joint.make_joints_dict(root, msg, JOINT_AXIS_OVERRIDES)
        if msg != success_msg:
            ui.messageBox(msg, title)
            return 0

        # Generate inertial_dict
        inertial_dict, msg = Link.make_inertial_dict(root, msg)
        if msg != success_msg:
            ui.messageBox(msg, title)
            return 0
        elif not 'base_link' in inertial_dict:
            msg = 'There is no base_link. Please set base_link and run again.'
            ui.messageBox(msg, title)
            return 0

        links_xyz_dict = {}

        # --------------------
        # Generate URDF
        Write.write_urdf(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
        Write.write_urdf_xacro(package_name, robot_name, save_dir)
        Write.write_materials_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
        Write.write_transmissions_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
        Write.write_gazebo_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
        Write.write_display_launch(package_name, robot_name, save_dir)
        Write.write_gazebo_launch(package_name, robot_name, save_dir)

        # copy over package files
        utils.create_package(package_name, save_dir, package_dir)
        utils.update_setup_py(save_dir, package_name)
        utils.update_setup_cfg(save_dir, package_name)
        utils.update_package_xml(save_dir, package_name)

        # Generate STl files
        utils.copy_occs(root)
        utils.export_stl(design, save_dir, components, robot_name)

        ui.messageBox(msg, title)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
