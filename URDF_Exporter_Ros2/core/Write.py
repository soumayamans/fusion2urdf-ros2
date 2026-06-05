# -*- coding: utf-8 -*-
"""
Created on Sun May 12 20:46:26 2019

@author: syuntoku
"""

import adsk, os
from xml.etree.ElementTree import Element, SubElement
from . import Link, Joint, launch_templates
from .Joint import _rmat_to_rpy, _rtranspose_mul_vec, _IDENTITY_3
from ..utils import utils

def write_link_urdf(joints_dict, repo, links_xyz_dict, file_name, inertial_dict):
    """
    Write links information into urdf "repo/file_name"


    Parameters
    ----------
    joints_dict: dict
        information of the each joint
    repo: str
        the name of the repository to save the xml file
    links_xyz_dict: vacant dict
        xyz information of the each link
    file_name: str
        urdf full path
    inertial_dict:
        information of the each inertial

    Note
    ----------
    In this function, links_xyz_dict is set for write_joint_tran_urdf.
    The origin of the coordinate of center_of_mass is the coordinate of the link
    """
    with open(file_name, mode='a') as f:
        # base_link — world-aligned by convention, xyz=[0,0,0]
        center_of_mass = inertial_dict['base_link']['center_of_mass']
        link = Link.Link(name='base_link', xyz=[0,0,0],
            center_of_mass=center_of_mass, repo=repo,
            mass=inertial_dict['base_link']['mass'],
            inertia_tensor=inertial_dict['base_link']['inertia'])
        links_xyz_dict['base_link'] = [0.0, 0.0, 0.0]  # world position
        link.make_link_xml()
        f.write(link.link_xml)
        f.write('\n')

        # other links
        for joint in joints_dict:
            name = joints_dict[joint]['child']
            joint_xyz = joints_dict[joint]['xyz']   # joint world-frame position (metres)
            Rc = joints_dict[joint].get('R_child', list(_IDENTITY_3))

            # Visual/collision origin: expressed in link frame.
            # The STL is exported in world coordinates, so we must undo the link frame
            # rotation (R_child^T) and shift by -joint_xyz to place it correctly.
            visual_xyz = [round(v, 6) for v in _rtranspose_mul_vec(Rc, [-j for j in joint_xyz])]
            Rc_T = [Rc[0],Rc[3],Rc[6], Rc[1],Rc[4],Rc[7], Rc[2],Rc[5],Rc[8]]
            visual_rpy = _rmat_to_rpy(Rc_T)

            # Centre of mass in link frame: R_child^T * (com_world - joint_xyz)
            com_world = inertial_dict[name]['center_of_mass']
            com_rel = [c - j for c, j in zip(com_world, joint_xyz)]
            center_of_mass = [round(v, 6) for v in _rtranspose_mul_vec(Rc, com_rel)]

            link = Link.Link(name=name, xyz=visual_xyz, visual_rpy=visual_rpy,
                center_of_mass=center_of_mass,
                repo=repo, mass=inertial_dict[name]['mass'],
                inertia_tensor=inertial_dict[name]['inertia'])
            links_xyz_dict[name] = joint_xyz  # store world position for joint xyz computation
            link.make_link_xml()
            f.write(link.link_xml)
            f.write('\n')


def write_joint_urdf(joints_dict, repo, links_xyz_dict, file_name):
    """
    Write joints and transmission information into urdf "repo/file_name"


    Parameters
    ----------
    joints_dict: dict
        information of the each joint
    repo: str
        the name of the repository to save the xml file
    links_xyz_dict: dict
        xyz information of the each link
    file_name: str
        urdf full path
    """

    with open(file_name, mode='a') as f:
        for j in joints_dict:
            parent = joints_dict[j]['parent']
            child = joints_dict[j]['child']
            joint_type = joints_dict[j]['type']
            upper_limit = joints_dict[j]['upper_limit']
            lower_limit = joints_dict[j]['lower_limit']
            try:
                # Vector from parent link origin to child joint, both in world frame.
                # Then rotate into parent link frame via R_parent^T.
                xyz_world = [c - p for c, p in
                             zip(links_xyz_dict[child], links_xyz_dict[parent])]
                Rp = joints_dict[j].get('R_parent', list(_IDENTITY_3))
                xyz = [round(v, 6) for v in _rtranspose_mul_vec(Rp, xyz_world)]
            except KeyError as ke:
                app = adsk.core.Application.get()
                ui = app.userInterface
                ui.messageBox("There seems to be an error with the connection between\n\n%s\nand\n%s\n\nCheck \
whether the connections\nparent=component2=%s\nchild=component1=%s\nare correct or if you need \
to swap component1<=>component2"
                % (parent, child, parent, child), "Error!")
                quit()

            joint = Joint.Joint(name=j, joint_type=joint_type, xyz=xyz,
                axis=joints_dict[j]['axis'], parent=parent, child=child,
                upper_limit=upper_limit, lower_limit=lower_limit,
                rpy=joints_dict[j].get('rpy', [0.0, 0.0, 0.0]))
            joint.make_joint_xml()
            f.write(joint.joint_xml)
            f.write('\n')

def write_gazebo_endtag(file_name):
    """
    Write about gazebo_plugin and the </robot> tag at the end of the urdf


    Parameters
    ----------
    file_name: str
        urdf full path
    """
    with open(file_name, mode='a') as f:
        f.write('</robot>\n')


def write_urdf(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir):
    try: os.mkdir(save_dir + '/urdf')
    except: pass
    try: os.mkdir(save_dir + '/urdf/' + robot_name)
    except: pass

    file_name = save_dir + '/urdf/' + robot_name + '/' + robot_name + '.xacro'  # the name of urdf file
    repo = package_name + '/meshes/' + robot_name + '/'  # the repository of binary stl files
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro">\n'.format(robot_name))
        f.write('\n')
        f.write('<xacro:include filename="$(find {})/urdf/{}/materials.xacro" />'.format(package_name, robot_name))
        f.write('\n')
        f.write('<xacro:include filename="$(find {})/urdf/{}/{}.trans" />'.format(package_name, robot_name, robot_name))
        f.write('\n')
        f.write('<xacro:include filename="$(find {})/urdf/{}/{}.gazebo" />'.format(package_name, robot_name, robot_name))
        f.write('\n')

    write_link_urdf(joints_dict, repo, links_xyz_dict, file_name, inertial_dict)
    write_joint_urdf(joints_dict, repo, links_xyz_dict, file_name)
    write_gazebo_endtag(file_name)

def write_materials_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir):
    try: os.mkdir(save_dir + '/urdf')
    except: pass
    try: os.mkdir(save_dir + '/urdf/' + robot_name)
    except: pass

    file_name = save_dir + '/urdf/' + robot_name + '/materials.xacro'  # the name of urdf file
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro" >\n'.format(robot_name))
        f.write('\n')
        f.write('<material name="silver">\n')
        f.write('  <color rgba="0.700 0.700 0.700 1.000"/>\n')
        f.write('</material>\n')
        f.write('\n')
        f.write('</robot>\n')

def write_transmissions_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir):
    """
    Write joints and transmission information into urdf "repo/file_name"


    Parameters
    ----------
    joints_dict: dict
        information of the each joint
    repo: str
        the name of the repository to save the xml file
    links_xyz_dict: dict
        xyz information of the each link
    file_name: str
        urdf full path
    """

    file_name = save_dir + '/urdf/{}/{}.trans'.format(robot_name, robot_name)  # the name of urdf file
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro" >\n'.format(robot_name))
        f.write('\n')

        for j in joints_dict:
            if joints_dict[j]['type'] == 'fixed':
                continue
            joint = Joint.Joint(name=j, joint_type=joints_dict[j]['type'],
                xyz=[0, 0, 0], axis=[0, 0, 0], parent='', child='',
                upper_limit=0.0, lower_limit=0.0)
            joint.make_transmission_xml()
            f.write(joint.tran_xml)
            f.write('\n')

        f.write('</robot>\n')

def write_gazebo_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir):
    try: os.mkdir(save_dir + '/urdf')
    except: pass
    try: os.mkdir(save_dir + '/urdf/' + robot_name)
    except: pass

    file_name = save_dir + '/urdf/' + robot_name + '/' + robot_name + '.gazebo'  # the name of urdf file
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro" >\n'.format(robot_name))
        f.write('\n')
        f.write('<xacro:property name="body_color" value="Gazebo/Silver" />\n')
        f.write('\n')

        gazebo = Element('gazebo')
        plugin = SubElement(gazebo, 'plugin')
        plugin.attrib = {'name':'control', 'filename':'libgazebo_ros_control.so'}
        gazebo_xml = "\n".join(utils.prettify(gazebo).split("\n")[1:])
        f.write(gazebo_xml)

        # for base_link
        f.write('<gazebo reference="base_link">\n')
        f.write('  <material>${body_color}</material>\n')
        f.write('  <mu1>0.2</mu1>\n')
        f.write('  <mu2>0.2</mu2>\n')
        f.write('  <self_collide>true</self_collide>\n')
        f.write('  <gravity>true</gravity>\n')
        f.write('</gazebo>\n')
        f.write('\n')

        # others
        for joint in joints_dict:
            name = joints_dict[joint]['child']
            f.write('<gazebo reference="{}">\n'.format(name))
            f.write('  <material>${body_color}</material>\n')
            f.write('  <mu1>0.2</mu1>\n')
            f.write('  <mu2>0.2</mu2>\n')
            f.write('  <self_collide>true</self_collide>\n')
            f.write('</gazebo>\n')
            f.write('\n')

        f.write('</robot>\n')

def write_display_launch(package_name, robot_name, save_dir):
    """
    write display launch file "save_dir/launch/display.launch"


    Parameter
    ---------
    robot_name: str
    name of the robot
    save_dir: str
    path of the repository to save
    """
    try: os.mkdir(save_dir + '/launch')
    except: pass

    file_text = launch_templates.get_display_launch_text(package_name, robot_name)

    file_name = os.path.join(save_dir, 'launch', 'display.launch.py')
    with open(file_name, mode='w') as f:
        f.write(file_text)

def write_gazebo_launch(package_name, robot_name, save_dir):
    """
    write gazebo launch file "save_dir/launch/gazebo.launch"


    Parameter
    ---------
    robot_name: str
        name of the robot
    save_dir: str
        path of the repository to save
    """

    try: os.mkdir(save_dir + '/launch')
    except: pass

    file_text = launch_templates.get_gazebo_launch_text(package_name, robot_name)

    file_name = os.path.join(save_dir, 'launch', 'gazebo.launch.py')
    with open(file_name, mode='w') as f:
        f.write(file_text)
