import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, IfElseSubstitution
from launch_ros.actions import Node

import xacro


def _pose(origin_el):
    """URDF <origin xyz rpy/> -> SDFormat <pose>x y z r p y</pose> text."""
    return (origin_el.get('xyz', '0 0 0') + ' ' + origin_el.get('rpy', '0 0 0')).strip()


def _geometry(el, package_path=None):
    out = ET.Element('geometry')
    for child in list(el):
        if child.tag == 'box':
            g = ET.SubElement(out, 'box')
            ET.SubElement(g, 'size').text = child.get('size')
        elif child.tag == 'sphere':
            g = ET.SubElement(out, 'sphere')
            ET.SubElement(g, 'radius').text = child.get('radius')
        elif child.tag == 'cylinder':
            g = ET.SubElement(out, 'cylinder')
            ET.SubElement(g, 'radius').text = child.get('radius')
            ET.SubElement(g, 'length').text = child.get('length')
        elif child.tag == 'mesh':
            g = ET.SubElement(out, 'mesh')
            uri = child.get('filename', '')
            # gz spawned from a string has no ament package index: rewrite
            # package:// URIs of THIS package to absolute file:// paths.
            prefix = 'package://' + _geometry.pkg_name + '/'
            if uri.startswith(prefix) and package_path:
                uri = 'file://' + os.path.join(package_path, uri[len(prefix):])
            ET.SubElement(g, 'uri').text = uri
            if child.get('scale'):
                ET.SubElement(g, 'scale').text = child.get('scale')
    return out


_geometry.pkg_name = 'palletizer_robot'


def _convert_link(el, model, materials, package_path=None):
    link = ET.SubElement(model, 'link', {'name': el.get('name', '')})
    nv = 0
    nc = 0
    for child in list(el):
        if child.tag == 'visual':
            v = ET.SubElement(link, 'visual', {'name': f'visual_{nv}'})
            nv += 1
            o = child.find('origin')
            if o is not None:
                ET.SubElement(v, 'pose').text = _pose(o)
            v.append(_geometry(child.find('geometry'), package_path))
            m = child.find('material')
            rgba = materials.get(m.get('name')) if m is not None else None
            if rgba:
                mat = ET.SubElement(v, 'material')
                for tag in ('ambient', 'diffuse'):
                    c = ET.SubElement(mat, tag)
                    c.text = rgba
        elif child.tag == 'collision':
            c_el = ET.SubElement(link, 'collision', {'name': child.get('name') or f'collision_{nc}'})
            nc += 1
            o = child.find('origin')
            if o is not None:
                ET.SubElement(c_el, 'pose').text = _pose(o)
            c_el.append(_geometry(child.find('geometry'), package_path))
        elif child.tag == 'inertial':
            i = ET.SubElement(link, 'inertial')
            o = child.find('origin')
            if o is not None:
                ET.SubElement(i, 'pose').text = _pose(o)
            mass = child.find('mass')
            if mass is not None:
                ET.SubElement(i, 'mass').text = mass.get('value', '0')
            inertia_el = child.find('inertia')
            if inertia_el is not None:
                ii = ET.SubElement(i, 'inertia')
                for attr in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz'):
                    v = ET.SubElement(ii, attr)
                    v.text = inertia_el.get(attr, '0')
    return link


def _convert_joint(el, model, child_link):
    """Emit an SDF <joint>. URDF joint <origin>s are baked into the CHILD LINK
    pose (relative_to the parent link), like gz's own URDF->SDF conversion does:
    bullet-featherstone does not apply joint-element <pose> offsets, so leaving
    them on the joint collapses the whole chain onto the model origin."""
    j = ET.SubElement(model, 'joint', {'name': el.get('name'), 'type': el.get('type', 'fixed')})
    p_el = ET.SubElement(j, 'parent'); p_el.text = el.find('parent').get('link')
    c_el = ET.SubElement(j, 'child'); c_el.text = el.find('child').get('link')
    o = el.find('origin')
    if o is not None and child_link is not None:
        pose_txt = _pose(o)
        if any(abs(float(v)) > 1e-12 for v in pose_txt.split()):
            pe = ET.Element('pose', {'relative_to': p_el.text})
            pe.text = pose_txt
            child_link.insert(0, pe)
    axis = el.find('axis')
    if j.get('type') != 'fixed' and axis is not None:
        ax = ET.SubElement(j, 'axis')
        ET.SubElement(ax, 'xyz').text = axis.get('xyz', '0 0 1')
        limit_el = el.find('limit')
        dyn_el = el.find('dynamics')
        if limit_el is not None or dyn_el is not None:
            lim = ET.SubElement(ax, 'limit')
            for attr in ('lower', 'upper'):
                v = ET.SubElement(lim, attr)
                v.text = limit_el.get(attr, '0') if limit_el is not None else '0'
            if limit_el is not None and 'velocity' in (limit_el.attrib or {}):
                ET.SubElement(lim, 'velocity').text = limit_el.get('velocity', '0')
            # SDF 2.x axis limit uses dissipation for the URDF damping+friction pair.
            if dyn_el is not None:
                damp = float(dyn_el.get('damping', '0')) + float(dyn_el.get('friction', '0'))
                ET.SubElement(lim, 'dissipation').text = repr(damp)


def robot_model_sdf_from_urdf(urdf_str, package_path=None):
    """Convert the URDF robot description to an SDFormat <model> and anchor its base
    link with a fixed joint to world, so bullet-featherstone anchors frame {0} at the
    world origin (a bare URDF root becomes a FLOATING base -> the robot 'flies')."""
    root = ET.fromstring(urdf_str)
    materials = {}
    for m in root.findall('material'):
        c = m.find('color')
        if c is not None:
            materials[m.get('name')] = c.get('rgba')

    model = ET.Element('model', {'name': root.get('name', 'robot_model')})
    link_els = {}
    for el in list(root):
        if el.tag == 'link':
            link_els[el.get('name')] = _convert_link(el, model, materials, package_path)
    for el in list(root):
        if el.tag == 'link':
            pass  # done above
        elif el.tag == 'joint':
            _convert_joint(el, model, link_els.get(el.find('child').get('link')))
        elif el.tag == 'gazebo':
            # gz only loads model-level <plugin> elements (same as the URDF->SDF
            # parser does): unwrap the <gazebo><plugin/></gazebo> wrapper. A
            # <gazebo reference="link"> carrying a <sensor/> is attached under that
            # SDF link instead (camera.urdf.xacro sensor, course camera pattern).
            ref = el.get('reference')
            for child in list(el):
                if child.tag == 'plugin':
                    model.append(child)
                elif ref in link_els:
                    link_els[ref].append(child)
        elif el.tag != 'material':  # <ros2_control> block stays as model child
            model.append(el)

    wj = ET.SubElement(model, 'joint', {'name': 'world_joint', 'type': 'fixed'})
    ET.SubElement(wj, 'parent').text = 'world'
    ET.SubElement(wj, 'child').text = root.find('link').get('name')

    sdf_root = ET.Element('sdf', {'version': '1.8'})
    sdf_root.append(model)
    return '<?xml version="1.0" ?>\n' + ET.tostring(sdf_root, encoding='unicode')


def generate_launch_description():

    # The name of the package the robot description is in.
    package_name = 'palletizer_robot'

    package_path = get_package_share_directory(package_name)

    gazebo_gui = DeclareLaunchArgument(
        'gazebo_gui', default_value='true',
        description='Set to "false" to run gz sim headless (-s).')

    # Compile the URDF XACRO description of the robot into pure XML.
    xacro_file = os.path.join(package_path, 'urdf', 'robot_description.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    my_robotarm_description = doc.toxml()

    params = {'robot_description': my_robotarm_description, 'use_sim_time': True}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

# forankring
    robot_model_sdf = robot_model_sdf_from_urdf(my_robotarm_description, package_path)

    world_file = os.path.join(package_path, 'worlds', 'pick_and_place.sdf')

    engine = ' --physics-engine gz-physics-bullet-featherstone-plugin'

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': IfElseSubstitution(
                LaunchConfiguration(gazebo_gui.name),
                if_value=['-r -v 4 ' + world_file + engine],
                else_value=['-s -r -v 3 ' + world_file + engine],
            )
        }.items(),
    )

    node_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_model_sdf,
            '-name', 'palletizer_robot',
        ],
        output='screen'
    )

    # Boxes spawn one at a time on the conveyor , spawn en ekstra boks når forrige har blitt beveget vekk
    box_pose_bridges = [f'/model/B{i}/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose'
                        for i in range(1, 13)]
    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # D435 over conveyor
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # Vacuum gripper box transport + placement service bridge
            '/world/pick_and_place/set_pose@ros_gz_interfaces/srv/SetEntityPose',
        ] + box_pose_bridges,
        output='screen'
    )

    node_conveyor_boxes = Node(
        package=package_name,
        executable='conveyor_boxes',
        name='conveyor_boxes',
        output='screen',
    )

    node_joint_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    node_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    return LaunchDescription([
        gazebo_gui,
        gazebo_sim,
        node_robot_state_publisher,
        node_spawn_entity,
        node_ros_gz_bridge,
        node_conveyor_boxes,
        node_joint_broadcaster,
        node_arm_controller,
    ])
