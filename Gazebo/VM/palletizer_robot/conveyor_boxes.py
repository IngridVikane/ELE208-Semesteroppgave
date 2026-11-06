#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Lag Bokser en av gangen på rullebåndet. spawner på pickup poiint. bokser etter spawner når boksen har blitt beveget vekk.
pollet fra gz world topic /world/pick and place/pose/info
"""

import re
import subprocess
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node

POSE_TOPIC = '/world/pick_and_place/pose/info'


def box_model_sdf(name, pose=None):
    """Definert dynaisk boks lag en via `create`'s -x/-y/-z flag"""
    s = 0.4
    i_flat = 5.0 * 2 * s ** 2 / 12  # uniform kube inertia for 5kg masse
    m = ET.Element('model', {'name': name})
    link = ET.SubElement(m, 'link', {'name': 'link'})
    box = ET.SubElement(link, 'collision', {'name': 'collision'})
    geom = ET.SubElement(box, 'geometry')
    bx = ET.SubElement(geom, 'box')
    ET.SubElement(bx, 'size').text = f'{s} {s} {s}'
    surf = ET.SubElement(box, 'surface')
    fric = ET.SubElement(surf, 'friction')
    od = ET.SubElement(fric, 'ode')
    ET.SubElement(od, 'mu').text = '1.2'
    ET.SubElement(od, 'mu2').text = '1.2'
    vis = ET.SubElement(link, 'visual', {'name': 'visual'})
    vgeom = ET.SubElement(vis, 'geometry')
    vbox = ET.SubElement(vgeom, 'box')
    ET.SubElement(vbox, 'size').text = f'{s} {s} {s}'
    mat = ET.SubElement(vis, 'material')
    for tag in ('ambient', 'diffuse'):
        c = ET.SubElement(mat, tag)
        c.text = '0.8 0.5 0.2 1'
    inertial = ET.SubElement(link, 'inertial')
    ET.SubElement(inertial, 'mass').text = '5.0'
    ii = ET.SubElement(inertial, 'inertia')
    for a in ('ixx', 'iyy', 'izz'):
        ET.SubElement(ii, a).text = f'{i_flat:.4f}'
    return ('<?xml version="1.0" ?>\n<sdf version="1.8">'
            + ET.tostring(m).decode() + '</sdf>')


def parse_world_pose(dump, name):
    """(x, y, z) of entity `name` from a gz topic -e textproto dump, or None."""
    lines = dump.splitlines()
    result = None
    for i, line in enumerate(lines):
        m = re.search(r'name:\s*"([^"]+)"', line)
        if not m:
            continue
        ent = m.group(1)
        if '/' in ent or ent != name:
            continue
        pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}  # textproto omits zero fields
        seen = False                          # position{} block present at all
        for j in range(i + 1, min(i + 30, len(lines))):
            if lines[j].strip().startswith('pose {'):
                break                          # next top-level entry
            if 'position' in lines[j]:
                seen = True
            pm = re.match(r'\s*(x|y|z):\s*([-+0-9.eE]+)', lines[j])
            if pm:
                pos[pm.group(1)] = float(pm.group(2))
            if lines[j].strip().startswith('orientation'):
                break
        if seen:
            result = (pos['x'], pos['y'], pos['z'])
    return result


class ConveyorBoxes(Node):

    def __init__(self):
        super().__init__('conveyor_boxes')

        #box size, T0B = T0C * T_CB pickup pose.
        self.spawn_pose = (0.0, -1.0, 0.6)   # boks senter på conveyor
        self.carry_away_distance = 0.3  # [m] from pickup point -> "gone"

        self.names = [f'B{i}' for i in range(1, 7)]
        self.spawned = 0

        self.timer = self.create_timer(1.0, self.update)
        self.get_logger().info('conveyor_boxes ready: waiting for controllers/world')

    def update(self):
        if self.spawned >= len(self.names):
            self.timer.cancel()
            return
        if self.spawned == 0:
            names = self.get_node_names()
            if 'arm_controller' not in names or 'joint_state_broadcaster' not in names:
                return
            if self.spawn_box(self.names[0]):
                self.spawned = 1
                self.get_logger().info(f'spawned {self.names[0]} at {self.spawn_pose}')
            return

        active = self.names[self.spawned - 1]
        pos = self.query_pose(active)
        if pos is None:
            return
        dx = pos[0] - self.spawn_pose[0]
        dy = pos[1] - self.spawn_pose[1]
        if (dx * dx + dy * dy) > self.carry_away_distance ** 2:
            nxt = self.names[self.spawned]
            if self.spawn_box(nxt):
                self.spawned += 1
                self.get_logger().info(f'{active} carried away -> spawned {nxt}')

    def query_pose(self, name):
        try:
            result = subprocess.run(
                ['gz', 'topic', '-e', '-t', POSE_TOPIC, '-n', '1'],
                capture_output=True, text=True, timeout=8)
        except (subprocess.TimeoutExpired, FileNotFoundError) as exc:
            self.get_logger().warn(f'cannot poll {name} pose yet: {exc}')
            return None
        if result.returncode != 0:
            self.get_logger().warn(f'gz topic poll rc={result.returncode}: '
                                   f'{(result.stderr or "").strip()[:200]}')
            return None
        return parse_world_pose(result.stdout, name)

    def spawn_box(self, name):
        sdf = box_model_sdf(name)
        x, y, z = self.spawn_pose
        cmd = ['ros2', 'run', 'ros_gz_sim', 'create',
               '-string', sdf, '-name', name, '-world', 'pick_and_place',
               '-x', str(x), '-y', str(y), '-z', str(z)]
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        except subprocess.TimeoutExpired:
            self.get_logger().warn(f'cannot spawn {name} yet: create timed out '
                                   '(world not ready?)')
            return False
        except FileNotFoundError as exc:
            self.get_logger().warn(f'cannot spawn {name} yet: {exc}')
            return False
        out = (result.stdout or '') + (result.stderr or '')
        if result.returncode != 0 or 'creation successful' not in out:
            self.get_logger().warn(f'spawn {name} failed rc={result.returncode}: '
                                   f'{out.strip()[-200:]}')
            return False
        return True


def main(args=None):
    rclpy.init(args=args)
    node = ConveyorBoxes()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
