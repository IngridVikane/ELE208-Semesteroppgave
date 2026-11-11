#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Conveyor box spawner and vacuum gripper pick-and-place controller.

Spawns boxes B1..B12 (0.4 m cubes, 5 kg) one at a time at the conveyor pickup
point (0.0, -1.0, 0.6). Tracks /joint_states via the robot's exact DH forward
kinematics (L = [0.1, 0.9, 0.8, 0.2, 0.15, 0.15], TCP = frame_6 * Tz(0.10)):
  - Latches (picks up) the active box when the TCP reaches the conveyor pickup
    pose (0.0, -1.0, 0.800).
  - Transports the held box rigidly with the TCP (box center = TCP + 0.20 * z_TCP)
    via /world/pick_and_place/set_pose (ros_gz_interfaces/srv/SetEntityPose).
  - Spawns the next box on the conveyor once the current box is carried > 0.3 m
    away from the pickup point.
  - Releases (places) the box onto its target pallet slot (Layer 1 z=0.344 for
    B1..B6, Layer 2 z=0.744 for B7..B12) when the TCP reaches the placement pose.
"""

import math
import random
import re
import subprocess
import threading
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node

try:
    from sensor_msgs.msg import JointState
    from ros_gz_interfaces.srv import SetEntityPose
except ImportError:
    JointState = None
    SetEntityPose = None

POSE_TOPIC = '/world/pick_and_place/pose/info'
SET_POSE_SRV = '/world/pick_and_place/set_pose'

# Exact DH table matching generate_robot_description.m / fk_check.m / run_joint_trajectory.m
# L = [0.1, 0.9, 0.8, 0.2, 0.15, 0.15], tool offset Tz(0.10)
DH_D = (0.10, 0.0, 0.0, 0.20, 0.15, 0.15)
DH_A = (0.0, 0.90, 0.80, 0.0, 0.0, 0.0)
DH_ALPHA = (math.pi / 2.0, 0.0, 0.0, -math.pi / 2.0, math.pi / 2.0, 0.0)
TCP_Z_OFFSET = 0.10
BOX_HALF_SIZE = 0.20
JOINT_NAMES = ('joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6')


def _mat_mul(A, B):
    """4x4 homogeneous matrix product A @ B."""
    return [
        [
            A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j] + A[i][3] * B[3][j]
            for j in range(4)
        ]
        for i in range(4)
    ]


def _dh_step(theta, d, a, alpha):
    """Standard DH matrix Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)."""
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return [
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0],
    ]


def fk_tcp(q):
    """Return (R_tcp, p_tcp) in world frame for 6-vector q = [q1..q6] (rad)."""
    T = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    for i in range(6):
        T = _mat_mul(T, _dh_step(q[i], DH_D[i], DH_A[i], DH_ALPHA[i]))
    # Apply tool offset Tz(0.10): p_tcp = p_6 + 0.10 * z_6, R_tcp = R_6
    R = [
        [T[0][0], T[0][1], T[0][2]],
        [T[1][0], T[1][1], T[1][2]],
        [T[2][0], T[2][1], T[2][2]],
    ]
    p = (
        T[0][3] + TCP_Z_OFFSET * R[0][2],
        T[1][3] + TCP_Z_OFFSET * R[1][2],
        T[2][3] + TCP_Z_OFFSET * R[2][2],
    )
    return R, p


def rot_to_quat(R):
    """Convert 3x3 rotation matrix R to unit quaternion (w, x, y, z)."""
    tr = R[0][0] + R[1][1] + R[2][2]
    if tr > 0.0:
        S = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * S
        x = (R[2][1] - R[1][2]) / S
        y = (R[0][2] - R[2][0]) / S
        z = (R[1][0] - R[0][1]) / S
    elif (R[0][0] > R[1][1]) and (R[0][0] > R[2][2]):
        S = math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2.0
        w = (R[2][1] - R[1][2]) / S
        x = 0.25 * S
        y = (R[0][1] + R[1][0]) / S
        z = (R[0][2] + R[2][0]) / S
    elif R[1][1] > R[2][2]:
        S = math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2.0
        w = (R[0][2] - R[2][0]) / S
        x = (R[0][1] + R[1][0]) / S
        y = 0.25 * S
        z = (R[1][2] + R[2][1]) / S
    else:
        S = math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2.0
        w = (R[1][0] - R[0][1]) / S
        x = (R[0][2] + R[2][0]) / S
        y = (R[1][2] + R[2][1]) / S
        z = 0.25 * S
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm > 1e-12:
        w, x, y, z = w / norm, x / norm, y / norm, z / norm
    else:
        w, x, y, z = 1.0, 0.0, 0.0, 0.0
    return (w, x, y, z)


def box_model_sdf(name, pose=None):
    """Definert dynamisk boks lagd via `create`'s -x/-y/-z flag.

    Includes VelocityControl (initial_linear=0, initial_angular=0) so the box
    does not accumulate downward freefall velocity in bullet-featherstone while
    transported via SetEntityPose, and slightly inset horizontal collision
    (0.34 x 0.34 x 0.40) so adjacent 0.40 m-grid stacks never clip laterally.
    """
    s = 0.4
    s_col_xy = 0.34
    i_flat = 5.0 * 2 * s ** 2 / 12  # uniform kube inertia for 5kg masse
    m = ET.Element('model', {'name': name})
    link = ET.SubElement(m, 'link', {'name': 'link'})
    box = ET.SubElement(link, 'collision', {'name': 'collision'})
    geom = ET.SubElement(box, 'geometry')
    bx = ET.SubElement(geom, 'box')
    ET.SubElement(bx, 'size').text = f'{s_col_xy} {s_col_xy} {s}'
    surf = ET.SubElement(box, 'surface')
    fric = ET.SubElement(surf, 'friction')
    od = ET.SubElement(fric, 'ode')
    ET.SubElement(od, 'mu').text = '1.2'
    ET.SubElement(od, 'mu2').text = '1.2'
    bul = ET.SubElement(fric, 'bullet')
    ET.SubElement(bul, 'friction').text = '1.2'
    ET.SubElement(bul, 'friction2').text = '1.2'
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
    vc = ET.SubElement(m, 'plugin', {
        'filename': 'gz-sim-velocity-control-system',
        'name': 'gz::sim::systems::VelocityControl',
    })
    ET.SubElement(vc, 'initial_linear').text = '0 0 0'
    ET.SubElement(vc, 'initial_angular').text = '0 0 0'
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

        # Box center on conveyor & corresponding TCP pickup pose
        self.spawn_pose = (0.0, -1.0, 0.6)
        self.pickup_tcp = (0.0, -1.0, 0.800)
        self.carry_away_distance = 0.3  # [m] from pickup point -> "gone"

        self.names = [f'B{i}' for i in range(1, 13)]   # 12 boxes = 2 layers of 6
        # Matching InverseKinematics_Palletizing12boxes.m:
        #   boxX = [1.0, 0.6, 1.0, 0.6, 1.0, 0.6]
        #   boxY = [0.4, 0.4, 0.0, 0.0, -0.4, -0.4]
        slots_xy = [
            (1.0,  0.4), (0.6,  0.4),
            (1.0,  0.0), (0.6,  0.0),
            (1.0, -0.4), (0.6, -0.4),
        ]
        # Target TCP placement coordinates (x, y, z_tcp_place) for B1..B12
        self.place_targets = (
            [(x, y, 0.544) for (x, y) in slots_xy] +
            [(x, y, 0.944) for (x, y) in slots_xy]
        )

        self.spawned = 0
        self.current_idx = 0
        self.held_box = None
        self.box_yaws = {}
        self.grasp_yaw_offset = 0.0
        self.in_pickup_zone = False
        self.pickup_min_z = None
        self.in_place_zone = False
        self.place_min_z = None

        self._lock = threading.Lock()
        self._spawning = False
        self._polling = False
        self._last_pose_send = 0.0
        self._pose_future = None
        self._snap_queue = []

        if SetEntityPose is not None:
            self.pose_client = self.create_client(SetEntityPose, SET_POSE_SRV)
        else:
            self.pose_client = None

        if JointState is not None:
            self.joint_sub = self.create_subscription(
                JointState, '/joint_states', self.on_joint_states, 10)
        else:
            self.joint_sub = None

        self.timer = self.create_timer(0.5, self.update)
        self.get_logger().info('conveyor_boxes ready: waiting for controllers/world')

    def send_box_pose(self, name, pos, quat_wxyz, force=False):
        """Send asynchronous SetEntityPose request for `name`."""
        if self.pose_client is None:
            return False
        now = time.monotonic()
        if not force:
            if (now - self._last_pose_send) < 0.022:
                return False
            if (self._pose_future is not None
                    and not self._pose_future.done()
                    and (now - self._last_pose_send) < 0.090):
                return False
        if not self.pose_client.service_is_ready():
            return False

        req = SetEntityPose.Request()
        req.entity.name = name
        req.pose.position.x = float(pos[0])
        req.pose.position.y = float(pos[1])
        req.pose.position.z = float(pos[2])
        req.pose.orientation.w = float(quat_wxyz[0])
        req.pose.orientation.x = float(quat_wxyz[1])
        req.pose.orientation.y = float(quat_wxyz[2])
        req.pose.orientation.z = float(quat_wxyz[3])

        self._last_pose_send = now
        self._pose_future = self.pose_client.call_async(req)
        return True

    def on_joint_states(self, msg):
        """Track arm TCP from /joint_states to pick, carry, and place boxes."""
        name_to_pos = dict(zip(msg.name, msg.position))
        if not all(jn in name_to_pos for jn in JOINT_NAMES):
            return
        q = [name_to_pos[jn] for jn in JOINT_NAMES]
        R_tcp, p_tcp = fk_tcp(q)

        # 1. Check for pickup at conveyor if no box is currently held
        if self.held_box is None:
            if self.current_idx < len(self.names) and (
                    self.current_idx < self.spawned or self._spawning):
                px, py, pz = self.pickup_tcp
                d_xy = math.hypot(p_tcp[0] - px, p_tcp[1] - py)
                if d_xy < 0.15 and p_tcp[2] <= (pz + 0.10):
                    self.in_pickup_zone = True
                    if self.pickup_min_z is None or p_tcp[2] < self.pickup_min_z:
                        self.pickup_min_z = p_tcp[2]

                should_pick = False
                if self.in_pickup_zone:
                    if p_tcp[2] <= (pz + 0.015):
                        should_pick = True
                    elif self.pickup_min_z is not None and p_tcp[2] > (self.pickup_min_z + 0.003):
                        should_pick = True
                    elif d_xy > 0.15:
                        self.in_pickup_zone = False
                        self.pickup_min_z = None

                if should_pick:
                    self.held_box = self.names[self.current_idx]
                    self.in_pickup_zone = False
                    self.pickup_min_z = None
                    self.in_place_zone = False
                    self.place_min_z = None
                    # Compute TCP yaw around world +Z (for R_tcp = Rz(psi_tcp) * diag(1,-1,-1))
                    psi_tcp = math.atan2(R_tcp[1][0], R_tcp[0][0])
                    psi_box = self.box_yaws.get(self.held_box, 0.0)
                    diff = psi_box - psi_tcp
                    # Wrap into (-pi/4, pi/4] for 90-deg symmetric square box
                    while diff <= -math.pi / 4.0:
                        diff += math.pi / 2.0
                    while diff > math.pi / 4.0:
                        diff -= math.pi / 2.0
                    self.grasp_yaw_offset = diff
                    self.get_logger().info(
                        f'Vacuum ON: picked up {self.held_box} '
                        f'(box_yaw={math.degrees(psi_box):+.1f} deg, '
                        f'tcp_yaw={math.degrees(psi_tcp):+.1f} deg, '
                        f'align_err={math.degrees(diff):+.1f} deg)')
            return

        # 2. A box is held: check if TCP has reached its pallet placement pose
        tx, ty, tz_place = self.place_targets[self.current_idx]
        tz_box = tz_place - BOX_HALF_SIZE
        d_place_xy = math.hypot(p_tcp[0] - tx, p_tcp[1] - ty)

        if d_place_xy < 0.15 and p_tcp[2] <= (tz_place + 0.12):
            self.in_place_zone = True
            if self.place_min_z is None or p_tcp[2] < self.place_min_z:
                self.place_min_z = p_tcp[2]

        should_release = False
        if self.in_place_zone:
            if p_tcp[2] <= (tz_place + 0.015):
                should_release = True
            elif self.place_min_z is not None and p_tcp[2] > (self.place_min_z + 0.004):
                should_release = True
            elif d_place_xy > 0.12:
                should_release = True

        if should_release:
            placed_name = self.held_box
            self.held_box = None
            self.in_place_zone = False
            self.place_min_z = None
            self.current_idx += 1

            psi_tcp_place = math.atan2(R_tcp[1][0], R_tcp[0][0])
            psi_final = psi_tcp_place + self.grasp_yaw_offset
            while psi_final <= -math.pi / 4.0:
                psi_final += math.pi / 2.0
            while psi_final > math.pi / 4.0:
                psi_final -= math.pi / 2.0
            # Snap to 0 deg when vision-aligned within 5 deg so stack is flush
            if abs(psi_final) < math.radians(5.0):
                psi_final = 0.0

            target_pos = (tx, ty, tz_box)
            placed_quat = (math.cos(psi_final / 2.0), 0.0, 0.0, math.sin(psi_final / 2.0))
            self.send_box_pose(placed_name, target_pos, placed_quat, force=True)
            with self._lock:
                self._snap_queue.append((placed_name, target_pos, placed_quat, 2))

            self.get_logger().info(
                f'Vacuum OFF: placed {placed_name} at '
                f'({tx:.3f}, {ty:.3f}, {tz_box:.3f}) yaw={math.degrees(psi_final):+.1f} deg')

            # Ensure next box is spawned if not already triggered during carry
            self._maybe_spawn_next(placed_name)
            return

        # 3. Still carrying self.held_box: compute box center pose from TCP
        bx = p_tcp[0] + BOX_HALF_SIZE * R_tcp[0][2]
        by = p_tcp[1] + BOX_HALF_SIZE * R_tcp[1][2]
        bz = p_tcp[2] + BOX_HALF_SIZE * R_tcp[2][2]

        # Clamp height slightly above support surfaces to prevent contact penetration
        if math.hypot(bx - self.spawn_pose[0], by - self.spawn_pose[1]) < 0.35:
            bz = max(bz, self.spawn_pose[2])
        elif 0.35 <= bx <= 1.25 and -0.65 <= by <= 0.65:
            bz = max(bz, tz_box)

        # Box orientation R_box = R_tcp * diag(1, -1, -1) * Rz(grasp_yaw_offset)
        R_base = [
            [R_tcp[0][0], -R_tcp[0][1], -R_tcp[0][2]],
            [R_tcp[1][0], -R_tcp[1][1], -R_tcp[1][2]],
            [R_tcp[2][0], -R_tcp[2][1], -R_tcp[2][2]],
        ]
        co, so = math.cos(self.grasp_yaw_offset), math.sin(self.grasp_yaw_offset)
        R_box = [
            [R_base[0][0] * co + R_base[0][1] * so, -R_base[0][0] * so + R_base[0][1] * co, R_base[0][2]],
            [R_base[1][0] * co + R_base[1][1] * so, -R_base[1][0] * so + R_base[1][1] * co, R_base[1][2]],
            [R_base[2][0] * co + R_base[2][1] * so, -R_base[2][0] * so + R_base[2][1] * co, R_base[2][2]],
        ]
        quat = rot_to_quat(R_box)
        self.send_box_pose(self.held_box, (bx, by, bz), quat)

        # Trigger next box spawn once carried > carry_away_distance from pickup
        dx = bx - self.spawn_pose[0]
        dy = by - self.spawn_pose[1]
        if (dx * dx + dy * dy) > self.carry_away_distance ** 2:
            self._maybe_spawn_next(self.held_box)

    def _maybe_spawn_next(self, carried_name):
        with self._lock:
            if (self.spawned == self.current_idx + (1 if self.held_box else 0)
                    and self.spawned < len(self.names)
                    and not self._spawning):
                nxt = self.names[self.spawned]
                self._spawning = True
                threading.Thread(
                    target=self._spawn_worker,
                    args=(nxt, carried_name),
                    daemon=True,
                ).start()

    def _spawn_worker(self, name, prev_name=None):
        ok, yaw = self.spawn_box(name)
        with self._lock:
            if ok:
                self.box_yaws[name] = yaw
                self.spawned += 1
                yaw_deg = math.degrees(yaw)
                if prev_name is None:
                    self.get_logger().info(
                        f'spawned {name} at {self.spawn_pose} yaw={yaw_deg:+.1f} deg')
                else:
                    self.get_logger().info(
                        f'{prev_name} carried away -> spawned {name} yaw={yaw_deg:+.1f} deg')
            self._spawning = False

    def update(self):
        # Flush any post-placement upright pose snaps
        with self._lock:
             pending = list(self._snap_queue)
             self._snap_queue.clear()
        requeue = []
        for name, pos, quat, rem in pending:
            self.send_box_pose(name, pos, quat, force=True)
            if rem > 1:
                requeue.append((name, pos, quat, rem - 1))
        if requeue:
            with self._lock:
                self._snap_queue.extend(requeue)

        if self.spawned == 0:
            with self._lock:
                if self._spawning:
                    return
            names = self.get_node_names()
            if 'arm_controller' not in names or 'joint_state_broadcaster' not in names:
                return
            with self._lock:
                self._spawning = True
            threading.Thread(
                target=self._spawn_worker,
                args=(self.names[0], None),
                daemon=True,
            ).start()
            return

        # Fallback poll if a box is moved externally while no box is held by the arm
        if (self.held_box is None
                and self.spawned < len(self.names)
                and self.current_idx < self.spawned):
            with self._lock:
                if self._spawning or self._polling:
                    return
                self._polling = True
            active = self.names[self.spawned - 1]
            threading.Thread(
                target=self._poll_worker,
                args=(active,),
                daemon=True,
            ).start()

    def _poll_worker(self, active):
        try:
            pos = self.query_pose(active)
            if pos is not None and self.held_box is None:
                dx = pos[0] - self.spawn_pose[0]
                dy = pos[1] - self.spawn_pose[1]
                if (dx * dx + dy * dy) > self.carry_away_distance ** 2:
                    with self._lock:
                        if self.spawned < len(self.names) and not self._spawning:
                            nxt = self.names[self.spawned]
                            self._spawning = True
                            if self.current_idx == self.spawned - 1:
                                self.current_idx = self.spawned
                            threading.Thread(
                                target=self._spawn_worker,
                                args=(nxt, active),
                                daemon=True,
                            ).start()
        finally:
            with self._lock:
                self._polling = False

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
        # Spawn with random yaw in [-35 deg, +35 deg] on the conveyor
        yaw = random.uniform(-math.radians(35.0), math.radians(35.0))
        cmd = ['ros2', 'run', 'ros_gz_sim', 'create',
               '-string', sdf, '-name', name, '-world', 'pick_and_place',
               '-x', str(x), '-y', str(y), '-z', str(z), '-Y', f'{yaw:.6f}']
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        except subprocess.TimeoutExpired:
            self.get_logger().warn(f'cannot spawn {name} yet: create timed out '
                                   '(world not ready?)')
            return False, 0.0
        except FileNotFoundError as exc:
            self.get_logger().warn(f'cannot spawn {name} yet: {exc}')
            return False, 0.0
        out = (result.stdout or '') + (result.stderr or '')
        if result.returncode != 0 or 'creation successful' not in out:
            self.get_logger().warn(f'spawn {name} failed rc={result.returncode}: '
                                   f'{out.strip()[-200:]}')
            return False, 0.0
        return True, yaw


def main(args=None):
    rclpy.init(args=args)
    node = ConveyorBoxes()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
