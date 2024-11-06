import math
import select
import threading
import time

import numpy as np
import torch

from go2_arx_deploy.lcm_types.leg_control_data_lcmt import leg_control_data_lcmt
from go2_arx_deploy.lcm_types.rc_command_lcmt import rc_command_lcmt
from go2_arx_deploy.lcm_types.state_estimator_lcmt import state_estimator_lcmt
from go2_arx_deploy.lcm_types.camera_message_lcmt import camera_message_lcmt
from go2_arx_deploy.lcm_types.camera_message_rect_wide import camera_message_rect_wide
from go2_arx_deploy.lcm_types.vr_command_lcmt import vr_command_lcmt


def quat_apply(a, b):
    if not isinstance(a, torch.Tensor):
        a = torch.tensor(a)
    if not isinstance(b, torch.Tensor):
        b = torch.tensor(b)
    shape = b.shape
    a = a.reshape(-1, 4)
    b = b.reshape(-1, 3)
    xyz = a[:, :3]
    t = xyz.cross(b, dim=-1) * 2
    return (b + a[:, 3:] * t + xyz.cross(t, dim=-1)).view(shape)

def get_rpy_from_quaternion(q):
    w, x, y, z = q
    r = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x ** 2 + y ** 2))
    p = np.arcsin(2 * (w * y - z * x))
    y = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y ** 2 + z ** 2))
    return np.array([r, p, y])


def get_rotation_matrix_from_rpy(rpy):
    """
    Get rotation matrix from the given quaternion.
    Args:
        q (np.array[float[4]]): quaternion [w,x,y,z]
    Returns:
        np.array[float[3,3]]: rotation matrix.
    """
    r, p, y = rpy
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(r), -math.sin(r)],
                    [0, math.sin(r), math.cos(r)]
                    ])

    R_y = np.array([[math.cos(p), 0, math.sin(p)],
                    [0, 1, 0],
                    [-math.sin(p), 0, math.cos(p)]
                    ])

    R_z = np.array([[math.cos(y), -math.sin(y), 0],
                    [math.sin(y), math.cos(y), 0],
                    [0, 0, 1]
                    ])

    rot = np.dot(R_z, np.dot(R_y, R_x))
    return rot


def quat_from_euler_xyz(roll, pitch, yaw):
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)

    qw = cy * cr * cp + sy * sr * sp
    qx = cy * sr * cp - sy * cr * sp
    qy = cy * cr * sp + sy * sr * cp
    qz = sy * cr * cp - cy * sr * sp

    return np.stack([qx, qy, qz, qw], axis=-1)

def quat_mul(a, b):
    assert a.shape == b.shape
    shape = a.shape
    a = a.reshape(-1, 4)
    b = b.reshape(-1, 4)

    x1, y1, z1, w1 = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
    x2, y2, z2, w2 = b[:, 0], b[:, 1], b[:, 2], b[:, 3]
    ww = (z1 + x1) * (x2 + y2)
    yy = (w1 - y1) * (w2 + z2)
    zz = (w1 + y1) * (w2 - z2)
    xx = ww + yy + zz
    qq = 0.5 * (xx + (z1 - x1) * (x2 - y2))
    w = qq - ww + (z1 - y1) * (y2 - z2)
    x = qq - xx + (x1 + w1) * (x2 + w2)
    y = qq - yy + (w1 - x1) * (y2 + z2)
    z = qq - zz + (z1 + y1) * (w2 - x2)

    quat = np.stack([x, y, z, w], axis=-1).reshape(shape)

    return quat

def quat_to_angle(quat):
    y_vector = torch.tensor([0., 1., 0.]).double()
    z_vector = torch.tensor([0., 0., 1.]).double()
    x_vector = torch.tensor([1., 0., 0.]).double()
    roll_vec = quat_apply(quat, y_vector) # [0,1,0]
    roll = torch.atan2(roll_vec[2], roll_vec[1]) # roll angle = arctan2(z, y)
    pitch_vec = quat_apply(quat, z_vector) # [0,0,1]
    pitch = torch.atan2(pitch_vec[0], pitch_vec[2]) # pitch angle = arctan2(x, z)
    yaw_vec = quat_apply(quat, x_vector) # [1,0,0]
    yaw = torch.atan2(yaw_vec[1], yaw_vec[0]) # yaw angle = arctan2(y, x)

    return torch.stack([roll, pitch, yaw], dim=-1)

def rpy_to_abg(roll, pitch, yaw):
    zero_vec = np.zeros_like(roll)
    q1 = quat_from_euler_xyz(zero_vec, zero_vec, yaw)
    q2 = quat_from_euler_xyz(zero_vec, pitch, zero_vec)
    q3 = quat_from_euler_xyz(roll, zero_vec, zero_vec)
    quats = quat_mul(q1, quat_mul(q2, q3))  # np, (4,)
    abg = quat_to_angle(quats).numpy()

    return abg

class StateEstimator:
    def __init__(self, lc, use_cameras=True):

        # reverse legs
        self.joint_idxs = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]
        self.wb_joint_idxs = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8,
                              12, 13, 14, 15, 16, 17]
        self.contact_idxs = [1, 0, 3, 2]

        self.lc = lc

        self.joint_pos = np.zeros(18)
        self.joint_vel = np.zeros(18)
        self.tau_est = np.zeros(12)
        self.world_lin_vel = np.zeros(3)
        self.world_ang_vel = np.zeros(3)
        self.euler = np.zeros(3)
        self.R = np.eye(3)
        self.buf_idx = 0

        self.smoothing_length = 12
        self.deuler_history = np.zeros((self.smoothing_length, 3))
        self.dt_history = np.zeros((self.smoothing_length, 1))
        self.euler_prev = np.zeros(3)
        self.timuprev = time.time()

        self.body_lin_vel = np.zeros(3)
        self.body_ang_vel = np.zeros(3)
        self.smoothing_ratio = 0.2

        self.contact_state = np.ones(4)

        self.mode = 0
        self.ctrlmode_left = 0
        self.ctrlmode_right = 0
        self.left_stick = [0, 0]
        self.right_stick = [0, 0]
        self.left_upper_switch = 0
        self.left_lower_left_switch = 0
        self.left_lower_right_switch = 0
        self.right_upper_switch = 0
        self.right_lower_left_switch = 0
        self.right_lower_right_switch = 0
        self.left_upper_switch_pressed = 0
        self.left_lower_left_switch_pressed = 0
        self.left_lower_right_switch_pressed = 0
        self.right_upper_switch_pressed = 0
        self.right_lower_left_switch_pressed = 0
        self.right_lower_right_switch_pressed = 0


        self.init_time = time.time()
        self.received_first_legdata = False
        self.received_first_armdata = False

        self.imu_subscription = self.lc.subscribe("leg_state_estimator_data", self._imu_cb)
        self.legdata_state_subscription = self.lc.subscribe("leg_control_data", self._legdata_cb)
        self.rc_command_subscription = self.lc.subscribe("rc_command", self._rc_command_cb)

        self.body_loc = np.array([0, 0, 0])
        self.body_quat = np.array([0, 0, 0, 1])

        self.cmd_l = 0.5
        self.cmd_p = 0.3
        self.cmd_y = 0.0
        self.cmd_alpha = 0.0
        self.cmd_beta = 0.0
        self.cmd_gamma = 0.0

        self.cmd_dx = 0.0
        self.cmd_dy = 0.0
        self.cmd_dz = 0.0


    def get_body_linear_vel(self):
        self.body_lin_vel = np.dot(self.R.T, self.world_lin_vel)
        return self.body_lin_vel

    def get_body_angular_vel(self):
        self.body_ang_vel = self.smoothing_ratio * np.mean(self.deuler_history / self.dt_history, axis=0) + (
                    1 - self.smoothing_ratio) * self.body_ang_vel
        return self.body_ang_vel

    def get_gravity_vector(self):
        grav = np.dot(self.R.T, np.array([0, 0, -1]))
        return grav

    def get_contact_state(self):
        return self.contact_state[self.contact_idxs]

    def get_rpy(self):
        return self.euler


    def get_command(self):
        commands_list = [
            [0.5, 0.3, 0, 0, 0, 0],
        ]


        # always in use
        cmd_x = 1 * self.left_stick[1]  # -1 ~ 1
        cmd_y = -1 * self.left_stick[0]  # -1 ~ 1
        cmd_yaw = -1 * self.right_stick[0]  # -1 ~ 1


        if self.right_upper_switch_pressed: # R1
            print("reset origin")
            self.right_upper_switch_pressed = False
            commands = commands_list[0]
            self.cmd_l, self.cmd_p, self.cmd_y, self.cmd_alpha, self.cmd_beta, self.cmd_gamma = commands

        if self.left_upper_switch_pressed: # L1
            self.ctrlmode_right = (self.ctrlmode_right + 1) % len(commands_list)
            self.left_upper_switch_pressed = False
            commands = commands_list[self.ctrlmode_right]
            self.cmd_l, self.cmd_p, self.cmd_y, self.cmd_alpha, self.cmd_beta, self.cmd_gamma = commands
            print(f"commands: {commands}")

        self.cmd_alpha, self.cmd_beta, self.cmd_gamma = rpy_to_abg(self.cmd_alpha, self.cmd_beta, self.cmd_gamma)

        return np.array([cmd_x, cmd_y, cmd_yaw, self.cmd_l, self.cmd_p, self.cmd_y, self.cmd_alpha, self.cmd_beta, self.cmd_gamma])


    def get_buttons(self):
        return np.array([self.left_lower_left_switch, self.left_upper_switch, self.right_lower_right_switch, self.right_upper_switch])

    def get_dof_pos(self):
        return self.joint_pos[self.wb_joint_idxs]

    def get_dof_vel(self):
        return self.joint_vel[self.wb_joint_idxs]

    def get_tau_est(self):
        return self.tau_est[self.joint_idxs]

    def get_yaw(self):
        return self.euler[2]

    def get_body_loc(self):
        return np.array(self.body_loc)

    def get_body_quat(self):
        return np.array(self.body_quat)

    def get_camera_front(self):
        return self.camera_image_front

    def get_camera_bottom(self):
        return self.camera_image_bottom

    def get_camera_rear(self):
        return self.camera_image_rear

    def get_camera_left(self):
        return self.camera_image_left

    def get_camera_right(self):
        return self.camera_image_right

    def _legdata_cb(self, channel, data):
        if not self.received_first_legdata:
            self.received_first_legdata = True
            print(f"First legdata: {time.time() - self.init_time}")

        msg = leg_control_data_lcmt.decode(data)
        self.joint_pos[:12] = np.array(msg.q)[:12]
        self.joint_vel[:12] = np.array(msg.qd)[:12]
        self.joint_pos[12:] = np.array(msg.q_arm)[:6]
        self.joint_vel[12:] = 0.
        self.tau_est = np.array(msg.tau_est)


    def _imu_cb(self, channel, data):
        msg = state_estimator_lcmt.decode(data)

        self.euler = np.array(msg.rpy)

        self.R = get_rotation_matrix_from_rpy(self.euler)

        self.contact_state = 1.0 * (np.array(msg.contact_estimate) > 200)

        self.deuler_history[self.buf_idx % self.smoothing_length, :] = msg.rpy - self.euler_prev
        self.dt_history[self.buf_idx % self.smoothing_length] = time.time() - self.timuprev

        self.timuprev = time.time()

        self.buf_idx += 1
        self.euler_prev = np.array(msg.rpy)

    def _sensor_cb(self, channel, data):
        pass

    def _rc_command_cb(self, channel, data):

        msg = rc_command_lcmt.decode(data)

        self.left_upper_switch_pressed = ((msg.left_upper_switch and not self.left_upper_switch) or self.left_upper_switch_pressed)
        self.left_lower_left_switch_pressed = ((msg.left_lower_left_switch and not self.left_lower_left_switch) or self.left_lower_left_switch_pressed)
        self.left_lower_right_switch_pressed = ((msg.left_lower_right_switch and not self.left_lower_right_switch) or self.left_lower_right_switch_pressed)
        self.right_upper_switch_pressed = ((msg.right_upper_switch and not self.right_upper_switch) or self.right_upper_switch_pressed)
        self.right_lower_left_switch_pressed = ((msg.right_lower_left_switch and not self.right_lower_left_switch) or self.right_lower_left_switch_pressed)
        self.right_lower_right_switch_pressed = ((msg.right_lower_right_switch and not self.right_lower_right_switch) or self.right_lower_right_switch_pressed)

        self.mode = msg.mode
        self.right_stick = msg.right_stick
        self.left_stick = msg.left_stick
        self.left_upper_switch = msg.left_upper_switch
        self.left_lower_left_switch = msg.left_lower_left_switch
        self.left_lower_right_switch = msg.left_lower_right_switch
        self.right_upper_switch = msg.right_upper_switch
        self.right_lower_left_switch = msg.right_lower_left_switch
        self.right_lower_right_switch = msg.right_lower_right_switch


    def poll(self, cb=None):
        t = time.time()
        try:
            while True:
                timeout = 0.01
                rfds, wfds, efds = select.select([self.lc.fileno()], [], [], timeout)
                if rfds:
                    self.lc.handle()
                else:
                    continue

        except KeyboardInterrupt:
            pass

    def spin(self):
        self.run_thread = threading.Thread(target=self.poll, daemon=False)
        self.run_thread.start()

    def close(self):
        self.lc.unsubscribe(self.legdata_state_subscription)


class StateEstimator_VR(StateEstimator):
    def __init__(self, lc, use_cameras=True):
        super().__init__(lc, use_cameras)
        self.vr_command_subscription = self.lc.subscribe("vr_command", self._vr_command_cb)
        self.delta_xyzrpy = np.zeros(6)


    def _vr_command_cb(self, channel, data):
        msg = vr_command_lcmt.decode(data)
        self.delta_xyzrpy = np.array(msg.ee_pose)[:6]


    def get_command(self):
        # always in use
        cmd_vel_x = 1 * self.left_stick[1]  # -1 ~ 1
        cmd_vel_y = -1 * self.left_stick[0]  # -1 ~ 1
        cmd_vel_yaw = -1 * self.right_stick[0]  # -1 ~ 1


        delta_x1, delta_y1, delta_z1, delta_roll, delta_pitch, delta_yaw = self.delta_xyzrpy
        delta_x1 += 0.3
        delta_l = np.sqrt(delta_x1**2 + delta_y1**2 + delta_z1**2)
        delta_y = np.arctan2(delta_y1, delta_x1)
        delta_p = np.arcsin(delta_z1 / delta_l) if delta_l != 0 else 0


        cmd_l = min(max(delta_l + 0.2, 0.3), 0.8)  # default=0.5, [0.3 ~ 0.8]
        cmd_p = min(max(delta_p + 0.3, -np.pi/3), np.pi/3)   # default=0.2, [-pi/3 ~ pi/3]
        cmd_y = min(max(delta_y, -np.pi/2), np.pi/2)  # default=0., [-pi/2 ~ pi/2]

        cmd_alpha = min(max(delta_roll, -np.pi * 0.45), np.pi * 0.45)  # default=0., [-pi/3 ~ pi/3]
        cmd_beta = min(max(delta_pitch, -1.5), 1.5)  # default=0., [-1.5 ~ 1.5]
        cmd_gamma = min(max(delta_yaw, -1.4), 1.4) # default=0., [-1.4 ~ 1.4]

        cmd_alpha, cmd_beta, cmd_gamma = rpy_to_abg(cmd_alpha, cmd_beta, cmd_gamma)

        cmd_alpha, cmd_beta, cmd_gamma = rpy_to_abg(cmd_alpha, cmd_beta, cmd_gamma)

        cmd_vel_x = np.clip(cmd_vel_x, -0.2, 1.5)
        cmd_vel_y = np.clip(cmd_vel_y, -0.2, 0.2)
        cmd_vel_yaw = np.clip(cmd_vel_yaw, -1.5, 1.5)

        return np.array([cmd_vel_x, cmd_vel_y, cmd_vel_yaw, cmd_l, cmd_p, cmd_y, cmd_alpha, cmd_beta, cmd_gamma])


if __name__ == "__main__":
    import lcm

    lc = lcm.LCM("udpm://239.255.76.67:7314?ttl=255")
    se = StateEstimator(lc)
    se.poll()
