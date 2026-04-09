import ctypes
import time
import xr
from casadi import arctan2
from xr import Space
import numpy as np
import cv2


class Controller:
    def __init__(self):
        self.context = xr.ContextObject(
                            instance_create_info=xr.InstanceCreateInfo(
                                enabled_extension_names=[
                                    # A graphics extension is mandatory (without a headless extension)
                                    xr.KHR_OPENGL_ENABLE_EXTENSION_NAME,
                                ],
                            ),
                        )
        self.context.__enter__()
        self.controller_paths = (xr.Path * 2)(
            xr.string_to_path(self.context.instance, "/user/hand/left"),
            xr.string_to_path(self.context.instance, "/user/hand/right"),
        )
        self.pose_action = xr.create_action(
            action_set=self.context.default_action_set,
            create_info=xr.ActionCreateInfo(
                action_type=xr.ActionType.POSE_INPUT,
                action_name="hand_pose",
                localized_action_name="Hand Pose",
                subaction_paths=self.controller_paths,
                count_subaction_paths=len(self.controller_paths),
            ),
        )

        self.select_action = xr.create_action(
            action_set=self.context.default_action_set,
            create_info=xr.ActionCreateInfo(
                action_type=xr.ActionType.FLOAT_INPUT,
                action_name="hand_click",
                localized_action_name="Hand Click",
                subaction_paths=self.controller_paths,
                count_subaction_paths=len(self.controller_paths),
            )
        )

        self.x_action = xr.create_action(
            action_set=self.context.default_action_set,
            create_info=xr.ActionCreateInfo(
                action_type=xr.ActionType.BOOLEAN_INPUT,
                action_name="button1",
                localized_action_name="Button1",
                count_subaction_paths=0,
                subaction_paths=None,
            ),
        )

        self.y_action = xr.create_action(
            action_set=self.context.default_action_set,
            create_info=xr.ActionCreateInfo(
                action_type=xr.ActionType.BOOLEAN_INPUT,
                action_name="button2",
                localized_action_name="Button2",
                count_subaction_paths=0,
                subaction_paths=None,
            ),
        )

        self.a_action = xr.create_action(
            action_set=self.context.default_action_set,
            create_info=xr.ActionCreateInfo(
                action_type=xr.ActionType.BOOLEAN_INPUT,
                action_name="button3",
                localized_action_name="Button3",
                count_subaction_paths=0,
                subaction_paths=None,
            ),
        )

        self.b_action = xr.create_action(
            action_set=self.context.default_action_set,
            create_info=xr.ActionCreateInfo(
                action_type=xr.ActionType.BOOLEAN_INPUT,
                action_name="button4",
                localized_action_name="Button4",
                count_subaction_paths=0,
                subaction_paths=None,
            ),
        )

        self.joystick_x = xr.create_action(
            action_set=self.context.default_action_set,
            create_info=xr.ActionCreateInfo(
                action_type=xr.ActionType.FLOAT_INPUT,
                action_name="joystickx",
                localized_action_name="Joystickx",
                count_subaction_paths=0,
                subaction_paths=None,
            ),
        )

        self.joystick_y = xr.create_action(
            action_set=self.context.default_action_set,
            create_info=xr.ActionCreateInfo(
                action_type=xr.ActionType.FLOAT_INPUT,
                action_name="joysticky",
                localized_action_name="Joysticky",
                count_subaction_paths=0,
                subaction_paths=None,
            ),
        )

        self.action_spaces, self.frame_state = self.setup()

    def setup(self):
        # bind actions to profiles
        suggested_bindings = (xr.ActionSuggestedBinding * 10)(
            xr.ActionSuggestedBinding(
                action=self.pose_action,
                binding=xr.string_to_path(
                    instance=self.context.instance,
                    path_string="/user/hand/left/input/aim/pose",
                ),
            ),
            xr.ActionSuggestedBinding(
                action=self.select_action,
                binding=xr.string_to_path(
                    instance=self.context.instance,
                    path_string="/user/hand/left/input/trigger/value",
                ),
            ),
            xr.ActionSuggestedBinding(
                action=self.pose_action,
                binding=xr.string_to_path(
                    instance=self.context.instance,
                    path_string="/user/hand/right/input/aim/pose",
                ),
            ),
            xr.ActionSuggestedBinding(
                action=self.select_action,
                binding=xr.string_to_path(
                    instance=self.context.instance,
                    path_string="/user/hand/right/input/trigger/value",
                ),
            ),
            xr.ActionSuggestedBinding(
                action=self.a_action,
                binding=xr.string_to_path(
                    instance=self.context.instance,
                    path_string="/user/hand/right/input/a/click",
                ),
            ),
            xr.ActionSuggestedBinding(
                action=self.b_action,
                binding=xr.string_to_path(
                    instance=self.context.instance,
                    path_string="/user/hand/right/input/b/click",
                ),
            ),
            xr.ActionSuggestedBinding(
                action=self.x_action,
                binding=xr.string_to_path(
                    instance=self.context.instance,
                    path_string="/user/hand/left/input/x/click",
                ),
            ),
            xr.ActionSuggestedBinding(
                action=self.y_action,
                binding=xr.string_to_path(
                    instance=self.context.instance,
                    path_string="/user/hand/left/input/y/click",
                ),
            ),
            xr.ActionSuggestedBinding(
                action=self.joystick_x,
                binding=xr.string_to_path(
                    instance=self.context.instance,
                    path_string="/user/hand/right/input/thumbstick/x",
                ),
            ),
            xr.ActionSuggestedBinding(
                action=self.joystick_y,
                binding=xr.string_to_path(
                    instance=self.context.instance,
                    path_string="/user/hand/right/input/thumbstick/y",
                ),
            ),
        )

        xr.suggest_interaction_profile_bindings(
            instance=self.context.instance,
            suggested_bindings=xr.InteractionProfileSuggestedBinding(
                interaction_profile=xr.string_to_path(
                    self.context.instance,
                    "/interaction_profiles/oculus/touch_controller",
                ),
                count_suggested_bindings=len(suggested_bindings),
                suggested_bindings=suggested_bindings,
            ),
        )

        # create action space
        action_spaces = [
            xr.create_action_space(
                session=self.context.session,
                create_info=xr.ActionSpaceCreateInfo(
                    action=self.pose_action,
                    subaction_path=self.controller_paths[0],
                ),
            ),
            xr.create_action_space(
                session=self.context.session,
                create_info=xr.ActionSpaceCreateInfo(
                    action=self.pose_action,
                    subaction_path=self.controller_paths[1],
                ),
            ),
        ]

        self.head_space = xr.create_reference_space(
            session=self.context.session,
            create_info=xr.ReferenceSpaceCreateInfo(
                pose_in_reference_space=xr.Posef(
                    orientation=xr.Quaternionf(0,0,0,1),
                    position=xr.Vector3f(0,0,0)
                )
            )
        )


        # xr.attach_session_action_sets(
        #     session=self.context.session,
        #     attach_info=xr.SessionActionSetsAttachInfo(
        #         count_action_sets=len(self.context.action_sets),
        #         action_sets=(xr.ActionSet * len(self.context.action_sets))(
        #             *self.context.action_sets
        #         )
        #     ),
        # )
        # frame_wait_info = xr.FrameWaitInfo(None)
        # frame_state = xr.wait_frame(self.context.session, frame_wait_info)
        # xr.begin_frame(self.context.session, None)
        frame_state = None
        return action_spaces, frame_state

    def get_action(self):
        for frame_index, frame_state in enumerate(self.context.frame_loop()):
            if self.context.session_state == xr.SessionState.FOCUSED:
                # synchronize action
                active_action_set = xr.ActiveActionSet(
                    action_set=self.context.default_action_set,
                    subaction_path=xr.NULL_PATH,
                )
                xr.sync_actions(
                    session=self.context.session,
                    sync_info=xr.ActionsSyncInfo(
                        count_active_action_sets=1,
                        active_action_sets=ctypes.pointer(active_action_set),
                    ),
                )

                grab_value_left = xr.get_action_state_float(
                    self.context.session,
                    xr.ActionStateGetInfo(
                        action=self.select_action,
                        subaction_path=self.controller_paths[0],
                    ),
                )

                grab_value_right = xr.get_action_state_float(
                    self.context.session,
                    xr.ActionStateGetInfo(
                        action=self.select_action,
                        subaction_path=self.controller_paths[1],
                    ),
                )

                space_location1 = xr.locate_space(
                    space=self.action_spaces[0],
                    base_space=self.context.space,
                    time=frame_state.predicted_display_time,
                )

                space_location2 = xr.locate_space(
                    space=self.action_spaces[1],
                    base_space=self.context.space,
                    time=frame_state.predicted_display_time,
                )

                a_button = xr.get_action_state_boolean(
                    self.context.session,
                    xr.ActionStateGetInfo(
                        action=self.a_action,
                        subaction_path=xr.NULL_PATH,
                    ),
                )

                b_button = xr.get_action_state_boolean(
                    self.context.session,
                    xr.ActionStateGetInfo(
                        action=self.b_action,
                        subaction_path=xr.NULL_PATH,
                    ),
                )

                x_button = xr.get_action_state_boolean(
                    self.context.session,
                    xr.ActionStateGetInfo(
                        action=self.x_action,
                        subaction_path=xr.NULL_PATH,
                    ),
                )

                y_button = xr.get_action_state_boolean(
                    self.context.session,
                    xr.ActionStateGetInfo(
                        action=self.y_action,
                        subaction_path=xr.NULL_PATH,
                    ),
                )

                joystick_x = xr.get_action_state_float(
                    self.context.session,
                    xr.ActionStateGetInfo(
                        action=self.joystick_x,
                        subaction_path=xr.NULL_PATH,
                    ),
                )

                joystick_y = xr.get_action_state_float(
                    self.context.session,
                    xr.ActionStateGetInfo(
                        action=self.joystick_y,
                        subaction_path=xr.NULL_PATH,
                    ),
                )

                head_pose = xr.locate_views(
                    session=self.context.session,
                    view_locate_info=xr.ViewLocateInfo(
                        display_time=frame_state.predicted_display_time,
                        view_configuration_type=xr.ViewConfigurationType.PRIMARY_STEREO,
                        space=self.head_space,
                    )
                )

                # print(head_pose[1][0].pose.position)

                head_position = head_pose[1][0].pose.position
                head_orientation = head_pose[1][0].pose.orientation
                # print(f'head_ori',head_orientation)

                head_x, head_y, head_z = head_position.x, head_position.y, head_position.z
                # head_roll, head_pitch, head_yaw = euler_from_quaternion([head_orientation.x, head_orientation.y, head_orientation.z, head_orientation.w])

                # head_T = np.eye(4, 4)
                # head_rot = quaternion_to_rotation_matrix([head_orientation.w, head_orientation.x, head_orientation.y, head_orientation.z])
                # head_T[:3, :3] = head_rot
                # head_T[:3, 3] =np.array((head_x, head_y, head_z))

                angle = quaternion_to_axis_angle((head_orientation.w,head_orientation.x, head_orientation.y, head_orientation.z))

                # print(angle)
                # print(head_roll, head_pitch, head_yaw)
                # print( head_orientation.y)
                position1 = space_location1.pose.position
                orientation1 = space_location1.pose.orientation

                # left_hand_T = np.eye(4, 4)
                # left_hand_rot = quaternion_to_rotation_matrix([orientation1.w, orientation1.x, orientation1.y, orientation1.z])
                # left_hand_T[:3, :3] = left_hand_rot
                # left_hand_T[:3, 3] =np.array((position1.x, position1.y, position1.z))


                q1 = [orientation1.x, orientation1.y, orientation1.z, orientation1.w]
                q1_inv= transform_quaternion(q1,angle)
                roll1, pitch1, yaw1 = euler_from_quaternion(q1_inv)


                R_rot = rotate_y(angle)
                position1.x = position1.x - head_x
                position1.y = position1.y - head_y
                position1.z = position1.z - head_z
                r1 = np.array([position1.x, position1.y, position1.z])
                position1.x, position1.y, position1.z = np.dot(r1,R_rot)


                position2 = space_location2.pose.position
                orientation2 = space_location2.pose.orientation


                q2 = [orientation2.x, orientation2.y, orientation2.z, orientation2.w]
                q2_inv= transform_quaternion(q2,angle)
                roll2, pitch2, yaw2  = euler_from_quaternion(q2_inv)

                R_rot = rotate_y(angle)
                position2.x = position2.x - head_x
                position2.y = position2.y - head_y
                position2.z = position2.z - head_z
                r2 = np.array([position2.x, position2.y, position2.z])
                position2.x, position2.y, position2.z = np.dot(r2,R_rot)

                # right_hand_T = np.eye(4, 4)
                # right_hand_rot = quaternion_to_rotation_matrix([orientation2.w, orientation2.x, orientation2.y, orientation2.z])
                # right_hand_T[:3, :3] = right_hand_rot
                # right_hand_T[:3, 3] =np.array((position2.x, position2.y, position2.z))
                #

                # left_on_head = np.linalg.inv(head_T) @ left_hand_T
                # right_on_head = np.linalg.inv(head_T) @ right_hand_T

                # left_rpy = rotation_matrix_to_rpy(left_on_head[:3, :3])
                # left_pos = left_on_head[:3, 3]
                # right_rpy = rotation_matrix_to_rpy(right_on_head[:3, :3])
                # right_pos = right_on_head[:3, 3]

                data = [roll1, pitch1, yaw1, position1.x, position1.y, position1.z, grab_value_left.current_state,
                       roll2, pitch2, yaw2, position2.x, position2.y, position2.z, grab_value_right.current_state,
                       a_button.current_state, b_button.current_state, x_button.current_state, y_button.current_state, joystick_x.current_state, joystick_y.current_state,
                       -head_z, -head_x, angle, head_y, -head_x]

                # print('left:',data[:6], 'right',data[7:13])

                yield data

                # yield [left_rpy[0], left_rpy[1], left_rpy[2], left_pos[0], left_pos[1], left_pos[2], grab_value_left.current_state,
                #        right_rpy[0], right_rpy[1], right_rpy[2], right_pos[0], right_pos[1], right_pos[2], grab_value_right.current_state,
                #        a_button.current_state, b_button.current_state, x_button.current_state, y_button.current_state, joystick_x.current_state, joystick_y.current_state,
                #        -head_z, -head_x, angle, -head_z, -head_x]

                # yield [grab_value_left.current_state
                #        , grab_value_right.current_state]
                # yield space_location1, space_location2
            else:
                print('no controller active')


def euler_from_quaternion(q):
    import math
    x,y,z,w = q
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z

# def quaternion_to_rotation_matrix(q):
#     """
#     将四元数转换为旋转矩阵。
#
#     参数:
#     q -- 四元数，形状为 (4,) 或者 (w, x, y, z)
#
#     返回:
#     3x3 旋转矩阵
#     """
#     # 提取四元数的四个分量
#     w, x, y, z = q
#
#     # 计算旋转矩阵
#     R = np.array([
#         [1 - 2*(y**2 + z**2),  2*(x*y - w*z),  2*(x*z + w*y)],
#         [2*(x*y + w*z),  1 - 2*(x**2 + z**2),  2*(y*z - w*x)],
#         [2*(x*z - w*y),  2*(y*z + w*x),  1 - 2*(x**2 + y**2)]
#     ])
#
#     return R


def quaternion_to_axis_angle(q):
    """
    将四元数转换为轴角表示的旋转。

    参数:
    q -- 四元数，形式为 [w, x, y, z]，其中 w 是标量部分，x, y, z 是向量部分。

    返回:
    axis -- 旋转轴，单位向量。
    angle -- 旋转角度，单位为弧度。
    """
    w, x, y, z = q
    # 计算旋转角度
    angle = 2 * np.arccos(w)*y/np.abs(y)
    theta = np.arcsin(x**2+z**2)/(y**2)
    angle =angle*np.cos(theta)

    #使用角度分量
    Zx = 2 * x * z + 2 * y * w
    Zz =  1 - 2 * x ** 2 - 2 * y ** 2
    angle2 = arctan2(Zx,Zz)
    # print('angle2',angle2)
    return  angle2


def quaternion_to_rotation_matrix(q):
    x, y, z, w = q
    R = np.array([
        [1 - 2 * y ** 2 - 2 * z ** 2, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
        [2 * x * y + 2 * z * w, 1 - 2 * x ** 2 - 2 * z ** 2, 2 * y * z - 2 * x * w],
        [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x ** 2 - 2 * y ** 2]
    ])
    return R

def rotation_matrix_to_rpy(R):
    """
    将旋转矩阵转换为RPY（滚转、俯仰、偏航）角度。

    参数:
    R -- 3x3 旋转矩阵

    返回:
    roll, pitch, yaw -- 分别对应绕 X、Y、Z 轴的旋转角度（单位：弧度）
    """
    # 确保 R 是一个 3x3 的矩阵
    assert R.shape == (3, 3), "输入必须是一个 3x3 旋转矩阵"

    # 计算滚转（Roll）角
    roll = np.arctan2(R[2, 1], R[2, 2])

    # 计算俯仰（Pitch）角
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[0, 0]**2 + R[1, 0]**2))

    # 计算偏航（Yaw）角
    yaw = np.arctan2(R[1, 0], R[0, 0])

    return roll, pitch, yaw


def rotation_matrix_to_quaternion(R):
    m00, m01, m02 = R[0]
    m10, m11, m12 = R[1]
    m20, m21, m22 = R[2]

    qw = np.sqrt(1.0 + m00 + m11 + m22) / 2.0
    qx = (m21 - m12) / (4.0 * qw)
    qy = (m02 - m20) / (4.0 * qw)
    qz = (m10 - m01) / (4.0 * qw)

    return np.array([qx, qy, qz, qw])


def rotate_y(phi):
    cos_phi = np.cos(phi)
    sin_phi = np.sin(phi)
    R_y = np.array([
        [cos_phi, 0, sin_phi],
        [0, 1, 0],
        [-sin_phi, 0, cos_phi]
    ])
    return R_y


def transform_quaternion(q, phi):
    R1 = quaternion_to_rotation_matrix(q)
    R2 = rotate_y(phi)
    R2_inv = np.linalg.inv(R2)
    R_total = np.dot(R2_inv, R1)
    q_total = rotation_matrix_to_quaternion(R_total)
    # print(q_total)

    return q_total

