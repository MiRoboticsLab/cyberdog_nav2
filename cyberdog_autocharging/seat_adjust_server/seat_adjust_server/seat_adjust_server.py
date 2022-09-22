# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import time

import std_msgs.msg as std_msgs
from rclpy.node import Node
import numpy as np

import sensor_msgs.msg as sensor_msgs

from protocol.msg import HeadTofPayload  # CHANGE
from protocol.msg import RearTofPayload  # CHANGE
from protocol.msg import SingleTofPayload  # CHANGE
from protocol.srv import MotionResultCmd  # CHANGE
from protocol.action import SeatAdjust

# from protocol.lcm import tof_lcmt    # CHANGE
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

z_array = [0] * 64
x_array = [0] * 64
y_array = [0] * 64
r_array = [0] * 64
indensity = [0] * 64


def GetRotationMatrix(theta_x, theta_y, theta_z):
    sx = np.sin(theta_x)
    cx = np.cos(theta_x)
    sy = np.sin(theta_y)
    cy = np.cos(theta_y)
    sz = np.sin(theta_z)
    cz = np.cos(theta_z)
    return np.array(
        [
            [cy * cz, cz * sx * sy - cx * sz, sx * sz + cx * cz * sy],
            [cy * sz, cx * cz + sx * sy * sz, cx * sy * sz - cz * sx],
            [-sy, cy * sx, cx * cy],
        ]
    )


left_head_t = np.array([0.259, 0.03, 0.102])
left_head_R = GetRotationMatrix(0.296, -0.266, 0.0)

right_head_t = np.array([0.259, -0.03, 0.102])
right_head_R = GetRotationMatrix(-0.296, -0.266, 0.0)

left_rear_t = np.array([-0.021, 0.042, -0.051])
left_rear_R = GetRotationMatrix(0.296, 0, 0.0)

right_rear_t = np.array([-0.021, -0.042, -0.051])
right_rear_R = GetRotationMatrix(-0.296, 0, 0.0)

row_depth_sum = np.zeros((4, 8))
col_depth_sum = np.zeros((4, 8))

every_point_depth = np.zeros((4, 64))

row_indensity_sum = np.zeros((4, 8))
col_indensity_sum = np.zeros((4, 8))

points_depth = np.zeros((4, 64))

trig_coeffs = [0] * 8


def compute_trig_coeffs():
    for i in range(0, 8):
        zone_centre_um = (i * 2) - 8 + 1
        zone_centre_um = (zone_centre_um * 4 * 15790) / 2
        trig_coeff = zone_centre_um * 4096
        if trig_coeff < 0:
            trig_coeff = trig_coeff - (587601 / 2)
        else:
            trig_coeff = trig_coeff + (587601 / 2)
        trig_coeff = trig_coeff / 587601
        trig_coeffs[i] = trig_coeff


def compute_xy(z, scale):
    temp = z * scale
    if temp < 0:
        temp = temp - 2048
    else:
        temp = temp + 2048
    temp = temp / 4096
    if temp < -32768:
        temp = -32768
    if temp > 32767:
        temp = 32767
    return temp


start_end_row_idx = np.zeros((4, 2))
timer_period = 0.2  # seconds
adjust_times = (int)(10 / timer_period)


class SeatAdjustServer(Node):
    def __init__(self):
        super().__init__("seat_adjust_server")
        self._action_server = ActionServer(
            self,
            SeatAdjust,
            "seatadjust",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.head_subscription = self.create_subscription(
            HeadTofPayload, "head_tof_payload", self.head_listener_callback, 10
        )

        self.rear_subscription = self.create_subscription(
            RearTofPayload, "rear_tof_payload", self.rear_listener_callback, 10
        )

        self.motion_result_client_ = self.create_client(
            MotionResultCmd, "/motion_result_cmd"
        )
        while not self.motion_result_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("motion service not available, waiting again...")

        compute_trig_coeffs()

        self.yaw_limit = 25 * 3.14 / 180
        self.yaw_adjust_sum = 0

        self.timer_nums = 0
        self.pcd_topic = []
        self.pcd_topic.append("left_head_pcd")
        self.pcd_topic.append("right_head_pcd")
        self.pcd_topic.append("left_rear_pcd")
        self.pcd_topic.append("right_rear_pcd")

        self.trans_R = []
        self.trans_R.append(left_head_R)
        self.trans_R.append(right_head_R)
        self.trans_R.append(left_rear_R)
        self.trans_R.append(right_rear_R)

        self.trans_t = []
        self.trans_t.append(left_head_t)
        self.trans_t.append(right_head_t)
        self.trans_t.append(left_rear_t)
        self.trans_t.append(right_rear_t)

        self.pcd_publisher = []
        for i in range(0, 4):
            pcd_publisher = self.create_publisher(
                sensor_msgs.PointCloud2, self.pcd_topic[i], 10
            )
            self.pcd_publisher.append(pcd_publisher)

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timer_count = 0
        self.action_start = 0
        self.action_complete = 0
        self.get_logger().info("__init__ complete")

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.action_start = 1
        self.action_complete = 0
        self.get_logger().info("Received goal request")

        self.motion_req = MotionResultCmd.Request()
        self.motion_req.motion_id = 111
        self.motion_req.vel_des = [0.0, 0.0, 0.0]
        self.motion_req.rpy_des = [0.0, 0.0, 0.0]
        self.motion_req.pos_des = [0.0, 0.0, 0.0]
        self.motion_req.acc_des = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.motion_req.ctrl_point = [0.0, 0.0, 0.0]
        self.motion_req.foot_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.motion_req.step_height = [0.0, 0.0]
        self.motion_req.duration = 0
        self.send_motion_request()

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    async def execute_callback(self, goal_handle):
        """Execute a goal."""
        self.get_logger().info("Executing goal...")

        # Append the seeds for the Fibonacci sequence
        feedback_msg = SeatAdjust.Feedback()
        feedback_msg.count = 0

        # Start executing the action
        # for i in range(1, goal_handle.request.order):
        while self.action_complete == 0:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Goal canceled")
                return SeatAdjust.Result()

            self.get_logger().info(
                "Publishing feedback: {0}".format(feedback_msg.count)
            )

            # Publish the feedback
            goal_handle.publish_feedback(feedback_msg)
            feedback_msg.count = feedback_msg.count + 1
            # Sleep for demonstration purposes
            time.sleep(timer_period)

        goal_handle.succeed()

        # Populate result message
        result = SeatAdjust.Result()
        result.result = result.SEATADJUST_RESULT_TYPE_SUCCESS

        self.get_logger().info("Returning result: {0}".format(result.result))

        return result

    def final_sit_down_action(self):
        self.motion_req.pos_des[0] = 0
        self.motion_req.pos_des[1] = 0
        self.motion_req.pos_des[2] = -0.05
        self.motion_req.rpy_des[2] = 0
        self.motion_req.motion_id = 212
        self.motion_req.duration = (int)(timer_period * 1000 - 10)
        self.send_motion_request()

        self.motion_req.pos_des[0] = 0
        self.motion_req.pos_des[1] = 0
        self.motion_req.pos_des[2] = 0
        self.motion_req.rpy_des[2] = 0
        self.motion_req.motion_id = 101
        self.motion_req.duration = 0
        self.send_motion_request()

    def send_motion_request(self):
        self.future = self.motion_result_client_.call_async(self.motion_req)

    def tof_pcd_generate(self, tof_msg):
        indensity_threshold = 3
        intensity_set_value = -0.2
        tof_position = tof_msg.tof_position
        # tof_time = (
        #     tof_msg.header.stamp.nanosec / 1000000 + tof_msg.header.stamp.sec * 1000
        # )
        for idx in range(0, 64):
            if (tof_position == SingleTofPayload.RIGHT_HEAD) or (
                tof_position == SingleTofPayload.LEFT_HEAD
            ):
                indensity[idx] = (
                    intensity_set_value
                    if ((tof_msg.intensity[idx]) > indensity_threshold)
                    else 0
                )
                z_array[idx] = -tof_msg.data[idx]
                x_array[idx] = (
                    compute_xy(1000 * z_array[idx], trig_coeffs[7 - (int)(idx % 8)])
                    / 1000
                )
                y_array[idx] = (
                    compute_xy(1000 * z_array[idx], trig_coeffs[(int)(idx / 8)]) / 1000
                )
            else:
                indensity[idx] = (
                    intensity_set_value
                    if ((tof_msg.intensity[63 - idx]) > indensity_threshold)
                    else 0
                )
                z_array[idx] = -tof_msg.data[63 - idx]
                x_array[idx] = (
                    compute_xy(1000 * z_array[idx], trig_coeffs[7 - (int)(idx % 8)])
                    / 1000
                )
                y_array[idx] = (
                    compute_xy(1000 * z_array[idx], trig_coeffs[(int)(idx / 8)]) / 1000
                )
            x_array[idx], y_array[idx], z_array[idx] = (
                np.dot(
                    self.trans_R[tof_position],
                    np.array([x_array[idx], y_array[idx], z_array[idx]]),
                )
                + self.trans_t[tof_position]
            )
        points_depth = np.vstack(
            (np.asarray(x_array), np.asarray(y_array), np.asarray(z_array))
        ).T
        points_indensity = np.vstack(
            (np.asarray(x_array), (np.asarray(y_array) - 0.5), np.asarray(indensity))
        ).T
        points = np.vstack((points_depth, points_indensity))

        pcd = point_cloud(points, "base_link")
        self.pcd_publisher[tof_position].publish(pcd)
        self.point_depth_compute(points_depth, tof_msg)
        self.point_indensity_compute(indensity, tof_msg)

    def head_listener_callback(self, msg):
        self.tof_pcd_generate(msg.left_head)
        self.tof_pcd_generate(msg.right_head)

    def rear_listener_callback(self, msg):
        self.tof_pcd_generate(msg.left_rear)
        self.tof_pcd_generate(msg.right_rear)

    def x_pose_adjust(self):
        tag_row_idx = 4
        max_points_num = 0
        points_num_threshold = 5
        move_step_length = 0.002
        for idx in range(0, 8):
            points_num = row_indensity_sum[2, idx] + row_indensity_sum[3, idx]
            tag_row_idx = (
                idx
                if (
                    (points_num >= points_num_threshold)
                    and (points_num > max_points_num)
                )
                else tag_row_idx
            )
            max_points_num = points_num
            self.get_logger().info("idx:%d  " % idx + "points_num:%d  " % points_num)
        self.motion_req.pos_des[0] = move_step_length * (tag_row_idx - 4)

    def y_pose_adjust(self):
        find_first_edge = 0
        first_edge_idx = 0
        last_edge_idx = 0
        judge_threshold = 0.01
        right_tof_depth = col_depth_sum[3]
        col_depth_sum_edge = (col_depth_sum[2] - right_tof_depth[::-1]) / 8
        for idx in range(0, 8):
            if abs(col_depth_sum_edge[idx]) > judge_threshold and (not find_first_edge):
                first_edge_idx = idx
                find_first_edge = 1
                last_edge_idx = first_edge_idx
            elif find_first_edge and abs(col_depth_sum_edge[idx]) > judge_threshold:
                last_edge_idx = idx
        delta_y = (
            col_depth_sum_edge[first_edge_idx]
            / abs(col_depth_sum_edge[first_edge_idx])
            * ((last_edge_idx - first_edge_idx) / 2)
        )
        self.motion_req.pos_des[1] = 0.002 * delta_y

    def yaw_adjust(self):
        # for idx in range(0,8):
        # threshold = 2.5
        left_rear_depth_8x8 = np.zeros((8, 8))
        right_rear_depth_8x8 = np.zeros((8, 8))
        diff_left_tof_depth = np.zeros((4, 8))
        diff_right_tof_depth = np.zeros((4, 8))
        depth_threshold = 0.015
        point_num_threshold = 2
        # heigh = 0.08
        step_length = 0.5
        for point_idx in range(0, 64):
            left_rear_depth_8x8[
                int(point_idx % 8), int(point_idx / 8)
            ] = every_point_depth[2, point_idx]
            right_rear_depth_8x8[
                int(point_idx % 8), int(point_idx / 8)
            ] = every_point_depth[3, point_idx]
        for row_num in range(0, 4):
            diff_left_tof_depth[row_num] = (
                left_rear_depth_8x8[row_num, :] - left_rear_depth_8x8[4 + row_num, :]
            )
            diff_right_tof_depth[row_num] = (
                right_rear_depth_8x8[row_num, :] - right_rear_depth_8x8[4 + row_num, :]
            )

        # diff_left_points_num = sum(abs(diff_left_tof_depth.flatten()) > 0.015)
        # diff_right_points_num = sum(abs(diff_right_tof_depth.flatten()) > 0.015)
        deviation_left_num = sum(
            (diff_left_tof_depth.flatten()) < -1 * depth_threshold
        ) + sum((diff_right_tof_depth.flatten()) > depth_threshold)
        deviation_right_num = sum(
            (diff_left_tof_depth.flatten()) > depth_threshold
        ) + sum((diff_right_tof_depth.flatten()) < -1 * depth_threshold)

        self.get_logger().info(
            "diff_left_points_num:%d  " % deviation_left_num
            + "diff_right_points_num:%d  " % deviation_right_num
        )
        need_adjust_points = max(deviation_left_num, deviation_right_num)
        delta_yaw = need_adjust_points / 2
        if (
            deviation_right_num > deviation_left_num
            and need_adjust_points >= point_num_threshold
        ):
            self.motion_req.rpy_des[2] = step_length * 3.14 / 180 * delta_yaw
        elif (
            deviation_left_num > deviation_right_num
            and need_adjust_points >= point_num_threshold
        ):
            self.motion_req.rpy_des[2] = -step_length * 3.14 / 180 * delta_yaw
        else:
            self.motion_req.rpy_des[2] = 0.00
        self.yaw_adjust_sum = self.yaw_adjust_sum + self.motion_req.rpy_des[2]
        self.motion_req.rpy_des[2] = (
            self.motion_req.rpy_des[2]
            if (abs(self.yaw_adjust_sum) < self.yaw_limit)
            else 0
        )

    def point_depth_compute(self, points, tof_msg):
        for row_col_idx in range(0, 8):
            row_depth_sum[tof_msg.tof_position, row_col_idx] = 0
            col_depth_sum[tof_msg.tof_position, row_col_idx] = 0
            for point_idx in range(0, 64):
                every_point_depth[tof_msg.tof_position, point_idx] = points[
                    point_idx, 2
                ]
                if int(point_idx % 8) == row_col_idx:
                    row_depth_sum[tof_msg.tof_position, int(point_idx % 8)] = (
                        row_depth_sum[tof_msg.tof_position, int(point_idx % 8)]
                        + points[point_idx, 2]
                    )
                if int(point_idx / 8) == row_col_idx:
                    col_depth_sum[tof_msg.tof_position, int(point_idx / 8)] = (
                        col_depth_sum[tof_msg.tof_position, int(point_idx / 8)]
                        + points[point_idx, 2]
                    )

    def point_indensity_compute(self, indensity, tof_msg):
        for row_col_idx in range(0, 8):
            row_indensity_sum[tof_msg.tof_position, row_col_idx] = 0
            col_indensity_sum[tof_msg.tof_position, row_col_idx] = 0
            for point_idx in range(0, 64):
                if int(point_idx % 8) == row_col_idx:
                    row_indensity_sum[
                        tof_msg.tof_position, int(point_idx % 8)
                    ] = row_indensity_sum[tof_msg.tof_position, int(point_idx % 8)] + (
                        indensity[point_idx] < -0.1
                    )
                if int(point_idx / 8) == row_col_idx:
                    col_indensity_sum[
                        tof_msg.tof_position, int(point_idx / 8)
                    ] = col_indensity_sum[tof_msg.tof_position, int(point_idx / 8)] + (
                        indensity[point_idx] < -0.1
                    )

    def timer_callback(self):
        if self.action_start and (self.timer_count < adjust_times):
            self.timer_count = self.timer_count + 1
            self.get_logger().info("timer_callback: %d " % (self.timer_count))
            # self.x_pose_adjust()
            self.y_pose_adjust()
            self.yaw_adjust()
            self.motion_req.motion_id = 212
            self.motion_req.duration = (int)(timer_period * 1000 - 10)
            self.send_motion_request()
        elif (self.timer_count >= adjust_times) and (
            self.timer_count < (adjust_times + 10)
        ):
            self.timer_count = self.timer_count + 1
            self.motion_req.pos_des[0] = 0
            self.motion_req.pos_des[1] = 0
            self.motion_req.pos_des[2] = -0.015
            self.motion_req.rpy_des[2] = 0
            self.motion_req.motion_id = 212
            self.motion_req.duration = (int)(timer_period * 1000 - 10)
            self.send_motion_request()
        elif self.timer_count >= (adjust_times + 10):
            self.motion_req.pos_des[0] = 0
            self.motion_req.pos_des[1] = 0
            self.motion_req.pos_des[2] = 0
            self.motion_req.rpy_des[2] = 0
            self.motion_req.motion_id = 101
            self.motion_req.duration = 0
            self.send_motion_request()

            self.timer_count = 0
            self.action_start = 0
            self.action_complete = 1
        # self.get_logger().info('timer_callback: %d ' % (self.timer_count))
        # if (self.action_start):
        #     self.timer_count = self.timer_count + 1
        #     self.get_logger().info('timer_callback: %d ' % (self.timer_count))
        # if (self.timer_count == 20):
        #     self.timer_count = 0
        #     self.action_start = 0
        #     self.action_complete = 1


def point_cloud(points, parent_frame):
    # In a PointCloud2 message, the point cloud is stored as an byte
    # array. In order to unpack it, we also include some parameters
    # which desribes the size of each individual point.

    ros_dtype = sensor_msgs.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize  # A 32-bit float takes 4 bytes.

    data = points.astype(dtype).tobytes()

    # The fields specify what the bytes represents. The first 4 bytes
    # represents the x-coordinate, the next 4 the y-coordinate, etc.
    fields = [
        sensor_msgs.PointField(name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate("xyz")
    ]

    # The PointCloud2 message also has a header which specifies which
    # coordinate frame it is represented in.
    header = std_msgs.Header(frame_id=parent_frame)

    return sensor_msgs.PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 3),  # Every point consists of three float32s.
        row_step=(itemsize * 3 * points.shape[0]),
        data=data,
    )


def main(args=None):

    rclpy.init(args=args)

    seatadjust_server = SeatAdjustServer()

    # We use a MultiThreadedExecutor to handle incoming goal requests concurrently
    executor = MultiThreadedExecutor()
    rclpy.spin(seatadjust_server, executor=executor)

    seatadjust_server.destroy()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
