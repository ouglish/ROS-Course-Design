#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
多点导航节点 (Multi-point Navigation Node)

解决的问题
----------
问题1：三轮差速小车在只改变角度（原地旋转）时，x、y可能产生偏移
    原因：差速驱动小车在原地旋转时，两侧轮子方向相反，由于机械误差、
          地面摩擦不均等因素，机器人重心实际上会发生轻微位移。
    解决方案：在完成"仅角度变化"的导航目标后，检测当前位置与目标位置
              的偏差，若超出阈值则自动发送位置修正目标，将小车精确
              移回目标 x、y 坐标。

问题2：小车导航默认起点坐标和起始方向可能与建图时不一致
    原因：使用 SLAM（如 gmapping）建图时，机器人的初始位置被记为地图
          坐标原点 (0, 0, 0)。在重新启动导航时，AMCL 默认以 (0, 0, 0)
          作为初始估计，但若机器人实际放置的位置或朝向与建图起点不同，
          定位会出现偏差。
    解决方案：在导航节点启动时，向 AMCL 的 /initialpose 话题发布明确的
              初始位姿（通过 ROS 参数配置），使 AMCL 从正确的位置开始
              粒子滤波，保证定位准确。

用法
----
    roslaunch multi_point_nav multi_point_nav.launch

参数（均可在 launch 文件或命令行中覆盖）
--------------------------------------
    ~waypoints            (list)  : 目标点列表，每个元素包含 x、y、theta
    ~initial_pose_x       (float) : 建图起始位置 x（默认 0.0）
    ~initial_pose_y       (float) : 建图起始位置 y（默认 0.0）
    ~initial_pose_theta   (float) : 建图起始朝向，弧度（默认 0.0）
    ~use_initial_pose     (bool)  : 是否在启动时向 AMCL 发布初始位姿（默认 True）
    ~position_tolerance   (float) : 判断是否到达目标的 x/y 容差，单位 m（默认 0.1）
    ~angle_tolerance      (float) : 判断是否到达目标的角度容差，单位 rad（默认 0.1）
    ~xy_correction_threshold (float): 触发 x/y 修正的漂移阈值，单位 m（默认 0.05）
    ~loop_waypoints       (bool)  : 是否循环遍历目标点（默认 False）
    ~amcl_convergence_wait (float): 发布初始位姿后等待 AMCL 收敛的时间，单位 s（默认 2.0）
    ~rotation_settle_wait  (float): 旋转完成后等待机器人停稳的时间，单位 s（默认 0.5）
    ~waypoint_pause        (float): 相邻目标点之间的停顿时间，单位 s（默认 0.3）
"""

import math

import actionlib
import rospy
import tf.transformations
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class MultiPointNav:
    """三轮差速小车多点导航控制器。"""

    def __init__(self):
        rospy.init_node('multi_point_nav', anonymous=False)

        # ── 导航目标点 ────────────────────────────────────────────────────────
        self.waypoints = rospy.get_param('~waypoints', [])

        # ── 初始位姿参数（解决问题2） ──────────────────────────────────────────
        # 这里填写建图时机器人的起始位置和朝向，使 AMCL 从正确的位置开始定位。
        # 如果建图时机器人从地图原点 (0, 0) 出发且朝向正东，则保持默认值即可。
        self.initial_pose_x = rospy.get_param('~initial_pose_x', 0.0)
        self.initial_pose_y = rospy.get_param('~initial_pose_y', 0.0)
        self.initial_pose_theta = rospy.get_param('~initial_pose_theta', 0.0)
        self.use_initial_pose = rospy.get_param('~use_initial_pose', True)

        # ── 容差与修正参数 ────────────────────────────────────────────────────
        # position_tolerance：判断"仅旋转"目标时使用的 x/y 容差
        self.position_tolerance = rospy.get_param('~position_tolerance', 0.1)
        # angle_tolerance：角度容差（暂时保留，供后续扩展使用）
        self.angle_tolerance = rospy.get_param('~angle_tolerance', 0.1)
        # xy_correction_threshold：旋转完成后，若 x/y 漂移超过此阈值则触发修正
        self.xy_correction_threshold = rospy.get_param('~xy_correction_threshold', 0.05)

        # ── 循环模式 ──────────────────────────────────────────────────────────
        self.loop_waypoints = rospy.get_param('~loop_waypoints', False)

        # ── 时间参数（均可通过 ROS 参数调整） ────────────────────────────────────
        # 发布初始位姿后等待 AMCL 收敛的时间
        self.amcl_convergence_wait = rospy.get_param('~amcl_convergence_wait', 2.0)
        # 旋转完成后等待机器人完全停稳的时间
        self.rotation_settle_wait = rospy.get_param('~rotation_settle_wait', 0.5)
        # 相邻目标点之间的停顿时间
        self.waypoint_pause = rospy.get_param('~waypoint_pause', 0.3)

        # ── 当前位姿（来自 AMCL） ─────────────────────────────────────────────
        self.current_pose = None
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self._pose_callback)

        # ── 发布初始位姿到 AMCL ───────────────────────────────────────────────
        self._initial_pose_pub = rospy.Publisher(
            '/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True
        )
        # 等待发布者与订阅者建立连接（最多 2 秒）
        deadline = rospy.Time.now() + rospy.Duration(2.0)
        while (self._initial_pose_pub.get_num_connections() == 0
               and rospy.Time.now() < deadline
               and not rospy.is_shutdown()):
            rospy.sleep(0.1)

        # ── 连接 move_base ────────────────────────────────────────────────────
        self._client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('等待 move_base 服务连接...')
        self._client.wait_for_server()
        rospy.loginfo('move_base 服务已连接')

    # ──────────────────────────────────────────────────────────────────────────
    # 公共接口
    # ──────────────────────────────────────────────────────────────────────────

    def run(self):
        """启动导航：（可选）设置初始位姿，然后依次导航到各目标点。"""
        # 解决问题2：在开始导航前设置与建图时一致的初始位姿
        if self.use_initial_pose:
            self._set_initial_pose()
            # 等待 AMCL 收敛，给粒子滤波一些时间
            rospy.sleep(self.amcl_convergence_wait)

        if not self.waypoints:
            rospy.logwarn(
                '未配置任何目标点。请在 launch 文件或参数服务器中设置 ~waypoints。'
            )
            return

        self._navigate_waypoints()

    # ──────────────────────────────────────────────────────────────────────────
    # 内部方法
    # ──────────────────────────────────────────────────────────────────────────

    def _pose_callback(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg

    def _set_initial_pose(self):
        """
        解决问题2：向 AMCL 的 /initialpose 话题发布初始位姿。

        背景说明
        --------
        使用 SLAM 建图时，机器人的起始位置被记录为地图坐标系原点 (0, 0, 0)。
        导航时若不显式设置初始位姿，AMCL 会默认使用 (0, 0, 0)，但实际上
        机器人可能被放置在不同的位置或朝向，导致定位出错。

        通过将建图时的起始位置和朝向以参数形式配置并发布给 AMCL，
        可以保证导航起点与建图起点一致，避免初始定位偏差。
        """
        rospy.loginfo(
            '设置 AMCL 初始位姿: x=%.3f, y=%.3f, theta=%.3f rad (%.1f°)',
            self.initial_pose_x,
            self.initial_pose_y,
            self.initial_pose_theta,
            math.degrees(self.initial_pose_theta),
        )

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = rospy.Time.now()

        msg.pose.pose.position.x = self.initial_pose_x
        msg.pose.pose.position.y = self.initial_pose_y
        msg.pose.pose.position.z = 0.0

        q = tf.transformations.quaternion_from_euler(0.0, 0.0, self.initial_pose_theta)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        # 协方差矩阵（6×6 展平）：x/y 标准差约 0.5 m，yaw 标准差约 0.26 rad
        # 这是 RViz "2D Pose Estimate" 的默认值，表示中等定位不确定性
        msg.pose.covariance = [
            0.25, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.25, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.06853892326654787,
        ]

        self._initial_pose_pub.publish(msg)
        rospy.loginfo('初始位姿已发布至 /initialpose，等待 AMCL 收敛...')

    def _send_goal(self, x: float, y: float, theta: float,
                   frame_id: str = 'map') -> int:
        """向 move_base 发送单个导航目标，阻塞直到完成，返回 GoalStatus 状态码。"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame_id
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0

        q = tf.transformations.quaternion_from_euler(0.0, 0.0, theta)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self._client.send_goal(goal)
        self._client.wait_for_result()
        return self._client.get_state()

    def _is_rotation_only_goal(self, wp: dict, prev_wp: dict) -> bool:
        """
        判断从 prev_wp 到 wp 是否为"仅角度变化"的原地旋转目标。

        仅当 x、y 变化量均在 position_tolerance 以内，且 theta 有明显变化时
        才被判定为旋转目标。
        """
        if prev_wp is None:
            return False

        dx = abs(wp.get('x', 0.0) - prev_wp.get('x', 0.0))
        dy = abs(wp.get('y', 0.0) - prev_wp.get('y', 0.0))
        # 使用正规化角度差，处理角度跨越 ±π 的情况（例如 3.1 → -3.1）
        raw_dtheta = wp.get('theta', 0.0) - prev_wp.get('theta', 0.0)
        dtheta = abs(math.atan2(math.sin(raw_dtheta), math.cos(raw_dtheta)))

        return (dx < self.position_tolerance
                and dy < self.position_tolerance
                and dtheta > self.angle_tolerance)

    def _correct_xy_drift(self, target_x: float, target_y: float,
                          target_theta: float):
        """
        解决问题1：原地旋转完成后，检测并修正 x/y 漂移。

        背景说明
        --------
        三轮差速小车在原地旋转时，左右驱动轮以相反方向旋转。由于轮子直径
        存在细微差异、地面摩擦不均匀、车体重心偏置等机械因素，机器人重心
        实际上会发生轻微位移，导致旋转结束后 x、y 坐标偏离目标值。

        修正策略
        --------
        旋转目标完成后，从 AMCL 获取当前位姿，计算与目标 (x, y) 的距离。
        若漂移超过 xy_correction_threshold，则重新向 move_base 发送一个
        精确位置目标，让小车修正回到正确坐标，同时保持目标朝向不变。
        """
        if self.current_pose is None:
            rospy.logwarn('尚未收到 AMCL 位姿，跳过 x/y 漂移修正。')
            return

        cur_x = self.current_pose.pose.pose.position.x
        cur_y = self.current_pose.pose.pose.position.y
        drift = math.hypot(cur_x - target_x, cur_y - target_y)

        if drift > self.xy_correction_threshold:
            rospy.loginfo(
                '检测到旋转后 x/y 漂移 %.3f m（阈值 %.3f m），正在发送位置修正目标…',
                drift, self.xy_correction_threshold,
            )
            state = self._send_goal(target_x, target_y, target_theta)
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo('x/y 位置修正成功。')
            else:
                rospy.logwarn('x/y 位置修正未完全成功，GoalStatus=%d。', state)
        else:
            rospy.loginfo(
                '旋转后 x/y 漂移 %.3f m，在阈值内，无需修正。', drift
            )

    def _navigate_waypoints(self):
        """依次导航到 self.waypoints 中的每个目标点。"""
        rospy.loginfo('开始多点导航，共 %d 个目标点。', len(self.waypoints))

        iteration = 0
        while not rospy.is_shutdown():
            prev_wp = None

            for idx, wp in enumerate(self.waypoints):
                if rospy.is_shutdown():
                    return

                x = float(wp.get('x', 0.0))
                y = float(wp.get('y', 0.0))
                theta = float(wp.get('theta', 0.0))

                is_rotation = self._is_rotation_only_goal(wp, prev_wp)
                rospy.loginfo(
                    '[%d/%d] 导航到目标点: x=%.3f, y=%.3f, theta=%.3f rad (%.1f°)%s',
                    idx + 1, len(self.waypoints),
                    x, y, theta, math.degrees(theta),
                    '  [仅旋转目标]' if is_rotation else '',
                )

                state = self._send_goal(x, y, theta)

                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo('已到达目标点 %d/%d。', idx + 1, len(self.waypoints))

                    # 解决问题1：旋转目标完成后检查并修正 x/y 漂移
                    if is_rotation:
                        rospy.sleep(self.rotation_settle_wait)  # 等待机器人完全停止，位姿稳定
                        self._correct_xy_drift(x, y, theta)
                else:
                    rospy.logwarn(
                        '未能到达目标点 %d/%d，GoalStatus=%d，跳过该点继续。',
                        idx + 1, len(self.waypoints), state,
                    )

                prev_wp = wp
                rospy.sleep(self.waypoint_pause)  # 短暂停顿后继续下一个目标点

            iteration += 1
            if not self.loop_waypoints:
                rospy.loginfo('所有目标点导航完成。')
                break

            rospy.loginfo('循环模式：开始第 %d 轮导航。', iteration + 1)


# ──────────────────────────────────────────────────────────────────────────────
# 入口
# ──────────────────────────────────────────────────────────────────────────────

def main():
    try:
        nav = MultiPointNav()
        nav.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('多点导航节点已停止。')


if __name__ == '__main__':
    main()
