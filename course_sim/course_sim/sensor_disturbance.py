"""Publish reproducible noisy measurements in the course sensor interfaces."""
import copy
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, Imu, JointState
from .cloud_format import encode, xt32_points
from .imu_attitude import Attitude


class SensorDisturbance(Node):
    def __init__(self, stream='imu'):
        if stream not in ('imu', 'cloud'):
            raise ValueError('stream must be imu or cloud')
        super().__init__('sensor_disturbance' if stream == 'imu' else 'sensor_cloud_disturbance')
        defaults = {'sigma_lidar': 0.005, 'p_lidar_drop': 0.0, 'sigma_imu_gyro': 0.001,
                    'sigma_imu_acc': 0.01, 'bias_imu_gyro': 0.0, 'bias_imu_acc': 0.0,
                    'seed': 0, 'lidar_frame': 'lidar_link', 'imu_frame': 'imu_link'}
        self.config = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        if not all(np.isfinite(self.config[k]) for k in defaults if k.startswith(('sigma_', 'bias_', 'p_'))):
            raise ValueError('Noise, bias and dropout parameters must be finite')
        if any(self.config[k] < 0 for k in ['sigma_lidar', 'sigma_imu_gyro', 'sigma_imu_acc']):
            raise ValueError('Noise standard deviations must be nonnegative')
        if not 0 <= self.config['p_lidar_drop'] <= 1:
            raise ValueError('Dropout must be in [0,1]')
        seeds = np.random.SeedSequence(self.config['seed']).spawn(2)
        self.lidar_rng, self.imu_rng = [np.random.default_rng(s) for s in seeds]
        self.attitude = Attitude()
        # Separate launch processes isolate the high-rate IMU from cloud work,
        # including the Python GIL and executor scheduling. Keep both spawned
        # RNG streams above so the existing seed-to-sensor mapping is unchanged.
        if stream == 'cloud':
            self.cloud_pub = self.create_publisher(PointCloud2, '/pointcloud', 10)
            self.create_subscription(PointCloud2, '/sim/raw/pointcloud', self.cloud, qos_profile_sensor_data)
        else:
            self.imu_pub = self.create_publisher(Imu, '/imu', 100)
            self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
            self.last_joint_stamp = None
            self.create_subscription(JointState, '/sim/raw/joint_states', self.joints,
                                     QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.create_subscription(Imu, '/sim/raw/imu', self.imu,
                                     QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.get_logger().info('XT32 snapshot time=0; IMU attitude from gyro/gravity measurements')

    def joints(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_joint_stamp is None or stamp < self.last_joint_stamp or stamp - self.last_joint_stamp >= 0.0099:
            self.joint_pub.publish(msg)
            self.last_joint_stamp = stamp

    def cloud(self, msg):
        try:
            points = xt32_points(msg, self.config['sigma_lidar'], self.config['p_lidar_drop'], self.lidar_rng)
            if len(points) == 0:
                self.get_logger().warn('No valid returns; skipping empty cloud for LIO-SAM', throttle_duration_sec=5)
                return
            header = copy.deepcopy(msg.header)
            header.frame_id = self.config['lidar_frame']
            self.cloud_pub.publish(encode(points, header))
        except ValueError as error:
            self.get_logger().error(str(error), throttle_duration_sec=5)

    def imu(self, msg):
        result = copy.deepcopy(msg)
        result.header.frame_id = self.config['imu_frame']
        for field, sigma, bias in [('angular_velocity', 'sigma_imu_gyro', 'bias_imu_gyro'),
                                   ('linear_acceleration', 'sigma_imu_acc', 'bias_imu_acc')]:
            vector = getattr(result, field)
            noise = self.imu_rng.normal(self.config[bias], self.config[sigma], 3)
            for axis, value in zip('xyz', noise):
                setattr(vector, axis, float(getattr(vector, axis) + value))
            covariance = np.eye(3) * self.config[sigma] ** 2
            setattr(result, field + '_covariance', covariance.reshape(-1).tolist())
        gyro = [getattr(result.angular_velocity, a) for a in 'xyz']
        acceleration = [getattr(result.linear_acceleration, a) for a in 'xyz']
        stamp = result.header.stamp.sec + result.header.stamp.nanosec * 1e-9
        try:
            quaternion = self.attitude.update(stamp, gyro, acceleration)
            for axis, value in zip('xyzw', quaternion):
                setattr(result.orientation, axis, float(value))
            result.orientation_covariance = [0.001, 0., 0., 0., 0.001, 0., 0., 0., 0.1]
            self.imu_pub.publish(result)
        except ValueError as error:
            self.get_logger().warn(str(error), throttle_duration_sec=5)


def main():
    run('imu')


def cloud_main():
    run('cloud')


def run(stream):
    rclpy.init()
    node = SensorDisturbance(stream)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
