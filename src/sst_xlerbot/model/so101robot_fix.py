import math
import numpy as np
from typing import List, Union, Tuple

# lerobot 관련 임포트는 사용 환경에 맞게 유지
try:
    from lerobot.robots.robot import Robot
    from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
    from lerobot.robots.utils import make_robot_from_config
    from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig
    from lerobot.cameras import ColorMode, Cv2Rotation
    from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
except ImportError:
    # 테스트를 위해 라이브러리가 없는 경우를 대비한 처리 (실제 환경에서는 무시됨)
    pass

def create_real_robot(port, camera_index, uid: str = "so101"):
    """Wrapper function to map string UIDS to real robot configurations."""
    if uid == "so101":
        robot_config = SO101FollowerConfig(
            port= port,
            use_degrees=True,
            cameras = {
                "base_camera": OpenCVCameraConfig(index_or_path= camera_index, 
                fps=30,
                width=640,
                height=480,
                color_mode=ColorMode.RGB,
                rotation=Cv2Rotation.NO_ROTATION)
            },
            id="robot1",
        )
        real_robot = make_robot_from_config(robot_config)
        return real_robot


class SO101Kinematics:
    """
    A class to represent the kinematics of a SO101 robot arm.
    All public methods use degrees for input/output.
    """

    def __init__(self, l1=0.11257, l2=0.1349):
        self.l1 = l1  # Length of the first link (upper arm) - from URDF
        self.l2 = l2  # Length of the second link (lower arm) - from URDF

    def _ik_core(self, x, y, l1, l2):
        """
        Core IK calculation.
        This remains unchanged as it provides the geometric basis.
        """
        # ==================== IK FORMULA START ====================
        # Calculate distance from origin to target point
        r = math.sqrt(x**2 + y**2)
        r_max = l1 + l2  # Maximum reachable distance

        # If target point is beyond maximum workspace, scale it to the boundary
        if r > r_max:
            scale_factor = r_max / r
            x *= scale_factor
            y *= scale_factor
            r = r_max

        # If target point is less than minimum workspace (|l1-l2|), scale it
        r_min = abs(l1 - l2)
        if r < r_min and r > 0:
            scale_factor = r_min / r
            x *= scale_factor
            y *= scale_factor
            r = r_min

        # Use law of cosines to calculate theta2
        cos_theta2 = -(r**2 - l1**2 - l2**2) / (2 * l1 * l2)

        # Clamp cos_theta2 to valid range [-1, 1] to avoid domain errors
        cos_theta2 = max(-1.0, min(1.0, cos_theta2))

        # Calculate theta2 (elbow angle)
        theta2 = math.pi - math.acos(cos_theta2)

        # Calculate theta1 (shoulder angle)
        beta = math.atan2(y, x)
        gamma = math.atan2(l2 * math.sin(theta2), l1 + l2 * math.cos(theta2))
        theta1 = beta + gamma
        # ==================== IK FORMULA END ====================

        return theta1, theta2

    def inverse_kinematics(self, x, y, l1=None, l2=None):
        """
        Calculate inverse kinematics for a 2-link robotic arm, considering joint offsets
        """
        # Use instance values if not provided
        if l1 is None:
            l1 = self.l1
        if l2 is None:
            l2 = self.l2

        # Calculate joint2 and joint3 offsets in theta1 and theta2
        theta1_offset = math.atan2(0.028, 0.11257)  # theta1 offset when joint2=0
        theta2_offset = math.atan2(0.0052, 0.1349) + theta1_offset  # theta2 offset when joint3=0

        # ==================== CALL CORE IK SOLVER ====================
        theta1, theta2 = self._ik_core(x, y, l1, l2)
        # =============================================================

        # Convert theta1 and theta2 to joint2 and joint3 angles
        joint2 = theta1 + theta1_offset
        joint3 = theta2 + theta2_offset

        # Ensure angles are within URDF limits
        joint2 = max(-0.8, min(3.92, joint2))  # broadened by ~0.7 rad (~40°) on both ends
        joint3 = max(-0.9, min(math.pi + 0.7, joint3))  # widen by ~0.7 rad (~40°)

        # Convert from radians to degrees
        joint2_deg = math.degrees(joint2)
        joint3_deg = math.degrees(joint3)

        # Apply coordinate system transformation
        joint2_deg = 90 - joint2_deg
        joint3_deg = joint3_deg - 90

        return joint2_deg, joint3_deg
    
    def forward_kinematics(self, joint2_deg, joint3_deg, l1=None, l2=None):
        """
        Calculate forward kinematics for a 2-link robotic arm
        [FIXED] Updated formula to match IK logic and coordinate system.
        """
        # Use instance values if not provided
        if l1 is None:
            l1 = self.l1
        if l2 is None:
            l2 = self.l2
            
        # Convert degrees to radians and apply inverse transformation
        joint2_rad = math.radians(90 - joint2_deg)
        joint3_rad = math.radians(joint3_deg + 90)
        
        # Calculate joint2 and joint3 offsets
        theta1_offset = math.atan2(0.028, 0.11257)
        theta2_offset = math.atan2(0.0052, 0.1349) + theta1_offset
        
        # Convert joint angles back to theta1 and theta2
        theta1 = joint2_rad - theta1_offset
        theta2 = joint3_rad - theta2_offset
        
        # ==================== FK FORMULA FIX ====================
        # 이전 코드: x = l1 * cos(t1) + l2 * cos(t1 + t2 - pi) (불일치 원인)
        # 수정 코드: x = l1 * cos(t1) + l2 * cos(t1 - t2)      (IK와 일치, 목표 좌표 도달)
        
        x = l1 * math.cos(theta1) + l2 * math.cos(theta1 - theta2)
        y = l1 * math.sin(theta1) + l2 * math.sin(theta1 - theta2)
        # ========================================================
        
        return x, y

    
    def generate_sinusoidal_velocity_trajectory(
        self,
        start_point: Union[List[float], np.ndarray],
        end_point: Union[List[float], np.ndarray],
        control_freq: float = 100.0,  # Hz
        total_time: float = 5.0,      # seconds
        velocity_amplitude: float = 1.0,  # m/s
        velocity_period: float = 2.0,     # seconds
        phase_offset: float = 0.0         # radians
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Generate a straight-line trajectory with sinusoidal velocity profile.
        (No changes made to this function)
        """
        
        # Convert to numpy arrays
        start = np.array(start_point, dtype=float)
        end = np.array(end_point, dtype=float)
        
        # Calculate direction and distance
        direction_vector = end - start
        total_distance = np.linalg.norm(direction_vector)
        direction_unit = direction_vector / total_distance if total_distance > 0 else np.zeros(3)
        
        # Generate time array
        dt = 1.0 / control_freq
        n_points = int(total_time * control_freq) + 1
        time_array = np.linspace(0, total_time, n_points)
        
        # Calculate angular frequency
        omega = 2 * np.pi / velocity_period
        
        # Generate sinusoidal velocity profile
        base_velocity = total_distance / total_time  # Average velocity needed
        velocities = base_velocity + velocity_amplitude * np.sin(omega * time_array + phase_offset)
        
        # Ensure non-negative velocities (optional - remove if negative velocities are desired)
        velocities = np.maximum(velocities, 0.1 * base_velocity)
        
        # Integrate velocity to get position along the path
        positions_1d = np.zeros(n_points)
        for i in range(1, n_points):
            positions_1d[i] = positions_1d[i-1] + velocities[i-1] * dt
        
        # Scale positions to fit exactly between start and end points
        if positions_1d[-1] > 0:
            positions_1d = positions_1d * (total_distance / positions_1d[-1])
        
        # Convert 1D positions to 3D trajectory
        trajectory = np.zeros((n_points, 3))
        for i in range(n_points):
            progress = positions_1d[i] / total_distance if total_distance > 0 else 0
            trajectory[i] = start + progress * direction_vector
        
        return trajectory, velocities, time_array