import math
import numpy as np
from typing import Tuple, List, Optional


class SO101Kinematics_3Link:
    """
    3-link IK for SO101 robot arm with full wrist control.
    All public methods use degrees for input/output.
    Based on three_link_ik_core.py
    """

    def __init__(self, l1=0.11257, l2=0.1349, l3=0.0845):
        """
        Initialize 3-link kinematics.

        Args:
            l1: Upper arm length (meters) - from URDF
            l2: Lower arm length (meters) - from URDF
            l3: Wrist to gripper length (meters) - from URDF (0.0601 + 0.0244)
        """
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3

        # Joint offsets (same as 2-link SO101Kinematics)
        self.theta1_offset = math.atan2(0.028, 0.11257)  # theta1 offset when joint2=0
        self.theta2_offset = math.atan2(0.0052, 0.1349) + self.theta1_offset  # theta2 offset when joint3=0

        # Max angular rate limit for safety (radians per step)
        self.max_angle_change = np.radians(5.0)

    def _ik_core(self, x: float, y: float, psi: float) -> List[List[float]]:
        """
        Core 3-link IK calculation (pure mathematical)

        Returns all possible solutions as list of [theta1, theta2, theta3].
        Each solution is in radians.
        """
        solutions = []

        # Calculate J3 position (wrist joint)
        x_j3 = x - self.l3 * np.cos(psi)
        y_j3 = y - self.l3 * np.sin(psi)
        R_squared = x_j3**2 + y_j3**2
        R = np.sqrt(R_squared)

        # Check workspace limits
        if R > (self.l1 + self.l2 + 1e-6) or R < abs(self.l1 - self.l2 - 1e-6):
            return solutions

        # Calculate theta2 using law of cosines
        cos_theta2 = (R_squared - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)

        theta2_plus = np.arccos(cos_theta2)
        theta2_minus = -theta2_plus

        # Calculate both elbow-up and elbow-down solutions
        for theta2 in [theta2_plus, theta2_minus]:
            gamma = np.arctan2(y_j3, x_j3)
            alpha = np.arctan2(self.l2 * np.sin(theta2), self.l1 + self.l2 * np.cos(theta2))
            theta1 = gamma - alpha
            theta3 = psi - (theta1 + theta2)

            # Normalize angles to [-pi, pi]
            theta1 = (theta1 + np.pi) % (2 * np.pi) - np.pi
            theta2 = (theta2 + np.pi) % (2 * np.pi) - np.pi
            theta3 = (theta3 + np.pi) % (2 * np.pi) - np.pi

            solutions.append([theta1, theta2, theta3])

        return solutions

    def _apply_offsets_and_limits(self, theta1, theta2, theta3):
        """
        Apply offsets and limits to theta angles (same as 2-link).

        Args:
            theta1, theta2, theta3: Pure mathematical theta angles (radians)

        Returns:
            joint2, joint3, joint4: Joint angles with offsets applied (radians)
        """
        # Convert theta to joint angles
        joint2 = theta1 + self.theta1_offset
        joint3 = theta2 + self.theta2_offset
        joint4 = theta3  # No offset for theta3

        # Ensure angles are within URDF limits (same broadening as 2-link)
        joint2 = max(-0.8, min(3.92, joint2))
        joint3 = max(-0.9, min(math.pi + 0.7, joint3))
        joint4 = max(-1.8, min(1.8, joint4))

        return joint2, joint3, joint4

    def get_next_safe_angles(
        self,
        x_target: float,
        y_target: float,
        psi_target: float,
        current_angles: List[float]
    ) -> Optional[List[float]]:
        """
        Calculate next safe joint angles with elbow flip prevention and rate limiting.

        Args:
            x_target: Target EE x position (meters)
            y_target: Target EE y position (meters)
            psi_target: Target EE orientation (radians)
            current_angles: Current theta angles [theta1, theta2, theta3] (radians, BEFORE offset)

        Returns:
            Next safe theta angles [theta1, theta2, theta3] (radians, BEFORE offset) or None if no solution
        """
        # Get all IK solutions (pure theta, no offset)
        all_solutions = self._ik_core(x_target, y_target, psi_target)

        if not all_solutions:
            return None

        current_t2 = current_angles[1]

        # Joint limits check (apply offset temporarily for validation)
        def is_valid(sol):
            t1, t2, t3 = sol
            j2, j3, j4 = self._apply_offsets_and_limits(t1, t2, t3)
            # Check if within limits (already clamped in _apply_offsets_and_limits)
            return True  # If we got here, it passed clamping

        # Prefer solutions that maintain elbow configuration (prevent flip)
        preferred_solutions = []
        for sol in all_solutions:
            if (sol[1] >= 0 and current_t2 >= 0) or (sol[1] < 0 and current_t2 < 0):
                preferred_solutions.append(sol)

        valid_preferred = [sol for sol in preferred_solutions if is_valid(sol)]

        if not valid_preferred:
            return None

        # Select solution with minimum joint movement
        best_solution = None
        min_distance_squared = float('inf')
        current_angles_arr = np.array(current_angles)

        for sol in valid_preferred:
            sol_arr = np.array(sol)
            distance_squared = np.sum((sol_arr - current_angles_arr)**2)

            if distance_squared < min_distance_squared:
                min_distance_squared = distance_squared
                best_solution = sol

        target_angles_arr = np.array(best_solution)

        # Apply angular rate limiting
        angle_diff = target_angles_arr - current_angles_arr

        # Normalize angle differences (shortest path)
        angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi

        max_diff = np.max(np.abs(angle_diff))

        if max_diff > self.max_angle_change:
            # Scale down to max allowed change
            scale_factor = self.max_angle_change / max_diff
            limited_diff = angle_diff * scale_factor
            new_angles = current_angles_arr + limited_diff
        else:
            new_angles = target_angles_arr

        # Final normalization
        new_angles = (new_angles + np.pi) % (2 * np.pi) - np.pi

        return list(new_angles)

    def inverse_kinematics(
        self,
        x: float,
        y: float,
        psi: float,
        current_angles: List[float]
    ) -> Tuple[float, float, float]:
        """
        Calculate inverse kinematics with safety logic and coordinate transformation.

        This is the WRAPPER function that (same structure as 2-link):
        1. Calls get_next_safe_angles() for IK calculation + safety logic
        2. Applies robot-specific offsets
        3. Applies coordinate system transformation
        4. Converts to degrees

        Args:
            x: End effector x coordinate (meters)
            y: End effector y coordinate (meters)
            psi: End effector orientation (radians)
            current_angles: Current theta angles [theta1, theta2, theta3] (radians, BEFORE offset)

        Returns:
            (shoulder_lift_deg, elbow_flex_deg, wrist_pitch_deg): Joint angles in degrees
            Returns current position if no solution found.
        """
        # ==================== CALL SAFE IK SOLVER ====================
        next_angles = self.get_next_safe_angles(x, y, psi, current_angles)
        # =============================================================

        if next_angles is None:
            # No solution: return current position (converted to degrees with offsets)
            theta1, theta2, theta3 = current_angles

            # Apply offsets
            joint2, joint3, joint4 = self._apply_offsets_and_limits(theta1, theta2, theta3)

            # Convert to degrees
            joint2_deg = np.degrees(joint2)
            joint3_deg = np.degrees(joint3)
            joint4_deg = np.degrees(joint4)

            # Apply coordinate transformation
            shoulder_lift_deg = 90 - joint2_deg
            elbow_flex_deg = joint3_deg - 90
            wrist_pitch_deg = joint4_deg  # No transform

            return shoulder_lift_deg, elbow_flex_deg, wrist_pitch_deg

        theta1, theta2, theta3 = next_angles

        # ==================== APPLY OFFSETS ====================
        joint2, joint3, joint4 = self._apply_offsets_and_limits(theta1, theta2, theta3)
        # ========================================================

        # Convert from radians to degrees
        joint2_deg = np.degrees(joint2)
        joint3_deg = np.degrees(joint3)
        joint4_deg = np.degrees(joint4)

        # Apply coordinate system transformation
        shoulder_lift_deg = 90 - joint2_deg
        elbow_flex_deg = joint3_deg - 90
        wrist_pitch_deg = joint4_deg  # No transform for theta3

        return shoulder_lift_deg, elbow_flex_deg, wrist_pitch_deg

    def forward_kinematics(
        self,
        shoulder_lift_deg: float,
        elbow_flex_deg: float,
        wrist_pitch_deg: float
    ) -> Tuple[float, float, float]:
        """
        Calculate forward kinematics for a 3-link robotic arm.

        This follows the same structure as 2-link SO101Kinematics.forward_kinematics():
        1. Reverse coordinate transformation (degrees → radians)
        2. Remove offsets (joint → theta)
        3. FK calculation with theta angles

        Args:
            shoulder_lift_deg: Shoulder lift angle (degrees)
            elbow_flex_deg: Elbow flex angle (degrees)
            wrist_pitch_deg: Wrist pitch angle (degrees)

        Returns:
            (x, y, psi): End effector position (meters) and orientation (degrees)
        """
        # Convert degrees to radians and apply inverse transformation
        joint2_rad = math.radians(90 - shoulder_lift_deg)
        joint3_rad = math.radians(elbow_flex_deg + 90)
        joint4_rad = math.radians(wrist_pitch_deg)  # No coordinate transform

        # Convert joint angles back to theta1, theta2, theta3 (remove offsets)
        theta1 = joint2_rad - self.theta1_offset
        theta2 = joint3_rad - self.theta2_offset
        theta3 = joint4_rad  # No offset for theta3

        # Forward kinematics calculations
        x2 = self.l1 * math.cos(theta1)
        y2 = self.l1 * math.sin(theta1)

        x3 = x2 + self.l2 * math.cos(theta1 + theta2)
        y3 = y2 + self.l2 * math.sin(theta1 + theta2)

        x_ee = x3 + self.l3 * math.cos(theta1 + theta2 + theta3)
        y_ee = y3 + self.l3 * math.sin(theta1 + theta2 + theta3)

        psi = theta1 + theta2 + theta3
        psi_deg = math.degrees(psi)

        return x_ee, y_ee, psi_deg
