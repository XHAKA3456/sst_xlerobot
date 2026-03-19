from sst_xlerbot.model.SO101Robot import SO101Kinematics
import math

kin = SO101Kinematics()

# 하드코딩된 값
target_x = 0.1629
target_y = 0.1131

print('=== 1. _ik_core (전처리/후처리 없음) ===')
theta1_rad, theta2_rad = kin._ik_core(target_x, target_y, kin.l1, kin.l2)
print(f'theta1 = {math.degrees(theta1_rad):.2f}°')
print(f'theta2 = {math.degrees(theta2_rad):.2f}°')

print('\n=== 2. inverse_kinematics (전체 파이프라인) ===')
joint2_deg, joint3_deg = kin.inverse_kinematics(target_x, target_y)
print(f'joint2 (shoulder_lift) = {joint2_deg:.2f}°')
print(f'joint3 (elbow_flex) = {joint3_deg:.2f}°')

print('\n=== 3. Zero position의 theta1, theta2 역계산 ===')
# Zero position (0°, 0°)을 역으로 계산
# joint2_deg = 90 - (theta1 + offset) → theta1 = (90 - joint2_deg) - offset (rad)
# joint3_deg = (theta2 + offset) - 90 → theta2 = (joint3_deg + 90) - offset (rad)

theta1_offset = math.atan2(0.028, 0.11257)
theta2_offset = math.atan2(0.0052, 0.1349) + theta1_offset

# Zero position: joint2_deg=0, joint3_deg=0
joint2_rad = math.radians(90 - 0)  # 좌표 변환 역
joint3_rad = math.radians(0 + 90)

theta1_at_zero = joint2_rad - theta1_offset
theta2_at_zero = joint3_rad - theta2_offset

print(f'Zero position의 theta1 = {math.degrees(theta1_at_zero):.2f}° ({theta1_at_zero:.4f} rad)')
print(f'Zero position의 theta2 = {math.degrees(theta2_at_zero):.2f}° ({theta2_at_zero:.4f} rad)')

print('\n=== 4. Zero position의 EE 위치 계산 (여러 FK 방식) ===')

# 방법 1: 표준 FK (theta1 + theta2)
x1 = kin.l1 * math.cos(theta1_at_zero) + kin.l2 * math.cos(theta1_at_zero + theta2_at_zero)
y1 = kin.l1 * math.sin(theta1_at_zero) + kin.l2 * math.sin(theta1_at_zero + theta2_at_zero)
print(f'방법 1 (표준 FK: theta1 + theta2):')
print(f'  EE = ({x1:.4f}, {y1:.4f})')

# 방법 2: theta1 + theta2 - π
x2 = kin.l1 * math.cos(theta1_at_zero) + kin.l2 * math.cos(theta1_at_zero + theta2_at_zero - math.pi)
y2 = kin.l1 * math.sin(theta1_at_zero) + kin.l2 * math.sin(theta1_at_zero + theta2_at_zero - math.pi)
print(f'방법 2 (현재 FK: theta1 + theta2 - π):')
print(f'  EE = ({x2:.4f}, {y2:.4f})')

# 방법 3: IK gamma 공식으로 검증
beta = math.atan2(target_y, target_x)
gamma = math.atan2(kin.l2 * math.sin(theta2_rad), kin.l1 + kin.l2 * math.cos(theta2_rad))
theta1_from_ik = beta + gamma
print(f'\n방법 3 (IK에서 계산된 theta1 검증):')
print(f'  IK의 theta1 = {math.degrees(theta1_from_ik):.2f}°')
print(f'  beta = {math.degrees(beta):.2f}°')
print(f'  gamma = {math.degrees(gamma):.2f}°')

print('\n=== 5. 비교 ===')
print(f'하드코딩 값:       ({target_x:.4f}, {target_y:.4f})')
print(f'표준 FK:          ({x1:.4f}, {y1:.4f}) - error: {math.sqrt((x1-target_x)**2 + (y1-target_y)**2):.6f}m')
print(f'현재 FK (- π):    ({x2:.4f}, {y2:.4f}) - error: {math.sqrt((x2-target_x)**2 + (y2-target_y)**2):.6f}m')
