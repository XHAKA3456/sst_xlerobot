from src.sst_xlerbot.model.so101robot_fix import SO101Kinematics
import math

kin = SO101Kinematics()

print("=" * 70)
print("FK/IK 왕복 테스트 - 수정된 FK 검증")
print("=" * 70)

# 하드코딩된 값
target_x = 0.1629
target_y = 0.1131

print(f"\n[1단계] 목표 EE 위치")
print(f"  x = {target_x:.4f} m")
print(f"  y = {target_y:.4f} m")

# IK 계산
print(f"\n[2단계] IK 계산")
joint2_deg, joint3_deg = kin.inverse_kinematics(target_x, target_y)
print(f"  joint2 (shoulder_lift) = {joint2_deg:.4f}°")
print(f"  joint3 (elbow_flex)    = {joint3_deg:.4f}°")

# FK로 역계산
print(f"\n[3단계] FK로 역계산")
fk_x, fk_y = kin.forward_kinematics(joint2_deg, joint3_deg)
print(f"  FK 결과: x = {fk_x:.4f} m, y = {fk_y:.4f} m")

# 에러 계산
error = math.sqrt((fk_x - target_x)**2 + (fk_y - target_y)**2)
error_x = abs(fk_x - target_x)
error_y = abs(fk_y - target_y)

print(f"\n[4단계] 에러 분석")
print(f"  Δx = {error_x:.6f} m = {error_x * 1000:.3f} mm")
print(f"  Δy = {error_y:.6f} m = {error_y * 1000:.3f} mm")
print(f"  Total error = {error:.6f} m = {error * 1000:.3f} mm")

# 판정
if error < 0.001:  # 1mm 이하
    print(f"\n✅ PASS: FK/IK가 일치합니다! (에러 < 1mm)")
elif error < 0.01:  # 1cm 이하
    print(f"\n⚠️  WARNING: 에러가 약간 있습니다 (1mm ~ 1cm)")
else:
    print(f"\n❌ FAIL: FK/IK 불일치! (에러 > 1cm)")

print("\n" + "=" * 70)
print("Zero Position 테스트")
print("=" * 70)

# Zero position FK 계산
print(f"\n[Zero Position] joint2=0°, joint3=0°")
zero_x, zero_y = kin.forward_kinematics(0.0, 0.0)
print(f"  FK 결과: x = {zero_x:.4f} m, y = {zero_y:.4f} m")
print(f"  하드코딩:  x = 0.1629 m, y = 0.1131 m")
print(f"  Δx = {abs(zero_x - 0.1629):.6f} m")
print(f"  Δy = {abs(zero_y - 0.1131):.6f} m")

zero_error = math.sqrt((zero_x - 0.1629)**2 + (zero_y - 0.1131)**2)
if zero_error < 0.001:
    print(f"\n✅ Zero position도 일치합니다!")
else:
    print(f"\n⚠️  Zero position 에러: {zero_error * 1000:.3f} mm")

print("\n" + "=" * 70)
print("추가 테스트: 여러 위치에서 IK → FK 왕복")
print("=" * 70)

test_points = [
    (0.15, 0.15),
    (0.20, 0.10),
    (0.10, 0.20),
    (0.12, 0.08),
]

max_error = 0.0
for i, (tx, ty) in enumerate(test_points, 1):
    j2, j3 = kin.inverse_kinematics(tx, ty)
    fx, fy = kin.forward_kinematics(j2, j3)
    err = math.sqrt((fx - tx)**2 + (fy - ty)**2)
    max_error = max(max_error, err)
    status = "✅" if err < 0.001 else "⚠️"
    print(f"{status} Test {i}: ({tx:.4f}, {ty:.4f}) → j=({j2:.2f}°, {j3:.2f}°) → ({fx:.4f}, {fy:.4f}) | err={err*1000:.3f}mm")

print(f"\nMax error across all tests: {max_error * 1000:.3f} mm")
if max_error < 0.001:
    print("✅ 모든 테스트 통과!")
else:
    print("⚠️  일부 테스트에서 에러 발생")
