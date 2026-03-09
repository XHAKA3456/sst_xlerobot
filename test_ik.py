from sst_xlerbot.model.SO101Robot import SO101Kinematics

kin = SO101Kinematics()

# Zero position 테스트
joint2 = 0.0  # shoulder_lift
joint3 = 0.0  # elbow_flex

# FK 계산
x, y = kin.forward_kinematics(joint2, joint3)
print(f"Zero position FK: x={x:.4f}, y={y:.4f}")
print(f"Expected:         x=0.1629, y=0.1131")
print(f"Difference:       Δx={abs(x-0.1629):.4f}, Δy={abs(y-0.1131):.4f}")

# IK → FK 왕복 테스트
target_x, target_y = 0.1629, 0.1131
j2, j3 = kin.inverse_kinematics(target_x, target_y)
fk_x, fk_y = kin.forward_kinematics(j2, j3)

print(f"\nIK → FK roundtrip:")
print(f"  Target: ({target_x:.4f}, {target_y:.4f})")
print(f"  IK result: joint2={j2:.2f}°, joint3={j3:.2f}°")
print(f"  FK result: ({fk_x:.4f}, {fk_y:.4f})")
print(f"  Error: {((fk_x-target_x)**2 + (fk_y-target_y)**2)**0.5:.6f} m")