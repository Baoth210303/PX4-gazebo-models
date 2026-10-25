#!/usr/bin/env python3
from pathlib import Path
import math

# ============================================================
# Cấu hình chính
# ============================================================

MODEL_NAME = "x500_cable"
OUT_FILE = Path("model.sdf")

# Tổng chiều dài dây
CABLE_TOTAL_LENGTH = 10.0   # mét

# Chiều dài mỗi đoạn dây.
# Không nên dùng 0.06 m cho dây 20 m vì sẽ cần ~333 segments.
# 0.20 m => 100 segments, còn có thể chạy được để test.
SEG_LENGTH = 0.5           # mét

# Bán kính dây
SEG_RADIUS = 0.005          # mét

# Khối lượng dây theo mét.
# 0.01 kg/m => dây 20 m nặng khoảng 0.2 kg.
# Nếu để 0.05 kg/m như bản cũ thì 20 m sẽ nặng ~1 kg, quá nặng cho x500.
LINEAR_DENSITY = 0.006       # kg/m

# Vị trí móc dây dưới drone, tính theo frame base_link
DRONE_ATTACH_X = 0.0
DRONE_ATTACH_Y = 0.0
DRONE_ATTACH_Z = -0.14

# Vị trí tải ở đầu dây, tính tương đối so với base_link lúc spawn.
# Mặc định: tải nằm phía sau drone theo trục X âm.
# Drone sẽ kéo dây theo hướng +X.
PAYLOAD_X = -CABLE_TOTAL_LENGTH
PAYLOAD_Y = 0.0
PAYLOAD_Z = DRONE_ATTACH_Z

# Tải ở đầu dây
PAYLOAD_MASS = 0.1
PAYLOAD_RADIUS = 0.08

# Joint damping
PAYLOAD_JOINT_DAMPING = 0.005
CABLE_JOINT_DAMPING = 0.0025
DRONE_JOINT_DAMPING = 0.008

# Ma sát contact thấp để dây không bị dính/kẹt vào vòng
CONTACT_MU = 0.02
CONTACT_MU2 = 0.02

# Contact mềm hơn để tránh dây bị khóa cứng khi chạm vòng
CONTACT_KP = 20000
CONTACT_KD = 5
CONTACT_MAX_VEL = 0.3
CONTACT_MIN_DEPTH = 0.001


def cylinder_inertia(mass: float, radius: float, length: float):
    """
    Quán tính cylinder đặc.
    Cylinder trong Gazebo có trục dọc theo local Z.
    Ở đây ta xoay link để local Z nằm theo hướng dây.
    """
    ixx = (1.0 / 12.0) * mass * (3.0 * radius * radius + length * length)
    iyy = ixx
    izz = 0.5 * mass * radius * radius
    return ixx, iyy, izz


def sphere_inertia(mass: float, radius: float):
    """
    Quán tính sphere đặc cho payload.
    """
    i = (2.0 / 5.0) * mass * radius * radius
    return i, i, i


def inertial_xml(mass: float, ixx: float, iyy: float, izz: float, indent: str = "      ") -> str:
    return f"""{indent}<inertial>
{indent}  <mass>{mass:.9f}</mass>
{indent}  <inertia>
{indent}    <ixx>{ixx:.12e}</ixx>
{indent}    <iyy>{iyy:.12e}</iyy>
{indent}    <izz>{izz:.12e}</izz>
{indent}    <ixy>0</ixy>
{indent}    <ixz>0</ixz>
{indent}    <iyz>0</iyz>
{indent}  </inertia>
{indent}</inertial>
"""


def contact_surface_xml(indent: str = "        ") -> str:
    return f"""{indent}<surface>
{indent}  <friction>
{indent}    <ode>
{indent}      <mu>{CONTACT_MU}</mu>
{indent}      <mu2>{CONTACT_MU2}</mu2>
{indent}    </ode>
{indent}  </friction>

{indent}  <contact>
{indent}    <ode>
{indent}      <kp>{CONTACT_KP}</kp>
{indent}      <kd>{CONTACT_KD}</kd>
{indent}      <max_vel>{CONTACT_MAX_VEL}</max_vel>
{indent}      <min_depth>{CONTACT_MIN_DEPTH}</min_depth>
{indent}    </ode>
{indent}  </contact>
{indent}</surface>
"""


def make_payload_link() -> str:
    """
    Link payload là tải ở đầu dây.
    Payload không fixed vào world, nên dây và drone có thể kéo tải này.
    """
    radius = PAYLOAD_RADIUS
    mass = PAYLOAD_MASS
    ixx, iyy, izz = sphere_inertia(mass, radius)

    return f"""
    <!-- ========================================================= -->
    <!-- Cable payload                                              -->
    <!-- Payload pose is relative to base_link at spawn time         -->
    <!-- ========================================================= -->

    <link name="cable_payload">
      <pose relative_to="base_link">{PAYLOAD_X:.6f} {PAYLOAD_Y:.6f} {PAYLOAD_Z:.6f} 0 0 0</pose>

{inertial_xml(mass, ixx, iyy, izz)}

      <collision name="collision">
        <geometry>
          <sphere>
            <radius>{radius}</radius>
          </sphere>
        </geometry>
      </collision>

      <visual name="visual">
        <geometry>
          <sphere>
            <radius>{radius}</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>0.8 0.05 0.05 1</ambient>
          <diffuse>0.8 0.05 0.05 1</diffuse>
        </material>
      </visual>
    </link>
"""


def make_cable_link(i: int, n_segments: int) -> str:
    """
    Tạo một đoạn dây.

    Quy ước:
    - Payload ở x = -20 m.
    - Drone attach point ở x = 0 m.
    - Dây nằm ban đầu theo phương X.
    - Cylinder mặc định dọc local Z, nên ta xoay link quanh Y 90 độ để local Z trùng world X.
    """
    name = f"cable_seg_{i:03d}"

    seg_mass = LINEAR_DENSITY * SEG_LENGTH
    ixx, iyy, izz = cylinder_inertia(seg_mass, SEG_RADIUS, SEG_LENGTH)

    # Segment i chạy từ payload về drone.
    # seg_000 gần payload, segment cuối gần drone.
    x = PAYLOAD_X + (i + 0.5) * SEG_LENGTH
    y = PAYLOAD_Y
    z = PAYLOAD_Z

    # Xoay cylinder: local Z -> world X
    pitch_90 = math.pi / 2.0

    return f"""
    <link name="{name}">
      <pose relative_to="base_link">{x:.6f} {y:.6f} {z:.6f} 0 {pitch_90:.12f} 0</pose>

{inertial_xml(seg_mass, ixx, iyy, izz)}

      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>{SEG_RADIUS}</radius>
            <length>{SEG_LENGTH}</length>
          </cylinder>
        </geometry>

{contact_surface_xml()}
      </collision>

      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>{SEG_RADIUS}</radius>
            <length>{SEG_LENGTH}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.02 0.02 0.02 1</ambient>
          <diffuse>0.02 0.02 0.02 1</diffuse>
        </material>
      </visual>
    </link>
"""


def joint_dynamics_xml(damping: float, indent: str = "      ") -> str:
    return f"""{indent}<axis>
{indent}  <xyz>1 0 0</xyz>
{indent}  <dynamics>
{indent}    <damping>{damping}</damping>
{indent}    <friction>0.0</friction>
{indent}  </dynamics>
{indent}</axis>

{indent}<axis2>
{indent}  <xyz>0 1 0</xyz>
{indent}  <dynamics>
{indent}    <damping>{damping}</damping>
{indent}    <friction>0.0</friction>
{indent}  </dynamics>
{indent}</axis2>
"""


def make_payload_to_first_joint() -> str:
    """
    Joint từ payload tới segment đầu tiên.
    Joint frame đặt tại payload.
    Xoay joint frame giống dây để local Z nằm theo phương dây.
    """
    pitch_90 = math.pi / 2.0

    return f"""
    <joint name="cable_payload_to_seg_000" type="universal">
      <parent>cable_payload</parent>
      <child>cable_seg_000</child>
      <pose relative_to="cable_payload">0 0 0 0 {pitch_90:.12f} 0</pose>

{joint_dynamics_xml(PAYLOAD_JOINT_DAMPING)}
    </joint>
"""


def make_cable_joint(i: int) -> str:
    """
    Joint giữa cable_seg_i và cable_seg_{i+1}.
    Vì cylinder local Z là trục dây, joint ở đầu local +Z của segment parent.
    """
    parent = f"cable_seg_{i:03d}"
    child = f"cable_seg_{i + 1:03d}"

    return f"""
    <joint name="{parent}_to_{child}" type="universal">
      <parent>{parent}</parent>
      <child>{child}</child>
      <pose relative_to="{parent}">0 0 {SEG_LENGTH / 2.0:.6f} 0 0 0</pose>

{joint_dynamics_xml(CABLE_JOINT_DAMPING)}
    </joint>
"""


def make_last_to_drone_joint(n_segments: int) -> str:
    """
    Joint nối đoạn dây cuối với base_link của drone.

    Đây là joint dùng để "nhả dây".
    Khi muốn thả dây khỏi drone, ta sẽ remove joint tên:
        cable_release_joint
    """
    last = f"cable_seg_{n_segments - 1:03d}"

    return f"""
    <joint name="cable_release_joint" type="universal">
      <parent>{last}</parent>
      <child>base_link</child>
      <pose relative_to="{last}">0 0 {SEG_LENGTH / 2.0:.6f} 0 0 0</pose>

{joint_dynamics_xml(DRONE_JOINT_DAMPING)}
    </joint>
"""


def make_motor_plugin(motor_number: int, turning_direction: str) -> str:
    return f"""
    <plugin filename="gz-sim-multicopter-motor-model-system"
      name="gz::sim::systems::MulticopterMotorModel">
      <jointName>rotor_{motor_number}_joint</jointName>
      <linkName>rotor_{motor_number}</linkName>
      <turningDirection>{turning_direction}</turningDirection>
      <timeConstantUp>0.0125</timeConstantUp>
      <timeConstantDown>0.025</timeConstantDown>
      <maxRotVelocity>1000.0</maxRotVelocity>
      <motorConstant>8.54858e-06</motorConstant>
      <momentConstant>0.016</momentConstant>
      <commandSubTopic>command/motor_speed</commandSubTopic>
      <motorNumber>{motor_number}</motorNumber>
      <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
      <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>
"""


def make_model_sdf() -> str:
    n_segments = int(round(CABLE_TOTAL_LENGTH / SEG_LENGTH))
    actual_length = n_segments * SEG_LENGTH

    if abs(actual_length - CABLE_TOTAL_LENGTH) > 1e-6:
        raise ValueError("CABLE_TOTAL_LENGTH phải chia hết cho SEG_LENGTH để tránh lệch hình học.")

    cable_links = "\n".join(make_cable_link(i, n_segments) for i in range(n_segments))

    joints = []
    joints.append(make_payload_to_first_joint())
    for i in range(n_segments - 1):
        joints.append(make_cable_joint(i))
    joints.append(make_last_to_drone_joint(n_segments))

    motor_plugins = "\n".join([
        make_motor_plugin(0, "ccw"),
        make_motor_plugin(1, "ccw"),
        make_motor_plugin(2, "cw"),
        make_motor_plugin(3, "cw"),
    ])

    sdf = f"""<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <model name="{MODEL_NAME}">

    <self_collide>false</self_collide>

    <include merge="true">
      <uri>model://x500_base</uri>
    </include>

    <!-- ========================================================= -->
    <!-- {actual_length:.2f} m cable tether                                      -->
    <!-- One end attached to a 1 kg payload, one end attached to drone base_link -->
    <!-- Cable direction at spawn: from x = {PAYLOAD_X:.2f} m to x = 0 m -->
    <!-- Segment length: {SEG_LENGTH:.2f} m                         -->
    <!-- Number of segments: {n_segments}                           -->
    <!-- Total cable mass: {LINEAR_DENSITY * actual_length:.3f} kg   -->
    <!-- Payload mass: {PAYLOAD_MASS:.3f} kg                         -->
    <!-- ========================================================= -->

{make_payload_link()}

{cable_links}

    <!-- ========================================================= -->
    <!-- Cable joints                                               -->
    <!-- ========================================================= -->

{''.join(joints)}

    <!-- ========================================================= -->
    <!-- Original x500 motor plugins                                -->
    <!-- ========================================================= -->

{motor_plugins}

    <plugin filename="MotorFailurePlugin"
      name="gz::sim::systems::MotorFailureSystem">
    </plugin>

  </model>
</sdf>
"""
    return sdf


def main():
    sdf = make_model_sdf()
    OUT_FILE.write_text(sdf)

    n_segments = int(round(CABLE_TOTAL_LENGTH / SEG_LENGTH))
    print(f"Generated: {OUT_FILE.resolve()}")
    print(f"Cable length     : {CABLE_TOTAL_LENGTH} m")
    print(f"Segment length   : {SEG_LENGTH} m")
    print(f"Number of segments: {n_segments}")
    print(f"Total cable mass : {LINEAR_DENSITY * CABLE_TOTAL_LENGTH:.3f} kg")
    print(f"Payload mass     : {PAYLOAD_MASS:.3f} kg")
    print(f"Payload position : x={PAYLOAD_X}, y={PAYLOAD_Y}, z={PAYLOAD_Z} relative to base_link")


if __name__ == "__main__":
    main()
