"""
Generate crs_sim/urdf/SR450.urdf and its link meshes (Bosch SR450 SCARA).

The arm links are boxes and cylinders built on the kinematics of
ctu_bosch_sr450.RobotBosch (link lengths 0.25 m / 0.2 m, flange height
0.5 m + q[2]) and sized after the SR450 data sheet drawing. The adapter is
robot_ee/bosch_adapter.step from https://github.com/pilcsimo/ROB1-2025-hw,
the hoop is the same mesh the other robots use.

    python tools/make_bosch_urdf.py [path/to/bosch_adapter.step]

Without the STEP path the existing meshes/bosch_adapter.stl is kept.
Needs trimesh (and cascadio for the STEP file), i.e. the `step` extra.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import trimesh

URDF_DIR = Path(__file__).resolve().parents[1] / "crs_sim" / "urdf"
MESH_DIR = URDF_DIR / "meshes"

L1, L2 = 0.25, 0.2          # link lengths of RobotBosch
Z_FLANGE = 0.5              # flange height at q[2] = 0 (RobotBosch._z_offset)
Q2_LIMITS = (-146520 / 444000.0, 2220 / 444000.0)


def box(center, size):
    return trimesh.creation.box(extents=size,
                                transform=trimesh.transformations.translation_matrix(center))


def cylinder(x, y, z0, z1, radius):
    return trimesh.creation.cylinder(
        radius=radius, height=z1 - z0, sections=48,
        transform=trimesh.transformations.translation_matrix([x, y, (z0 + z1) / 2]))


def save(name, parts):
    mesh = trimesh.util.concatenate(parts)
    mesh.apply_scale(1000.0)        # the URDF scales the meshes from mm
    mesh.export(MESH_DIR / f"{name}.stl")


def make_meshes():
    # base: floor pedestal behind axis 1, column, head with the arm 1 motor
    save("sr450_base_link", [
        box([-0.30, 0, 0.04], [0.24, 0.20, 0.08]),
        cylinder(-0.30, 0, 0.08, 0.62, 0.06),
        box([-0.30, 0, 0.74], [0.26, 0.20, 0.24]),
        box([-0.14, 0, 0.70], [0.28, 0.14, 0.10]),
        cylinder(0, 0, 0.64, 0.76, 0.07),
    ])
    # arm 1 (link_1 frame = base rotated by q0)
    save("sr450_link_1", [
        box([L1 / 2, 0, 0.60], [L1, 0.13, 0.06]),
        cylinder(0, 0, 0.57, 0.63, 0.065),
        cylinder(L1, 0, 0.57, 0.63, 0.065),
    ])
    # arm 2 (link_2 frame at axis 2), motor housing of the quill at its end
    save("sr450_link_2", [
        box([L2 / 2, 0, 0.67], [L2, 0.11, 0.08]),
        cylinder(0, 0, 0.63, 0.71, 0.055),
        cylinder(L2, 0, 0.63, 0.71, 0.055),
        cylinder(L2, 0, 0.71, 0.82, 0.05),
    ])
    # quill (link_3 frame at the flange face, z up)
    save("sr450_link_3", [cylinder(0, 0, 0.012, 0.40, 0.0125)])
    # rotating flange disc (link_4)
    save("sr450_link_4", [cylinder(0, 0, 0.0, 0.012, 0.025)])


def convert_adapter(step_path):
    mesh = trimesh.load(step_path, force="mesh")     # STEP is loaded in m
    mesh.apply_scale(1000.0)
    mesh.export(MESH_DIR / "bosch_adapter.stl")


INERTIAL = ('<inertial><mass value="1"/><inertia ixx="0.01" iyy="0.01" izz="0.01" '
            'ixy="0" ixz="0" iyz="0"/></inertial>')


def link(name, mesh=None, rgba="0.92 0.92 0.9 1", rpy="0 0 0"):
    if mesh is None:
        return f'    <link name="{name}">\n        {INERTIAL}\n    </link>\n'
    return f'''    <link name="{name}">
        {INERTIAL}
        <visual>
            <origin xyz="0 0 0" rpy="{rpy}"/>
            <geometry><mesh filename="meshes/{mesh}.stl" scale="0.001 0.001 0.001"/></geometry>
            <material name="{name}_color"><color rgba="{rgba}"/></material>
        </visual>
    </link>
'''


def joint(name, kind, parent, child, xyz="0 0 0", rpy="0 0 0", limits=None):
    s = (f'    <joint name="{name}" type="{kind}">\n'
         f'        <origin xyz="{xyz}" rpy="{rpy}"/>\n'
         f'        <parent link="{parent}"/>\n        <child link="{child}"/>\n')
    if kind != "fixed":
        lo, hi = limits
        s += ('        <axis xyz="0 0 1"/>\n'
              f'        <limit effort="100" velocity="10" lower="{lo:.6f}" upper="{hi:.6f}"/>\n')
    return s + "    </joint>\n"


def make_urdf():
    q_lim = 80536 / 54286.97186604, 73434 / 35061.99184485
    green, grey = "0.72 0.78 0.3 1", "0.3 0.3 0.32 1"
    parts = [
        link("base_link", "sr450_base_link", grey),
        joint("joint1", "revolute", "base_link", "link_1", limits=(-q_lim[0], q_lim[0])),
        link("link_1", "sr450_link_1", green),
        joint("joint2", "revolute", "link_1", "link_2", xyz=f"{L1} 0 0",
              limits=(-q_lim[1], q_lim[1])),
        link("link_2", "sr450_link_2", green),
        joint("joint3", "prismatic", "link_2", "link_3", xyz=f"{L2} 0 {Z_FLANGE}",
              limits=Q2_LIMITS),
        link("link_3", "sr450_link_3", "0.8 0.8 0.82 1"),
        # the URDF joint is relative to arm 2, RobotBosch's q[3] is absolute:
        # crs_sim.RobotBosch.model_q converts
        joint("joint4", "revolute", "link_3", "link_4", limits=(-20.0, 20.0)),
        link("link_4", "sr450_link_4", grey),
        # flange: z along the tool (down), the hoop points along phi
        joint("joint_flange", "fixed", "link_4", "flange", rpy="0 3.14159265359 0"),
        link("flange"),
        joint("joint_adapter", "fixed", "flange", "adapter",
              rpy="3.14159265359 0 3.14159265359"),
        link("adapter", "bosch_adapter", "0.14 0.55 0.68 0.8", rpy="0 0 1.5707963268"),
        joint("joint_hoop", "fixed", "adapter", "hoop", xyz="0 0 -0.008"),
        link("hoop", "hoop", "0.14 0.55 0.68 0.8", rpy="0 0 1.5707963268"),
    ]
    header = ('<?xml version="1.0"?>\n'
              "<!-- Bosch SR450 SCARA, generated by tools/make_bosch_urdf.py from the\n"
              "     kinematics of ctu_bosch_sr450.RobotBosch: the flange frame is\n"
              "     RobotBosch.flange_pose(q). The link shapes are boxes / cylinders\n"
              "     approximating the real arm, the adapter and the hoop are the CAD\n"
              "     models. joint4 is relative to arm 2 (see RobotBosch.model_q). -->\n"
              '<robot name="SR450">\n')
    (URDF_DIR / "SR450.urdf").write_text(header + "".join(parts) + "</robot>\n")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        convert_adapter(sys.argv[1])
    make_meshes()
    make_urdf()
