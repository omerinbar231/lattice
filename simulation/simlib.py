from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


Vec3 = Tuple[float, float, float]
Mat4 = List[List[float]]


@dataclass(frozen=True)
class Link:
    name: str
    mass: Optional[float] = None


@dataclass(frozen=True)
class Joint:
    name: str
    type: str
    parent: str
    child: str
    origin_xyz: Vec3
    origin_rpy: Vec3
    axis: Vec3
    lower: Optional[float]
    upper: Optional[float]
    effort: Optional[float]
    velocity: Optional[float]


@dataclass(frozen=True)
class MeshRef:
    filename: str
    resolved_path: Optional[Path]
    link_name: str
    role: str


@dataclass
class UrdfModel:
    path: Path
    name: str
    links: List[Link]
    joints: List[Joint]
    meshes: List[MeshRef]

    @property
    def link_names(self) -> List[str]:
        return [link.name for link in self.links]

    @property
    def joints_by_name(self) -> Dict[str, Joint]:
        return {joint.name: joint for joint in self.joints}

    @property
    def active_joints(self) -> List[Joint]:
        return [joint for joint in self.joints if joint.type in {"revolute", "continuous", "prismatic"}]

    @property
    def root_links(self) -> List[str]:
        children = {joint.child for joint in self.joints}
        return [link.name for link in self.links if link.name not in children]

    @property
    def leaf_links(self) -> List[str]:
        parents = {joint.parent for joint in self.joints}
        return [link.name for link in self.links if link.name not in parents]


def parse_vec(text: Optional[str], default: Vec3 = (0.0, 0.0, 0.0)) -> Vec3:
    if not text:
        return default
    parts = [float(value) for value in text.split()]
    if len(parts) != 3:
        raise ValueError(f"expected 3 vector values, got {len(parts)} in {text!r}")
    return parts[0], parts[1], parts[2]


def parse_float(text: Optional[str]) -> Optional[float]:
    if text is None:
        return None
    return float(text)


def identity() -> Mat4:
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def matmul(a: Mat4, b: Mat4) -> Mat4:
    out = [[0.0] * 4 for _ in range(4)]
    for row in range(4):
        for col in range(4):
            out[row][col] = sum(a[row][idx] * b[idx][col] for idx in range(4))
    return out


def transform_point(t: Mat4, point: Vec3 = (0.0, 0.0, 0.0)) -> Vec3:
    x, y, z = point
    return (
        t[0][0] * x + t[0][1] * y + t[0][2] * z + t[0][3],
        t[1][0] * x + t[1][1] * y + t[1][2] * z + t[1][3],
        t[2][0] * x + t[2][1] * y + t[2][2] * z + t[2][3],
    )


def rpy_transform(xyz: Vec3, rpy: Vec3) -> Mat4:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rot = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    out = identity()
    for row in range(3):
        for col in range(3):
            out[row][col] = rot[row][col]
        out[row][3] = xyz[row]
    return out


def axis_angle(axis: Vec3, angle: float) -> Mat4:
    x, y, z = axis
    norm = math.sqrt(x * x + y * y + z * z)
    if norm == 0.0:
        return identity()
    x, y, z = x / norm, y / norm, z / norm
    c = math.cos(angle)
    s = math.sin(angle)
    v = 1.0 - c
    out = identity()
    out[0][0] = x * x * v + c
    out[0][1] = x * y * v - z * s
    out[0][2] = x * z * v + y * s
    out[1][0] = y * x * v + z * s
    out[1][1] = y * y * v + c
    out[1][2] = y * z * v - x * s
    out[2][0] = z * x * v - y * s
    out[2][1] = z * y * v + x * s
    out[2][2] = z * z * v + c
    return out


def axis_translation(axis: Vec3, distance: float) -> Mat4:
    x, y, z = axis
    norm = math.sqrt(x * x + y * y + z * z)
    if norm == 0.0:
        return identity()
    out = identity()
    out[0][3] = distance * x / norm
    out[1][3] = distance * y / norm
    out[2][3] = distance * z / norm
    return out


def resolve_mesh_path(urdf_dir: Path, filename: str) -> Optional[Path]:
    if filename.startswith("package://"):
        return None
    candidate = Path(filename)
    if not candidate.is_absolute():
        candidate = urdf_dir / candidate
    return candidate.resolve()


def load_urdf(path: str | Path) -> UrdfModel:
    urdf_path = Path(path).resolve()
    root = ET.parse(urdf_path).getroot()
    links: List[Link] = []
    joints: List[Joint] = []
    meshes: List[MeshRef] = []

    for link_el in root.findall("link"):
        mass = None
        mass_el = link_el.find("inertial/mass")
        if mass_el is not None:
            mass = parse_float(mass_el.get("value"))
        link_name = link_el.get("name", "")
        links.append(Link(name=link_name, mass=mass))
        for role in ("visual", "collision"):
            for mesh_el in link_el.findall(f"{role}/geometry/mesh"):
                filename = mesh_el.get("filename", "")
                meshes.append(
                    MeshRef(
                        filename=filename,
                        resolved_path=resolve_mesh_path(urdf_path.parent, filename),
                        link_name=link_name,
                        role=role,
                    )
                )

    for joint_el in root.findall("joint"):
        origin_el = joint_el.find("origin")
        axis_el = joint_el.find("axis")
        limit_el = joint_el.find("limit")
        joints.append(
            Joint(
                name=joint_el.get("name", ""),
                type=joint_el.get("type", "fixed"),
                parent=joint_el.find("parent").get("link", ""),
                child=joint_el.find("child").get("link", ""),
                origin_xyz=parse_vec(origin_el.get("xyz") if origin_el is not None else None),
                origin_rpy=parse_vec(origin_el.get("rpy") if origin_el is not None else None),
                axis=parse_vec(axis_el.get("xyz") if axis_el is not None else None),
                lower=parse_float(limit_el.get("lower")) if limit_el is not None else None,
                upper=parse_float(limit_el.get("upper")) if limit_el is not None else None,
                effort=parse_float(limit_el.get("effort")) if limit_el is not None else None,
                velocity=parse_float(limit_el.get("velocity")) if limit_el is not None else None,
            )
        )

    return UrdfModel(path=urdf_path, name=root.get("name", urdf_path.stem), links=links, joints=joints, meshes=meshes)


def joint_midpoint(joint: Joint) -> float:
    if joint.lower is None or joint.upper is None:
        return 0.0
    return 0.5 * (joint.lower + joint.upper)


def clamp_joint(joint: Joint, value: float) -> float:
    if joint.lower is not None:
        value = max(joint.lower, value)
    if joint.upper is not None:
        value = min(joint.upper, value)
    return value


def joint_motion_transform(joint: Joint, value: float) -> Mat4:
    if joint.type in {"revolute", "continuous"}:
        return axis_angle(joint.axis, value)
    if joint.type == "prismatic":
        return axis_translation(joint.axis, value)
    return identity()


def child_joints_by_parent(model: UrdfModel) -> Dict[str, List[Joint]]:
    children: Dict[str, List[Joint]] = {}
    for joint in model.joints:
        children.setdefault(joint.parent, []).append(joint)
    return children


def forward_kinematics(
    model: UrdfModel,
    joint_positions: Optional[Dict[str, float]] = None,
    root_link: Optional[str] = None,
) -> Tuple[Dict[str, Mat4], Dict[str, Mat4]]:
    joint_positions = joint_positions or {}
    roots = model.root_links
    if root_link is None:
        if not roots:
            raise ValueError("URDF has no root link")
        root_link = roots[0]

    link_poses: Dict[str, Mat4] = {root_link: identity()}
    joint_poses: Dict[str, Mat4] = {}
    unresolved = set(range(len(model.joints)))

    while unresolved:
        changed = False
        for idx in list(unresolved):
            joint = model.joints[idx]
            parent_pose = link_poses.get(joint.parent)
            if parent_pose is None:
                continue
            joint_pose = matmul(parent_pose, rpy_transform(joint.origin_xyz, joint.origin_rpy))
            value = joint_positions.get(joint.name, 0.0)
            child_pose = matmul(joint_pose, joint_motion_transform(joint, value))
            joint_poses[joint.name] = joint_pose
            link_poses[joint.child] = child_pose
            unresolved.remove(idx)
            changed = True
        if not changed:
            names = ", ".join(model.joints[idx].name for idx in sorted(unresolved))
            raise ValueError(f"could not resolve joint tree from root {root_link!r}: {names}")

    return link_poses, joint_poses


def find_chain_joints(model: UrdfModel, end_link: str, root_link: Optional[str] = None) -> List[Joint]:
    roots = [root_link] if root_link else model.root_links
    children = child_joints_by_parent(model)
    for root in roots:
        queue = deque([(root, [])])
        seen = {root}
        while queue:
            link_name, path = queue.popleft()
            if link_name == end_link:
                return path
            for joint in children.get(link_name, []):
                if joint.child in seen:
                    continue
                seen.add(joint.child)
                queue.append((joint.child, path + [joint]))
    raise ValueError(f"no kinematic path found to link {end_link!r}")


def choose_end_link(model: UrdfModel, preferred: Optional[str] = None) -> str:
    if preferred and preferred in model.link_names:
        return preferred
    for candidate in ("gripper_frame_link", "tool0", "ee_link", "end_effector_link", "gripper_link"):
        if candidate in model.link_names:
            return candidate
    leaves = model.leaf_links
    if not leaves:
        raise ValueError("URDF has no leaf links")
    return leaves[-1]


def ordered_active_joints(model: UrdfModel, preferred_order: Optional[Sequence[str]] = None) -> List[Joint]:
    by_name = model.joints_by_name
    ordered: List[Joint] = []
    if preferred_order:
        for name in preferred_order:
            joint = by_name.get(name)
            if joint and joint.type in {"revolute", "continuous", "prismatic"}:
                ordered.append(joint)
    seen = {joint.name for joint in ordered}
    for joint in model.active_joints:
        if joint.name not in seen:
            ordered.append(joint)
    return ordered


def wave_joint_positions(joints: Sequence[Joint], time_s: float, duration_s: float) -> Dict[str, float]:
    positions: Dict[str, float] = {}
    period = max(duration_s, 0.001)
    for index, joint in enumerate(joints):
        center = joint_midpoint(joint)
        if joint.lower is not None and joint.upper is not None:
            span = max(0.0, joint.upper - joint.lower)
            amplitude = min(0.35, 0.25 * span)
        elif joint.type == "prismatic":
            amplitude = 0.02
        else:
            amplitude = 0.35
        value = center + amplitude * math.sin(2.0 * math.pi * time_s / period + index * 0.7)
        positions[joint.name] = clamp_joint(joint, value)
    return positions


def skeleton_points(
    link_poses: Dict[str, Mat4],
    joint_poses: Dict[str, Mat4],
    chain_joints: Sequence[Joint],
    root_link: str,
    end_link: str,
) -> List[Vec3]:
    points = [transform_point(link_poses[root_link])]
    for joint in chain_joints:
        points.append(transform_point(joint_poses[joint.name]))
    points.append(transform_point(link_poses[end_link]))
    compact: List[Vec3] = []
    for point in points:
        if not compact or any(abs(point[idx] - compact[-1][idx]) > 1e-9 for idx in range(3)):
            compact.append(point)
    return compact


def point_bounds(points: Iterable[Vec3]) -> Dict[str, Vec3]:
    pts = list(points)
    return {
        "min": (min(p[0] for p in pts), min(p[1] for p in pts), min(p[2] for p in pts)),
        "max": (max(p[0] for p in pts), max(p[1] for p in pts), max(p[2] for p in pts)),
    }


def as_list(vec: Vec3) -> List[float]:
    return [vec[0], vec[1], vec[2]]

