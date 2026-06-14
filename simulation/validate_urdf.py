from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from collections import Counter
from pathlib import Path
from typing import Any, Dict, List, Sequence

SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE = SCRIPT_DIR.parent
VENDOR_DIR = SCRIPT_DIR / "_vendor"
if VENDOR_DIR.exists():
    sys.path.insert(0, str(VENDOR_DIR))
sys.path.insert(0, str(SCRIPT_DIR))

from simlib import as_list, choose_end_link, find_chain_joints, forward_kinematics, load_urdf, transform_point


DEFAULT_URDF = SCRIPT_DIR / "lattice.urdf"
DEFAULT_EXPECTED_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


def duplicates(values: Sequence[str]) -> List[str]:
    counts = Counter(values)
    return sorted([value for value, count in counts.items() if count > 1])


def run_pybullet_direct_check(urdf_path: Path) -> Dict[str, Any]:
    if importlib.util.find_spec("pybullet") is None:
        return {"available": False, "loaded": None, "message": "pybullet is not installed"}

    import pybullet as p

    client = p.connect(p.DIRECT)
    try:
        p.setAdditionalSearchPath(str(urdf_path.parent))
        robot_id = p.loadURDF(str(urdf_path), useFixedBase=True)
        return {
            "available": True,
            "loaded": True,
            "body_id": robot_id,
            "joint_count": p.getNumJoints(robot_id),
        }
    except Exception as exc:  # pragma: no cover - depends on optional native package
        return {"available": True, "loaded": False, "message": str(exc)}
    finally:
        p.disconnect(client)


def validate(urdf_path: Path, expected_joints: Sequence[str], end_link: str | None, check_pybullet: bool) -> Dict[str, Any]:
    model = load_urdf(urdf_path)
    link_names = model.link_names
    joint_names = [joint.name for joint in model.joints]
    missing_meshes = [
        {
            "filename": mesh.filename,
            "link": mesh.link_name,
            "role": mesh.role,
            "resolved_path": str(mesh.resolved_path) if mesh.resolved_path else None,
        }
        for mesh in model.meshes
        if mesh.resolved_path is None or not mesh.resolved_path.exists()
    ]
    expected_missing = [name for name in expected_joints if name not in joint_names]
    active_without_limits = [
        joint.name
        for joint in model.active_joints
        if joint.type != "continuous" and (joint.lower is None or joint.upper is None)
    ]

    selected_end_link = choose_end_link(model, end_link)
    root_link = model.root_links[0] if model.root_links else None
    fk_report: Dict[str, Any]
    if root_link:
        chain = find_chain_joints(model, selected_end_link, root_link)
        zero_poses, _ = forward_kinematics(model, {}, root_link)
        ee_zero = transform_point(zero_poses[selected_end_link])
        fk_report = {
            "root_link": root_link,
            "end_link": selected_end_link,
            "chain_joints": [joint.name for joint in chain],
            "zero_pose_end_position_m": as_list(ee_zero),
        }
    else:
        fk_report = {"root_link": None, "end_link": selected_end_link, "error": "no root link"}

    issues: List[str] = []
    if missing_meshes:
        issues.append(f"{len(missing_meshes)} mesh references are missing")
    if len(model.root_links) != 1:
        issues.append(f"expected exactly one root link, found {len(model.root_links)}")
    if duplicates(link_names):
        issues.append("duplicate link names found")
    if duplicates(joint_names):
        issues.append("duplicate joint names found")
    if expected_missing:
        issues.append("expected LeRobot/SO101 joint names are missing")
    if active_without_limits:
        issues.append("active joints without lower/upper limits found")

    report: Dict[str, Any] = {
        "urdf": str(model.path),
        "robot_name": model.name,
        "link_count": len(model.links),
        "joint_count": len(model.joints),
        "active_joint_count": len(model.active_joints),
        "root_links": model.root_links,
        "leaf_links": model.leaf_links,
        "active_joints": [
            {
                "name": joint.name,
                "type": joint.type,
                "parent": joint.parent,
                "child": joint.child,
                "axis": as_list(joint.axis),
                "lower": joint.lower,
                "upper": joint.upper,
            }
            for joint in model.active_joints
        ],
        "mesh_reference_count": len(model.meshes),
        "missing_meshes": missing_meshes,
        "duplicate_links": duplicates(link_names),
        "duplicate_joints": duplicates(joint_names),
        "expected_joints": list(expected_joints),
        "expected_joints_missing": expected_missing,
        "active_joints_without_limits": active_without_limits,
        "forward_kinematics": fk_report,
        "issues": issues,
        "ok": not issues,
    }

    if check_pybullet:
        pybullet_report = run_pybullet_direct_check(model.path)
        report["pybullet_direct_check"] = pybullet_report
        if pybullet_report.get("available") and not pybullet_report.get("loaded"):
            report["ok"] = False
            report["issues"].append("pybullet could not load the URDF")

    return report


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate a local SO101/Lattice URDF and its mesh references.")
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF, help="URDF to validate")
    parser.add_argument("--end-link", default="gripper_frame_link", help="End link used for FK smoke testing")
    parser.add_argument("--expected-joint", action="append", dest="expected_joints", help="Expected active joint name")
    parser.add_argument("--check-pybullet", action="store_true", help="Also try loading the URDF with PyBullet if installed")
    parser.add_argument(
        "--json-output",
        type=Path,
        default=SCRIPT_DIR / "lattice_validation.json",
        help="Where to write the validation report",
    )
    args = parser.parse_args()

    expected_joints = args.expected_joints or DEFAULT_EXPECTED_JOINTS
    report = validate(args.urdf, expected_joints, args.end_link, args.check_pybullet)
    args.json_output.parent.mkdir(parents=True, exist_ok=True)
    args.json_output.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(f"URDF: {report['urdf']}")
    print(f"Robot: {report['robot_name']}")
    print(f"Links/joints: {report['link_count']} links, {report['joint_count']} joints")
    print(f"Active joints: {', '.join(joint['name'] for joint in report['active_joints'])}")
    print(f"Missing meshes: {len(report['missing_meshes'])}")
    print(f"FK end link: {report['forward_kinematics']['end_link']}")
    print(f"Report: {args.json_output}")
    if report["issues"]:
        print("Issues:")
        for issue in report["issues"]:
            print(f"- {issue}")
        return 1
    print("Validation passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
