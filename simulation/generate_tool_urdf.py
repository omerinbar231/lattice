from __future__ import annotations

import argparse
import os
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Iterable, List


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_BASE = SCRIPT_DIR / "lattice_base.urdf"
DEFAULT_TOOLS_DIR = SCRIPT_DIR / "tools"
DEFAULT_OUTPUT_DIR = SCRIPT_DIR / "generated"


def tool_files(tools_dir: Path = DEFAULT_TOOLS_DIR) -> List[Path]:
    return sorted(path for path in tools_dir.glob("*.xml") if path.is_file())


def available_tools(tools_dir: Path = DEFAULT_TOOLS_DIR) -> List[str]:
    return [path.stem for path in tool_files(tools_dir)]


def load_tool(tool_name: str, tools_dir: Path = DEFAULT_TOOLS_DIR) -> ET.Element:
    tool_path = tools_dir / f"{tool_name}.xml"
    if not tool_path.exists():
        names = ", ".join(available_tools(tools_dir)) or "(none)"
        raise ValueError(f"unknown tool {tool_name!r}; available tools: {names}")
    root = ET.parse(tool_path).getroot()
    if root.tag != "tool":
        raise ValueError(f"{tool_path} must use a <tool> root element")
    return root


def rewrite_mesh_paths(robot: ET.Element, source_dir: Path, output_dir: Path) -> None:
    for mesh in robot.findall(".//mesh"):
        filename = mesh.get("filename", "")
        if not filename or filename.startswith("package://"):
            continue
        mesh_path = Path(filename)
        if mesh_path.is_absolute():
            continue
        resolved = (source_dir / mesh_path).resolve()
        rel = os.path.relpath(resolved, output_dir.resolve()).replace(os.sep, "/")
        mesh.set("filename", rel)


def build_urdf(
    tool_name: str,
    output_path: Path | None = None,
    base_path: Path = DEFAULT_BASE,
    tools_dir: Path = DEFAULT_TOOLS_DIR,
) -> Path:
    base_tree = ET.parse(base_path)
    robot = base_tree.getroot()
    tool = load_tool(tool_name, tools_dir)
    robot.set("name", f"lattice_arm_{tool_name}")

    for child in list(tool):
        robot.append(child)

    if output_path is None:
        output_path = DEFAULT_OUTPUT_DIR / f"lattice_{tool_name}.urdf"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    rewrite_mesh_paths(robot, base_path.parent, output_path.parent)
    ET.indent(robot, space="  ")
    base_tree.write(output_path, encoding="utf-8", xml_declaration=True)
    return output_path


def build_all(output_dir: Path = DEFAULT_OUTPUT_DIR, tools_dir: Path = DEFAULT_TOOLS_DIR) -> List[Path]:
    return [build_urdf(name, output_dir / f"lattice_{name}.urdf", tools_dir=tools_dir) for name in available_tools(tools_dir)]


def print_tools(names: Iterable[str]) -> None:
    for name in names:
        print(name)


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate Lattice Arm URDF variants from a tool module.")
    parser.add_argument("--tool", default="gripper", help="Tool module to attach")
    parser.add_argument("--output", type=Path, help="Output URDF path")
    parser.add_argument("--all", action="store_true", help="Generate every tool module into simulation/generated")
    parser.add_argument("--list-tools", action="store_true", help="List available tool modules")
    parser.add_argument(
        "--update-default",
        action="store_true",
        help="After generating the gripper variant, copy it to simulation/lattice.urdf for backwards compatibility.",
    )
    args = parser.parse_args()

    if args.list_tools:
        print_tools(available_tools())
        return 0

    if args.all:
        outputs = build_all()
        for path in outputs:
            print(path)
        if args.update_default:
            build_urdf("gripper", SCRIPT_DIR / "lattice.urdf")
        return 0

    output = build_urdf(args.tool, args.output)
    if args.update_default and args.tool == "gripper":
        build_urdf("gripper", SCRIPT_DIR / "lattice.urdf")
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
