"""Lattice Arm MuJoCo workbench.

A single-window, embedded MuJoCo viewer for inspecting the Lattice Arm with
interchangeable end-of-arm tools. The 3D scene is rendered offscreen with
``mujoco.Renderer`` and shown directly inside a dark Tkinter dashboard, so there
is exactly one window and no stray default MuJoCo panels. Controls: an
end-effector selector, forward-kinematics joint sliders, Cartesian jog buttons
driven by damped-least-squares IK, pose presets, a live tool0 readout, and a
screenshot button.

Headless modes (no display required) share the same render core:

    python mujoco_viewer.py                      # interactive workbench
    python mujoco_viewer.py --tool empty
    python mujoco_viewer.py --shot arm.png        # one offscreen frame
    python mujoco_viewer.py --contact-sheet cs.png  # multi-angle sheet
    python mujoco_viewer.py --direct --all-tools  # load + IK + asset self-check
    python mujoco_viewer.py --passive             # native MuJoCo window (fallback)
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from generate_tool_urdf import available_tools, build_urdf
from simlib import (
    Joint,
    UrdfModel,
    clamp_joint,
    forward_kinematics,
    joint_midpoint,
    load_urdf,
    ordered_active_joints,
    transform_point,
    wave_joint_positions,
)

ARM_JOINT_ORDER = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
DEFAULT_TOOL = "gripper"
Vec3 = Tuple[float, float, float]

# Dark "instrument workbench" palette (see DESIGN.md).
THEME = {
    "bg": "#0d1413",
    "panel": "#17211f",
    "panel_hi": "#1d2a27",
    "fg": "#e7efe9",
    "muted": "#91a09a",
    "accent": "#45c2ad",       # teal: verified geometry / interaction
    "amber": "#e0a526",        # CAD / workspace accent
    "danger": "#d9603b",       # deltas / errors
    "ok": "#67c98b",
    "button": "#1f302d",
    "button_hi": "#2b4b45",
    "trough": "#263531",
    "viewport_bg": "#05080a",
}


# --------------------------------------------------------------------------- #
# MuJoCo loading                                                              #
# --------------------------------------------------------------------------- #
def require_mujoco():
    """Import mujoco + numpy or raise a friendly error."""
    import importlib.util

    if importlib.util.find_spec("mujoco") is None:
        raise RuntimeError(
            "MuJoCo is not installed. Install the optional simulation dependencies with:\n"
            "  python -m pip install -r simulation/requirements-sim.txt"
        )
    import mujoco
    import numpy as np

    return mujoco, np


@dataclass
class Variant:
    """A tool-specific build of the arm: the generated URDF plus a simlib model."""

    tool: str
    urdf_path: Path
    kinematic_model: UrdfModel
    end_link: str
    active_joints: List[Joint]
    arm_joints: List[Joint]


def load_variant(tool_name: str) -> Variant:
    urdf_path = build_urdf(tool_name)
    model = load_urdf(urdf_path)
    end_link = "tool0" if "tool0" in model.link_names else model.leaf_links[-1]
    active = ordered_active_joints(model)
    arm = [j for j in ordered_active_joints(model, ARM_JOINT_ORDER) if j.name in ARM_JOINT_ORDER]
    return Variant(tool_name, urdf_path, model, end_link, active, arm)


def build_view_model(mujoco, urdf_path: Path, offw: int, offh: int, dressed: bool = True,
                     shadows: bool = False):
    """Compile the URDF into an MjModel for rendering, via the MjSpec API.

    Sets an offscreen framebuffer big enough for ``offw`` x ``offh`` and, when
    ``dressed``, gives the scene a studio look: a soft gradient sky, a 10 cm
    checkered grid floor (a Cartesian reference for jogging), and a directional
    key light with shadows.

    NOTE: URDF import defaults to ``compiler.discardvisual = True``, which drops
    visual geoms AND prunes every material/texture (so the floor + sky would
    silently vanish). We force it off; the arm keeps its per-link colours and the
    geom count is unchanged (visual/collision meshes are identical here).
    (The URDF ``<mujoco>`` passthrough also ignores ``<visual><global>``, which
    is why we edit the compiled spec instead of injecting XML.)
    """
    spec = mujoco.MjSpec.from_file(str(urdf_path))
    spec.compiler.discardvisual = False
    spec.visual.global_.offwidth = max(offw, 64)
    spec.visual.global_.offheight = max(offh, 64)
    spec.visual.quality.shadowsize = 4096

    if not dressed:
        spec.visual.headlight.ambient = [0.4, 0.4, 0.4]
        spec.visual.headlight.diffuse = [0.7, 0.7, 0.7]
        spec.visual.headlight.specular = [0.12, 0.12, 0.12]
        return spec.compile()

    spec.visual.headlight.ambient = [0.4, 0.41, 0.44]
    spec.visual.headlight.diffuse = [0.32, 0.32, 0.34]
    spec.visual.headlight.specular = [0.08, 0.08, 0.08]
    # World-axis triad size (X=red, Y=green, Z=blue); shown via the render option.
    spec.visual.scale.framelength = 0.55
    spec.visual.scale.framewidth = 0.035

    sky = spec.add_texture()
    sky.name = "skybox"
    sky.type = mujoco.mjtTexture.mjTEXTURE_SKYBOX
    sky.builtin = mujoco.mjtBuiltin.mjBUILTIN_GRADIENT
    sky.rgb1 = [0.80, 0.84, 0.90]   # zenith
    sky.rgb2 = [0.55, 0.61, 0.70]   # horizon
    sky.width = sky.height = 512

    grid = spec.add_texture()
    grid.name = "grid"
    grid.type = mujoco.mjtTexture.mjTEXTURE_2D
    grid.builtin = mujoco.mjtBuiltin.mjBUILTIN_CHECKER
    grid.rgb1 = [0.25, 0.29, 0.34]
    grid.rgb2 = [0.41, 0.46, 0.52]
    grid.markrgb = [0.52, 0.60, 0.68]
    grid.mark = mujoco.mjtMark.mjMARK_EDGE
    grid.width = grid.height = 512

    mat = spec.add_material()
    mat.name = "gridmat"
    mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "grid"
    mat.texrepeat = [13, 13]   # 1.3 m table / 13 ≈ 10 cm cells
    mat.texuniform = False
    mat.reflectance = 0.0      # a reflection pass roughly doubles render cost

    floor = spec.worldbody.add_geom()
    floor.name = "floor"
    floor.type = mujoco.mjtGeom.mjGEOM_PLANE
    floor.size = [0.65, 0.65, 0.05]
    floor.material = "gridmat"
    floor.pos = [0.12, 0.0, 0.0]

    light = spec.worldbody.add_light()
    light.type = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
    light.pos = [0.5, 0.4, 1.3]
    light.dir = [-0.35, -0.3, -1.0]
    light.diffuse = [0.55, 0.55, 0.57]
    light.specular = [0.2, 0.2, 0.2]
    light.castshadow = shadows   # shadows ~triple software-render cost; off for live

    return spec.compile()


def degenerate_meshes(mujoco, np, model, threshold: float = 1.0e-3) -> List[Tuple[str, float]]:
    """Return (name, max_extent_m) for meshes that collapse below ``threshold``.

    A visual mesh smaller than ~1 mm almost always means a unit/scale mistake in
    the URDF (the exact bug that once made the arm shells invisible). Surfacing
    it loudly stops that class of regression from being silent again.
    """
    bad: List[Tuple[str, float]] = []
    for i in range(model.nmesh):
        adr = int(model.mesh_vertadr[i])
        n = int(model.mesh_vertnum[i])
        if n <= 0:
            continue
        verts = np.asarray(model.mesh_vert[adr : adr + n])
        extent = float((verts.max(axis=0) - verts.min(axis=0)).max())
        if extent < threshold:
            bad.append((model.mesh(i).name, extent))
    return bad


# --------------------------------------------------------------------------- #
# Kinematics helpers + IK (position-only damped least squares)               #
# --------------------------------------------------------------------------- #
def current_end_position(model: UrdfModel, values: Dict[str, float], end_link: str) -> Vec3:
    link_poses, _ = forward_kinematics(model, values)
    return transform_point(link_poses[end_link])


def vec_sub(a: Vec3, b: Vec3) -> Vec3:
    return a[0] - b[0], a[1] - b[1], a[2] - b[2]


def vec_norm(v: Vec3) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def solve_3x3(a: List[List[float]], b: List[float]) -> List[float]:
    mat = [row[:] + [b[idx]] for idx, row in enumerate(a)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda row: abs(mat[row][col]))
        if abs(mat[pivot][col]) < 1e-12:
            return [0.0, 0.0, 0.0]
        mat[col], mat[pivot] = mat[pivot], mat[col]
        scale = mat[col][col]
        for jdx in range(col, 4):
            mat[col][jdx] /= scale
        for row in range(3):
            if row == col:
                continue
            factor = mat[row][col]
            for jdx in range(col, 4):
                mat[row][jdx] -= factor * mat[col][jdx]
    return [mat[row][3] for row in range(3)]


def numerical_jacobian(
    model: UrdfModel,
    joints: Sequence[Joint],
    values: Dict[str, float],
    end_link: str,
    eps: float = 1e-4,
) -> List[List[float]]:
    base = current_end_position(model, values, end_link)
    columns: List[Vec3] = []
    for joint in joints:
        perturbed = dict(values)
        perturbed[joint.name] = clamp_joint(joint, perturbed.get(joint.name, 0.0) + eps)
        moved = current_end_position(model, perturbed, end_link)
        columns.append(((moved[0] - base[0]) / eps, (moved[1] - base[1]) / eps, (moved[2] - base[2]) / eps))
    return [[columns[col][row] for col in range(len(columns))] for row in range(3)]


def ik_step(jacobian: List[List[float]], error: Vec3, damping: float) -> List[float]:
    joint_count = len(jacobian[0]) if jacobian else 0
    if joint_count == 0:
        return []
    jj_t = [[0.0] * 3 for _ in range(3)]
    for row in range(3):
        for col in range(3):
            jj_t[row][col] = sum(jacobian[row][joint] * jacobian[col][joint] for joint in range(joint_count))
        jj_t[row][row] += damping * damping
    y = solve_3x3(jj_t, [error[0], error[1], error[2]])
    return [sum(jacobian[row][joint] * y[row] for row in range(3)) for joint in range(joint_count)]


@dataclass
class IkResult:
    values: Dict[str, float]
    residual: float          # metres
    converged: bool
    iterations: int


def solve_ik(
    model: UrdfModel,
    joints: Sequence[Joint],
    start_values: Dict[str, float],
    end_link: str,
    target: Vec3,
    iterations: int = 160,
    tolerance: float = 0.0005,
    damping: float = 0.06,
    max_step: float = 0.12,
) -> IkResult:
    """Move ``end_link`` toward ``target`` and report the residual so callers can
    give the user real feedback instead of failing silently."""
    values = dict(start_values)
    residual = vec_norm(vec_sub(target, current_end_position(model, values, end_link)))
    used = 0
    for used in range(1, iterations + 1):
        current = current_end_position(model, values, end_link)
        error = vec_sub(target, current)
        residual = vec_norm(error)
        if residual <= tolerance:
            return IkResult(values, residual, True, used)
        jacobian = numerical_jacobian(model, joints, values, end_link)
        delta = ik_step(jacobian, error, damping)
        for joint, amount in zip(joints, delta):
            limited = max(-max_step, min(max_step, amount))
            values[joint.name] = clamp_joint(joint, values.get(joint.name, joint_midpoint(joint)) + limited)
    # Budget exhausted: report the residual of the pose we actually return, not
    # the one measured before this iteration's joint update.
    residual = vec_norm(vec_sub(target, current_end_position(model, values, end_link)))
    return IkResult(values, residual, residual <= tolerance, used)


# --------------------------------------------------------------------------- #
# Camera + render core                                                        #
# --------------------------------------------------------------------------- #
@dataclass
class OrbitCamera:
    azimuth: float = 120.0
    elevation: float = -18.0
    distance: float = 0.6
    lookat: List[float] = field(default_factory=lambda: [0.12, 0.0, 0.12])

    def to_mjv(self, mujoco):
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = self.lookat
        cam.distance = self.distance
        cam.azimuth = self.azimuth
        cam.elevation = self.elevation
        return cam

    def orbit(self, d_az: float, d_el: float) -> None:
        self.azimuth = (self.azimuth + d_az) % 360.0
        self.elevation = max(-89.0, min(89.0, self.elevation + d_el))

    def zoom(self, factor: float) -> None:
        self.distance = max(0.05, min(5.0, self.distance * factor))

    def pan(self, dx: float, dy: float) -> None:
        az = math.radians(self.azimuth)
        right = (-math.sin(az), math.cos(az), 0.0)
        scale = self.distance * 0.0018
        self.lookat[0] += (-dx * right[0]) * scale
        self.lookat[1] += (-dx * right[1]) * scale
        self.lookat[2] += dy * scale


def frame_camera(mujoco, np, model, data, fill: float = 1.6) -> OrbitCamera:
    mujoco.mj_forward(model, data)
    pts = np.asarray(data.geom_xpos)
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    center = ((lo + hi) / 2.0).tolist()
    diag = float(np.linalg.norm(hi - lo)) or 0.5
    return OrbitCamera(distance=diag * fill, lookat=center)


def joint_qpos_map(mujoco, model, joints: Sequence[Joint]) -> Dict[str, int]:
    out: Dict[str, int] = {}
    for joint in joints:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint.name)
        if jid >= 0:
            out[joint.name] = int(model.jnt_qposadr[jid])
    return out


def write_qpos(model, data, qadr: Dict[str, int], values: Dict[str, float]) -> None:
    for name, value in values.items():
        adr = qadr.get(name)
        if adr is not None:
            data.qpos[adr] = value


class Scene:
    """A compiled MuJoCo model + data + offscreen renderer for one variant.

    Constructing a Scene creates an OpenGL context (the Renderer), so it is only
    used by paths that actually draw pixels (the GUI, --shot, --contact-sheet).
    The headless --direct check stays GL-free.
    """

    def __init__(self, mujoco, np, variant: Variant, width: int, height: int,
                 shadows: bool = False, framebuffer: Optional[Tuple[int, int]] = None):
        self.mujoco = mujoco
        self.np = np
        self.variant = variant
        # The offscreen framebuffer is fixed at compile; the renderer may be any
        # size up to it. We compile a generous framebuffer so the renderer can be
        # resized to match the viewport later without recompiling the model.
        self.fb_w, self.fb_h = framebuffer if framebuffer else (width, height)
        self.model = build_view_model(mujoco, variant.urdf_path, self.fb_w, self.fb_h, shadows=shadows)
        self.data = mujoco.MjData(self.model)
        self.r_w, self.r_h = min(width, self.fb_w), min(height, self.fb_h)
        self.renderer = mujoco.Renderer(self.model, self.r_h, self.r_w)
        self._qadr = joint_qpos_map(mujoco, self.model, variant.active_joints)
        # Draw the world coordinate triad (X=red, Y=green, Z=blue) every frame.
        self._opt = mujoco.MjvOption()
        self._opt.frame = mujoco.mjtFrame.mjFRAME_WORLD

    def apply(self, values: Dict[str, float]) -> None:
        write_qpos(self.model, self.data, self._qadr, values)
        self.mujoco.mj_forward(self.model, self.data)

    def resize(self, width: int, height: int) -> None:
        """Re-make the renderer at a new size (≤ framebuffer) so frames come out
        at the viewport's exact pixels — no per-frame rescale needed. Closes the
        old renderer first (close-before-create keeps the GL context valid)."""
        width, height = max(64, min(width, self.fb_w)), max(64, min(height, self.fb_h))
        if (width, height) == (self.r_w, self.r_h):
            return
        try:
            self.renderer.close()
        except Exception:
            pass
        self.r_w, self.r_h = width, height
        self.renderer = self.mujoco.Renderer(self.model, self.r_h, self.r_w)

    @property
    def size(self) -> Tuple[int, int]:
        return self.r_w, self.r_h

    def render(self, camera: OrbitCamera):
        self.renderer.update_scene(self.data, camera.to_mjv(self.mujoco), scene_option=self._opt)
        return self.renderer.render()

    def warnings(self) -> List[Tuple[str, float]]:
        return degenerate_meshes(self.mujoco, self.np, self.model)

    def close(self) -> None:
        try:
            self.renderer.close()
        except Exception:
            pass


def save_png(img, path: Path) -> None:
    from PIL import Image

    path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(img).save(path)


def zero_pose(variant: Variant) -> Dict[str, float]:
    return {j.name: clamp_joint(j, 0.0) for j in variant.active_joints}


# --------------------------------------------------------------------------- #
# Headless commands                                                           #
# --------------------------------------------------------------------------- #
def cmd_direct(mujoco, np, tools: Sequence[str]) -> int:
    """Headless self-check: compile each variant, run a jog-IK probe, and flag
    any degenerate (collapsed-scale) meshes. Needs no display or GL context."""
    status = 0
    for tool_name in tools:
        variant = load_variant(tool_name)
        model = mujoco.MjModel.from_xml_path(str(variant.urdf_path))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        start = zero_pose(variant)
        current = current_end_position(variant.kinematic_model, start, variant.end_link)
        target = (current[0] + 0.02, current[1], current[2])
        result = solve_ik(variant.kinematic_model, variant.arm_joints, start, variant.end_link, target)
        bad = degenerate_meshes(mujoco, np, model)
        flag = "OK" if result.converged and not bad else "WARN"
        if not result.converged or bad:
            status = 1
        print(
            f"[{flag}] {tool_name}: {variant.urdf_path.name}  "
            f"{model.njnt} joints, {model.ngeom} geoms, "
            f"IK residual {result.residual * 1000:.2f} mm in {result.iterations} it"
        )
        for name, extent in bad:
            print(f"        ! degenerate mesh '{name}' extent {extent * 1000:.3f} mm (scale/units bug?)")
    return status


def cmd_shot(mujoco, np, tool: str, out: Path, res: Tuple[int, int],
             az: Optional[float], el: Optional[float]) -> int:
    variant = load_variant(tool)
    scene = Scene(mujoco, np, variant, res[0], res[1], shadows=True)
    try:
        scene.apply(zero_pose(variant))
        camera = frame_camera(mujoco, np, scene.model, scene.data)
        if az is not None:
            camera.azimuth = az
        if el is not None:
            camera.elevation = el
        img = scene.render(camera)
    finally:
        scene.close()
    save_png(img, out)
    print(f"wrote {out}  ({img.shape[1]}x{img.shape[0]}, tool={tool})")
    return 0


def cmd_contact_sheet(mujoco, np, tool: str, out: Path, res: Tuple[int, int]) -> int:
    from PIL import Image

    variant = load_variant(tool)
    values = zero_pose(variant)
    tile_w, tile_h = res
    angles = [(120, -18), (210, -18), (300, -18), (75, -55)]
    scene = Scene(mujoco, np, variant, tile_w, tile_h, shadows=True)
    try:
        scene.apply(values)
        base = frame_camera(mujoco, np, scene.model, scene.data)
        tiles = []
        for az, el in angles:
            base.azimuth, base.elevation = az, el
            tiles.append(np.asarray(scene.render(base)).copy())
    finally:
        scene.close()
    cols, rows = 2, 2
    sheet = Image.new("RGB", (cols * tile_w, rows * tile_h), THEME["viewport_bg"])
    for idx, tile in enumerate(tiles):
        sheet.paste(Image.fromarray(tile), ((idx % cols) * tile_w, (idx // cols) * tile_h))
    out.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(out)
    print(f"wrote {out}  (contact sheet {cols * tile_w}x{rows * tile_h}, tool={tool})")
    return 0


def run_passive(mujoco, np, tool: str) -> int:
    """Native MuJoCo window playing a slow wave demo. Fallback that needs no
    Pillow/ImageTk; also handy for users who prefer MuJoCo's own navigation."""
    import mujoco.viewer

    variant = load_variant(tool)
    model = build_view_model(mujoco, variant.urdf_path, 128, 128)
    data = mujoco.MjData(model)
    qadr = joint_qpos_map(mujoco, model, variant.active_joints)
    mujoco.mj_forward(model, data)
    camera = frame_camera(mujoco, np, model, data)
    with mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False) as viewer:
        with viewer.lock():
            viewer.opt.frame = mujoco.mjtFrame.mjFRAME_WORLD
            viewer.cam.lookat[:] = camera.lookat
            viewer.cam.distance = camera.distance
            viewer.cam.azimuth = camera.azimuth
            viewer.cam.elevation = camera.elevation
        start = time.time()
        while viewer.is_running():
            values = wave_joint_positions(variant.active_joints, time.time() - start, 7.0)
            write_qpos(model, data, qadr, values)
            mujoco.mj_forward(model, data)
            viewer.sync()
            time.sleep(1 / 60)
    return 0


# --------------------------------------------------------------------------- #
# Embedded single-window workbench                                            #
# --------------------------------------------------------------------------- #
class JointSlider:
    """A flat, themed joint slider drawn on a Canvas: a name + degree readout, a
    track with limit ticks, a teal fill from the zero anchor, and a draggable
    thumb. Values are stored/reported in radians. Nicer and fully theme-able
    compared with tk.Scale, whose colours/shape are largely fixed by the OS."""

    H = 60

    def __init__(self, parent, tk, name, lower, upper, value, on_change, width=520):
        self.tk = tk
        self.name = name
        self.lower = lower if lower is not None else -math.pi
        self.upper = upper if upper is not None else math.pi
        if self.upper <= self.lower:
            self.upper = self.lower + 1e-3
        self.value = value
        self.on_change = on_change
        self.canvas = tk.Canvas(parent, height=self.H, width=width, bg=THEME["panel"],
                                highlightthickness=0, bd=0)
        self.canvas.bind("<Configure>", lambda _e: self._redraw())
        self.canvas.bind("<Button-1>", self._on_mouse)
        self.canvas.bind("<B1-Motion>", self._on_mouse)

    def pack(self, **kw):
        self.canvas.pack(**kw)

    def _bounds(self):
        w = self.canvas.winfo_width() or int(self.canvas["width"])
        return 18, w - 18

    def _x_for(self, value):
        x0, x1 = self._bounds()
        t = (value - self.lower) / (self.upper - self.lower)
        return x0 + max(0.0, min(1.0, t)) * (x1 - x0)

    def _value_for(self, x):
        x0, x1 = self._bounds()
        t = max(0.0, min(1.0, (x - x0) / max(1, x1 - x0)))
        return self.lower + t * (self.upper - self.lower)

    def _on_mouse(self, event):
        self.value = self._value_for(event.x)
        self._redraw()
        self.on_change(self.name, self.value)

    def set_value(self, value):  # programmatic update; no callback
        self.value = max(self.lower, min(self.upper, value))
        self._redraw()

    def _redraw(self):
        c = self.canvas
        c.delete("all")
        x0, x1 = self._bounds()
        ymid = self.H - 19
        c.create_text(x0, 16, anchor="w", text=self.name, fill=THEME["fg"], font=("Segoe UI", 11))
        c.create_text(x1, 16, anchor="e", text=f"{math.degrees(self.value):+6.1f}°",
                      fill=THEME["accent"], font=("Consolas", 12, "bold"))
        c.create_line(x0, ymid, x1, ymid, fill=THEME["trough"], width=6, capstyle="round")
        for limit in (self.lower, self.upper):
            xv = self._x_for(limit)
            c.create_line(xv, ymid - 7, xv, ymid + 7, fill=THEME["panel_hi"], width=2)
        anchor = min(max(0.0, self.lower), self.upper)
        ax, vx = self._x_for(anchor), self._x_for(self.value)
        c.create_line(ax, ymid, vx, ymid, fill=THEME["accent"], width=6, capstyle="round")
        r = 9
        c.create_oval(vx - r, ymid - r, vx + r, ymid + r, fill=THEME["accent"], outline=THEME["fg"], width=2)


class LatticeWorkbench:
    DOCK_W = 580
    JOG_AXES = (("X", 0), ("Y", 1), ("Z", 2))
    AXIS_COLORS = {"X": "#ff5a4d", "Y": "#46d65f", "Z": "#5b8dff"}  # match MuJoCo's RGB triad

    def __init__(self, mujoco, np, initial_tool: str, jog_step: float):
        import tkinter as tk
        from tkinter import ttk
        from PIL import Image, ImageTk

        self.mujoco, self.np = mujoco, np
        self.tk, self.ttk = tk, ttk
        self._Image, self._ImageTk = Image, ImageTk

        self.root = tk.Tk()
        self.root.title("Lattice viewer")
        self.root.configure(bg=THEME["bg"])
        self.root.minsize(1100, 680)

        # Render at roughly the maximized viewport size, then scale the frame to
        # the live viewport (so window resizing never recreates a GL context).
        sw, sh = self.root.winfo_screenwidth(), self.root.winfo_screenheight()
        # Generous offscreen framebuffer so the renderer can be resized to the
        # exact viewport size (avoids a per-frame image rescale) without recompile.
        self._fb_w, self._fb_h = min(sw, 2560), min(sh, 1600)
        self._view_w = max(640, min(sw - self.DOCK_W - 80, self._fb_w))
        self._view_h = max(480, min(sh - 170, self._fb_h))

        self.variant: Optional[Variant] = None
        self.scene: Optional[Scene] = None
        self.camera = OrbitCamera()
        self.joint_values: Dict[str, float] = {}
        self.sliders: Dict[str, JointSlider] = {}

        self.tool_var = tk.StringVar(value=initial_tool)
        self.step_var = tk.DoubleVar(value=jog_step)
        self.status_var = tk.StringVar(value="Loading…")
        self.meta_var = tk.StringVar(value="")
        self.tcp_var = tk.StringVar(value="tool0  —")

        self._photo = None
        self._dirty = True
        self._suppress = False
        self._animating = False
        self._anim_t0 = 0.0
        self._drag: Optional[Tuple[int, int, str]] = None
        self._fullscreen = False
        self._closing = False
        self._tick_after: Optional[str] = None
        self._resize_after: Optional[str] = None   # debounce for viewport resize

        self._configure_style()
        self._build_layout()
        self.load_tool(initial_tool)

        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.bind("<F11>", self._toggle_fullscreen)
        self.root.bind("<Escape>", self._exit_fullscreen)
        try:
            self.root.state("zoomed")   # start maximized (Windows/most WMs)
        except Exception:
            pass
        self._tick_after = self.root.after(33, self._tick)
        self.root.after(80, lambda: (self.root.lift(), self.root.focus_force()))

    def _toggle_fullscreen(self, *_):
        self._fullscreen = not self._fullscreen
        self.root.attributes("-fullscreen", self._fullscreen)

    def _exit_fullscreen(self, *_):
        self._fullscreen = False
        self.root.attributes("-fullscreen", False)

    # -- styling -----------------------------------------------------------
    def _configure_style(self) -> None:
        style = self.ttk.Style()
        style.theme_use("clam")
        style.configure(".", background=THEME["bg"], foreground=THEME["fg"], fieldbackground=THEME["panel"])
        style.configure("TFrame", background=THEME["bg"])
        style.configure("Panel.TFrame", background=THEME["panel"])
        style.configure("TLabel", background=THEME["bg"], foreground=THEME["fg"], font=("Segoe UI", 11))
        style.configure("Panel.TLabel", background=THEME["panel"], foreground=THEME["fg"], font=("Segoe UI", 11))
        style.configure("Muted.TLabel", background=THEME["panel"], foreground=THEME["muted"], font=("Segoe UI", 10))
        style.configure("Head.TLabel", background=THEME["bg"], foreground=THEME["fg"], font=("Segoe UI", 19, "bold"))
        style.configure("Sec.TLabel", background=THEME["panel"], foreground=THEME["accent"],
                        font=("Segoe UI", 12, "bold"))
        style.configure("TButton", background=THEME["button"], foreground=THEME["fg"], borderwidth=0,
                        padding=(10, 10), font=("Segoe UI", 11))
        style.map("TButton", background=[("active", THEME["button_hi"])])
        style.configure("Accent.TButton", background=THEME["button_hi"], foreground=THEME["fg"],
                        padding=(10, 10), font=("Segoe UI", 11, "bold"))
        style.map("Accent.TButton", background=[("active", THEME["accent"])])
        style.configure("Jog.TButton", background=THEME["button"], foreground=THEME["fg"], borderwidth=0,
                        padding=(10, 8), font=("Consolas", 15, "bold"))
        style.map("Jog.TButton", background=[("active", THEME["button_hi"])])
        style.configure("TCombobox", fieldbackground=THEME["panel_hi"], background=THEME["button"],
                        foreground=THEME["fg"], arrowcolor=THEME["fg"], bordercolor=THEME["panel_hi"],
                        lightcolor=THEME["panel_hi"], darkcolor=THEME["panel_hi"], padding=6)
        style.map("TCombobox",
                  fieldbackground=[("readonly", THEME["panel_hi"])],
                  foreground=[("readonly", THEME["fg"])],
                  selectbackground=[("readonly", THEME["panel_hi"])],
                  selectforeground=[("readonly", THEME["fg"])],
                  background=[("active", THEME["button_hi"])],
                  arrowcolor=[("readonly", THEME["fg"])])
        for _opt, _val in (("*TCombobox*Listbox.font", ("Segoe UI", 11)),
                           ("*TCombobox*Listbox.background", THEME["panel"]),
                           ("*TCombobox*Listbox.foreground", THEME["fg"]),
                           ("*TCombobox*Listbox.selectBackground", THEME["button_hi"]),
                           ("*TCombobox*Listbox.selectForeground", THEME["fg"])):
            self.root.option_add(_opt, _val)

    # -- layout ------------------------------------------------------------
    def _build_layout(self) -> None:
        tk, ttk = self.tk, self.ttk
        outer = ttk.Frame(self.root, padding=12)
        outer.pack(fill="both", expand=True)

        header = ttk.Frame(outer)
        header.pack(fill="x", pady=(0, 10))
        ttk.Label(header, text="Lattice viewer", style="Head.TLabel").pack(side="left")

        body = ttk.Frame(outer)
        body.pack(fill="both", expand=True)
        body.columnconfigure(0, weight=1)   # viewport grows
        body.columnconfigure(1, weight=0)   # dock fixed
        body.rowconfigure(0, weight=1)

        # Viewport (left, fills available space)
        view_wrap = tk.Frame(body, bg=THEME["viewport_bg"], highlightthickness=1,
                             highlightbackground=THEME["panel_hi"])
        view_wrap.grid(row=0, column=0, sticky="nsew")
        self._photo = self._ImageTk.PhotoImage(
            self._Image.new("RGB", (self._view_w, self._view_h), THEME["viewport_bg"])
        )
        self.view = tk.Label(view_wrap, bg=THEME["viewport_bg"], image=self._photo, bd=0, cursor="fleur")
        self.view.pack(fill="both", expand=True)
        self.view.bind("<Configure>", self._on_view_configure)
        self.view.bind("<ButtonPress-1>", lambda e: self._begin_drag(e, "orbit"))
        self.view.bind("<ButtonPress-3>", lambda e: self._begin_drag(e, "pan"))
        self.view.bind("<Shift-ButtonPress-1>", lambda e: self._begin_drag(e, "pan"))
        self.view.bind("<B1-Motion>", self._on_drag)
        self.view.bind("<B3-Motion>", self._on_drag)
        self.view.bind("<ButtonRelease-1>", self._end_drag)
        self.view.bind("<ButtonRelease-3>", self._end_drag)
        self.view.bind("<MouseWheel>", self._on_wheel)

        # Control dock (right, fixed width)
        dock = ttk.Frame(body, style="Panel.TFrame", padding=12, width=self.DOCK_W)
        dock.grid(row=0, column=1, sticky="ns", padx=(12, 0))
        dock.grid_propagate(False)
        self._build_tool_row(dock)
        self._build_jog(dock)
        self._build_presets(dock)
        ttk.Label(dock, textvariable=self.tcp_var, style="Panel.TLabel",
                  font=("Consolas", 14, "bold"), foreground=THEME["accent"]).pack(anchor="w", pady=(16, 6))
        ttk.Label(dock, text="FORWARD KINEMATICS", style="Sec.TLabel").pack(anchor="w", pady=(8, 4))
        self.slider_frame = ttk.Frame(dock, style="Panel.TFrame")
        self.slider_frame.pack(fill="both", expand=True)

    def _build_tool_row(self, dock) -> None:
        ttk = self.ttk
        ttk.Label(dock, text="END EFFECTOR", style="Sec.TLabel").pack(anchor="w")
        row = ttk.Frame(dock, style="Panel.TFrame")
        row.pack(fill="x", pady=(4, 14))
        combo = ttk.Combobox(row, textvariable=self.tool_var, values=available_tools(), state="readonly",
                             width=22, font=("Segoe UI", 12))
        combo.pack(side="left", ipady=3)
        combo.bind("<<ComboboxSelected>>", lambda _e: self.load_tool(self.tool_var.get()))

    def _build_jog(self, dock) -> None:
        tk, ttk = self.tk, self.ttk
        ttk.Label(dock, text="CARTESIAN JOG (tool0)", style="Sec.TLabel").pack(anchor="w", pady=(6, 0))
        ttk.Label(dock, text="coloured to match the 3D axes", style="Muted.TLabel").pack(anchor="w", pady=(0, 6))
        jog = ttk.Frame(dock, style="Panel.TFrame")
        jog.pack(fill="x", pady=(0, 10))
        for r, (name, axis) in enumerate(self.JOG_AXES):
            tk.Label(jog, text=name, bg=THEME["panel"], fg=self.AXIS_COLORS[name],
                     font=("Consolas", 17, "bold"), width=2).grid(row=r, column=0, padx=(0, 12), pady=3)
            ttk.Button(jog, text="–", width=4, style="Jog.TButton",
                       command=lambda a=axis: self.jog(a, -1)).grid(row=r, column=1, padx=3, sticky="ew")
            ttk.Button(jog, text="+", width=4, style="Jog.TButton",
                       command=lambda a=axis: self.jog(a, +1)).grid(row=r, column=2, padx=3, sticky="ew")
        step_row = ttk.Frame(jog, style="Panel.TFrame")
        step_row.grid(row=0, column=3, rowspan=3, padx=(20, 0))
        ttk.Label(step_row, text="step (m)", style="Muted.TLabel").pack(anchor="w")
        tk.Spinbox(step_row, from_=0.001, to=0.05, increment=0.001, textvariable=self.step_var, width=8,
                   font=("Consolas", 13), bg=THEME["panel_hi"], fg=THEME["fg"], insertbackground=THEME["fg"],
                   buttonbackground=THEME["button"], relief="flat").pack(anchor="w", pady=4)

    def _build_presets(self, dock) -> None:
        ttk = self.ttk
        ttk.Label(dock, text="POSE", style="Sec.TLabel").pack(anchor="w", pady=(6, 3))
        row = ttk.Frame(dock, style="Panel.TFrame")
        row.pack(fill="x", pady=(0, 5))
        ttk.Button(row, text="Zero", command=self.preset_zero).pack(side="left", expand=True, fill="x", padx=3)
        self.wave_btn = ttk.Button(row, text="Wave ▶", style="Accent.TButton", command=self.toggle_wave)
        self.wave_btn.pack(side="left", expand=True, fill="x", padx=3)
        row2 = ttk.Frame(dock, style="Panel.TFrame")
        row2.pack(fill="x")
        ttk.Button(row2, text="Reset view", command=self.reset_view).pack(side="left", expand=True, fill="x", padx=3)
        ttk.Button(row2, text="Screenshot", command=self.screenshot).pack(side="left", expand=True, fill="x", padx=3)

    # -- tool loading ------------------------------------------------------
    def load_tool(self, tool_name: str) -> None:
        self._stop_wave()
        prev_tool = self.variant.tool if self.variant is not None else None
        reframe = prev_tool is None   # only auto-frame the camera on first load
        # Close the OLD scene before building the new one. Keeping two live GL
        # contexts and then freeing one blanks every subsequent render on
        # Windows, which is what broke tool switching previously.
        if self.scene is not None:
            self.scene.close()
            self.scene = None
        try:
            self._activate(tool_name, reframe)
        except Exception as exc:  # noqa: BLE001 - surfaced to the status bar
            recovered = False
            if prev_tool and prev_tool != tool_name:
                try:
                    self._activate(prev_tool, reframe=False)
                    self.tool_var.set(prev_tool)
                    recovered = True
                except Exception:
                    pass
            note = " (kept previous tool)" if recovered else ""
            self._set_status(f"could not load tool '{tool_name}': {exc}{note}", THEME["danger"])

    def _activate(self, tool_name: str, reframe: bool) -> None:
        variant = load_variant(tool_name)
        scene = Scene(self.mujoco, self.np, variant, self._view_w, self._view_h,
                      framebuffer=(self._fb_w, self._fb_h))
        previous = self.joint_values
        self.variant, self.scene = variant, scene
        # Carry the pose across tool changes so the arm does not jump; joints that
        # only exist on the new tool (e.g. the gripper) start at zero.
        self.joint_values = {j.name: previous.get(j.name, clamp_joint(j, 0.0)) for j in variant.active_joints}
        scene.apply(self.joint_values)
        if reframe or self.camera is None:
            self.camera = frame_camera(self.mujoco, self.np, scene.model, scene.data)
        self._build_sliders()
        self._refresh_readouts()
        self._dirty = True
        self.meta_var.set(f"{len(variant.active_joints)} joints · {variant.urdf_path.name}")
        bad = scene.warnings()
        if bad:
            names = ", ".join(n for n, _ in bad)
            self._set_status(f"{tool_name} loaded — WARNING: degenerate meshes ({names})", THEME["danger"])
        else:
            self._set_status(f"{tool_name} loaded — drag to orbit · wheel to zoom · right-drag to pan · F11 fullscreen",
                             THEME["ok"])

    def _build_sliders(self) -> None:
        for child in self.slider_frame.winfo_children():
            child.destroy()
        self.sliders.clear()
        assert self.variant is not None
        for joint in self.variant.active_joints:
            slider = JointSlider(self.slider_frame, self.tk, joint.name, joint.lower, joint.upper,
                                 self.joint_values[joint.name], self._slider_changed)
            slider.pack(fill="x", pady=1)
            self.sliders[joint.name] = slider

    # -- interaction -------------------------------------------------------
    def _slider_changed(self, name: str, value_rad: float) -> None:
        if self._suppress:
            return
        self._stop_wave()
        self.joint_values[name] = value_rad
        self._apply_and_refresh()

    def _safe_step(self) -> float:
        # The Spinbox is freely editable; an empty/partial value makes
        # DoubleVar.get() raise TclError. Fall back to the default step.
        try:
            step = float(self.step_var.get())
        except Exception:
            step = 0.005
        return step if step > 0 else 0.005

    def jog(self, axis: int, direction: int) -> None:
        if self.variant is None:
            return
        self._stop_wave()
        step = self._safe_step()
        current = current_end_position(self.variant.kinematic_model, self.joint_values, self.variant.end_link)
        target = list(current)
        target[axis] += direction * step
        result = solve_ik(self.variant.kinematic_model, self.variant.arm_joints, self.joint_values,
                          self.variant.end_link, tuple(target))
        self.joint_values.update(result.values)
        self._sync_sliders()
        self._apply_and_refresh()
        axis_name = "XYZ"[axis]
        if result.converged:
            self._set_status(f"jog {axis_name}{'+' if direction > 0 else '−'} {step * 1000:.0f} mm  "
                             f"→ residual {result.residual * 1000:.2f} mm", THEME["ok"])
        else:
            self._set_status(f"jog {axis_name}: target out of reach — residual {result.residual * 1000:.1f} mm",
                             THEME["danger"])

    def preset_zero(self) -> None:
        self._stop_wave()
        self.joint_values = zero_pose(self.variant)
        self._sync_sliders()
        self._apply_and_refresh()
        self._set_status("pose: zero", THEME["muted"])

    def toggle_wave(self) -> None:
        if self._animating:
            self._stop_wave()
        else:
            self._animating = True
            self._anim_t0 = time.time()
            self.wave_btn.configure(text="Stop ◼")
            self._set_status("wave demo running…", THEME["accent"])

    def _stop_wave(self) -> None:
        if self._animating:
            self._animating = False
            self.wave_btn.configure(text="Wave ▶")

    def reset_view(self) -> None:
        if self.scene is not None:
            self.camera = frame_camera(self.mujoco, self.np, self.scene.model, self.scene.data)
            self._dirty = True

    def screenshot(self) -> None:
        if self.variant is None or self.scene is None:
            return
        # Grab from the live scene's renderer; spinning up a second GL context
        # while the viewport's context is current can blank the embedded view.
        out = SCRIPT_DIR / "screenshots" / f"lattice_{self.variant.tool}_{int(time.time())}.png"
        save_png(self.scene.render(self.camera), out)
        self._set_status(f"saved {out.relative_to(SCRIPT_DIR)}", THEME["accent"])

    def _begin_drag(self, event, mode: str) -> None:
        self._drag = (event.x, event.y, mode)

    def _on_drag(self, event) -> None:
        if self._drag is None:
            return
        x0, y0, mode = self._drag
        dx, dy = event.x - x0, event.y - y0
        if mode == "pan":
            self.camera.pan(dx, dy)
        else:
            self.camera.orbit(-dx * 0.4, dy * 0.4)
        self._drag = (event.x, event.y, mode)
        self._dirty = True

    def _end_drag(self, _event) -> None:
        self._drag = None

    def _on_wheel(self, event) -> None:
        self.camera.zoom(0.9 if event.delta > 0 else 1.0 / 0.9)
        self._dirty = True

    # -- state plumbing ----------------------------------------------------
    def _sync_sliders(self) -> None:
        for name, slider in self.sliders.items():
            slider.set_value(self.joint_values[name])

    def _apply_and_refresh(self) -> None:
        if self.scene is not None:
            self.scene.apply(self.joint_values)
        self._refresh_readouts()
        self._dirty = True

    def _refresh_readouts(self) -> None:
        if self.variant is None:
            return
        pos = current_end_position(self.variant.kinematic_model, self.joint_values, self.variant.end_link)
        self.tcp_var.set(f"tool0   X {pos[0]:+.3f}   Y {pos[1]:+.3f}   Z {pos[2]:+.3f}  m")

    def _set_status(self, text: str, color: str) -> None:
        return  # status bar removed from the UI; no-op keeps call sites simple

    def _on_view_configure(self, event) -> None:
        if event.width > 1 and event.height > 1 and (event.width != self._view_w or event.height != self._view_h):
            self._view_w, self._view_h = event.width, event.height
            self._dirty = True   # crop the current frame immediately…
            # …and re-make the renderer at the new size once resizing settles.
            if self._resize_after is not None:
                try:
                    self.root.after_cancel(self._resize_after)
                except Exception:
                    pass
            self._resize_after = self.root.after(140, self._apply_view_resize)

    def _apply_view_resize(self) -> None:
        self._resize_after = None
        if self.scene is not None and not self._closing:
            self.scene.resize(self._view_w, self._view_h)
            self._dirty = True

    def _fit_cover(self, img, tw: int, th: int, resample=None):
        """Scale a rendered frame to exactly (tw, th), cropping overflow so the
        viewport is filled at any window size without distorting the arm."""
        tw, th = max(1, tw), max(1, th)
        iw, ih = img.size
        scale = max(tw / iw, th / ih)
        nw, nh = max(tw, int(iw * scale + 0.5)), max(th, int(ih * scale + 0.5))
        img = img.resize((nw, nh), resample if resample is not None else self._Image.BILINEAR)
        left, top = (nw - tw) // 2, (nh - th) // 2
        return img.crop((left, top, left + tw, top + th))

    # -- render loop -------------------------------------------------------
    def _tick(self) -> None:
        if self._closing:
            return
        if self._animating and self.variant is not None:
            self._suppress = True
            try:
                self.joint_values = wave_joint_positions(self.variant.active_joints,
                                                         time.time() - self._anim_t0, 7.0)
                for name, slider in self.sliders.items():
                    slider.set_value(self.joint_values[name])
            finally:
                self._suppress = False
            self._apply_and_refresh()
        if self._dirty and self.scene is not None:
            frame = self._Image.fromarray(self.scene.render(self.camera))
            # Usually rendered at the exact viewport size (no rescale); only crop
            # during the brief window before a resize settles.
            if frame.size != (self._view_w, self._view_h):
                frame = self._fit_cover(frame, self._view_w, self._view_h)
            self._photo = self._ImageTk.PhotoImage(frame)
            self.view.configure(image=self._photo)
            self._dirty = False
        self._tick_after = self.root.after(33, self._tick)

    def close(self) -> None:
        if self._closing:
            return
        self._closing = True
        for after_id in (self._tick_after, self._resize_after):
            if after_id is not None:
                try:
                    self.root.after_cancel(after_id)
                except Exception:
                    pass
        self._tick_after = self._resize_after = None
        if self.scene is not None:
            self.scene.close()
            self.scene = None
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


# --------------------------------------------------------------------------- #
# Entry point                                                                 #
# --------------------------------------------------------------------------- #
def parse_res(text: str) -> Tuple[int, int]:
    parts = text.lower().split("x")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("resolution must look like 1280x960")
    try:
        width, height = int(parts[0]), int(parts[1])
    except ValueError:
        raise argparse.ArgumentTypeError("resolution must look like 1280x960")
    if width < 64 or height < 64:
        raise argparse.ArgumentTypeError("resolution must be at least 64x64")
    return width, height


def main() -> int:
    parser = argparse.ArgumentParser(description="Embedded MuJoCo workbench for Lattice Arm tool variants.")
    parser.add_argument("--tool", default=DEFAULT_TOOL, choices=available_tools(), help="Initial tool module")
    parser.add_argument("--jog-step", type=float, default=0.005, help="IK jog step size in metres")
    parser.add_argument("--direct", action="store_true", help="Headless load + IK + asset self-check (no GUI)")
    parser.add_argument("--all-tools", action="store_true", help="In --direct mode, check every tool")
    parser.add_argument("--shot", type=Path, help="Render one offscreen PNG and exit")
    parser.add_argument("--contact-sheet", type=Path, help="Render a multi-angle PNG sheet and exit")
    parser.add_argument("--res", type=parse_res, default=(1280, 960),
                        help="Render resolution WxH for --shot/--contact-sheet")
    parser.add_argument("--az", type=float, help="Camera azimuth for --shot")
    parser.add_argument("--el", type=float, help="Camera elevation for --shot")
    parser.add_argument("--passive", action="store_true", help="Use the native MuJoCo window (no embedded GUI)")
    args = parser.parse_args()

    modes = [name for name, on in (("--direct", args.direct), ("--shot", bool(args.shot)),
                                   ("--contact-sheet", bool(args.contact_sheet)), ("--passive", args.passive)) if on]
    if len(modes) > 1:
        parser.error(f"choose only one mode at a time (got {', '.join(modes)})")
    if args.all_tools and not args.direct:
        parser.error("--all-tools only applies with --direct")

    try:
        mujoco, np = require_mujoco()
        if args.direct:
            tools = available_tools() if args.all_tools else [args.tool]
            return cmd_direct(mujoco, np, tools)
        if args.shot:
            return cmd_shot(mujoco, np, args.tool, args.shot, args.res, args.az, args.el)
        if args.contact_sheet:
            return cmd_contact_sheet(mujoco, np, args.tool, args.contact_sheet, args.res)
        if args.passive:
            return run_passive(mujoco, np, args.tool)
        try:
            from PIL import ImageTk  # noqa: F401
        except Exception:
            print("Pillow's ImageTk is unavailable; falling back to the native MuJoCo window (--passive).",
                  file=sys.stderr)
            return run_passive(mujoco, np, args.tool)
        LatticeWorkbench(mujoco, np, args.tool, args.jog_step).run()
        return 0
    except (RuntimeError, ValueError, OSError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
