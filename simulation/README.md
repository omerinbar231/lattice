# Lattice Simulation

This folder contains a lightweight URDF and MuJoCo workflow for inspecting the Lattice Arm with different tool states.

## Files

- `lattice_base.urdf` - canonical tool-less Lattice Arm base ending at `lattice_tool_mount_link` (carries the `ConnectorPartA` coupling cylinder; the tool servo and the rotating `ConnectorPartAcap` coupling cap live in the tool modules so they vary per tool).
- `tools/empty.xml` - no end-effector tool, but exposes the tool-changer coupling: a continuous `tool_coupling` joint rolls the `ConnectorPartAcap` cap inside the cylinder, with `tool0` riding on it.
- `tools/gripper.xml` - default gripper module: drive servo, ConnectorPartB, the gripper body, a moving jaw (`gripper` joint), and the (locked) coupling cap.
- `tools/screw_gripper.xml` - alternate screw-driven gripper module: drive servo, ConnectorPartB, motor casing, a rotating screw (`screw_gripper_spin` continuous joint), and the (locked) coupling cap.
- `generate_tool_urdf.py` - builds attached-tool URDF variants from the base and a tool module.
- `lattice.urdf` - default generated gripper variant for backwards compatibility.
- `generated/` - generated `empty`, `gripper`, and `screw_gripper` variants.
- `mujoco_viewer.py` - MuJoCo workbench with tool selection, FK sliders, and jog-style IK controls.
- `validate_urdf.py` - dependency-free URDF smoke test.

The SO-101 mesh assets live in `assets/so101/`. Lattice printed parts are referenced from the hardware folder, for example:

- `../hardware/Shoulder/Shoulder.stl`
- `../hardware/Shoulder Connector/ConnectorPartA.stl`
- `../hardware/End Effector Connector/ConnectorPartB.stl`
- `../hardware/End Effectors/Gripper/GripperPartA.stl`
- `../hardware/End Effectors/Screw Gripper/ScrewComponent.stl`

## Generate Tool Variants

From the repository root:

```bash
python simulation/generate_tool_urdf.py --all --update-default
```

This writes:

- `simulation/generated/lattice_empty.urdf`
- `simulation/generated/lattice_gripper.urdf`
- `simulation/generated/lattice_screw_gripper.urdf`
- `simulation/lattice.urdf` as the default gripper variant

## Validate

```bash
python simulation/validate_urdf.py --urdf simulation/generated/lattice_empty.urdf --expected-tool empty --end-link tool0
python simulation/validate_urdf.py --urdf simulation/generated/lattice_gripper.urdf --expected-tool gripper --end-link tool0
python simulation/validate_urdf.py --urdf simulation/generated/lattice_screw_gripper.urdf --expected-tool screw_gripper --end-link tool0
```

The validator checks XML structure, link/joint graph, mesh paths, expected active joints, joint limits, and a zero-pose forward-kinematics smoke test.

### Screw Gripper Note

The `screw_gripper` module is an inferred first-pass tool model. It reuses the default gripper's ConnectorPartB placement and mounts `GeneralMotorCasing.stl` in the same local frame as the motor-casing portion of `GripperPartA.stl`. `ScrewComponent.stl` is modeled as a continuous rotating child link on the recovered motor output axis, with `tool0` at the screw tip. Replace this transform with a mounted assembly export if a production-accurate TCP or clearance model is needed.

## MuJoCo Viewer

Install the optional viewer dependency:

```bash
python -m pip install -r simulation/requirements-sim.txt
```

Open the workbench:

```bash
python simulation/mujoco_viewer.py --tool gripper
```

This is a **single-window** dark workbench: the MuJoCo scene is rendered
offscreen straight into the control panel, so there is one window and no stray
default MuJoCo UI. The arm stands on a 10 cm checkered grid floor under a soft
gradient sky with shadows, and a world-axis triad (**X = red, Y = green,
Z = blue**) marks the origin so directions are unambiguous; the Cartesian jog
labels are coloured to match. The window opens maximized; press **F11** for true
fullscreen (**Esc** to exit). The viewport is interactive — **drag** to orbit,
**wheel** to zoom, **right-drag** (or shift-drag) to pan; it scales to fill the
window. Controls:

- an end-of-arm **tool selector**, including `empty`
- **FK sliders** for the active joints (degrees, clamped to joint limits) with a live `tool0` readout
- **Cartesian jog** buttons that move `tool0` in X/Y/Z via damped-least-squares IK, reporting the residual (and flagging out-of-reach targets)
- **pose presets** (Zero / Wave demo) and **Reset view** / **Screenshot**

The viewer also runs a preflight check that warns about degenerate (collapsed,
mis-scaled) meshes — the failure mode that previously left the arm shells
invisible.

### Headless modes (no display required)

```bash
python simulation/mujoco_viewer.py --direct --all-tools          # load + IK + asset self-check
python simulation/mujoco_viewer.py --shot arm.png --res 1280x960 # one offscreen frame
python simulation/mujoco_viewer.py --contact-sheet sheet.png     # 4-angle PNG sheet
python simulation/mujoco_viewer.py --passive                     # native MuJoCo window (fallback)
```

`--shot`/`--contact-sheet` render the zero pose; `--shot` also accepts
`--az`/`--el` to set the camera. `--passive` (also the automatic fallback when
Pillow's `ImageTk` is unavailable) opens MuJoCo's own window running a slow wave
demo.

### Performance

The embedded viewport renders MuJoCo offscreen and copies each frame back to the
CPU (`glReadPixels`) for display, so its frame rate is bound by that read-back,
not the render resolution (lowering resolution does not help). To reduce cost the
live view omits shadows and floor reflections (both restored in
`--shot`/`--contact-sheet`) and renders at the exact viewport size so no
per-frame rescale is needed. On integrated GPUs (e.g. Intel Iris Xe) interaction
is still only ~10–15 fps — that read-back is the hard floor.

For genuinely smooth (60 fps) navigation use **`--passive`**, which lets MuJoCo
render straight to its own GPU window with no read-back. The trade-off is that it
is a separate native window rather than the embedded panel; that is an inherent
MuJoCo limitation, not a tuning knob.

## Scope

This is a lightweight visual/kinematic workflow, not a final dynamics model. Inertial properties are approximate, and `tool0` should be refined after measured TCPs are selected for each tool.

**Mesh units:** the `assets/so101/*.stl` shell meshes are authored in **metres** and carry **no** `scale` (their `sts3215` servo meshes likewise). Only the `../hardware/*.stl` Lattice parts are in millimetres and use `scale="0.001 0.001 0.001"`. Do not add a millimetre scale to the so101 meshes — that collapses them to invisible specks (the viewer's degenerate-mesh preflight will flag it).
