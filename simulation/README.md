# Lattice URDF

This folder contains an initial URDF robot description for the Lattice arm mounted on the SO-101 base, plus an optional MuJoCo viewer for inspecting it.

## Files

- `lattice.urdf` - visual/kinematic model for the six active Lattice/SO-101 joints.
- `assets/so101/` - SO-101 base and servo mesh assets used by the URDF.
- `validate_urdf.py` - dependency-free URDF smoke test.
- `mujoco_viewer.py` - optional MuJoCo viewer for FK/IK inspection of the URDF.
- `requirements-sim.txt` - optional viewer dependencies (`mujoco`, `pillow`).

The Lattice-specific printed parts are referenced from the existing hardware files in this repository, for example:

- `../hardware/Shoulder/Shoulder.stl`
- `../hardware/Shoulder Connector/ConnectorPartA.stl`
- `../hardware/End Effector Connector/ConnectorPartB.stl`
- `../hardware/End Effectors/Gripper/GripperPartA.stl`

## Joint Model

The active joint names match the Lattice/LeRobot motor names:

- `shoulder_pan`
- `shoulder_lift`
- `elbow_flex`
- `wrist_flex`
- `wrist_roll`
- `gripper`

The Lattice tool-change geometry keeps the SO-101 wrist chain through `wrist_flex`. `HolderArmPart` replaces the stock SO-101 wrist-roll-pitch printed carrier on `wrist_link`, and `wrist_roll` rotates the downstream tool-side servo and gripper assembly.

The gripper range is modeled as `-30 deg` closed to `70 deg` open.

## Validate

From the repository root:

```bash
python simulation/validate_urdf.py --end-link tool0
```

The validator checks XML structure, link/joint graph, mesh paths, expected active joints, joint limits, and a zero-pose forward-kinematics smoke test.

## MuJoCo Viewer

Install the optional viewer dependencies:

```bash
python -m pip install -r simulation/requirements-sim.txt
```

Open the viewer:

```bash
python simulation/mujoco_viewer.py
```

A single-window dark workbench renders the arm offscreen straight into the control
panel (one window, no stray default MuJoCo UI), standing on a grid floor under a
soft sky with a world-axis triad (**X = red, Y = green, Z = blue**). The window
opens maximized; **F11** toggles fullscreen (**Esc** to exit). **Drag** to orbit,
**wheel** to zoom, **right-drag** (or shift-drag) to pan. Controls:

- **FK sliders** for the active joints (degrees, clamped to joint limits) with a live `tool0` readout
- **Cartesian jog** buttons that move `tool0` in X/Y/Z via damped-least-squares IK, coloured to match the axes and reporting the residual
- **Zero** / **Wave** pose presets, plus **Reset view** and **Screenshot**

A preflight check warns about degenerate (collapsed / mis-scaled) meshes.

### Headless modes (no display required)

```bash
python simulation/mujoco_viewer.py --direct                      # load + IK + asset self-check
python simulation/mujoco_viewer.py --shot arm.png --res 1280x960 # one offscreen frame
python simulation/mujoco_viewer.py --contact-sheet sheet.png     # 4-angle PNG sheet
python simulation/mujoco_viewer.py --passive                     # native MuJoCo window
python simulation/mujoco_viewer.py --urdf path/to.urdf           # load a different URDF
```

### Performance

The embedded viewport renders MuJoCo offscreen and copies each frame back to the
CPU, so its frame rate is bound by that read-back, not the render resolution. On
integrated GPUs interaction is ~10-15 fps; for smooth (60 fps) navigation use
`--passive`, which renders straight to a GPU window.

## Scope

This is an initial visual/kinematic description, not a final dynamics model. Inertial properties are approximate or inherited from the SO-101 reference where practical, and `tool0` should be refined after a measured TCP is selected for each end effector.

**Mesh units:** the `assets/so101/*.stl` shell meshes are authored in **metres** and carry **no** `scale`; only the millimetre `../hardware/*.stl` Lattice parts use `scale="0.001 0.001 0.001"`. A stray millimetre scale on the so101 shells collapses them to invisible specks (the previous behaviour, fixed here); the viewer's degenerate-mesh preflight flags this class of error.
