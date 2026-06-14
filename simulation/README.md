# Lattice URDF

This folder contains an initial URDF robot description for the Lattice arm mounted on the SO-101 base.

## Files

- `lattice.urdf` - visual/kinematic model for the six active Lattice/SO-101 joints.
- `assets/so101/` - SO-101 base and servo mesh assets used by the URDF.
- `validate_urdf.py` - dependency-free URDF smoke test.

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

## Scope

This is an initial visual/kinematic description, not a final dynamics model. Inertial properties are approximate or inherited from the SO-101 reference where practical, and `tool0` should be refined after a measured TCP is selected for each end effector.
