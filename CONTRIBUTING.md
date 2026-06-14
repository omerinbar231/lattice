# Contributing to Lattice

Thanks for your interest in Lattice. Whether you've built the arm, found a bug, or want to tackle one of the items below, contributions are welcome.

## Core Contributors

- Ahad Jawaid
- Juan Luna

## Roadmap

These are the priorities we'd love help with:

- [x] **URDF** — create a URDF for the arm for simulation and motion planning.
- [ ] **Simulation** — set up a simulation environment for the arm.
- [ ] **Camera Extension** - create a camera hardware extension that can clearly see the end effector gripper.
- [ ] **Assembly videos** — film and upload video instructions for building the arm.
- [ ] **Data collection** — implement data collection, or validate that the current teleoperation setup transfers correctly.
- [ ] **Policy inference** — implement and/or validate that existing code can run policy inference to control the arm.
- [ ] **End effectors** — design new swappable end effectors (the current ones live in [`hardware/End Effectors/`](hardware/End%20Effectors/)).
- [ ] **Easier self-swapping** — the arm can already detach and reattach end effectors on its own; improve the design to make this easier and more reliable, including holding bays to dock the swappable end effectors.

## How to Contribute

1. **Check existing issues** so you don't duplicate work in progress.
2. **Open or comment on an issue** describing what you plan to do.
3. **Fork → branch → PR**, referencing the issue (e.g. `Fixes #12`).
4. Keep PRs small and update docs when behavior changes.

Software lives in the [lerobot `lattice` branch](https://github.com/ahadjawaid/lerobot/tree/lattice); hardware and build resources live in this repo.

## Hardware Changes

The hardware design is largely settled. Please discuss major hardware changes in an issue before submitting, and keep cost and assembly complexity low.
