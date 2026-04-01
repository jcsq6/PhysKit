# Pool Demo Controls

Run:

 - Build the `tests/pool` demo (see project README/CMake). After building, run the executable `pool` in the tests/pool directory.

Basic controls

- Left mouse: hold to charge shot, release to shoot. Aim by moving the mouse left/right.
- W/A/S/D: move camera.
- Space / Shift: move camera up/down.
- Esc: toggle drag (click-and-drag camera) mode.
- R: reset / rerack the balls.
- Tab: toggle debug overlay (if supported by the app — shows collision info).

Notes

- You cannot shoot while any ball's speed exceeds a small threshold (wait for table to settle).
- The HUD displays current power while charging.
