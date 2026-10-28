<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# MuJoCo parity reference

`mujoco_reference.cc` drives the installed gz-physics MuJoCo plugin through its
public feature API (SDF world construction, `ForwardStep` with a 1 ms input,
link `FrameDataRelativeToWorld` and basic joint state) and prints every tenth
step of three scenarios: a free-falling sphere, a sphere dropped onto a static
box, and a world-revolute pendulum. `../mujoco_reference.json` is its output.

Generated on 2026-10-27 with gz-physics 10.0.0~pre2
(`libgz-physics-mujoco-plugin.so.10.0.0~pre2`, sha256
`ba0b4f86b26f85873722bcb9872fb426af9d6a1d75933be533caa85bcf9daec0`) and
MuJoCo 3.11.0 (`libmujoco.so.3.11.0`, sha256
`e21f664ea5896fc5aba4de57902ac3fd215584e36214ee692c93a36c0099578a`):

```sh
cmake -S crates/gz-physics-mujoco/tests/fixtures/reference -B build/mujoco-reference \
  -DCMAKE_PREFIX_PATH=/home/nkoenig/local -DCMAKE_BUILD_TYPE=Release
cmake --build build/mujoco-reference
LD_LIBRARY_PATH=/home/nkoenig/local/lib build/mujoco-reference/mujoco_reference \
  /home/nkoenig/local/lib/libgz-physics-mujoco-plugin.so \
  > crates/gz-physics-mujoco/tests/fixtures/mujoco_reference.json
```

`mujoco_reference <plugin> --load 100 200 400` instead prints the best-of-five
time to construct N spheres, free and then each with a world revolute, and take
one 1 ms step, the native side of
`examples/load_time.rs` (results in `docs/MUJOCO.md`).
