# One millisecond at the second glancing strike

The first glancing impact on the rough shelf occurred at 2.170 s. The first 0.1° yaw difference appeared at the second strike at 2.194 s. Starting from the GPU's exact preceding state, the independent CPU solver gave an outgoing yaw speed between 1.72 and 4.64 rad/s as its step size changed. The continuous GPU run gave 1.617 rad/s. [Native gz-web-rs replay](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A11407%2F) · [Spatial friction and original release](gpu-shelf-spatial.md)

Each row below starts a fresh Rapier cuboid and static shelf from the saved position, orientation, mass-center velocity and world angular velocity at 2.193 s. The columns use the GPU and continuous Rapier snapshots, respectively. Within either column every native refinement starts from exactly the same state, with an empty contact history.

| Native substeps in 1 ms | Outgoing yaw from GPU state (rad/s) | Outgoing yaw from Rapier state (rad/s) |
| ---: | ---: | ---: |
| 8 | 1.722 | 2.200 |
| 32 | 4.641 | 4.801 |
| 64 | 1.747 | 4.751 |
| 128 | 1.777 | 4.686 |
| 256 | 1.817 | 4.447 |
| 512 | 3.932 | 3.207 |
| 1,024 | 1.865 | 4.230 |

The original uninterrupted CPU simulation left this interval at 4.808 rad/s. Reversing only the local frame of the same finite shelf changed the two cold 32-substep results by less than 0.000001 rad/s. Normal impulse from the named shelf agreed with the gravity-corrected change in linear momentum to within 10⁻¹² N s in all 16 native controls. Each result records the complete signed linear and angular impulse, outgoing twist and the first substep with a loaded contact. [States and corner impulses from the original recording](gpu-shelf-glance-state.json) · [Matched interval results and exact configuration](gpu-shelf-glance.json)

At 2.193 s the first corner of the GPU box already touched the shelf, with a normal speed below 10⁻⁸ m/s. The next corner was 339 µm above the surface, approaching at 0.782 m/s. In the continuous CPU snapshot the first corner penetrated by 28.4 µm while the second was 293 µm above the surface. Resolving that transition from one supported point to two points is the next accuracy check. The results show why fitting to one native step size would give an unreliable target.

The Rust example reads the frozen source states embedded in the checkout and reproduces the table without GPU hardware. Rapier 0.36.1 uses a finite horizontal shelf, a free eccentric cuboid, gravity of 9.81 m/s², friction 0.45, zero restitution and hard per-point Coulomb contact. It runs 32 solver iterations and eight internal PGS iterations. Auxiliary stabilization, prediction, allowed error and correction velocity are zero. The qualification executable covered 84/84 lines and 192/195 regions.

```bash
RUSTC_WRAPPER= cargo run --release --locked -p gz-sim-cli --example gpu_box_shelf_glance
```
