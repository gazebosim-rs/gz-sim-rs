# First bench impulse after a dropped grasp

Each replay follows the folded sheet from supported pickup through complete escape from both fingers and ends on the first impact with the original ideal plane. The visible workbench is finite. In these four overload cases the sheet lands beyond its edge. A finite receiving box is qualified separately. [Finite box contact](static-soft-bench.md)

| Mesh and controller | Time after matched grip | First upward impulse | Lateral impulse | Native replay |
| --- | ---: | ---: | ---: | --- |
| 5 columns, guarded, 1.25 N | 272 ms | 4.673 mN·s | -3.271 mN·s | [gz-web-rs](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9851%2F) |
| 5 columns, control, 1.25 N | 265 ms | 1.903 mN·s | -1.332 mN·s | [gz-web-rs](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9852%2F) |
| 7 columns, guarded, 1.30 N | 275 ms | 11.096 mN·s | -7.767 mN·s | [gz-web-rs](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9853%2F) |
| 7 columns, control, 1.30 N | 267 ms | 1.757 mN·s | -1.230 mN·s | [gz-web-rs](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9854%2F) |

Each run has an additional native frame at the exact first impulse. Scrub to the final frame to see first contact. The WebGPU checker verified 560 frames, 20,752 visual poses and shapes, and 13,472 measured particle positions. All four native triangle checks found zero crossings through a jaw core or the opposite panel.

The vertical audit uses the individual particle masses from the authored material and each measured particle velocity immediately before and after impact. It subtracts the weight, jaw and bench impulses from that momentum change. The lateral audit independently combines the material momentum and external forces in each 1 ms solver interval. The largest absolute balance residual on either axis was 9.2 × 10⁻¹⁷ N·s. All four replay streams were byte-identical on the independent particle-data rerun.
