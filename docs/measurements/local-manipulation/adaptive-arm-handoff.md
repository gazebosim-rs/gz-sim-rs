<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Park and wake the working serial arm

The same 1 kg pick-and-place controller placed the parcel at 9.195 s, retired the empty pickup tray at 18 s and returned above the vacant station. After settling, the arm parked at **31.032 s**. On the next pickup command it woke before the solve ending at **31.533 s**. In a separate run, removing the loaded receiving shelf during a pause woke the arm before the solve ending at **31.283 s**. The parcel then began falling on that first solve. [First-resumed states, commands and trace checksums](adaptive-arm-handoff.json)

| Difference from the continuously dynamic run | Next pickup command | Receiving shelf removed |
|---|---:|---:|
| Prescribed / parked steps | 500 | 250 |
| Matched post-delivery solves | 14,032 | 13,782 |
| Largest center-position difference, six moving bodies | 2.827 µm | 2.078 µm |
| Largest orientation difference | 15.77 µrad | 11.59 µrad |
| Largest arm linear momentum difference | 0.000505 kg·m/s | 0.000572 kg·m/s |
| Largest arm angular momentum difference about the world origin | 0.000321 kg·m²/s | 0.000456 kg·m²/s |
| First resumed mount impulse difference | 0.000508 N·s | 0.000290 N·s |
| First resumed shelf impulse, both lanes | 0.00981 N·s | 0 N·s |
| First resumed pad impulse, both lanes | 0 N·s | 0 N·s |

The first resumed mount impulse is the measured change in linear momentum of the entire rail-mounted arm after subtracting its gravitational impulse. All five arm links were clear of external contact at that boundary. The peak link velocity differences over the full run were 0.824 mm/s on the normal route and 9.356 mm/s at the empty right finger 41 ms after the support-loss boundary. The parcel followed the same trajectory in both lanes.

## What made standby possible

The original sideways retreat swept a raised finger into the delivered parcel. The return now raises the spindle from 1.6 to 1.98 rad before moving sideways. Shelf support is checked at every return solve. Its minimum vertical impulse was 0.00981 N·s in both qualified runs. Once the rail returns home, the spindle lowers under control to its physical stop and the empty fingers close. This avoids exciting the undamped wrist during a free drop. The ordinary 18-second placement is preserved. The adaptive and continuous routes receive identical commands, workcell edits and simulated time.

Both modes release rail effort and disengage the two position drives. The native solver must then certify clearance, bounded link motion and passive equilibrium for 32 consecutive 1 ms intervals. The eligible fixed-base assembly includes a stable hanging universal wrist. On demotion, Rapier removes the complete native island from active solving together with the screw and wrist speed-reference bodies. On the initial sleeping implementation each link incorrectly accumulated one gravity step (9.81 mm/s). Its first resumed mount impulse differed by 0.02768 N·s. Correcting the native island transition reduced that error to 0.000508 N·s on the command wake.

During standby the receiving shelf must still exist and carry positive vertical impulse. A queued pickup command or missing load requests promotion before the next solve. The test removes the confirmed loaded receiver at the same paused clock in both lanes. The shelf supplied 0.00981 N·s on its final loaded step and zero after removal. The detector wakes the idle arm for its next motion command. The model without the retired stands has open space below the falling parcel.

Each run issued 29,506 successful joint force writes, 238 target writes and four drive-enable writes. Both modes recorded the same SHA-256 of their ordered input stream, including the empty tray removal and optional receiver removal. The comparison reads the six rigid bodies on every advancing solve following placement and checks that the reference uses the same clock, link ordering, wake boundary and input digest. Reset restored both supports, repeated the numerical report and produced the same six-link pose digest. All four full pose archives, post-delivery traces and JSON reports matched byte for byte across one and eight Bevy compute threads.

## Reproduce with the installed CPU preview

Choose fresh output directories and files. Use `--timings path.json` for a separate timing run without tracing. The normal 18-second task still runs when `--fidelity` is omitted.

```bash
./bin/serial_transfer clear-obstacle dynamic --parcel-kg 1 --fidelity dynamic \
  --threads 1 --handoff-trace dynamic.ndjson > dynamic.json
./bin/serial_transfer clear-obstacle adaptive --parcel-kg 1 --fidelity adaptive \
  --threads 1 --handoff-reference dynamic.ndjson > adaptive.json

./bin/serial_transfer clear-obstacle loss-dynamic --parcel-kg 1 --fidelity dynamic \
  --threads 1 --remove-receiver-during-standby --handoff-trace loss.ndjson > loss-dynamic.json
./bin/serial_transfer clear-obstacle loss-adaptive --parcel-kg 1 --fidelity adaptive \
  --threads 1 --remove-receiver-during-standby --handoff-reference loss.ndjson > loss-adaptive.json
```

The installed route, standby controller and per-step comparison exercised 517 of 521 changed executable lines (99.23%) and 926 of 970 changed LLVM regions (95.46%). Removing required load or joint feedback at six boundaries stopped the route before its next motion. [Watch both modes in gz-web-rs](adaptive-arm-view.md) · [Compare paired step costs](adaptive-arm-switching.md)
