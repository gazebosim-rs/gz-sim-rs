<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M1 real browser observations

The actual WebGL2 viewer displayed all 20 Rapier boxes and spheres plus ground,
with 21 truthful visual rows on the executor 1.14.1 graph. The same saved and
displayed frames show motion at 21.690 s, committed pause at 25.675 s, then
reset to paused epoch 1 at zero time
with initial poses restored. Unreported metrics remain unavailable. The current
preview emitted no automatic unsupported SDF query.

Resumed motion, fresh advancing state after reload, committed pause after
reconnect, clean shutdown and automatic recovery to the restarted paused world
were also observed. Reload first showed a transient closed connection, then
recovered automatically. Background rendering was often 1 fps; these are not
performance measurements. Spheres may roll off the finite ground during a long
run and continue to be simulated.

M1 is accepted after mandatory qualification and independent review; see
[the acceptance record](../../M1_ACCEPTANCE.md). These images record the bounded
local browser observations. Bundle
and image hashes are in [observations.json](observations.json).

![Initial paused world](initial.jpg)

![Actual motion](motion.jpg)

![Committed pause](pause.jpg)

![Committed reset](reset.jpg)

Additional frames: [resume](resume.jpg), [transient reconnect closure](reconnect-transient-closed.jpg),
[recovered live scene](reconnect.jpg), [pause after reconnect](reconnect-paused.jpg),
[clean shutdown](shutdown.jpg), and [restarted paused world](restart.jpg).

Earlier observations remain available in commit
`1a2648de6090b3b25e47a47c3de534252a8d3c5f`; they used the previous dependency graph.
