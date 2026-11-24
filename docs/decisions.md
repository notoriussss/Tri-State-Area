# Decisions

Each record lists its provenance. "Proposed" means it has not yet been ratified by all three groups.

| # | Decision | Source | Date | Status |
| --- | --- | --- | --- | --- |
| D1 | `overhead` sees, publishes the world, and computes virtual sensors. `fleet` decides and moves. `manipulator` sorts cubes | professor, group assignment | 17-11-2026 | decided |
| D2 | Hybrid decentralized paradigm: each robot plans autonomously on its own Pi 5 using the overhead world state and keeps two local reflexes (black line stop and internal HAL command watchdog) | recommendation from Perry report | 17-11-2026 | proposed |
| D3 | Each robot marker is a cardstock triangle; its centroid gives position and its tip gives heading | professor | 17-11-2026 | decided |
| D4 | There is no physical projector. The "virtual projection" is drawn on-screen over the camera image (`viewer_node` in `overhead`) | professor | 17-11-2026 | decided |
| D5 | Python with ROS 2 Lyrical on Ubuntu 26.04 (including on the Raspberry Pi 5 boards); neither Go nor Rust is used unless measurements justify it | latency analysis (see [`architecture.md`](architecture.md)); version confirmed by `overhead` | 17-11-2026 | decided |
| D6 | Red cubes are not delivered; they must be avoided. If a red cube reaches the delivery zone, Perry discards it | professor | 17-11-2026 | decided |
| D7 | Two options will be evaluated for red-cube avoidance: full safety radius (17 cm) vs. 12 cm radius plus a clear push corridor | professor (concept) and `fleet` (variant) | 17-11-2026 | to evaluate in simulator |
| D8 | Perry's delivery-zone arbiter remains unchanged; each robot requests and releases the zone lease directly from its own Pi 5 | stage 1 contract | 17-11-2026 | proposed |
| D9 | Single `Tri-State-Area` monorepo with top-level folders per system group and explicit group ownership | team (`overhead` set up the layout) | 17-11-2026 | decided (layout finalized) |
| D10 | A simulator (`sim/`) publishes the exact same contract as the real overhead camera | `fleet` | 17-11-2026 | proposed |
| D11 | Units in meters and radians (REP 103); `arena` frame with origin at the inner bottom-left corner of the overhead image | `fleet` | 17-11-2026 | proposed |
| D12 | Front cameras on the robots may remain mounted, but navigation uses the `overhead` camera | confirmed by Ignacio | 17-11-2026 | decided |
| D13 | Anti-collision uses standard navigation-light rules (port = red, starboard = green) computed virtually from the existing physical roof triangles (D3) and overhead collision alerts (< 34 cm), with plan-before-moving validation (inside margin and ≥ 12 cm from red cubes, re-evaluated at 20 Hz); no physical red/green stickers or LEDs on the chassis | fleet | 24-11-2026 | proposed |
