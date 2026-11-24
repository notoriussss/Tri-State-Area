# Open Questions

Each question lists the group to ask. These should be tracked as repository issues; remaining details will be resolved during integration testing.

## Overhead

1. At what rate and latency will `/world/state` be published? Controller gains depend on this.
2. What color and size will each robot's triangle marker have? We propose an elongated isosceles triangle (~10 cm long) in two colors distinct from the cubes and the delivery zone.
3. Is the camera mounted directly above the painted red center dot? The nadir point `c` used in parallax correction depends on this.
4. Which virtual sensors will be published and in what exact format? See the proposal in [`contract.md`](contract.md).
5. Do you accept the origin and axes proposed in [`arena.md`](arena.md)?
6. Will the working capture resolution be 1280 × 720?

## Manipulator

7. Do you confirm the stage-1 delivery-zone arbiter contract stays unchanged with each robot calling `/zone/request` and `/zone/release` directly from its own Pi 5?
8. How far past the green line must a cube be pushed? Do the 12–19.5 cm band from Perry's axis and the 1.5 cm minimum separation between cubes still apply?

## Professor

9. How will this stage be graded (e.g., cubes per minute, collisions, red cubes touched, or total completion time)?
10. Do you accept the two-step red-cube avoidance variant (12 cm radius plus clear push corridor) instead of the full 17 cm radius?
11. Does VCI stand for *Virtual Communication Interface*? That is our current assumption.

## Team

12. Will we use a dedicated Wi-Fi router for all three groups? Which host will run `overhead` and the viewer on demo day?
13. Who is responsible for `sim/` (while `viewer` belongs to `overhead`)?

## To Measure

14. How much clear space is available on each robot's roof to mount the triangle marker?
15. What is the actual network latency from the `overhead` host to each robot's Raspberry Pi 5 (and between Phineas and Ferb)?
