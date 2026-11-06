# TRANSFER - measure before relying on (from sim session 7c03aa)
- Leg axis by x-slices (right edge + 1.6 cm): verify on real depth noise; the sim fit had res < 1 mm.
- IK approach compensation (cmd yaw +2..4, pitch -15): re-derive by preview on the real backend.
- Grip creep per walk (~1-2 mm/walk in sim) and the 2-walk limit: test with short walks over the table first.
- Hanging puppy: lowest point ~.27 below the hand; body along the finger axis (sim physics of the plush joints).
- Push-in finish: verify the box does not slide on the real table under a 12 N-class push.
- Stale backend command after an operator placement (sim backend): check r1_status 'command' before moving.
