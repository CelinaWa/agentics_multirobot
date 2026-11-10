---
name: object-localization
description: Where to find each robot's own object-localization skill in a G1 + R1 team, and how to relate what the two cameras see. Read before locating anything from an image.
---

# Object localization in the G1 + R1 team

Each robot localizes with its own camera, in its own pelvis frame, by its own skill:

- G1 (`g1_observe`, `g1_pixel_to_position`): `.agents/robots/g1/skills/object-localization/SKILL.md`
- R1 (`r1_observe`, `r1_pixel_to_position`): `.agents/robots/r1/skills/object-localization/SKILL.md`

A robot's folder has that skill only when its camera and observation profile support it; otherwise its
`robot-interface` says what the camera gives.

A position measured by one robot is in that robot's frame. To use it for the other robot, go through the team frame
([robot-interface](../robot-interface/SKILL.md)): the point in the room is the measuring robot's pose in the team
frame composed with the point in its pelvis frame, and the other robot's pose takes it into the other pelvis frame.
The result carries both poses' uncertainties, which are usually several centimetres after a walk. For anything a
hand must touch, have the robot that will touch it measure it with its own camera.

When both robots see the same landmark (a table corner, a box edge), the two measurements tie the frames together:
use them to correct one robot's pose with `team_set_pose`.
