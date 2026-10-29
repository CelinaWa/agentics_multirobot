---
name: learn-from-incidents
description: Keep a cumulative table of falls, near-falls, unexpected contacts, stops and stalls, cluster them by situation into hazard zones and change the plan there. Read after a fall, a near-fall or an operator stop, and when they keep recurring.
---

# Learn from incidents: map where things go wrong and change the plan there

On a real robot a fall is not a reset; it is damage, a long pause and a reason the operator may
end the session. Every near-fall is a fall you were lucky about.

- **An incident table.** Falls, near-falls (a sharp rise in the reported tracking error, a tilt
  that climbs in `transition.feedback`, a stop the backend triggered, a pause or disarm by the
  operator), unexpected contacts and stalls, each with the positions of the robot and of the
  objects involved relative to the scene's fixed structure, the phase, the last few actions and
  the body signals, in one cumulative table in `memory/` across attempts and sessions.
- **Clusters by situation, not by the last command:** location, nearby objects, phase, posture
  (leaning, crouched, an arm far out), body signals. The last command is usually only the trigger.
- **A cluster is a hazard zone.** Strategies that avoid it, pass through it quickly or finish the
  task from farther away worked better than patches; a keep-out or a dedicated, conservative
  behaviour covers the time the robot has to spend there. The escape route from one hazard can lead
  into the next, so it is worth checking.
- **A fix is real** once the robot has passed the same situation safely often enough to bound the
  rate ([evidence-and-regression](../evidence-and-regression/SKILL.md)).
- **A stop is evidence that the plan is wrong there.** After the backend's safety stop or the
  operator's pause, resuming the same action rarely helped; backing off to a known-safe
  configuration and approaching differently, or switching methods, did. Ask the operator what they
  saw: they stand next to the robot and see what the camera does not.
- **Near-miss counts** estimate how often a situation would end in a fall: one near-fall in a few
  passes means a fall within the session if nothing changes.
