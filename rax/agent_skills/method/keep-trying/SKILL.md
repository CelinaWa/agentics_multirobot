---
name: keep-trying
description: Use all of a session's robot time - recovery and retries after a failure, practising another stage when an attempt is lost, and exercising recovery paths; mastery is repeated success counted k/n. Read when an attempt fails, when a session seems lost, and before you end robot work early.
---

# Keep going: every minute of robot time is practice

Robot time is the scarcest thing a session has; the model's time and the notes are cheap next to it. While the robots
stand, the session can still teach something, even after a failure or a fault.

- **After a failure, recovery and a retry** teach the most: re-perceive, back off, re-approach, look again. A
  diagnosis before repeating a motion saves attempts.
- **When retrying the stage is hopeless, practise another stage** you have not mastered yet: a walk to a measured
  target, a precise turn, a contact, a lift, a put-down. Practice with the real objects is worth more than a
  simplified stand-in, which leaves the real path untested.
- **Exercise the paths a reliable controller will need**, failure detection, recovery and retries included: a
  recovery branch that never ran in practice may not work when it matters.
- **Mastery means repeated success** at each stage, counted as k/n ([practice-by-stage](../practice-by-stage/SKILL.md)),
  not one stage further than last time.
{% if simulated %}
- **In simulation a fault is not a reason to stop.** A fall or a guard trip costs a restart of a minute or two.
  Check from the attempt's review and the timeline that your own motion caused it, record it as the outcome of its
  stage, change what caused it, and go on. Ending robot work early gives the remaining minutes away.
{% else %}
- **On the real robot, stop when going on is unsafe**, and only then: say why in one line, with the evidence (what
  happened, how often, what would make it safe), so the operator can decide.
{% endif %}
- **Before ending robot work early, check the list:** an untried strategy on the strategy page, a stage with fewer
  than five tries, a rule in the notes resting on one or two tries, a recovery path never run. If any is left and
  the robots stand, it is not time to stop.
