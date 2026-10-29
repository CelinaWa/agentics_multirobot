---
name: verify-committal-actions
description: Define the expected outcome of an action you cannot take back (releasing, pushing, stepping, turning away, moving out of view), check it at once and plan the recovery. Read before such an action and when one did not do what you meant.
---

# Verify the outcome of committal actions

**Committal actions** are actions after which you no longer control or see the object as before:
releasing or dropping it, pushing a free object, taking a step or a turn (the body moves more or
less than commanded and cannot step back), or moving where the camera cannot see.

- **An expected outcome, defined before acting:** an observable signature of success (where the
  object should be, what the camera should show, what the body should feel, what the feedback's
  tracking error and tilt should do) and how soon it should appear.
- **A check at the next decision.** Re-perceiving and comparing the result with that signature
  catches the failures that an assumed success hides: a can that looked lifted but was only pushed
  over, a release that bounced out.
- **A recovery for each failure configuration** (the object at your feet, out of view, displaced,
  still in the hand), starting from where the failure leaves the robot, not from where the attempt
  began. An object on the floor is the operator's to pick up: ask, do not reach for it.
- **Actions after which a failure is still recoverable** are worth preferring, and each action's
  outcomes counted per attempt before you rely on it.
- **A cap on repetitions** of the same committal action, after which another method takes over or
  you ask the operator, keeps a failing move from eating the session and from pushing the object
  out of reach.
