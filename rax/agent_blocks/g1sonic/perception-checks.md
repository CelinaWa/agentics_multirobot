- **Re-projection.** A 3-D estimate (an outline, a contact point, the hand) projected back into the
  image lands on the object when the pipeline is right. A hand that occludes part of an object can
  look like contact.
- **Consistency over frames.** A static object seen from two poses gives the same estimate once your
  own motion is accounted for.
- **A residual log** in `memory/` (run, posture, residuals, frames) shows when to re-measure;
  investigate residuals before changing a threshold that rejects them.
