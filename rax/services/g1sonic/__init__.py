"""The real Unitree G1 + Dex3 under NVIDIA's SONIC whole-body controller, driven through the
official C++ deploy stack (`gear_sonic_deploy`, `g1_deploy_onnx_ref --input-type zmq`).

The agent's interface is HumanoidArena's: 50 Hz semantic40 reference frames (`g1_step`), a cuRobo
arm planner (`g1_free_space_move`), the front camera with its geometry (`g1_observe`,
`g1_pixel_to_position`) and `g1_hold`. The backend converts each frame the way HumanoidArena's
action provider does and streams it to the deploy stack over ZMQ; the stack runs the frozen SONIC
encoder and decoder and its own PD loop. Nothing here commands a motor.
"""
