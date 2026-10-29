# agentics_multirobot — 真机 Unitree G1 + Dex3：SONIC 控制，RAX agent 驱动

这是从 DGX 上 sim2sim 验证过的那套东西打出来的包（2026-10）。**这里的代码还没有在真机上跑过一次**，sim 里跑过
900 多个 session。按下面的顺序走，哪一步没过就停在哪一步，不要跳。

布局：**一台本地主机同时跑 SONIC（全身控制器）和 RAX（robot service + agent）**，G1 头上的 Jetson 只负责发相机画面。

```
rax/                 RAX 真机分支 + 完整 G1 移植（services/g1sonic、5 个 G1 技能、操作台、bring-up 脚本）
rax/scripts/         g1_up.sh（一个 tmux 开齐 camera/sonic/console/dashboard/work 五个窗格）
                     g1_check_all.sh（第一天标定）、g1_check_quick.sh（每次开机检查）、g1_motion_check.py、g1_console.py
jetson_camera/       Jetson 上跑的相机服务：RealSense（--camera realsense）或 ZED（--camera zedsdk|zed），TCP 7790
sonic/               SONIC 的 commit 号、v1.1 模型下载 + 哈希校验、站立参考动作、sim 专用 patch、手动安装说明
tasks/               agent 任务描述：trash.md（桌上的罐子扔进垃圾桶）；文件名就是 session 的任务 id
docs/                REAL_ROBOT_WORKFLOW.md（开机→标定→agent 全流程）、REAL_ROBOT_DEPLOY*.md、AGENT_WORKFLOW.md、
                     g1_real_options.example.json
setup_rax_host.sh    主机一次性安装：两个 venv、设置模板，--with-sonic 顺带装 SONIC
requirements-*.txt   robot service 环境 / agent 环境 / 相机机器
```

---

## 0. 需要准备的硬件和网络

| 东西 | 要求 |
|---|---|
| RAX/SONIC 主机 | Ubuntu 22.04，NVIDIA GPU，**CUDA 12 + TensorRT 10 已装好**（sim 容器里是 TensorRT 10.13 / CUDA 12.9；安装脚本不装这两样），一张**有线网卡**接机器人网段，静态 IP 如 `192.168.123.50`。有外网（下模型、装包） |
| G1 + Dex3 | 电池满。Jetson 地址默认 `192.168.123.164`，用户 `unitree`。头部 RealSense D435 |
| 吊架 + 带子 | 开机、第一次站、第一次走、出事全靠它 |
| Unitree 遥控器 | 开机自检、兜底急停 |
| 卷尺、地面胶带 | 第一天标定要量 3 个数 |
| 人 | 两个：一个按键，一个守吊架和急停 |
| 桌子 + 罐子 | 以后跑任务用；标定的 camera 步要量桌面高度 |

---

## 1. 主机一次性安装（没有机器人也能做，约半天）

```bash
git clone https://github.com/CelinaWa/agentics_multirobot.git ~/agentics_multirobot
cd ~/agentics_multirobot
sudo apt install -y tmux bubblewrap socat   # bubblewrap + socat: Claude Code 的沙箱，缺了 agent 直接拒绝启动
bash setup_rax_host.sh --with-sonic
```

它做的事：
1. 建两个 venv：`~/envs/rax-g1`（robot service，装 `requirements-robot.txt`）、`~/envs/rax-agent`（agent 自己的环境，不带任何机器人包）。
2. 写 `~/.g1_up.env`（来自 `g1_up.env.same_host.example`）和 `~/g1_real_options.json`（来自 `docs/g1_real_options.example.json`）。
3. `--with-sonic`：clone `Jay-Ye/GR00T-WholeBodyControl` 到 `~/GR00T-WholeBodyControl`，checkout `sonic/GR00T_WBC_COMMIT.txt` 里的 commit；
   把 `sonic/reference_standing/stand_000` 拷进 `gear_sonic_deploy/reference/standing/`；用 `sonic/download_models.py` 下载
   **SONIC v1.1** 模型（encoder、decoder、observation_config、planner）并用 `sonic/models.sha256` 校验，不对就退出；
   跑 `scripts/install_deps.sh`（apt，要 sudo）和 `just build` 编译。
4. 跑一遍不需要机器人的单元测试 `rax/local_tests/unit/test_g1sonic_backend.py`。

> 为什么要钉 v1.1：fork 的 checkout 自带的 `observation_config.yaml` 和它的 `download_from_hf.py` 对应的是 v1.0 模型，不是我们 sim 验证的那套。不要用它们。

安装完**检查两份设置文件**：

`~/.g1_up.env` 要改的只有这些（其它保持默认）：

| 变量 | 填什么 |
|---|---|
| `ROBOT_IP`、`JETSON_SSH` | Jetson 的 IP 和 `unitree@IP`（默认 `192.168.123.164`） |
| `RAX_HOST_IP` | `127.0.0.1`（SONIC 和 RAX 同机） |
| `SONIC_SSH` | 留空（SONIC 在本机） |
| `SONIC_DEPLOY_DIR` | `--with-sonic` 已自动填好 |

`~/g1_real_options.json`：`state_host` 填 `127.0.0.1`，`camera_host` 填 Jetson IP，其它保持默认（`camera: "network"`、端口 7790、`console_start_mode: "stand"`）。

没机器人时能验证到哪：`cd ~/GR00T-WholeBodyControl/gear_sonic_deploy && ./deploy.sh --help` 能打印出用法说明编译是好的；
`~/envs/rax-g1/bin/python -m pytest rax/local_tests/unit -q -k g1sonic` 全绿。

---

## 2. Jetson 一次性安装（约 20 分钟，机器人开机即可，不需要站起来）

```bash
# 主机上
ssh-copy-id unitree@192.168.123.164            # 以后 g1_up.sh 要免密 ssh 过去开相机
scp -r jetson_camera unitree@192.168.123.164:~/
# Jetson 上
ssh unitree@192.168.123.164
python3 -c "import pyrealsense2, cv2, numpy, PIL" || pip3 install pyrealsense2 numpy opencv-python-headless pillow
cd ~/jetson_camera && python3 camera_server.py --camera realsense --port 7790
```

看到 `serving realsense on port 7790`、之后每 5 秒一行 `frames sent N`，Ctrl-C 退出。这个服务**只读相机、不碰电机**。
相机内参从 RealSense 固件里读，外参用 URDF 的 `d435_link`，**都不用标**。

（以后换头盔 ZED：相机服务改在主机本地跑 `--camera zedsdk`，`camera_host` 填 `127.0.0.1`，并在 `g1_real_options.json`
里填量出来的 `camera_pose_override`。先不要走这条路。）

---

## 3. 真机第一天：标定（约 40 分钟）

目的：拿到这台机器人在 SONIC 下的真实数字（转 30° 实际转多少、走 1 m 实际走多少、停车延迟、相机高度误差），
和 sim 的数字对照，写进 `G1.md`。**不跑任何任务。**

1. 机器人挂上吊架，脚离地约 20 cm，周围 1.5 m 清空。开机，等 Unitree 自检完成，遥控器放手边。
   Unitree 自带的运动服务不用手动停：SONIC 启动时自己接管（`docs/REAL_ROBOT_WORKFLOW.md` 第 B 节第 3 行）。
2. 主机上：
   ```bash
   cd ~/agentics_multirobot/rax && bash scripts/g1_up.sh check
   ```
   它先做预检（ping Jetson、免密 ssh、设置文件无占位符、deploy.sh 和 `reference/standing/` 存在），任何一项失败会直接告诉你缺什么，不会动机器人。
   然后开一个 tmux，一屏五个窗格：

   | 窗格 | 里面跑的 | 你要按的 |
   |---|---|---|
   | `camera` | ssh 到 Jetson 的相机服务 | 没有 |
   | `sonic` | `deploy.sh real …`，编译检查后问 `Proceed? [Y/n]` | **`Y`**，然后等 `Init Done`（机器人自动摆成站姿，僵硬、不平衡） |
   | `console` | `g1_console.py` | **`s`** 站起（SONIC 开始平衡，腿会伸直一下）；**`SPACE`** 上锁/暂停切换；**`o`** 软停（先挂带子） |
   | `dashboard` | 状态、相机画面 | 没有 |
   | `work` | `g1_check_all.sh` | 每一步前它写清楚"这步动什么"，等你 **ENTER** |

3. `work` 窗格按提示走，顺序：`status` → `disarmed`（确认没上锁时指令会被拒）→（按 `s`、等站稳、按 `SPACE`）→
   `hold`（10 s 不动，看漂移）→ `arms` → `waist` → `hands` → `pause`（手臂动起来时**你按 SPACE**，测停车延迟）→
   （放低吊架让脚吃力、带子放松、再按 `SPACE`）→ `turn`（±30° 两次）→（地上贴胶带）→ `walk`（1 m 两次，每次问你卷尺读数）→
   输入骨盆高度和桌面高度 → `camera`。
4. 结束时 `work` 窗格打一张 Markdown 表。把表发给 Junjie / 存成 `G1.md`。

**随时停：** `SPACE` 是暂停（动作在当前帧停住，机器人继续站着）；`o` 是软停（全身阻尼，机器人会软掉，**必须先挂着带子**）。
倾斜超过 60° 时 robot service 会自动软停 + 拔钥匙。

**第一天预期会看到的、不算故障的事**（sim 里的数据）：转 30° 实际约 22°、走 1 m 实际约 0.8 m（规划器保守）、
相机算出的地面高度和卷尺差 1–3 cm。**算故障的事**：`s` 之后站不稳、`hold` 期间倾斜持续增大、任何一步脚步乱、`pause` 按了不停。
出现就 `o`，把 `work` 窗格的输出发过来。

---

## 4. 以后每次开机

```bash
cd ~/agentics_multirobot/rax && bash scripts/g1_up.sh check --quick-check
```
同样的五个窗格，`work` 只跑 `status`、`hold`、`arms` 三步（1 分钟），数字和 `G1.md` 对一下。

---

## 5. 跑任务（agent 模式）

**前提：** 第 3 节的表拿到了，第 4 节的三步正常。另外还需要一次性装好 Claude Code 并登录实验账号：

```bash
npm install -g @anthropic-ai/claude-code             # 需要 node >= 18
cd ~/agentics_multirobot/rax && ~/envs/rax-g1/bin/python -m harness.claude_cli.experiment_login
```
登录流程、账号选择、`--personal` 的含义：`docs/AGENT_WORKFLOW.md`。

每次跑任务：

1. 任务文件。`tasks/trash.md` 是现成的（桌上的罐子扔进垃圾桶：场景、成功条件、规则、sim 里学到的提示、什么时候问人），
   `tasks/crouch_pick.md` 是蹲下从矮面（椅子/矮架）拿罐子再放回（用 `g1_crouch`，只在 sim 验证过：0.1/0.2/0.3 m 指令实际下蹲 0.08/0.21/0.29 m，不倒）；
   开 session 前按房间改一下里面垃圾桶的位置。自己写新任务照它的结构：物品、位置、目标、禁区、什么时候停下来问人。
   **第一次真机建议先用更小的任务**，例如"把桌上的罐子拿起来再放回去；同一个物体失败两次就停下来问我"。
2. 机器人已经在 SONIC 下站着（`g1_up.sh check --quick-check` 跑过），然后：
   ```bash
   bash scripts/g1_up.sh agent --task-file ../tasks/trash.md
   ```
   tmux 还在时它只替换 `console` 和 `work` 两个窗格，**不要再按 `s`**（机器人已经站着）。`work` 窗格就是 Claude Code 的对话，
   agent 想动机器人会先 `request_operator`，`console` 按 `SPACE` 给钥匙，按 `r` 回复它的具体请求。
3. 中途想改任务、提示它、让它停：直接在 `work` 窗格打字。
4. 可选：给它看"做完是什么样"的照片（RAX 论文里的 in-context action learning，论文里也是可选项，默认没有示教）：
   ```bash
   bash scripts/g1_up.sh agent --task-file ../tasks/trash.md --demo done.jpg
   bash scripts/g1_up.sh agent --task-file ../tasks/trash.md --demo 1.jpg --demo 2.jpg --demo-kind completion_sequence
   ```
   照片只给它看意图（最终物体关系 / 动作顺序），不含任何机器人状态，它不会照抄位置。没有 `--demo` 时对应技能不会出现在它的工作目录里。

agent 的工作目录里除了机器人技能还有一层"方法"技能（从 RAX 论文的 HumanoidArena 分支搬过来、改成真机语境）：
按阶段统计每次尝试的漏斗、k/n 证据、预算、事故地图、不可逆动作先验证、控制器工程规范、结束前写 `handoff/README.md`。
它们是人写的 harness 技能，不是 agent 自己探索出来的；完整列表见 `rax/agent_skills/README.md`。

agent 在 sim 里**没有**端到端跑过（sim 的 900 多个 session 是脚本直接调工具），所以第一次 agent 真机前建议先让 Junjie 在 DGX 的 sim 上
做一次 dry run。时间或 token 用完 session 自然结束时：Claude 先停，robot service 后停，**机器人保持最后一帧姿势站着**（SONIC 继续平衡），
不会软掉；之后要停只能靠 SONIC 窗格的 `O` 或遥控器。

---

## 6. 关机顺序

1. 挂带子、吊起（脚离地）。
2. `console` 按 `o`（SONIC 软停并退出，机器人软掉挂着）。
3. `tmux kill-session -t g1`（关掉相机服务和其它窗格）。
4. 机器人关机、取电池。

---

## 6.5 在 DGX sim 里用真 agent 跑过一次（2026-10-29）

`start_agent.sh` + Claude Code（不是脚本）在 sim2sim 里跑了 `tasks/crouch_pick.md` 的 sim 版本：robot service、MCP、console 的 arm/reply、
`request_operator`、`g1_observe`/`g1_pixel_to_position`/`g1_crouch`/`g1_free_space_move` 都被 agent 正常调用，agent 自己量了蹲 0.1/0.2 m 的
骨盆高度和倾角、发现场景摆错时停下来问人。暴露出的两点：(1) 主机必须装 `bubblewrap socat`（上面已加）；(2) Docker 容器里沙箱建不了
user namespace，agent 的 Bash 用不了，所以 sim 里只验证了手动调工具的路径，`code/` 下写控制器和 skill library 提交要在真主机上才跑得通。

## 7. 已知的坑 / 还没验证的事

- **`g1_free_space_move`（cuRobo 手臂规划器）没装就不可用。** `requirements-robot.txt` 故意不含 torch/cuRobo（装起来重，要 CUDA）；
  没有它 agent 会按 skill 里写的自己拼手臂帧（sim 里 agent 已经验证了这条退路：preview 报错后它改用 `g1_step`）。想要规划器：在 robot service 的
  venv 里装 torch + cuRobo（`services/g1sonic/g1/solvers.py` 第一次调用时才 import），再用 `g1_free_space_move preview_only=true` 验证。

- **这里的一切都没在真机上跑过。** 顺序：第 1、2 节 → 第 3 节 → 第 4 节 → 第 5 节，每一节的结果都决定下一节要不要做。
- sim 里 SONIC 开始走的第一步有约 5–10% 概率扑倒（策略本身，不是我们的代码），所以第一次 `turn`/`walk` 带子一定挂着。
- 机器人伸手时骨盆会后退 5–10 cm，agent 的技能里已经考虑，但真机数字要用第 3 节的 `arms` 步核对。
- **MA-Teleop 的 `g1_client` 和这里的 robot service 不能同时跑**，两个都往 5556 发命令。
- 没做的：agent 沙箱审计、sim 里的 agent 端到端 dry run、`G1.md`（等第 3 节的数字）。
- 不要把 `~/.g1_up.env`、`~/g1_real_options.json`、任何 token 提交进来。`.gitignore` 挡了常见的几个。

遇到对不上的地方，先看 `docs/REAL_ROBOT_WORKFLOW.md`（更细、带"系统此时在做什么"的表），再把 `work` 和 `sonic` 窗格的输出发过来。
