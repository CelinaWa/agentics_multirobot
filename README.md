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
docs/                REAL_ROBOT_WORKFLOW.md（开机→标定→agent 全流程）、REAL_ROBOT_DEPLOY*.md、AGENT_WORKFLOW.md、RAX_INSIDE.md（打开黑盒：进程、说明书、安检关卡、真实 session 逐步）、
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
3. `--with-sonic`：clone `Jinghy123/GR00T-WholeBodyControl`（公开 fork，含 pinned commit） 到 `~/GR00T-WholeBodyControl`，checkout `sonic/GR00T_WBC_COMMIT.txt` 里的 commit；
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
`cd rax && ~/envs/rax-g1/bin/python -m pytest local_tests/unit -q -k g1sonic` 全绿（要在 `rax` 目录里跑，测试从那里 import）。

### 1.5 装完之后、没机器人时就能做的事（照顺序敲）

```bash
# a. 编译和单元测试（约 2 分钟）
cd ~/GR00T-WholeBodyControl/gear_sonic_deploy && ./deploy.sh --help          # 能打印用法 = 编译没问题
cd ~/agentics_multirobot/rax && ~/envs/rax-g1/bin/python -m pytest local_tests/unit -q -k g1sonic   # 必须在 rax 目录里跑；应全绿

# b. 两份设置文件（上面的表）
nano ~/.g1_up.env            # ROBOT_IP / JETSON_SSH 改成 Jetson 的；RAX_HOST_IP=127.0.0.1；SONIC_SSH 留空
nano ~/g1_real_options.json  # state_host 127.0.0.1，camera_host 填 Jetson IP

# c. Claude Code + 登录（agent 模式用，第 5 节）
curl -fsSL https://claude.ai/install.sh | bash     # 原生安装到 ~/.local/bin，不需要 node；之后 export PATH="$HOME/.local/bin:$PATH"
cd ~/agentics_multirobot/rax && bash scripts/setup_claude_experiment_login.sh --personal   # 用你自己登录的账号付费；去掉 --personal 则用单独的实验账号

# d. 预检（不动机器人；没接机器人时 ping Jetson 这一项会失败，其它项应全过）
cd ~/agentics_multirobot/rax && bash scripts/g1_up.sh check --preflight-only
```

然后按第 2 节装 Jetson（机器人开机即可），第 3 节是真机第一天。

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

1. 机器人挂上吊架，脚离地约 20 cm，周围 1.5 m 清空。开机，等 Unitree 自检完成，遥控器放手边。（吊着只是为了开机和 SONIC 启动；按 `s` 之前要放到脚承重，见 `docs/CALIBRATION_WALKTHROUGH.md`。）
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
curl -fsSL https://claude.ai/install.sh | bash     # 原生安装到 ~/.local/bin，不需要 node；之后 export PATH="$HOME/.local/bin:$PATH"
cd ~/agentics_multirobot/rax && bash scripts/setup_claude_experiment_login.sh --personal   # 用你自己登录的账号付费；去掉 --personal 则用单独的实验账号
```
登录流程、账号选择、`--personal` 的含义：`docs/AGENT_WORKFLOW.md`。

每次跑任务：

1. 任务文件。**真机第一个任务建议用 `tasks/puppy_box.md`**：把桌上的毛绒小狗抓起来放进旁边的透明塑料箱（Hugging Face 数据集 `Celina717/g1_pnp_pour` 里遥操作示教过 29 次的那个任务：站着不走、只用右手），文件里**没有要人量的数字**：距离和桌高都由机器人自己测；带 sim 经验跑见 5.5；示教帧在 `tasks/demos/puppy_box/`，用 `--demo` 按顺序喂给 agent：
   ```bash
   bash scripts/g1_up.sh agent --task-file ../tasks/puppy_box.md \
       --demo ../tasks/demos/puppy_box/1_start.jpg --demo ../tasks/demos/puppy_box/2_grasp.jpg \
       --demo ../tasks/demos/puppy_box/3_over_box.jpg --demo ../tasks/demos/puppy_box/4_done.jpg --demo-kind completion_sequence
   ```
   `tasks/trash.md` 是第二个（桌上的罐子扔进垃圾桶：场景、成功条件、规则、sim 里学到的提示、什么时候问人），
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

agent 在 sim 里已经端到端跑过很多次（2026-10-29 起；puppy_box 最后一次不改代码的评测 7/8，见 `experience/README.md`）。时间或 token 用完 session 自然结束时：Claude 先停，robot service 后停，**机器人保持最后一帧姿势站着**（SONIC 继续平衡），
不会软掉；之后要停只能靠 SONIC 窗格的 `O` 或遥控器。

---

### 5.5 Sim → 真机迁移：带着 sim 经验跑 puppy_box（照顺序敲）

**带过去的是什么**（`experience/puppy_box/`，DGX sim session 3b3f99 导出，详见 `experience/README.md`）：

| 内容 | 作用 |
|---|---|
| `skill-library-g1sonic.bundle` | 技能库（11 个技能，主技能 `plush-pick-place-box-any-layout`），由 `import_experience.sh` 装进本机 |
| `workspace/code/puppy_box/run.py` v18.8 | 一个程序处理所有布局（小狗在箱子左/右/前），sim 里最后一次不改代码评测 7/8、0 次摔倒 |
| `workspace/memory/` | `TIPS.md`（带 `[strategy]`/`[param]`/`[sim-only]` 标签）、`PARAMS.json`（49 个 sim 测得的数字）、RUNS/EVAL 记录 |
| `workspace/handoff/README.md` | 开头 "Start here"：先跑什么、先量哪些参数 |

`--experience` 开 session 时自动做 sim→真机转换：`[sim-only]` 行移到 `memory/history/<session>/SIM_ONLY.md`（agent 不再照它做），
生成 `memory/TRANSFER.md`，列出 49 个要在真机上重新测的参数。agent 先测（观察、空手 hover、慢速局部动作），用 `record()` 记下真机值，
再做抓取等不可逆动作；代码通过 `api.params.param()` 读数，还没测过的会打印"用的是 sim 值"。

**第 0 步（只做一次）：** 第 1–3 节装好、标定过；第 5 节 Claude Code 已登录。

**第 1 步：更新代码和经验（每次）**
```bash
cd ~/agentics_multirobot && git pull                          # 至少要到 c176d9d（修了全新主机导入后技能库为空的 bug）
bash rax/scripts/import_experience.sh experience/puppy_box    # 第一次：建 rax/runtime/skill-library/g1sonic；以后：合并，不丢真机学到的技能
```
应该打印 cloned / merged，`git -C rax/runtime/skill-library/g1sonic log --oneline | head -1` 至少是 `604b743`。

**第 2 步：机器人站起来（每次）**
```bash
cd ~/agentics_multirobot/rax && bash scripts/g1_up.sh check --quick-check
```
SONIC 窗格 `Y`，等 `Init Done`；**脚先着地承重再按 `s`**，按完几秒内把带子放松（挂着按 `s` 会乱甩）；`work` 窗格 status/hold/arms 三步通过。

**第 3 步：摆场景（目测即可，不用量）**
- 桌子（0.68–0.78 m 高都行）放在机器人前面，骨盆离桌沿大约 0.3 m（手垂下碰不到桌子）。
- 小狗侧躺、腿朝机器人；透明塑料箱在小狗旁边，间隔几厘米到 12 cm。**第一次先摆"箱子在小狗右边"**（sim 里最稳，5/5），
  再试箱子在左边（右手跨身体）、箱子在小狗后面。
- agent 会自己用深度测桌高和桌沿距离，然后通过 `request_operator` 让你把桌子推 "closer"/"further"，推到桌沿离骨盆约 0.2 m。
  机器人不走路，靠人推桌子。

**第 4 步：启动 agent**
```bash
cd ~/agentics_multirobot/rax
bash scripts/g1_up.sh agent --task-file ../tasks/puppy_box.md --experience ../experience/puppy_box/workspace
```
tmux 已经在时只替换 `console` 和 `work` 两个窗格，**不要再按 `s`**。`console` 窗格：`SPACE` 给钥匙/暂停，`r` 回复 agent 的请求，
`o` 软停（只在带子挂好时按，机器人会软掉）。`work` 窗格就是 Claude 对话，可以直接打字提示它。

agent 一开始会：读 `TRANSFER.md` 和 handoff → 测桌高、桌沿、可达距离等参数 → 请你推桌子 → 按 `run.py` 的阶段跑
（观察 → 接近 → 抓取+抬起 → 搬运 → 放下 → 收手），每个阶段前有 LOOK 检查点。

**看什么、什么时候停：**
- 每次请求先看清它要做什么再按 `r`；左手应一直收在身后上方，碰到小狗不算成功。
- 机器人在 planner 下不停踏步、倾斜明显、手压着箱子往下推：按 `SPACE` 暂停，必要时 `o`。
- 任务失败可以接受；**摔倒不行**，宁可早停。
- 一次成功后它会请你重新摆（reset），换一种布局再试。

**第 5 步：跑完后把真机经验带回来**
```bash
cd ~/agentics_multirobot
ls -t rax/runtime/sessions | head -1                          # 刚跑完的 session，例如 20261104T..._puppy_box_xxxxxx
bash rax/scripts/export_experience.sh rax/runtime/sessions/<那个 session> experience/puppy_box_real
git add experience/puppy_box_real && git commit -m "experience: first real puppy_box session" && git push
```
导出的 `SOURCE.txt` 写着 `environment: real`，`PARAMS.json` 里多了真机值。DGX 这边用它重新调 sim 场景（例如用真机测的
`scene.table_height_m`），再跑下一批 sim。请把 `work` 窗格的输出和 `memory/PARAMS.json` 也发给我。

**第 6 步：关机** 按第 6 节。

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

第二次 sim 跑（书架版 crouch_pick）暴露了 backend 的一个真问题：第一次 `g1_walk` 的 planner 热身转身之后，机器人在 SONIC planner 下
原地踏步不停（15 分钟，倾角 3–10°），backend 只在机器人"安静"后才把控制交回 streamed hold，所以所有运动工具一直回答 still stepping，
agent 被卡死；而 robot service 退出时 planner keepalive 一停，deploy 1 s 后超时跳回参考动作第 0 帧，机器人倒了。已修（见第 7 节）：
`g1_hold` 现在会用约 20 s 尝试把 planner 下踏步的机器人接回来，接不回来会明说让 agent 找 operator；service 退出前也先这样试一次。

## 7. 已知的坑 / 还没验证的事

- **`g1_free_space_move`（cuRobo 手臂规划器）没装就不可用。** `requirements-robot.txt` 故意不含 torch/cuRobo（装起来重，要 CUDA）；
  没有它 agent 会按 skill 里写的自己拼手臂帧（sim 里 agent 已经验证了这条退路：preview 报错后它改用 `g1_step`）。想要规划器：在 robot service 的
  venv 里装 torch + cuRobo（`services/g1sonic/g1/solvers.py` 第一次调用时才 import），再用 `g1_free_space_move preview_only=true` 验证。
  **Warp 版本坑（2026-11-01 在 sim 里踩到）**：cuRobo 依赖的 `warp-lang` 从 1.10 起 PyPI 轮子是 CUDA 13 编译的，驱动低于 580（CUDA 13.0）的机器上
  Warp 只看到 CPU，规划器报 `list index out of range`。驱动是 5xx 时装 GitHub release 里的 `+cu12` 轮子：
  `pip install --no-deps --force-reinstall https://github.com/NVIDIA/warp/releases/download/v<版本>/warp_lang-<版本>%2Bcu12-py3-none-manylinux_2_28_x86_64.whl`
  （`<版本>` 用 `python -c "import warp; print(warp.__version__)"` 查），然后 `python -c "import warp as wp; wp.init(); print(wp.get_cuda_devices())"` 要列出 GPU。
  规划器还要求起始姿态无自碰撞：手贴在髋部时它会返回 `start_state_in_self_collision` 和碰到的连杆对，先用 `g1_step` 把手挪开几厘米再规划。

- **这里的一切都没在真机上跑过。** 顺序：第 1、2 节 → 第 3 节 → 第 4 节 → 第 5 节，每一节的结果都决定下一节要不要做。
- sim 里 SONIC 开始走的第一步有约 5–10% 概率扑倒（策略本身，不是我们的代码），所以第一次 `turn`/`walk` 带子一定挂着。
- **走完后原地踏步不停（"still stepping"）。** 偶发（sim 里脚本时代约 1/10 轮，agent 第二次 sim 跑撞上一次 15 分钟；之后 84 次短
  planner 动作里 0 次）。现在 backend 有限时恢复：下一次 armed 调用（`g1_hold` 就行）先把 planner 的 facing 对齐到实测朝向等安静，再让
  它慢走一步（`walk_stuck_reset_s` 1 s、0.1 m/s，约前进 0.1 m）后 idle 重置步态，总共等 `walk_stuck_recover_s`（默认 20 s）。还不安静
  就让 planner 继续平衡（这是安全的），调用报错叫 agent 找 operator（operator 挂带子按 STOP 或等它停）。**不会**硬把 hold 压到踏步中的
  机器人上：sim 里试过挑关节最慢瞬间接管再缓到站姿，2 次倒 1 次；要这种行为得显式开 `walk_stuck_force_handover`。工具回复
  `feedback.warm_up` / `feedback.control` 会告诉 agent 热身转身跑了、是否还在踏步。
- 机器人伸手时骨盆会后退 5–10 cm，agent 的技能里已经考虑，但真机数字要用第 3 节的 `arms` 步核对。
- **MA-Teleop 的 `g1_client` 和这里的 robot service 不能同时跑**，两个都往 5556 发命令。
- 没做的：agent 沙箱审计、sim 里的 agent 端到端 dry run、`G1.md`（等第 3 节的数字）。
- 不要把 `~/.g1_up.env`、`~/g1_real_options.json`、任何 token 提交进来。`.gitignore` 挡了常见的几个。

遇到对不上的地方，先看 `docs/REAL_ROBOT_WORKFLOW.md`（更细、带"系统此时在做什么"的表），再把 `work` 和 `sonic` 窗格的输出发过来。
