# agentics_multirobot — 真机 Unitree G1 + Dex3：SONIC 控制，RAX agent 驱动

这是从 DGX 上 sim2sim 验证过的那套东西打出来的包（2026-10）。**这里的代码还没有在真机上跑过一次**，sim 里跑过
900 多个 session。按下面的顺序走，哪一步没过就停在哪一步，不要跳。

**两台机器人从 sim 迁到真机，分别看这里：**

| 机器人 | 任务 | 现状（2026-11-06） | 怎么跑 |
|---|---|---|---|
| G1 + Dex3 | `tasks/puppy_box.md`：站着不走，右手把小狗放进箱子 | sim 最后一次不改代码评测 7/8，可以上真机 | 第 1–4 节装机和标定，然后第 5.5 节带 sim 经验跑 |
| R1 + PiPER 两指夹爪 | `tasks/puppy_box_r1.md`：夹小狗的腿，走两三小步，放进箱子 | sim 里还不可靠；连接真机控制栈的 bridge 有几处必须先改 | 先看第 8.0 节：现在能做的是阶段 A（装 R1 部署栈、sim 和真机分阶段测） |

两台都用同一个 RAX：同一个 launcher、同一套 operator 请求和上锁、同一种经验导出导入（`experience/<任务>/`）。

布局：**一台本地主机同时跑 SONIC（全身控制器）和 RAX（robot service + agent）**，G1 头上的 Jetson 只负责发相机画面。

```
rax/                 RAX 真机分支 + 完整 G1 移植（services/g1sonic、5 个 G1 技能、操作台、bring-up 脚本）
rax/scripts/         g1_up.sh（一个 tmux 开齐 camera/sonic/console/dashboard/work 五个窗格）
                     g1_check_all.sh（第一天标定）、g1_check_quick.sh（每次开机检查）、g1_motion_check.py、g1_console.py
jetson_camera/       Jetson 上跑的相机服务：RealSense（--camera realsense）或 ZED（--camera zedsdk|zed），TCP 7790
sonic/               SONIC 的 commit 号、v1.1 模型下载 + 哈希校验、站立参考动作、sim 专用 patch、手动安装说明
tasks/               agent 任务描述：trash.md（桌上的罐子扔进垃圾桶）；文件名就是 session 的任务 id
r1/                  R1 的迁移计划（TRANSFER_PLAN.md）、sim 和诊断脚本（只在 sim 用）；R1 的 robot service 在 rax/services/r1sonic/
experience/          sim 导出的经验：puppy_box（G1）、puppy_box_r1（R1）
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
# Jetson 上（登录后一行一行敲：ros:foxy(1) noetic(2) 的提示会吞掉粘贴进去的后续命令）
ssh unitree@192.168.123.164
sudo /unitree/sbin/mscli stopservice video_hub_pc4   # 宇树的 video_hub 占着 RealSense 的彩色流
cd ~/jetson_camera && ~/miniconda3/envs/teleop/bin/python3 camera_server.py --camera realsense --port 7790
```

Jetson 上**不要** pip 装 pyrealsense2：系统 `python3` 是 3.8，`/usr/lib/python3/dist-packages/pyrealsense2`
里只有 cp310/cp312 的 `.so`，而且 Jetson 不通外网。用 `teleop` 这个 conda 环境（Python 3.10 +
pyrealsense2 2.57.6）；JPEG 由 cv2 编码，Pillow 用不到。`~/.g1_up.env` 的 `JETSON_CAMERA_CMD`
已指向这个解释器。

`video_hub_pc4` 不停的话相机只报 `VIDIOC_S_FMT ... Device or resource busy`（只有彩色流冲突，深度和
红外能开）。`stopservice` 只停当前这次（重启后 `enable` 又回到 1），所以**每次机器人上电后都要再 stop 一次**；停掉期间宇树 App 看不到头部相机，
**不影响电机、SONIC 和状态流**。恢复：`sudo /unitree/sbin/mscli startservice video_hub_pc4`。

看到 `serving realsense on port 7790`、之后每 5 秒一行 `frames sent N`，Ctrl-C 退出。没有客户端连上来时 `N` 一直是 0，这是正常的：`serving` 这行就说明相机已经打开。这个服务**只读相机、不碰电机**。
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

**最少要敲的（必做）：**
```bash
cd ~/agentics_multirobot && git pull
bash rax/scripts/import_experience.sh experience/puppy_box
cd rax && bash scripts/g1_up.sh agent --task-file ../tasks/puppy_box.md --experience ../experience/puppy_box/workspace
```
下面逐步说明；标【可选】的可以不做。

**第 1 步：更新代码和经验【必做，每次】**
```bash
cd ~/agentics_multirobot && git pull                          # 要拉到最新：导入脚本修过两次（全新主机技能库为空；之前跑过 session 的主机导入报 unrelated histories）
bash rax/scripts/import_experience.sh experience/puppy_box    # 第一次：建 rax/runtime/skill-library/g1sonic；以后：合并，不丢真机学到的技能
```
应该打印 cloned / merged，`git -C rax/runtime/skill-library/g1sonic log --oneline | head -1` 至少是 `604b743`。

**第 2 步：单独做一次检查【可选】**
```bash
cd ~/agentics_multirobot/rax && bash scripts/g1_up.sh check --quick-check
```
不做也行：第 4 步的 `g1_up.sh agent` 在没有 `g1` tmux 时会自己起 camera / SONIC / console / dashboard，站起来在那里做。
做的话：SONIC 窗格 `Y`，等 `Init Done`；**脚先着地承重再按 `s`**，按完几秒内把带子放松（挂着按 `s` 会乱甩）；`work` 窗格
status/hold/arms 三步通过（约 1 分钟）。隔了很久第一次开机、或刚升级过，建议做；也可以改成第 4 步命令末尾加 `--quick-check`，
效果一样（先检查再自动起 agent）。

**第 3 步：摆场景【必做；目测即可，不用量】**
- 桌子（0.68–0.78 m 高都行）放在机器人前面，骨盆离桌沿大约 0.3 m（手垂下碰不到桌子）。
- 小狗侧躺、腿朝机器人；透明塑料箱在小狗旁边，间隔几厘米到 12 cm。**第一次先摆"箱子在小狗右边"**（sim 里最稳，5/5），
  再试箱子在左边（右手跨身体）、箱子在小狗后面。
- agent 会自己用深度测桌高和桌沿距离，然后通过 `request_operator` 让你把桌子推 "closer"/"further"，推到桌沿离骨盆约 0.2 m。
  机器人不走路，靠人推桌子。

**第 4 步：启动 agent【必做】**
```bash
cd ~/agentics_multirobot/rax
bash scripts/g1_up.sh agent --task-file ../tasks/puppy_box.md --experience ../experience/puppy_box/workspace
```
- **跳过了第 2 步：** 先 `tmux kill-session -t g1` 关掉上次留下的 tmux（没有就不用）。然后在这个 tmux 里站起来：SONIC 窗格 `Y`，
  等 `Init Done`；console 窗格**脚先着地承重再按 `s`**，几秒内放松带子。
- **做了第 2 步（tmux 还在）：** 只替换 `console` 和 `work` 两个窗格，**不要再按 `s`**（机器人已经站着）。
- `console` 窗格：`SPACE` 给钥匙/暂停，`r` 回复 agent 的请求，`o` 软停（只在带子挂好时按，机器人会软掉）。`work` 窗格就是
  Claude 对话，可以直接打字提示它。
- 可加的参数【都可选】：
  - `--quick-check`：先跑 1 分钟检查（同第 2 步）再起 agent。
  - `--demo ../tasks/demos/puppy_box/1_start.jpg ... --demo-kind completion_sequence`：给它看示教照片（见第 5 节）。
    带了 sim 经验后不需要：`run.py` 和 tips 已经比照片具体。
  - `--experience`：**本节的关键，必带**；去掉就是从零开始。

agent 一开始会：读 `TRANSFER.md` 和 handoff → 测桌高、桌沿、可达距离等参数 → 请你推桌子 → 按 `run.py` 的阶段跑
（观察 → 接近 → 抓取+抬起 → 搬运 → 放下 → 收手），每个阶段前有 LOOK 检查点。

**看什么、什么时候停：**
- 每次请求先看清它要做什么再按 `r`；左手应一直收在身后上方，碰到小狗不算成功。
- 机器人在 planner 下不停踏步、倾斜明显、手压着箱子往下推：按 `SPACE` 暂停，必要时 `o`。
- 任务失败可以接受；**摔倒不行**，宁可早停。
- 一次成功后它会请你重新摆（reset），换一种布局再试。

**第 5 步：跑完后把真机经验带回来【建议做；不做的话下一次真机和 sim 都学不到这次的东西】**
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

---

## 8. R1（Unitree R1 + PiPER 两指夹爪）：从 sim 迁到真机

**现状（2026-11-06）：bridge 在 sim 里改好了，可以开始真机测试（先做完阶段 A，再按 8.4 一步步来），完整任务还没在真机上跑过。**
RAX sim 里 13 个 session：agent 一共把小狗放进箱子约 21 次，0 次摔倒；从远处走过来完成全程是 2/10，经验在 `experience/puppy_box_r1/`。
连接真机控制栈的 bridge（`rax/services/r1sonic/bridge_real.py`）已经在 sim 里接**真正的 R1 部署程序**测过（8.2）：站立、伸手、走路、头部、
整条搬运链都通，右手腕加 0.3 kg / 0.6 kg 模拟夹爪和小狗的重量也各 4/4。**没测过的只有真机能回答**：真 PiPER、夹真小狗、R1 的相机。
RAX 那一半（launcher、上锁、operator 请求、经验导入导出）和 G1 相同，不同的地方都写在下面。

### 8.0 先看这里：R1 按什么顺序跑

和 G1 对照：G1 的第 1–4 节是装机和让机器人在控制器下站稳，第 5 节开始跑 agent。R1 一样分阶段，但中间多一块
**bridge**（RAX 和 R1 部署栈之间的翻译层）。bridge 在 sim 里已经测通（阶段 B），所以阶段 A 做完就可以做 C。

| 阶段 | 做什么 | 谁做 | 现在能做吗 | 对应 G1 |
|---|---|---|---|---|
| A | 装 R1 部署栈，在 sim 里测，再在真机上分阶段测。只用键盘或 PICO 遥操作，不碰 RAX | 你 | **能，现在就做** | 第 1–4 节 |
| B | 改 bridge（8.2），在 sim 里接真正的 R1 部署栈测通 | Claude | **sim 里做完了**。真机才能回答的三项留给 C：真 PiPER、夹真小狗、相机 | G1 不需要，`g1_up.sh` 已接好 |
| C | 真机第一天：不开 agent，逐项检查（8.4） | 你 + Claude | **能，A 通过后就做** | 第 3 节标定 |
| D | 带 sim 经验开 RAX agent 跑 puppy_box_r1（8.5、8.6） | 你当 operator | C 做完之后 | 第 5.5 节 |

**阶段 A 要敲的命令**（摘自 Hugging Face `Jay-Ye/SONIC_R1` 的 `TESTING.md`，版本号和细节以那份为准）：

```bash
# A1. 下载并校验 R1 模型（有外网时做）
python3 -m venv ~/.venvs/sonic-r1-check && source ~/.venvs/sonic-r1-check/bin/activate
python -m pip install huggingface_hub numpy onnx onnxruntime
hf download Jay-Ye/SONIC_R1 --local-dir ~/models/SONIC_R1
export SONIC_R1_BUNDLE=~/models/SONIC_R1
python "$SONIC_R1_BUNDLE/integration/verify_bundle.py"

# A2. MA-Teleop 和部署子模块切到 TESTING.md 第 2 节写的固定版本，装 R1 预设，编译
export MA_TELEOP=~/workspace/MA-Teleop
cd "$MA_TELEOP" && git submodule update --init submodules/GR00T-WholeBodyControl
git -C submodules/GR00T-WholeBodyControl fetch origin r1-deploy-heading-fix
git -C submodules/GR00T-WholeBodyControl switch --detach 5de0e1316837fec5204dd8a824e7b664a092b353
python "$SONIC_R1_BUNDLE/integration/install_ma_teleop.py" --ma-root "$MA_TELEOP"
cd "$MA_TELEOP/submodules/GR00T-WholeBodyControl/gear_sonic_deploy" && source scripts/setup_env.sh && just build

# A3. sim 测试：终端 1 开实验室打过 R1 补丁的 unitree_mujoco（config.py 里 ROBOT="r1"、DOMAIN_ID=42、INTERFACE="lo"）
cd /path/to/R1-patched-unitree_mujoco/simulate_python && python unitree_mujoco.py
#     终端 2：键盘控制回放参考动作。] 请求控制，T 播放，O 软停；R1 上不要按 Enter 选 planner 模式
cd "$MA_TELEOP/submodules/GR00T-WholeBodyControl/gear_sonic_deploy"
SONIC_DDS_DOMAIN=42 ./deploy.sh sim --robot r1 --input-type keyboard \
  --cp policy/r1/50k/model_step_050000 --obs-config policy/r1/50k/observation_config.yaml \
  --motion-data reference/r1_example/ --zmq-port 6556 --zmq-out-port 6557
```

A4 是真机分阶段测试，按 `TESTING.md` 第 6–7 节：先在目标工作站上查 50 Hz 时序；真机上先不启动策略，核对接口、关节方向和急停；
再在吊架上做中立站姿、单臂小幅动作。真机部署命令见下面 8.4 的终端 A（网卡名按实际改，真机用 DDS domain 0，不能加
`--disable-crc-check`）。

**阶段 A 通过的标准：** 吊架上站姿稳定，单臂小幅动作正常，软件停和物理急停都试过有效。做完直接进 8.4。

### 8.1 带过去的和不带的

| 路径 | 是什么 |
|---|---|
| `experience/puppy_box_r1/` | sim 经验：技能库 `skill-library-r1sonic.bundle`、`workspace/memory/`（TIPS 带 `[strategy]`/`[param]`/`[sim-only]` 标签，PARAMS 是 sim 测的数，到真机要重测）、`workspace/handoff/README.md` |
| `tasks/puppy_box_r1.md` | 真机任务文件：人当 operator，5 次 reset |
| `rax/services/r1sonic/` | R1 的 robot service：backend、给 agent 的工具（含 `r1_leg_lift`、`r1_carry_release` 两个原语）、`bridge_real.py` |
| `r1/TRANSFER_PLAN.md` | 迁移计划：什么带过去、真机第一天的顺序、成功标准 |

**只在 sim 用，不要带到真机：** `r1/sim/`、`rax/scripts/r1_sim_operator.py`、`rax/scripts/r1_sim_options.json`、
`tasks/puppy_box_r1_sim.md`、`r1/tools/diag/carry_chain_test.py`。最后这个脚本用 sim 的上帝视角指令摆场景，真机上跑不了。

### 8.2 上真机前必须先改完的（大部分没有机器人也能做）

R1 的 SONIC 策略**没有 locomotion planner**：`deploy.sh --robot r1` 默认 `--no-planner`，部署栈只吃 `pose` topic 上的流式参考动作。
`rax/services/r1sonic/bridge_real.py` 已经按这个改写（2026-11-06），并在 sim 里接**真正的 R1 部署栈**测过：

| 项 | 状态 |
|---|---|
| 手臂指令 | **已完成，已测**。`pose` topic 协议 1 + `vr_position`/`vr_orientation`，部署栈进入 teleop 模式（encoder mode 1）。RAX backend 经 bridge 伸右手到三个桌前目标，lockstep sim 里 5/5 次、15/15 个目标全部到位，误差 0.3–1.8 cm，倾角 < 2° |
| 关节顺序 | **已确认**。发出去的 `joint_pos` 是 IsaacLab 顺序，遥测的 `body_q` 是 MuJoCo 顺序 |
| 端口 | **已完成**。默认改成实验室的 6556/6557 |
| 走路（`r1_walk`，搬运时那两三小步） | **已完成，已在真部署栈 sim 里测**（自由运行，和真机一样）。之前的摔倒是实验室 sim 的 bug，不是部署也不是步态：`r1_sonic_sim.py` 用 `mj_objectVelocity(mjOBJ_BODY)` 算陀螺仪，那是 R1 骨盆的惯性主轴坐标系，y/z 轴对调，站着转得慢看不出来，走起来策略收到错的角速度就摔。`r1/deploy_sim/run.sh` 在拷贝里改成 `mjOBJ_XBODY`（实验室原文件没动；真机 IMU 不受影响）。证据：同一个 C++ 部署的输入输出和 teleop.onnx 逐 tick 一致（误差 1e-8）；把同样的错误陀螺仪放进我们自己的 Python 循环也在 4.7 s 摔倒。修好后：bridge 的 `walk` 流式发 RAX sim 的参数化步态（每 tick 50 帧前瞻），走 0.5 / 0.3 / −0.2 / 3×0.07 m 和原地转 ±30°，2/2 轮 8/8 次不倒，倾角 < 2°，转角误差 ≈ 1°，原地转会漂 7–12 cm。距离靠腿部运动学里程计估计，**偏少约 20–30%**（0.51 m 估计 vs 0.62 m 真实），所以修正步会多走一点；走完要重新观察。**搬运链也测了**（自由运行，夹爪用替身，手里没重量）：伸手到夹取姿态 → 夹紧 → 抬高 15 cm → backend `r1_carry_release`（默认每步 0.07 m）→ 放低 → 张开。最初 0.07 m 的短步只会原地挪 0–4 cm，因为步态太短、幅度起不来；现在短步用完整长度的步态、按距离缩小髋摆幅，最短一步 0.14 m（请求更短会返回 `note`），修正步只往前不往回。之后 22 次里 20 次走到 0.21 m 目标并放下（真实 0.11–0.31 m），0 次摔倒，倾角 ≤ 2.4°；1 次几乎没走动、被 backend 判为卡住安全停下；1 次 bridge 进程崩溃（SIGFPE，加锁前约 25 次运行出现 2 次，加锁并开 faulthandler 后 17 次没再出现，原因未证实）。长距离会走过头：0.5 m 实走 0.6–0.9 m，靠相机重新观察校正 |
| 相机（`r1_observe`） | **软件通路已接，用测试视频测过**。bridge 加 `--camera-port <端口> --camera-intrinsics fx,fy,cx,cy`，从实验室的 `MA-Teleop tools/r1/publish_camera.py` 取最新帧（超过 0.5 s 的旧帧会拒绝），backend 生成完整观测（图像、内参、由运动学模型算出的相机位姿）。还缺：R1 上用哪个相机（`publish_camera.py --source` 填什么；SONIC_R1 指南写着方案没定）、它的标定内参、真实安装位置是否和模型一致 |
| 夹爪（右手） | **已接，用协议一致的替身测过，真机未测**。bridge 加 `--piper-right "<命令>"`，运行实验室的 `MA-Teleop tools/r1/piper_can_bridge.py --serve`（本机或 ssh 到机器人）。指令宽度 < 3.5 cm 就是夹紧（2 mm），否则张开（70 mm），实测宽度来自夹爪反馈；夹爪任何报错都会让 bridge 的 `state` 报错，session 停下。端到端测过：backend 夹紧 → 实测 0.032 m（碰到东西），张开 → 0.07 m。左手夹爪还没有验证过的 USB 序列号 |
| 头部俯仰 | **已接，已测**。bridge 每帧带 `r1_head`（俯仰、偏航、单调时钟时间戳），部署要加 `--r1-head-tracking --r1-head-pitch-range 0.5` 才会执行（默认关，范围默认 ±0.35）。sim 里 0.5 / 0.3 / −0.2 / 0 rad 都跟到 ±0.04 rad，0.7 被限到 0.5。backend 的 `head_pitch`（默认 0.5）随第一个动作下发（实测伸手后 0.54 rad），和我们 sim 一样：第一次动作之前的观察，头还是平视 |
| 里程计 | 遥测里没有底盘位置。bridge 用支撑脚运动学 + IMU 估计（`state` 里的 `odometry_xy_m`），偏少约 20–30%，真机要用相机重新观察来校正 |

**不走路的版本可行性（同一套真部署栈 lockstep sim，2026-11-06）：** 站在一个位置不动，backend 依次做 夹取姿态 (0.30, -0.21, -0.02) → 抬高 15 cm →
右前方放置点 (0.30, -0.38, 0.10) → 正前偏右放置点 (0.35, -0.28, 0.12) → 收回，3/3 次全部完成，每步误差 ≤ 2.1 cm，倾角 ≤ 2.4°（坐标是骨盆坐标系，单位 m；
没有带小狗的重量）。也就是说，只要箱子放在机器人右前方约 40 cm 内，不走路也够得到。

**时序：** 之前自由运行时伸手 7 次摔 5 次，当时以为是延迟，其实也是上面那个陀螺仪 bug。修好后自由运行伸手 9/9 轮、27/27 个目标全部到位。
上真机前仍要按 `TESTING.md` 第 6 节查 50 Hz 时序。

**自己复现：**

```bash
# 1. 在有 r1_sim_cpu 和 unitree_sdk2py 的容器里起 R1 DDS sim + 真部署栈（DDS 域 42，回环网卡，端口 6556/6557）
docker exec sonic-sim2sim bash r1/deploy_sim/run.sh start        # 等 deploy.out 里出现 "Init Done"，约 30 s
# 2. 起 bridge，跑端到端测试（backend -> bridge -> 部署栈 -> sim）
cd rax && PYTHONPATH=. ~/envs/rax-g1/bin/python -m services.r1sonic.bridge_real --port 5594 &
~/envs/rax-g1/bin/python ../r1/deploy_sim/bridge_e2e.py
docker exec sonic-sim2sim bash r1/deploy_sim/run.sh stop
```

不要用 `r1/sim/` 里我们自己写的 R1 sim 来验证 bridge：它不是真部署栈，测不出上面这些问题。

### 8.3 主机安装（只做一次）

1. RAX 部分和 G1 一样：第 1 节的 `bash setup_rax_host.sh`（建 `~/envs/rax-g1` 和 `~/envs/rax-agent`），第 1.5 节 c 装 Claude Code 并登录。
   不需要 `--with-sonic`，那是 G1 的 SONIC。
2. R1 的 SONIC 部署栈：按 Hugging Face 仓库 `Jay-Ye/SONIC_R1` 里的 `TESTING.md` 第 1–6 节做：下载并校验模型、
   MA-Teleop 和部署子模块切到固定版本、编译、sim 里测、目标工作站的 50 Hz 时序检查。
3. robot service 的设置文件 `~/r1_real_options.json`（bridge 的地址和端口，头部俯仰）：
   ```json
   {"stack_host": "127.0.0.1", "stack_port": 5591, "head_pitch": 0.5}
   ```
4. session 目录放到 `rax/` 外面。launcher 会把自己的 `rax/` 目录设成 agent 不可读，默认的 `rax/runtime/` 在里面，
   agent 的工作目录会变成只读（R1 在 sim 里前 4 个 session 就是这样坏的）。每个终端先设：
   ```bash
   export RAX_ARTIFACT_ROOT=$HOME/rax-runtime
   ```
   G1 用的是同一个 launcher，跑 G1 时也建议这样设。

### 8.4 真机第一天：不开 agent，逐项检查（你要做的）

前提：阶段 A 通过（吊架上中立站姿、小幅手臂动作、软件停和物理急停都试过）。全程挂吊架或保护绳，有人拿急停。
**PICO 的 manager 和 bridge 不要同时开**：两个都往部署栈的同一个端口发指令。

```bash
# 终端 A：R1 部署栈。不要用 deploy.sh：它不转发 --r1-head-tracking，认不出的参数会被当成网卡名。
#   这里直接用 deploy.sh 最后执行的那条命令，多加头部跟踪（不加头不会低头，相机只能平视）。enp2s0 换成实际网卡。
#   bridge 的 sim 测试都用 100k 模型（exports/100000 + observation_config_sonic_r1_v1_1）。TESTING.md 的预设选 50k（评测过的那个），
#   用 50k 就把 M 和 --obs-config 换成 policy/r1/50k/...，但 50k 没有接 bridge 测过。
cd "$MA_TELEOP/submodules/GR00T-WholeBodyControl/gear_sonic_deploy" && source scripts/setup_env.sh
M=~/models/SONIC_R1/exports/100000/model/exported      # A1 下载的包里同时有 50k 和 100k
SONIC_DDS_DOMAIN=0 just run r1_deploy_onnx_ref enp2s0 "$M/model_step_100000_decoder.onnx" reference/r1_example/ \
  --obs-config policy/r1/observation_config_sonic_r1_v1_1.yaml --encoder-file "$M/model_step_100000_encoder.onnx" \
  --input-type zmq_manager --output-type all --zmq-host localhost --zmq-port 6556 --zmq-out-port 6557 \
  --r1-head-tracking --r1-head-pitch-range 0.5

# 终端 B：bridge。--piper-right 是右手 PiPER（实验室的 piper_can_bridge.py，按 USB 序列号找适配器；
#   机器人上先 sudo ip link set <can口> up type can bitrate 1000000）。相机定下来之前先不加 --camera-*。
cd ~/agentics_multirobot/rax
PYTHONPATH=. ~/envs/rax-g1/bin/python -m services.r1sonic.bridge_real --port 5591 --stack-in 6556 --stack-out 6557 \
  --piper-right "ssh -o BatchMode=yes unitree@192.168.123.164 python3 -u <MA-Teleop>/tools/r1/piper_can_bridge.py --serve"

# 相机定下来以后（终端 C + bridge 加两个参数）：
#   python3 <MA-Teleop>/tools/r1/publish_camera.py --source <R1 的相机> --port 5600
#   bridge 加 --camera-port 5600 --camera-intrinsics fx,fy,cx,cy   （该分辨率下的标定内参）
```

**按顺序做，每步把结果发给 Claude，没过就停在那一步：**

| 步 | 做什么 | 看什么 / 过的标准 |
|---|---|---|
| 1 | 只测夹爪：机器人不上电或不动，单独跑 `piper_can_bridge.py --serve`，再通过 bridge 发张开、夹紧 | 张开读数约 70 mm，夹紧空夹约 2 mm，夹住小狗的腿读数停在腿的粗细；没有报错 |
| 2 | 开终端 A、B，只站立 | 倾角稳定在 2° 以内，不晃。带着两个 PiPER 的重量，这一步最能暴露问题（sim 里手腕加 0.6 kg 没问题，但左手的重量没模拟） |
| 3 | 头部：发 `head` 0.5，再回 0 | 头低下去又抬起来，不影响站姿 |
| 4 | 右手伸到桌前 3–5 个点（`r1_move_hand`） | 用尺子量指尖和目标的差。差几厘米是夹爪指尖偏移没标，把数告诉 Claude 改模型 |
| 5 | 不拿东西走 0.14 m 前进、0.14 m 后退、原地转 30°，各 3 次 | 不倒；量实际距离和侧移（sim 里短步误差 1–3 cm，长距离会走过头） |
| 6 | **夹爪能不能夹住小狗**（sim 里最不可信的一项）：夹住伸出桌沿的那条腿，抬 10 cm，停 60 s，再走两步 | 宽度读数不变、腿不滑出。夹不住就先换夹法（夹在爪子后面、双手），别的都往后放 |
| 7 | 相机（定了型号以后）：在 0.3–0.4 m 处用卷尺核对算出来的爪子位置 | 误差 2 cm 以内 |
| 8 | 手工调 `r1_leg_lift`（抬多高）和 `r1_carry_release`（步长、前伸、放低） | 记成真机值，写进 `experience/puppy_box_r1/workspace/memory/` 的 PARAMS |

步 1–6 不需要相机，现在就能做。**需要你告诉 Claude 的**：R1 用哪个相机（`publish_camera.py --source` 填什么）和它的标定内参；
左手夹爪的 USB 序列号（目前只有右手的）。

### 8.5 真机第二天：带 sim 经验开 RAX session

终端 A、B 照 8.4 开着，机器人站着。

```bash
cd ~/agentics_multirobot && git pull
export RAX_ARTIFACT_ROOT=$HOME/rax-runtime PYTHONPATH=$PWD/rax
bash rax/scripts/import_experience.sh experience/puppy_box_r1          # 装 R1 技能库（第一次是 clone，以后是合并）
CFG=$(~/envs/rax-g1/bin/python rax/scripts/configure_session.py --task puppy_box_r1 \
  --task-file tasks/puppy_box_r1.md --robot r1sonic --backend-options ~/r1_real_options.json \
  --agent-python ~/envs/rax-agent/bin/python --experience experience/puppy_box_r1/workspace \
  --environment real --session-hours 2 | tail -1)
~/envs/rax-g1/bin/python rax/harness/claude_cli/launcher.py --config "$CFG" --no-gemini
```

- `--experience` 必须指向导出目录里的 `workspace/`，指向上一层会什么都不拷。开 session 时 `[sim-only]` 的行会被移走，
  并生成 `memory/TRANSFER.md`，列出要在真机上重测的参数。
- 上面的 launcher 是交互式的，可以直接打字提示 agent。sim 里用的是 `--non-interactive`（无人值守），真机上不建议。

**operator 怎么回 agent：**
- 给钥匙、暂停：另开终端 `cd ~/agentics_multirobot/rax && RAX_ARTIFACT_ROOT=$HOME/rax-runtime ~/envs/rax-g1/bin/python scripts/g1_console.py`，
  只用 `SPACE`（给钥匙/暂停）和 `r`（回复 "done"，reset 完用）。**不要按 `s` 和 `o`**：这两个是给 G1 robot service 的，R1 的不读。
  R1 要停机用终端 A 的部署栈或遥控器、急停。
- 要文字回答的请求（例如"小狗在箱子里吗"）：
  ```bash
  cd ~/agentics_multirobot/rax
  S=$(ls -td $HOME/rax-runtime/sessions/*_puppy_box_r1_* | head -1)
  ~/envs/rax-g1/bin/python -c "import sys; from services.controller import operator as o; [print(r['id'], r['kind'], r['message']) for r in o.pending(sys.argv[1])]" "$S"
  ~/envs/rax-g1/bin/python -c "import sys; from services.controller import operator as o; o.reply(sys.argv[1], sys.argv[2], sys.argv[3])" "$S" <请求id> "在箱子里，靠前壁"
  ```
  请求大约 4–5 分钟后过期，尽量一分钟内回。像人一样说看到了什么，不要报坐标。
- 任务文件里 operator 要做的事：摆场景（小狗侧躺，一条腿伸出桌沿约 6 cm）、reset（最多 5 次）、回答问题。

### 8.6 跑完：把真机经验导出来

```bash
cd ~/agentics_multirobot
S=$(ls -td $HOME/rax-runtime/sessions/*_puppy_box_r1_* | head -1)
bash rax/scripts/export_experience.sh "$S" experience/puppy_box_r1_real
git add experience/puppy_box_r1_real && git commit -m "experience: first real R1 puppy_box session" && git push
```

**算迁移成功**（`r1/TRANSFER_PLAN.md` 第 5 节）：一个真机 session 里 5 次 reset 至少 3 次从远处走过来把小狗放进箱子，
不摔倒，导出的 PARAMS 里抬起和搬运的参数是真机值。
