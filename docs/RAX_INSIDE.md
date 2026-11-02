# RAX 真机 agent：打开黑盒

读完这份你应该能回答：agent 启动后第一句话收到什么、它按什么顺序做事、哪些是写死的、哪些是它临场决定的、
它的每一次调用在到机器人之前经过哪些关卡、它做不到什么、你在旁边能看到什么。所有内容来自代码和 2026-10-29 在
sim 里真实跑过的一次 trash session（`sessions/20261029T170144Z_trash_e5abb4`），不是设想。

---

## 1. 五个进程和它们之间的线

```
 你                     RAX 主机                                            机器人网络 192.168.123.x
 ┌──────────┐   ┌─────────────────────────────────────────────────┐   ┌──────────────────────┐
 │ console  │──▶│ robot service (services/g1sonic/backend.py)      │──▶│ SONIC deploy          │──DDS──▶ G1 电机
 │ s/space/r│   │  · 安检 + 50 Hz 参考帧流  ──pose tcp 5556──────▶│   │ (g1_deploy_onnx_ref)  │
 └──────────┘   │  · 本体状态           ◀──state tcp 5557─────────│◀──│                       │◀─DDS─── 关节/IMU
 ┌──────────┐   │  · 相机帧             ◀──tcp 7790───────────────│◀──│ Jetson camera_server  │◀─────── RealSense
 │ agent 终端│──▶│  · MCP unix socket ◀──────────────┐             │   └──────────────────────┘
 │ (打字)   │   └───────────────────────────────────┼─────────────┘
 └──────────┘   ┌───────────────────────────────────┼─────────────┐
 ┌──────────┐   │ Claude Code（bubblewrap 沙箱里）     │             │
 │ dashboard│◀──│  工作目录 agent-workspace/，只能通过 MCP 工具碰机器人 │
 │ :8770    │   └─────────────────────────────────────────────────┘
 └──────────┘
```

- **robot service** 是唯一会向 SONIC 发数据的进程。它把 agent 的"语义 40 维姿势帧"检查、限幅后以 50 Hz 流给 SONIC；SONIC 的策略网络负责平衡和落脚，电机命令由 SONIC 发。
- **Claude Code** 就是你平时用的那个 CLI，以 `--permission-mode dontAsk` + 沙箱启动。它和机器人之间只有一条线：`RAX_MCP_SOCKET` 这个 unix socket，另一头是 robot service。沙箱里本机回环端口是关的，所以它连不上 5556/5557/7790；SONIC 目录、rax 源码、`~/.ssh`、`~/.config/rax` 对它不可读（`RAX_PROTECTED_PATHS`）。
- **console**（`g1_console.py`）、**dashboard**（8770 端口）、**agent 终端**是你的三个入口，分工在第 6 节。

`g1_up.sh` 把这些开在一个 tmux 里的五个窗格：camera、sonic、console、work（agent）、dashboard。

---

## 2. 一个 session 从建立到结束

| 步骤 | 谁做 | 发生了什么 |
|---|---|---|
| `configure_session.py --task-file task.md` | 你（`g1_up.sh agent` 代劳） | 建 `sessions/<时间>_<任务>_<id>/`，写 `configuration.json`（机器人、观测档、预算：默认 4 小时、4000 次模型调用、10 美元）、`operator-input.json`；你的 task.md 复制为 `agent-workspace/TASK.md` |
| robot service 启动 | `g1_up.sh` | 连 SONIC 的 5557 读状态、绑 5556 准备发帧、连 Jetson 相机、开 MCP socket；写 `state.json`、`operator/stack_status.json` 供 console 和 dashboard 读 |
| `start_agent.sh` | `g1_up.sh` | launcher 生成 Claude Code 的 settings（允许：Read/Edit 只限 workspace、Bash（沙箱内）、Glob、Grep、WebFetch、MCP 工具；禁止：WebSearch、定时、跨会话消息；拒绝读保护路径），把 `AGENTS.md`、`MCP_SESSION.md`、`START_PROMPT.md`、技能目录 `.agents/skills/` 写进 workspace，`CLAUDE.md` 只有一行 `@AGENTS.md`，然后启动 `claude` |
| 第一句话 | launcher → agent | 原文：**"Read AGENTS.md, then MCP_SESSION.md and TASK.md, and call `status`. Follow the AGENTS.md workflow."** 这是它收到的全部开场指令 |
| 任务中 | agent ↔ robot service ↔ 你 | 见第 3、4、5 节 |
| 结束 | agent 自己结束回合 / 预算用完 / 你 Ctrl-C | `agent-lifecycle.json` 记 `exited`；`memory/`、`handoff/`、`skill-library/` 留在磁盘上 |

**中断恢复**：同一 workspace 再启动时第一句话换成："This session was interrupted and has now been resumed… reread AGENTS.md… then reread memory/PROGRESS.md, call status, and continue from the live state. Do not repeat finished work."

---

## 3. 它读到的三份文件（它的"说明书"）

**`AGENTS.md`**（harness 生成，你不用改）六节：

1. *Safety comes first*：每个动作物理且不可撤销，小步、慢、动后看；拒绝是一堵墙，不许绕，不许通过工具以外的路径碰机器人、控制器、相机；工具报错先读完整个 `reason` 和 `robot_state`；停过之后先观察再动；拿不准就不动并写进 `memory/PROGRESS.md`。再加 G1 专用规矩（钥匙、每次以可保持姿势结尾、从测得关节出发、手按比例合）。
2. *Start here*：读 `TASK.md` → 读已有的 `memory/PROGRESS.md`、`memory/INDEX.md` → 调 `status` → 读 `skill-library/INDEX.md` 看前人留下的技能。
3. *Workflow*（四步，第 4 节详述）。
4. *Tools and code*：普通 Python 里 `from api.runtime import Context; ctx.call(tool, **args)` 调的是同一套工具；直接调用和脚本共用一个机器人，不许两个脚本同时动它。
5. *Workspace and artifacts*：`code/`、`memory/`、`handoff/`、`skill-library/`、`output/` 各放什么。
6. *Notes and compaction*：对话上下文会被压缩，所以持续写 `memory/PROGRESS.md`，压缩后靠它接着干。

**`MCP_SESSION.md`**：一段话，列出本 session 的工具名（`g1_crouch, g1_execute_motion_plan, g1_free_space_move, g1_hand_shape, g1_hold, g1_observe, g1_pixel_to_position, g1_status, g1_step, g1_walk, request_operator, skill_library_merge, status`），参数和限制看工具自带的 schema。

**`TASK.md`**：你写的。它把这份当事实。

它还被指向这些技能文件（`.agents/skills/`），G1 相关的：`control/g1sonic/robot-interface`（40 维帧的布局、钥匙、拒绝语义）、`control/g1sonic/controller-field-guide`（SONIC 上怎么走、转、伸手，碰到东西看什么反馈）、`control/g1sonic/controller-tracking`、`control/g1sonic/free-space-move`（cuRobo 规划）、`perception/depth-g1sonic/object-localization`（像素→三维）、`task/manual-robot-control`、`task/autonomous-control`、`task/continual-learning`、`auto-research/transfer-handoff`，以及一组方法论技能（小步验证、匹配精度、从事故学习等）。

---

## 4. 写死的 vs 它临场决定的

| 写死的（代码 / 说明书） | 它临场决定的（模型） |
|---|---|
| 开场指令、读文件的顺序、四步 workflow 的框架 | 每一步具体做什么：先看哪里、点哪个像素、用哪只手、帧怎么填、走不走、什么时候问人 |
| 11 个机器人工具及其参数范围 | 工具调用的顺序和次数 |
| robot service 的安检和限幅（第 5 节的数字） | 动作的大小（在限幅以内）、速度（帧数） |
| 拒绝之后不许绕、停止之后不许自恢复 | 被拒后怎么改方案 |
| 四步必须留下的产物：`memory/PROGRESS.md`、`memory/RUNS.md`、`handoff/README.md`、skill-library | 写什么内容 |
| 预算：时长、模型调用数、费用 | 怎么分配 |

四步 workflow 原文大意：
1. **先自己把任务做一遍（手动）**：按 `manual-robot-control`，逐条对照 `TASK.md` 的条件。
2. **把动作写成代码，眼睛留在回路里**：`code/<项目>/` 下写可以一步步跑的控制器，每步重新定位、检查前提和结果，场景不对就停下来带图报告；在关键点（碰之前、松手之前、东西可能被碰动之后）自己看图决定，不盲跑。
3. **真实场景反复测**：`request_operator(reset)` 请你复位，跑控制器，看报告和关键帧，改代码，换布局再来；每次记 `memory/RUNS.md`。
4. **教给下一个 session**：做通的流程进 skill-library（先 `draft`，验证后升级），`skill_library_merge` 合并；`handoff/README.md` 写"换个布局怎么规划每个子任务"。

今天 sim 里的 session 只走到第 1 步的末尾（抓起、转身、搬运中掉了），因为容器里沙箱坏了没有 Bash，第 2 步写不了代码。真机主机装了 bubblewrap + socat，第 2 到 4 步可以走。

---

## 5. 一次 `g1_step` 从 agent 到电机经过的关卡

agent 发 `g1_step(actions=[40 维 × N 行])`，robot service 按顺序做：

1. **形状**：N × 40，N ≤ 100（50 Hz，最多 2 秒）。
2. **关节限位**：每个关节在 `[下限+0.05, 上限−0.05] rad` 之内，否则拒绝并列出越界的关节。
3. **姿态有效**：第 3–9 维的 rot6d 必须是合法旋转。
4. **第一行不能跳**：和机器人**现在保持的姿势**比，任一关节差 ≤ 0.35 rad，否则拒绝（防"从零开始"把所有关节拉到零位）。
5. **相邻行不能跳**：相邻两行任一关节差 ≤ 0.15 rad（7.5 rad/s）。
6. **结尾可保持**：超过 5 行的调用，最后 5 行必须相同（机器人会一直跟踪最后一帧）。
7. **根旋转限速**：每拍根姿态变化 > 26° 的被 slerp 压下去。
8. **钥匙**：`operator/armed.json` 不是 armed → 拒绝，原因写"未上钥匙 / 操作员暂停"。
9. **状态新鲜**：SONIC 的状态帧超过 0.2 s 没来 → 视为断链，拒绝或中途停止。
10. 通过后以 50 Hz 流给 SONIC；流的过程中**倾斜 > 60°** → 告诉 SONIC 阻尼坐下、拔钥匙、记 `fault: fall`，回复里带 `MotionStopped`。
11. 回复 JSON：执行了多少行、跟踪误差、倾斜、测得关节、本体状态（`proprio`）。

拒绝叫 `RejectedBeforeAction`：机器人没动。中途停止叫 `MotionStopped`：机器人停在当前姿势。两种都原样回到 agent 手里，它的规矩是读完整个原因再决定。

`g1_walk` 走的是 SONIC 的规划器模式：service 发速度/朝向命令，SONIC 自己迈步；转向限速 30°/s（45°/s 时腰部 7 rad/s 摔过）；超过 0.5 m 自动拆段；第一次走先左右各扭一下热身；走完等 1.5 s 静止再把保持姿势接回来，最多等 6 s；若 20 s 后还在原地踏步，下一个调用先把朝向扶正、慢走一步，再不行就要求操作员。`carry=true` 时手臂冻住。

**G1 上没有的保护**：没有笛卡尔工作空间盒（RB-Y1 有），手能伸到关节限位允许的任何地方，包括桌面以下；没有自碰撞检查（`g1_free_space_move` 的 cuRobo 规划有，`g1_step` 没有）。所以 `task.md` 里的"别碰桌子"、"抬高再移"是给模型的规矩，不是硬件挡的。

---

## 6. 真实的一次：2026-10-29 sim trash session（32 分钟，74 次工具调用）

时间从 agent 第一次调用算起。每行是 `mcp.jsonl` 里的真实记录。

| 时刻 | 调用 | 它在干什么 / 得到什么 |
|---|---|---|
| 0 s | `status` | 预算：0 次调用、0 美元、上限 10 美元 |
| 20 s | `g1_status` | 已上钥匙（脚本操作员 16 s 时上的）、streamed 模式、倾斜 1° |
| 23 s | `g1_observe` | 第一张彩色+深度图 |
| 26 s | `g1_pixel_to_position` | 点了罐子上下三点和左右两点：罐子在前方 0.51 m、略左；桌沿 0.47 m |
| 112 s | `g1_walk 0.1 m` | 想再近一点。走了 0.1（这是第一次走，带热身） |
| 128–140 s | 定位、`g1_status` | 重新量罐子位置 |
| 152 s | `g1_crouch 0, arms_home` | 想把手臂归位；发现从站立调它不归位 |
| 277 s | `g1_step`（第 1 个手动帧序列） | 抬左臂到罐子上方。每个帧序列它要手算 25–40 行，花 1.5–2 分钟 |
| 453–578 s | `g1_crouch` 两次（lean 0.3 / 0.45） | 试图前倾增加够程；测得骨盆反而后仰 4–8°、后滑 2–3 cm。**放弃这条路** |
| 600 s | `g1_crouch 0` | 站回来 |
| 638–684 s | `g1_walk` 0.1 / 0.08 / 0.12 | 三次短走，分别前进 3.5 / 0 / 0 cm（原地踏步） |
| 855–1281 s | `g1_step` × 6，每次后 `observe` + 定位 | 腰前倾 0.3 rad、左手从罐子上方下到侧面、向内贴近。中间发现"腕 roll 0 = 掌心朝内"而不是它以为的朝下，改了方向 |
| 1364–1550 s | `g1_step` × 4 | 合手（hand 0→1 分 30 行）、确认手指停住、抬起、收到胸前 |
| 1614 s | `g1_step` | 搬运姿势 |
| 1628–1659 s | `g1_walk turn 45/45/45/40, carry` | 四次原地转，测得 36.5/36.7/37.7/33.1° = 138°，面向垃圾桶 |
| 1718–1723 s | `g1_step`、定位 | 看到桶，桶沿在前方 1.3–1.5 m |
| 1745 s, 1773 s | `g1_walk 0.5 m, carry` × 2 | 每段实际前进约 0.27 m。第一段后食指继续合拢（滑），第二段后所有手指到握拳 = 罐子没了 |
| 1787 s | `g1_hold` | 停住 |
| 1792 s | `request_operator(help)` | 原话摘要："罐子搬运中丢了…我看不到它…在地上的话请复位…还剩 28 分钟，要不要停" |
| 1885 s | `request_operator(help)` | "时间不够再来一轮，我停在这里，机器人站着、手臂归位、倾斜 2°。罐子到底在哪？" |
| 结束 | 写 `memory/PROGRESS.md`、`TIPS.md`、`handoff/README.md` | 明确写了"任务未完成"，以及它量到的每个数字 |

从这条轨迹能看出三件事：它的流程是"看 → 定位 → 算一小段 → 发 → 读反馈 → 再看"的循环；每个决定都有记录；它停下来的两个点（丢了罐子、时间不够）都是自己判断后请求操作员，没有硬撑。

---

## 7. 你在旁边能看什么、能做什么

| 想要 | 去哪 |
|---|---|
| 每一次工具调用和返回 | `<session>/mcp.jsonl`（一行一条 JSON，含时间、参数、返回） |
| 它看过的每张图 | `agent-workspace/runtime/frames/` |
| 它自己的进度笔记 | `agent-workspace/memory/PROGRESS.md`（当前步骤、确认的结果、失败和教训、剩余预算、下一步）、`RUNS.md`、`TIPS.md` |
| 给下一个 session 的交接 | `agent-workspace/handoff/README.md` |
| 它写的代码 | `agent-workspace/code/` |
| 它在等你什么 | `<session>/operator/requests/`，console 和 dashboard 也显示 |
| 机器人实时状态 | dashboard `http://<RAX 主机>:8770`；`<session>/state.json` |
| 模型用量 | `<session>/model-usage.jsonl` |
| 实时录像 | `<session>/stream/` |

三个入口的分工：
- **console**：`s` 启动控制、空格上/拔钥匙、`r` 回答它正在等的那条 `request_operator`（固定回 done）、`o` 停止。这是**安全开关**，对它来说表现为"下一个调用被拒"。
- **agent 终端**：直接打字，它当作你的新指令读（"先只做罐子"、"桶在你左后方 1.5 m"、"停下别动"）。这是**对话**。
- **dashboard**：只看不碰。

---

## 8. 老实说哪里还是"盒子"

- **什么时候走、往哪伸手，是模型决定的**，service 只保证每一步在限幅内、钥匙在、没摔。它会反复试同一个抓取，可能把东西越推越远——`task.md` 里写"失败两次就问我"就是为这个。
- **没有笛卡尔保护和自碰撞检查**（第 5 节末）。真机第一天先跑 `g1_check_all.sh`，让它从小动作开始，人站在停止键旁边。
- **它看不到的**：真值位置、自己真正走了多远（只有命令值和它用相机量的）、你的脸。它不知道你在旁边，除非你在终端里告诉它。
- **每次工具调用之间它要想几秒到几十秒**；一个"看、定位、伸手、看"循环 30–90 秒；今天 sim 里到第一次像样的预抓取用了 25 分钟（没有 cuRobo，全部手算帧；真机上有 cuRobo 会快很多）。
- **沙箱是启动的前提**：bubblewrap 起不来，launcher 直接拒绝启动，不会无沙箱运行。

相关文件：`docs/AGENT_WORKFLOW.md`（按你的操作顺序讲它每一步在做什么）、`docs/REAL_ROBOT_WORKFLOW.md`（真机清单）、`docs/CALIBRATION_WALKTHROUGH.md`（第一天标定）、`tasks/puppy_box.md`（第一个任务）。

## 9. 经验怎么在 session 之间、在 sim 和真机之间传

| 通道 | 谁写 | 装什么 | 怎么到下一个 session |
|---|---|---|---|
| `skill-library/<robot>/` | agent，`skill_library_merge` 合并 | 可执行技能（`main(ctx)` 代码 + 证据 + 图） | 自动：每个新 session clone 这个库（同一台机器）；换机器用 `export_experience.sh` 打 git bundle，`import_experience.sh` 装进去 |
| `memory/` | agent | 这个任务的 TIPS（条件→动作→测得结果）、RUNS、INDEX、`memory/skills/` | `configure_session.py --experience <上一个 session>`（`g1_up.sh agent --experience`）：整份拷进新工作区，上一个 session 的 `PROGRESS.md` 放到 `memory/history/<session>/` 当历史 |
| `handoff/README.md` | agent（transfer-handoff 技能） | 换个布局怎么规划每个子任务、每次操作怎么定位、参数范围、失败怎么认 | 同上，随 `--experience` 拷入 |
| `code/` | agent | 它写的控制器 | 同上拷入；但真正可复用的应进 skill-library |

2026-10-31 之前只有第一行是自动的，后三行没有任何代码搬运（技能文件假设有，harness 没做）。现在 `--experience` 补上了；
AGENTS.md 多了一段：继承来的东西可能来自 sim 或另一个布局，每个数字先自己观测验证。

**sim 里的限制（已解决）**：2026-10-31 sim2sim 容器重建（加了 SYS_ADMIN）后 bubblewrap 沙箱能用了，sim session 的 agent
可以写、跑 `code/` 和 skill-library 代码（例如 puppy_box 的 `code/puppy_box/pb3.py`）。

### 9.1 统一的 sim↔real 框架（2026-11-02）

sim 和真机是**同一套** agent、工具、memory 格式、代码；区别只有一个字段：

- **session 的环境**：`configure_session.py --environment real|sim`（默认 real；`g1_up.sh` 起的是真机，sim 批跑脚本传 `sim`）。
  agent 的进程环境里有 `RAX_ENVIRONMENT`，AGENTS.md 和技能里写着"这个 session 跑在 real/sim"。
- **三类知识，三个标签**（技能 `sim-to-real`）：TIPS.md / handoff 的每一行开头标
  `[strategy]`（怎么做、为什么，换环境当规则用）、`[param]`（测出来的数，换环境当先验、要重测）、
  `[sim-only]`（只因为是仿真才成立：脚本 operator、接触模型、场景生成器）。
- **数字放在 `memory/PARAMS.json`，按环境分层**：代码里 `from api.params import param, record`；
  `param('reach.forward_m')` 取本环境的值，没有就取另一环境的值并在 stderr 说明（真机上用了 sim 的数一定看得见）；
  `param(..., verified=True)` 在本环境没测过时直接报错，用在提交性动作（抓取高度、放置点、走路距离）上；
  `record(...)` 写进本环境那一层，另一层保留。`python -m api.params` 列出哪些还没在本环境测过。
- **继承时自动适配**（launcher → `services/sim_to_real.py`）：sim 经验进真机 session 时，`[sim-only]` 行移到
  `memory/history/<session>/SIM_ONLY.md`；生成 `memory/TRANSFER.md`：来源环境、只在 sim 测过的数字清单（勾选框）、先测什么。
- **导出时检查**：`export_experience.sh` 在 `SOURCE.txt` 记 `environment:`（旧 session 用 `RAX_EXPORT_ENVIRONMENT=sim`），
  并写 `TRANSFER_REPORT.txt`：各标签行数、PARAMS 两层各多少、哪些没打标签却写了数字。
- **operator 契约**：真 operator 能做的（arm、reset、扶 gantry、大致放回、说看到什么）sim operator 才能做；
  sim operator 不报 GT 数字。依赖 GT 的东西标 `[sim-only]`。
- **real → sim 回流**：真机测到的数（如 `scene.table_height_m` 的 real 层）喂回 sim 场景随机化
  （`3/eng/recon_dog/build_dog_scene.py` 读 `REAL_PARAMS=<PARAMS.json>`，桌高改成真值 ±2 cm）。
- **能不能上真机**：`3/eng/console_art/transfer_gate.py` 按摆放（左/右/前）统计 GT 确认的成功、摔倒，再对最新 session 的
  memory 做标签检查；每种摆放 ≥2 次成功、0 摔倒、memory 打完标签才 PASS。

真机第一个 session 的顺序（技能里写死）：读 TRANSFER.md → 列出第一个子任务要用的 `[param]` → 每个用最便宜、不提交的方式测
（先看、再空手 hover、再慢速局部动作）并 `record()` → 才执行。机器人层面的数（手在桌高能伸多远、Dex3 闭合后抬 5 cm 还抓不抓得住、
短距离 walk 实际走多少、真相机下桌边/物体检测读数）用 `robot.` / `walk.` / `hand.` 命名，后面所有任务都继承。
