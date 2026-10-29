# 真机工作流程：从 G1 开机到关机

> 写于 2026-10-27，同日晚更新（一键控制台、长流程 sim 结论），2026-10-29 更新（`g1_up.sh` 一条命令起所有窗口、`g1_check_all.sh` 第一天标定、`g1_check_quick.sh` 开机自检：清单 A 第 5–8 步和第 6 节）。这份按时间顺序写"你在现场一步步做什么"，从机器人上电开始。
> 每一步背后的参数、文件、安全规则在同目录的 `REAL_ROBOT_DEPLOY.md`（人话版）和 `REAL_ROBOT_DEPLOY_technical.md`（技术版）里，这里只在需要时引用，不重复。
> 依据：`2/homebody/.cache/rax-g1real`（RAX 真机分支）、容器里的 SONIC 部署代码 `gear_sonic_deploy`（`deploy.sh`、`g1_deploy_onnx_ref.cpp`、键位表）、`MA-Teleop/inference`（你现在用的真机推理流程）、`3/eng` 的 sim 实验（FAILURE_LOG 第 160 到 190 行；长流程任务 234 到 235 行）。
> 标了 **[按 Unitree 手册]** 的步骤是 G1 本体的操作，代码里查不到，以 Unitree 官方说明为准。

---

## 两张清单

### A. 你要动手做的事（按顺序，只有这些）

用一键控制台（第 4 节第 6 条）的版本。括号里是不用控制台时对应的老命令。

1. 机器人挂上吊架，脚离地约 20 cm（只在开机、SONIC 启动那一下需要吊着），周围 1.5 m 清空，急停放在手边。
2. 装电池、开机、等自检完；开遥控器。**不要用遥控器让它站起来。**
3. 关掉所有别的会发命令的程序（teleop、MA-Teleop 的 g1_client 等）。
4. SONIC 主机 `ping 192.168.123.164` 确认在机器人网段。
5. RAX 主机一条命令起所有窗口（第一天先标定）：`bash scripts/g1_up.sh check`；以后每天 `bash scripts/g1_up.sh agent --task-file task.md --quick-check`。它先 ping 机器人、ssh 两台主机、检查参数文件，然后开一个 tmux（`tmux attach -t g1` 进去看，Ctrl-b d 出来，出来不关任何东西），**一个屏幕分 5 格**：左边大格 work（标定脚本，或 robot service + agent），右边一列 camera（ssh 到 Jetson 开 `camera_server.py`）、sonic（ssh 到 SONIC 主机跑 `deploy.sh real …`）、console（`g1_console.py`）、dashboard。Ctrl-b + 方向键把光标移到哪一格，按键就进哪一格（想要分页签而不是分格加 `--windows`）。配置只填一次：`scripts/g1_up.env.example` 拷成 `~/.g1_up.env`。（老做法分四步：开相机、`deploy.sh`、`configure_session.py`、`start_agent.sh` + `run_dashboard.sh` + `g1_console.py`，见第 2、3、4 节。）
6. 光标移到 sonic 格，看到 `Proceed? [Y/n]` 按 **Y**，等 `Init Done`。之后这一格**不再碰**。
7. （第一天）光标移到 work 格，`g1_check_all.sh` 会一步步提示（`>>> … [ENTER]`）：什么时候去 console 格按 `s`、按空格，什么时候放吊架、什么时候拿尺子（量了把数字敲进去回车）。全部做完，这一格回到 shell，其他四格照常跑。详见第 6 节。空格是**开关**：第一下上钥匙（允许动），再按一下暂停，再按一下继续；console 格只管钥匙，动作命令都从 work 格发。
8. 标定做完或第二天直接做任务：**不要关 tmux**（关 sonic 窗口 = 关电机），在任意终端敲 `bash scripts/g1_up.sh agent --task-file task.md`，它只换 console 和 work 两格。机器人已经站着的话**不要再按 `s`**，agent 要钥匙时按空格。
9. **先放吊架，直到机器人自己站在地上、带子变松**（带子还挂着做保护；SONIC 启动时已把它拉成刚性的标准站姿，能像雕像一样站住），**然后**控制台按 **`s`**（START）：机器人在地上站住、自己平衡，**不踏步**（SONIC 直接盯着站立参考动作站着；接管那一瞬间腿会伸直一下，带子兜着）。**不要在脚离地、或只是刚碰地的时候按 `s`**：没有地面反力 SONIC 会在空中乱蹬（sim 里腿速 35 rad/s、倾斜到 10°、永远安静不下来），旁边的人会被踢到；脚刚碰地但带子还托着也只是轻一点（2 分钟内接不上 hold）；脚真正承重时 3.5 s 就站稳。脚承重、它安静下来后 robot service 自动切成"定住"，屏幕上"控制模式"变成 streamed 就是站稳了（sim 里 13 秒）。（老做法：`g1 arm` 后等 agent 的第一个 `g1_hold`。）
10. 第一天按第 6 节一层层做检查（status → 小动作 → 相机量位置 → 走路量距离 → 拿着东西走路量距离）。做检查需要 agent 能动，所以先按一下空格上钥匙；检查间隙再按空格暂停。
11. 正式开始任务：控制台按 **空格**（ARM）。agent 开始动。（老做法：`g1 arm 5`，输入 `ARM`。）
12. 第一次 `g1_walk` 走完、两脚左右分开约一肩宽、前后不错开、安静 2 秒，**才摘带子**。
13. 任务进行中：站在急停旁。觉得不对 → **空格**（PAUSE：动作 20 ms 内停住，机器人保持姿势）；处理完 → 再按**空格**继续，agent 问"能继续吗"就按 **`r`** 回答它。（老做法：`g1 disarm` / `g1 arm` / `g1 reply done`。）
14. 真要让它停下坐地 → **先挂带子**，再按 **`o`**（STOP：拔钥匙 + SONIC 软停，机器人软掉挂在带子上，SONIC 程序随后退出）。SONIC 终端的 `O`、手柄 Select、硬件急停仍然是最外层的保险，随时可用。
15. 结束：按 **`o`**（已经停了就不用）→ agent 终端 Ctrl-C → 挂带子吊起 → SONIC 终端 Ctrl-C（`o` 之后它已自己退出）→ 关相机 → 关机取电池。控制台 `q` 退出。

### B. 这个过程里系统自己在做什么（程序启动顺序）

| 顺序 | 哪台机器 | 启动的程序 | 它干什么 | 机器人表现 |
|---|---|---|---|---|
| 1 | G1 本体 | Unitree 自带控制器（开机自启） | 自检、电机上电、DDS 通信 | 软的，吊着 |
| 2 | Jetson（G1 头上） | `camera_server.py` | 头部相机彩色 + 深度流，端口 7790 | 不动 |
| 3 | SONIC 主机 | `deploy.sh real` → 编译 → `g1_deploy_onnx_ref` | ① 停掉 Unitree 自带运动服务（`ReleaseMode`）② 等第一帧 `LowState` ③ 3 s 把全身拉到标准站姿、手握一下张开 ④ 打印 `Init Done`，进 `WAIT_FOR_CONTROL`，PD 保持站姿等"开始"；此时 5557 还不发状态 | **自动摆成站姿**（僵硬，不平衡），之后不动 |
| 4 | RAX 主机 | `configure_session.py` | 只写 session 配置文件 | 不动 |
| 5 | RAX 主机 | `start_agent.sh` → 拉起 **robot service**（robot-SDK 环境）+ **Claude agent**（无 SDK 环境）+ MCP 工具 | robot service 连 SONIC 的 5556/5557；agent 开始读任务、调 `g1_status`（报"没有状态"，正常） | 不动 |
| 6 | RAX 主机 | `run_dashboard.sh` | 显示钥匙、请求、相机画面、姿态 | 不动 |
| 7 | RAX 主机 | agent 想动 → robot service 拒绝 → `request_operator` | 等你 `g1 arm` | 不动 |
| 8 | robot service | 控制台 `s`（或没按 `s` 时 agent 的第一个 `g1_hold`）：发"开始 + 规划器模式"到 5556，重复到 5557 出现状态（≤ 8 s），然后**立刻切到流姿势模式** | SONIC 从保持站姿切到规划器，再马上切回盯着站立参考动作（第 0 帧，暂停） | **开始自己平衡**，带子里站着不踏步（接管瞬间腿伸直一下） |
| 9 | robot service | 等 ≥ 3 s + 安静 1 s（倾斜 < 10°，关节不动），最多 15 s | — | 安静下来 |
| 10 | robot service | 记下当前姿势，切 SONIC 到 streamed 模式，1.5 s 缓到标准站姿 | 从此 agent 的每个 `g1_step` 变成姿势流发给 SONIC | 站稳，听指令 |
| 11 | agent | `g1_observe` / `g1_pixel_to_position` / `g1_walk`（带热身 ±15°、分段、carry）/ `g1_step` / `g1_hand_shape` | 每次调用先过 robot service 的安检（钥匙、状态 ≤ 0.2 s 新、倾斜、关节限位） | 做任务 |
| 12 | robot service（自动） | 倾斜 > 60° → 叫 SONIC 软掉 + 自动拔钥匙；状态 0.2 s 没更新 → 拒绝/中断动作；2 s 没更新 → 认为 SONIC 死了 | 保护 | 停住或坐下 |
| 13 | 关机 | `g1 disarm` → agent Ctrl-C → SONIC `O`（全身阻尼）→ Ctrl-C | — | 软掉，挂在带子上 |

和 MA-Teleop 对照：第 1–3 步和你现在完全一样；MA-Teleop 的第 4–10 步是 `controller_server` + `g1_client`（g1_client 先发 start(planner)，收到状态后切 streamed），RAX 把这两下放进第 8–10 步，多了"等安静"和"钥匙"。

---

## 0. 现场需要的东西

| 东西 | 作用 |
|---|---|
| G1 + Dex3 手，电池充满 | 机器人 |
| 吊架 + 带子 | 开机、第一次走路、出事时都靠它。整个流程里带子"挂着但放松"的时间比你想的长 |
| Unitree 遥控器 | 开机自检、必要时进调试模式、兜底急停 **[按 Unitree 手册]** |
| 急停（物理）| 最外层保险，始终有人站在旁边 |
| SONIC 主机 | 带 NVIDIA GPU + TensorRT 的电脑，接在机器人网络 `192.168.123.x` 上，跑 `g1_deploy_onnx_ref`。MA-Teleop 里这台叫 "server machine" |
| RAX 主机 | 跑 robot service + Claude agent + dashboard。可以和 SONIC 主机是同一台；不是同一台时要能互通 ZMQ 5556/5557 |
| G1 自带的 Jetson（`192.168.123.164`）| 跑头部相机服务，给 agent 彩色图 + 深度 |
| 卷尺 | 第一天标定走路距离和相机定位 |
| 任务物品 | 易拉罐（sim 里的尺寸：高 16 cm、直径 5.4 cm）、废纸团（**揉一张 A4，直径 6 到 8 cm**；sim 里原来的纸团只有 4.2 cm，抓取成功率 1/6，今晚换成 7 cm 的在测）、垃圾桶（开口约 34 × 36 cm，桶沿高 41 cm）、冰箱（上门，门沿离地约 1.0 m）。物品尺寸和 sim 差太多，agent 的抓法参数就不对 |

**只允许一个程序给机器人发指令。** 开 SONIC 之前，把 teleop、MA-Teleop 的 `g1_client`、其他 DDS 发布者全部关掉。

---

## 1. 上电（机器人本体）

1. 把机器人挂到吊架上，**脚离地约 20 cm**，带子受力，身体大致竖直，周围 1.5 m 清空。（这是开机和 SONIC 启动时的状态；按 `s` 之前要把它放到脚承重。）
2. 装电池，按电源键开机（Unitree 的惯例是短按一下、再长按到灯亮），等它自检完成，大约 1 到 2 分钟 **[按 Unitree 手册]**。
3. 开遥控器，确认和机器人连上 **[按 Unitree 手册]**。此时机器人在 Unitree 自带控制器手里，关节是软的或阻尼状态，手不会动。
4. 在 SONIC 主机上确认网络：`ping 192.168.123.164`（Jetson）通。电脑在机器人网段上。
5. **先不要做任何会让它站起来的事**（不要用遥控器让它进入自带的站立/行走模式）。SONIC 一启动就会把 Unitree 的运动控制服务停掉并接管电机（见第 3 节），所以机器人应该从"软的、吊着"这个状态交给 SONIC。

---

## 2. 开相机（Jetson 上）

两种选一种，和 RAX 的 `camera` 配置对应：

- **HomeBody 的 `camera_server.py`**（RAX 默认，`camera: network`，端口 `7790`，彩色 + 深度）：ssh 到 `192.168.123.164` 启动它。
- **MA-Teleop 的 `g1_camera_publisher.py --port 5620`**：你现在用的那个，只有彩色图。RAX 也能接它（`camera: ma-teleop`），但因为它的流里不带相机内参，要你把 ZED 的标定矩阵填进配置；而且没有深度就没有 `g1_pixel_to_position`，桌面任务基本做不了。

**结论：用第一种，什么都不用填**（camera_server 的流里自带深度和内参）。

---

## 3. 开 SONIC（SONIC 主机上）

```
cd <gear_sonic_deploy>
./deploy.sh real --input-type zmq_manager --output-type zmq \
    --planner planner/target_vel/V2/planner_sonic.onnx \
    --motion-data <只有站立动作的参考目录> \
    --zmq-host <robot service 所在主机>
```

脚本会：自动找 `192.168.123.x` 的网卡 → 检查模型文件 → 编译 → 打印一张配置表 → 问你 `Proceed with deployment? [Y/n]`。真机模式下**不会**加 `--disable-crc-check`（那是 sim 专用）。

按 Y 之后下面 4 步**全是程序自动做的，你不用动手，也不要碰键盘**。写出来是让你知道机器人为什么会动、什么时候必须吊着。这和你用 MA-Teleop 时 `./deploy.sh real --input-type zmq_manager` 之后看到的完全是同一个过程（同一个脚本、同一个程序）：

1. **停掉 Unitree 自带的运动控制服务**（代码里 `MotionSwitcherClient.ReleaseMode`，失败会打印 `Failed to switch to Release Mode` 并每 5 秒重试）。如果一直失败，用遥控器把机器人切到调试/开发者模式再试 **[按 Unitree 手册，通常是 L2+R2]**。
2. 等机器人第一帧状态（`LowState`）。
3. **收到第一帧后，立刻用 3 秒把全身关节从当前位置拉到标准站姿，电机已经上力；Dex3 手先握一下再张开。** 这就是你用 MA-Teleop 时看到的"一开 SONIC 机器人就摆成站姿"。注意此时它只是**僵硬地摆着站姿**，还没有在平衡；真正开始自己平衡要等"开始"信号（MA-Teleop 里是 g1_client 发的 start，RAX 里是第一次 `g1_hold`）。**所以这一刻机器人必须是吊着的**，不然它会在地上以不受控的姿势被拉成站姿。
4. 打印 `Init Done`，进入 `WAIT_FOR_CONTROL`：机器人用 PD 保持标准站姿，等"开始"信号。**这时 ZMQ 5557 还没有状态发出来**，RAX 那边 `g1_status` 会报"没有状态"，是正常的。

**在这个终端不要按任何键**（MA-Teleop 的 `g1_client.py` 注释原话："do NOT press any keys"）。SONIC 程序自己带一套键盘命令，你用 MA-Teleop 时从没按过，因为 g1_client 替你通过网络发了"开始"；RAX 的 robot service 也是这样替你发。所以键盘只需要记住一个用来兜底：

| 键 | 作用 |
|---|---|
| `O` | **软件急停**：所有关节变成阻尼，机器人坐下。手柄上是 Select。关机前也用它 |
| `]` | 开始控制。**不要按**：RAX 会通过网络发"开始"，手动按了就是两个人同时在指挥。手柄上是 Start |

**命令里两个尖括号填什么**：
- `<只有站立动作的参考目录>` → `reference/standing/`。这个目录是 sim 容器里用 `3/eng/make_standing_ref.py` 生成的（`stand_000`：SONIC 标准站姿、500 帧、10 秒、速度全零），在 `/opt/sonic/gear_sonic_deploy/reference/standing/`，把整个目录拷到真机 SONIC 主机的 `gear_sonic_deploy/reference/` 下面就行。不拷也能跑：不写 `--motion-data` 就是默认的 `reference/example/`（几段舞蹈），MA-Teleop 一直这么用。
- `<robot service 所在主机>` → 跑 RAX `start_agent.sh` 的那台电脑的 IP（robot service 在它上面开 5556 口，SONIC 连过去）。**和 SONIC 主机是同一台就不用写**（默认 localhost）。

上面命令里多出来的参数说明：`--planner` 其实是 `deploy.sh` 的默认值，你 MA-Teleop 那条短命令用的就是它，写出来只是防止有人改默认；`--motion-data` 默认是自带的 example 舞蹈，MA-Teleop 也一直用这个默认，没出过事，这里换成"只有站立"的参考目录是一个保险（规划器流中断时 SONIC 会跳回参考动作的第 0 帧，站立比舞蹈安全）。**你照 MA-Teleop 的那条短命令开也可以**，只是少了这层保险。

---

## 4. 开 RAX（RAX 主机上）

先说 RAX 是什么：它替代了 MA-Teleop 里的 `controller_server` + `g1_client`。MA-Teleop 是一个训练好的策略网络在流姿势；RAX 是 Claude（AI agent）通过一组工具（`g1_observe` 看图、`g1_walk` 走、`g1_step` 动手臂、`g1_hold` 站住……）来指挥机器人，中间隔着一个 **robot service**，它做安全检查并把工具调用翻译成发给 SONIC 的姿势流。SONIC 那一层和 MA-Teleop 完全相同。

一次性准备（只做一次，Junjie 之前应该已经弄好了，没弄好看技术版第 3 节）：
- **两个 Python 环境**：一个给 robot service（装了 unitree SDK、能直接碰机器人），一个给 agent（**故意不装任何机器人 SDK**，这样 AI 写的任何脚本都不可能绕过安全检查直接给机器人发命令，只能走工具）。这就是 `RAX_PYTHON` 和 `RAX_AGENT_PYTHON` 两个变量的含义。
- `RAX_PROTECTED_PATHS`：agent 不允许修改的目录（SONIC 代码、unitree_sdk2、teleop、robot service 的环境），同样是防 AI 乱改。
- 模型账号、真机参数 JSON。

每次实验：

1. **写配置**（不动机器人）：
   ```
   $RAX_PYTHON scripts/configure_session.py --task <任务名> --task-file task.md \
       --robot g1sonic --backend-options g1_real_options.json --observation-profile rgbd
   ```
   **`g1_real_options.json`（真机参数 JSON）是什么**：robot service 的一张设置表，一个普通的 JSON 文件，你自己写、放在 RAX 主机上随便哪个目录，用 `--backend-options` 指给它。里面只写你要改的项，没写的项用 robot service 代码里的默认值（`services/g1sonic/backend.py` 开头的 `DEFAULTS`，共 50 多项，大部分是安全阈值，不用动）。同目录有一份模板 `g1_real_options.example.json`，复制过去把尖括号填掉就能用。真机和 sim 必须不一样的几项：

   | 项 | 真机写什么 | 意思 |
   |---|---|---|
   | `state_host` | SONIC 主机的 IP（同一台就 `127.0.0.1`） | 去哪里收机器人状态（5557） |
   | `zmq_host` | `0.0.0.0`（同一台也行 `127.0.0.1`） | robot service 在哪个网卡上开姿势口（5556），SONIC 的 `--zmq-host` 指过来 |
   | `wall_time_scale` | `1.0` | sim 里可以放慢时间，真机必须 1.0 |
   | `state_max_age_s` | `0.2` | 状态 0.2 秒没更新就拒绝/停止动作（sim 里放宽到 1.5） |
   | `console_start_mode` | `stand` | 控制台 `s` 怎么站：`stand` = 启动后立刻切流姿势模式，盯着站立参考站着**不踏步**（默认，sim 两次验证）；`planner` = 老做法，规划器原地碎步站稳再接管（sim 对照里接管时冻住了单脚姿势，见第 5 节） |
   | `planner_settle_s` | `3.0` | 只在 `planner` 模式和 agent 自己启动控制的路上用：至少让它在规划器下站 3 秒再接管（sim 里 2 秒） |
   | `camera` / `camera_host` / `camera_port` | `network` / `192.168.123.164` / `7790` | 用 Jetson 上 HomeBody 的 `camera_server.py` |
   | `camera_pose_override` | 先 `null` | 相机装的位置和 URDF 不一样时再填（第 6 节第 4 条量出来之后） |
2. **启动 agent + robot service**：`bash scripts/start_agent.sh --config <session>/operator-input.json`，另开 `scripts/run_dashboard.sh` 看钥匙、请求、相机画面、实时姿态。
3. agent 先调 `g1_status`（报"没有状态"，正常）。它想动会被拒绝，并通过 `request_operator` 请你上钥匙。
4. **钥匙命令**：真机分支里是 `scripts/g1`（和 `rby1` 同一套，建议软链到 PATH）：

   | 命令 | 作用 |
   |---|---|
   | `g1 arm 5` | 上钥匙 5 分钟（不写数字就一直有效）。终端会要你输入 `ARM`，意思是"有人在急停旁、周围已清空" |
   | `g1 disarm` | 拔钥匙：正在做的动作 20 ms 内停住，机器人**保持姿势**（不会坐下；坐下是 SONIC 的 `O`） |
   | `g1 status` / `g1 requests` / `g1 reply done` | 看状态 / 看 agent 在请求什么 / 回答它 |

5. **怎么告诉 agent 这个 session 做什么、怎么和它说话**：
   - **任务写在一个 markdown 文件里**（比如 `task.md`：桌上有一个易拉罐和一个纸团，把它们扔进垃圾桶，然后打开冰箱门再关上；能写多细写多细：物品在哪、桶在哪、不许做什么、什么时候来问你）。`configure_session.py --task <短名> --task-file task.md` 把它写进 session；`start_agent.sh` 启动时会把它放到 agent 工作目录的 `TASK.md`，agent 的第一句指令就是"读 AGENTS.md、MCP_SESSION.md、TASK.md，然后调 status，按 AGENTS.md 的流程做"。不想写文件也可以 `--task-text "一句话"`。
   - **agent 的终端就是一个 Claude Code 对话窗口**（`start_agent.sh` 默认交互模式）。任何时候你可以在里面直接打字和它说话：改任务（"先只做罐子，纸团不管"）、给提示（"桶在你左后方 1.5 m"）、让它停（"停下，站着别动"——它会调 `g1_hold`）。它说的话、每次调工具的结果也都显示在这个窗口里。
   - **它需要你的时候会来问**（`request_operator`）：比如要钥匙、场景复位了没有、摔了之后能不能继续。你在控制台按 `r`（或 `g1 reply done`）就是回答"好了"。想回答别的内容就在 agent 终端里打字。
   - 它做的每一步记录在 `<session>/mcp.jsonl`，看到的每张图在 `agent-workspace/runtime/frames/`。
6. **一键控制台（可选，推荐）**：`RAX_PYTHON scripts/g1_console.py`（robot service 那个 python 环境；默认接最新的 session）。一个键盘管完整个过程，不用在两个终端和 `g1` 命令之间切：

   | 键 | 作用 | 对应原来的操作 |
   |---|---|---|
   | `s` | **START**：SONIC 开始控制，机器人在带子里站起来、自己平衡，**不踏步**。此时你放吊架让脚承重；它在地上安静下来后，robot service 自动接管成"定住"模式（最多等 3 分钟，期间按 `o` 随时可停）。agent 这时仍然动不了 | agent 的第一次 `g1_hold`（那条路走规划器，会踏步） |
   | 空格 | **ARM / PAUSE 切换**：第一次按 = 上钥匙，agent 开始动；再按 = 拔钥匙，正在做的动作 20 ms 内停住、机器人保持姿势、agent 会来问你；再按 = 继续 | `g1 arm` / `g1 disarm` |
   | `r` | 回答 agent 最旧的一条请求"done"（场景复位好了、暂停后让它继续） | `g1 reply done` |
   | `o` | **STOP**：SONIC 软停（全身阻尼，机器人软掉，**必须挂着带子**）+ 拔钥匙。SONIC 程序随后自己退出，下次 START 前要重新跑 `deploy.sh` | SONIC 终端 `O` + `g1 disarm` |
   | `q` | 退出控制台（机器人状态不变） | — |

   屏幕上实时显示：钥匙状态、控制模式（idle / streamed / planner）、倾斜角、状态延迟、故障、robot service 最近几条事件、agent 的待回答请求。控制台只写 session 的 `operator/` 目录（`armed.json`、`stack.json`），由 robot service 执行；SONIC 终端的 `O`、手柄 Select 和硬件急停**仍然是最外层的停止手段**，控制台不替代它们。在 sim 里完整测过一遍：START 16 秒站稳（倾斜 1.3 度）、ARM 后 agent 的 hold 能过、PAUSE 后 `g1_walk` 被拒、STOP 后 1 秒内状态停止、SONIC 退出。

---

## 5. 第一次启动控制：第一次 `g1_hold`（整个流程最危险的一步）

> 对应 MA-Teleop 里你"按站立、再按推理"那两下。这里由 robot service 做，触发点是控制台的 `s`（你没按 `s` 的话，就是上钥匙后 agent 的第一个动作指令）。

**控制台 `s`（默认，`console_start_mode: stand`）：**

| 时间 | robot service 在做什么 | 你在做什么 |
|---|---|---|
| 0 s | 通过 ZMQ 发"开始 + 规划器模式"，重复发直到 5557 上出现第一帧状态（最多 8 s），**然后立刻切到流姿势模式**。这时还没有人给它发姿势，SONIC 就盯着自己启动时加载的站立参考动作（`reference/standing/` 第 0 帧，暂停）平衡：**原地站着，不踏步**。接管那一瞬间腿会猛地伸直一下（sim 里脚离地 5 到 6 cm、不到 1 秒、倾斜 6 度，带子兜着） | 手放在吊架上，看它站住 |
| 之后 | 等"安静"：倾斜 < 10 度、关节都不怎么动、持续 1 s；最多等 3 分钟（`console_start_timeout_s`），期间 `o` 随时可停 | **慢慢放吊架让脚承重，带子留着、放松地挂着，先别摘** |
| 安静后 | 记下当时的姿势，开始发"定住"姿势流，1.5 s 内缓慢过渡到标准站姿；屏幕上控制模式变 streamed | 什么都不用做 |

sim 里跑了两次：`s` 到站稳 13 s，除了接管那一下两脚全程没离地，站稳后倾斜 1.2 到 1.8 度，之后 90 秒纹丝不动。

**老做法（`console_start_mode: planner`，也是你没按 `s`、直接上钥匙让 agent 的第一个 `g1_hold` 去启动控制时走的路）：** 开始后先让 SONIC 在**规划器**下站。规划器是走路用的，速度命令为零它也要原地碎步几秒把自己站稳（这就是你在 MA-Teleop 里按"站立"后看到的那几步）；robot service 至少等 `planner_settle_s`（3 s）再等"安静"（最多 15 s），然后才切流姿势模式接管。sim 里同一天对照跑了一次：踏了 2 到 8 秒，接管那一刻左脚正抬着 7 cm，"定住"把这个单脚姿势冻住了，带子一松机器人就倾着 11 到 12 度单脚站了一分钟。没倒，但就是以前记录过的"放开后站姿歪/窄"那类问题（FAILURE_LOG 171 到 173、236）。所以**默认改成 `stand`**，`planner` 留着备用。

**为什么以前要踏步、现在不用了？** SONIC 通过网络收到的"开始"只在规划器模式下生效（流姿势模式下的"开始"只认键盘的 `]`），所以控制必须用规划器模式启动。新做法只是启动后马上切走，不给规划器迈步的机会；切过去之后 SONIC 盯着的是一段纯站立的参考动作（第 3 节的 `reference/standing/`）。这也是那个目录必须存在的原因：参考动作如果不是站姿，切过去它就会去摆那个姿势。
**脚已经着地再按 `s` 可以吗？** 可以：带子留着、放松地挂着，脚底不要有东西。sim 里就是先把吊架放下 0.2 m 让脚着地，再按的 `s`。**要先着地**，不要学官方文档"先按开始再放下去"的顺序：sim 里试过吊在空中按 `s`，策略脚下没有地面就一直在空中蹬腿找支撑（两脚来回 13 到 20 cm、倾斜 7 到 12 度），带子放松落地后虽然站住了，但之后半分钟一直小幅挪脚（倾斜到 5 度、漂 7 cm）；脚先着地再按 `s` 的两次全程脚不离地、倾斜 1.3 度。

**别人是怎么让 SONIC 站起来的（都查过原文）：**
- **NVIDIA 官方（GR00T-WholeBodyControl 仓库的 SONIC 部署文档，键盘流程）**：deploy 一启动就先把 29 个关节用策略的 PD 增益在 3 秒内线性拉到默认站姿（`default_angles`）并一直保持（"Init Done" 之后机器人在吊架上就是硬邦邦的标准站姿）；按 `]` 开始控制 = 策略开始跟踪**预加载参考动作的第 0 帧（暂停）**，默认参考是 `neutral_kick_R_001` 这段踢腿动作的第一帧，一个站姿；然后 sim 里按 `9` 把机器人放到地上，真机就是放吊架。也就是说官方从来不靠规划器站立，和我们现在的 `stand` 模式一样，只是我们换成了纯站立的参考段。官方 PICO 遥操作文档对真机的描述是"让 G1 松松地挂在吊架上，策略启动后它自己开始平衡"。
- **实验室的 ZMQ 客户端（`g1_sonic_client.py`、`psi_sonic_client.py`、MA-Teleop）**：都是 `start=True, planner=True`，先进规划器站——因为网络上的"开始"只在规划器模式生效。我们之前的踏步就是从这里继承的。
- **HomeBody（自己的 AMO 策略，不是 SONIC）**：上电后所有关节零刚度只留阻尼（软着），遥控器 `Y` = 在限速下把关节拉到中立姿势，`A` = 策略以站姿接管；停止就是全身回到阻尼。和 SONIC deploy 的 INIT→`]` 是同一个思路：先 PD 拉到站姿，再把策略放上去。

两条规矩，都是 sim 里 1100 多次开机总结出来的（FAILURE_LOG 167 到 173 行）：

- **带子挂到第一次 `g1_walk` 走完再摘。** 第一次真正迈步那一下，策略本身约有 5% 到 10% 的概率迈不好（膝盖超调、该落地时脚还在空中，1 秒内从 10 度倒到 60 度）。和参考动作、起步速度、站姿检查都无关，治不了，只能用带子兜住。原地小碎步（转身用的那种）从来没摔过，之后的走路也基本不摔。
- **摘带子前看两脚。** 两脚左右分开约一肩宽（26 cm 左右，不是 13 到 16 cm 并拢）、前后没有错开、机器人安静 2 秒以上。脚并拢或前后错开就别摘，重新吊起来让它再站一次。

启动失败（15 秒不安静：吊得不平、靠着东西）：摆正、清掉接触物，`g1 arm` 再来一次。

---

## 6. 第一天的检查顺序：`g1_check_all.sh`（标定，一次性；SONIC 或固件升级后重做）

这是 RAX 的 bring-up 清单里"有人盯着的特性标定"那一步（RB-Y1 的 `RBY1.md` 第 6 章）。量到的数字直接决定 agent 的行为规则（走完必须重看、手够不够得着、暂停多久停住）和后端的保护。

`g1_up.sh check` 的 work 窗口跑 `scripts/g1_check_all.sh <session>`，每一步是 `scripts/g1_motion_check.py <session> <步骤>`（走真实后端、和 agent 同一套安检；每步结束机器人保持姿势），步与步之间它停下来等你按 ENTER，并告诉你这一步要做什么。数字打印出来并存到 `<session>/runtime/characterization/g1_motion_check.jsonl`，填进 `G1.md`。

| 步骤 | 机器人 | 量什么 | 你做什么 | sim 里的数（2026-10-29，参考） |
|---|---|---|---|---|
| `status` | 吊着，没按 `s` | 栈连上没有、状态龄、钥匙 | 看 | 无本体状态（正常） |
| `disarmed` | 同上 | 没钥匙的 `g1_step` 必须被拒 | 看拒绝文本 | "motion is not armed" |
| `hold` | 放吊架到脚承重、带子松，按 `s` 站住，空格 | 10 s 静止：关节漂移、倾斜范围、控制器自身跟踪 RMSE（这是基线不是碰撞） | 等 | 漂移 0.005 rad，倾斜 1.2°，RMSE 0.097 rad |
| `arms` | 脚承重，带子松挂 | 每臂肩 −0.3 / 肘 +0.3 / 腕 +0.3 rad 各一次，手的位移命令 vs 实测（FK，cm） | 等 | 肩到 0.25–0.29，手误差 0.7–1.5 cm；腕 0.20–0.24，误差 1.4–2.0 cm |
| `waist` | 脚承重，带子松挂 | 腰 yaw +0.2、腰 pitch +0.15 | 等 | yaw 只到 0.05（只跟 1/4）；pitch 到 0.15 |
| `hands` | 脚承重，带子松挂 | 合手张手，手指到位误差 | 等 | 合手最大误差 0.19 rad，张手 0.06 |
| `pause` | 脚承重，带子松挂 | 一个 2 s 手臂动作，中途按空格：多久停、多走几度、停后 3 s 漂多少 | **动了就按空格** | 当拍停住，多走 0.064 rad，3 s 漂 0.003 rad |
| `turn` | **脚着地**，带子松着，空格 | 转 ±30° 两次，IMU 实测角、耗时、倾斜峰值 | 清场 | 实转 22.5°（0.75 倍），13 s 和 5 s，倾斜峰 5.5° |
| `walk` | 脚着地 | 走 1 m 两次，然后问你尺子读数 | **胶带标脚尖，量实际距离，输进去** | 命令 0.5 m 规划器自报 0.41，14 s，倾斜峰 3.8–5.2° |
| `camera` | 脚着地 | 一张观测：深度有效率、内参、后端假设的相机位姿、视野里水平面的高度（level 系），和你尺子量的地面/桌面比 | **尺子量骨盆离地、桌面离地，输进去** | 地面 −0.795（真值 −0.79），桌面 −0.03（真值 −0.03）：误差 < 1 cm |

脚着地的三步在 sim 里走路前都带 ±15° 热身转。走路和转弯的真机数会和 sim 不一样，这正是要量的。

另外两个真机前要做、不在脚本里的：**沙盒审计**（起一次 agent，从它里面确认连不上 SONIC 的 5556/5557 和相机 7790，写不了钥匙文件）和**干跑 agent**（`task.md` 只写"抬右手到胸前再放回去"）。

以后每天开机：`g1_up.sh agent … --quick-check`，work 窗口先跑 `g1_check_quick.sh`（status + hold + arms，1 分钟），数字和 `G1.md` 对得上才接着起 agent。

这些做完再做任务。先做只用手臂的（桌边的罐子），**冰箱门和多次转弯的导航放最后**。拿着东西走路（`carry=true`）真机上没量过之前，不要让 agent 做"两手都拿着去垃圾桶"（sim：一手 1.2 m 命令走 0.9 m，两手只走 0.3 m，一次倒退 0.2 m，FAILURE_LOG 235）。

---

## 7. 任务进行中要知道的事

**走路**（`g1_walk`，robot service 内部已经加了这些保护，你只需知道它会这样表现）：
- 只能往前，不能后退、不能横移；每次转最多 ±45 度，速度 0.1 到 0.6 m/s。
- 第一次走之前自动左右各扭 15 度热身；超过 0.5 m 拆段走；转弯时会顺带往前挪 10 到 20 cm，前面要留空间。
- 走完如果一直原地踏步停不下来，这是已知的坑（规划器没交回控制权），**你直接 `g1 disarm` 就行**，规划器会继续替它平衡。
- `carry=true` 走路时腰、双臂、双手冻住，手里有东西时必用。
- 走的距离是命令值，不是实测。没有里程计上传。`3/eng/legodo.py` 用关节角算了一个腿部里程计，真机可用但没验过，先用卷尺对一次。

**手臂作业，这是今天 sim 里新确认的、真机上也会遇到的一件事**（FAILURE_LOG 189 行，`sonic-pelvis-retreat` 记录）：
- 机器人前倾在桌边作业时，SONIC 靠**移动盆骨**来保持平衡，而不是硬撑：双臂往前伸出去，盆骨后退 5 到 10 cm；双手拖一个东西往回拉，盆骨反而往前跟 7 cm；手按到桌面合上，再退 5 cm。
- 后果：任何"先量好位置、再伸手过去"的动作，到手落下的时候机器人已经不在量的那个位置了。sim 里桌子上的盘子因为这个连续 50 轮没抓起来。
- 真机上的对策：把会引起后退的动作放在定位之前做（先前倾、先把手举到作业高度，再走最后一步），或者把东西拖到身边再抓；手臂舒服的作业距离是盆骨前方 0.35 到 0.45 m，再远接触就会把机器人推开。

**长流程任务（罐子 + 纸团 → 垃圾桶 → 冰箱）今晚在 sim 里跑了 8 次，没有一次全程成功，但摔倒 0 次。真机上照着下面做能避开 sim 里踩过的坑**（FAILURE_LOG 234、235；视频 `3/gifs/long_run6_can_ball_to_bin_4x.mp4`）：
- **哪只手拿什么**：右手抓罐子今晚 6 次成 5 次，可以用；纸团不管哪只手都是 1/6 左右，问题在纸团不在手。真机先用大一点的纸团（揉 A4，6 到 8 cm），别用小的。
- **纸团要放在手前方 0.37 到 0.42 m 再下手**，太近太远都抓不到。**每次没抓到，手指会把它往桌子里推 2 到 6 cm**，推四五次就到任何站位都够不着的地方（离桌边 0.34 m）。真机上纸团没抓到两次，就伸手把它拨回桌边再试，别让 agent 一直原地重抓。
- **手里有东西的时候不要重新走位到桌边**。sim 里两次都出事：重新走位的程序会把手臂交回给走路策略（东西掉桌上），180 度转身只转了 115 度，然后斜着走到桌子上去，最后一次是身体顶着桌沿、倾 15 度、原地踏步 8 分钟停不下来（没摔，但这种状态真机上必须立刻按空格暂停或 `o` 停）。要重新走位，先把东西放下。
- **扔进垃圾桶前，最后一眼必须看到桶**。拿着东西走路走多远完全不准（见第 6 节第 6 条），靠"走了多少"推算桶在哪会把东西扔在地上（sim 里两次扔在离桶 1 m 的地上）。规矩：走完看一次桶，看不到就把手举高举开再看，看到了才松手；看到的桶要有 26 cm 以上的宽度，小的黑盒子不是桶（sim 里认错过一次，走过了头）。
- **去冰箱前先用桶定位**：从桶走去冰箱的路线短（约 1.3 m，一个转弯），从桶边重新起算位置就准；拿着东西走完那段再去推算位置，sim 里差了 1.2 到 1.7 m，机器人在房间另一头对着柜子拉"门"。
- **冰箱门站位：门沿在身前 0.37 到 0.39 m**。近了（0.25 m）手臂没有可行的拉门轨迹；机器人不能后退，所以宁可停远一点再走半步。另外开门后的"门开了没有"相机判断有过一次误报（门没动它说开了 16 度），真机上你自己看一眼。

**手**：`g1_hand_shape` 定义 Dex3 "张开"和"握紧"长什么样，`g1_step` 里的手部分数在两者之间插值。真盘子的重量、Dex3 的柔性和 sim 不同，抓取成败会变，摔的机制不变。

**随时可以 `g1 disarm`。** 它只是让机器人定住，不是让它坐下，所以不会造成任何损伤；拿不准就拔钥匙。

---

## 8. 出事了

| 现象 | 系统自动做什么 | 你做什么 |
|---|---|---|
| 倾斜超过 60 度 | robot service 叫 SONIC 软掉（所有关节阻尼）+ 自动拔钥匙，记"摔倒故障" | 扶住或让它坐下，重新吊回吊架，检查有没有损伤，`g1 arm` 重新上钥匙，下一次 `g1_hold` 重新启动控制（第 5 节） |
| 状态 0.2 秒没更新 | 拒绝新动作 / 中断动作，机器人在 SONIC 里保持最后姿势 | 看 SONIC 终端和网线 |
| 状态 2 秒没更新 | 认为 SONIC 死了，忘掉姿势，下次 hold 重新启动控制 | 先挂带子，再重启 SONIC |
| 走完一直原地踏步 | 不切回 streamed，规划器继续平衡 | `g1 disarm` |
| 身体顶着桌子/柜子，倾着 10 到 15 度一直踏步（sim 第 7 次长流程里出现过 8 分钟） | 不会自动处理（没到 60 度） | 立刻 `g1 disarm`（控制台空格）；它还在踏步就挂带子、SONIC `O` |
| 任何你觉得不对的时候 | — | `g1 disarm`（定住）；真要让它停下来坐地：SONIC 终端 `O` / 手柄 Select / 急停，**能的话先挂带子** |

sim 里"仿真器重启导致瘫倒"这一类真机上不存在；但 SONIC 程序崩了，机器人会软掉在地上，所以 SONIC 主机不要跑别的重活。

---

## 9. 收尾和关机

1. 控制台**空格**暂停（老做法 `g1 disarm`）。
2. agent 终端 Ctrl-C（agent 和 robot service 一起停）。
3. **把吊架带子挂上、吊起一点让脚不承重**。
4. 控制台 **`o`**，或 SONIC 终端按 `O`：全身阻尼，机器人软掉（吊着就只是挂在带子上）。按过 `o` 的话 SONIC 程序自己退出；按的是 SONIC 终端的 `O` 就再 Ctrl-C 退出它。
5. 关相机服务。
6. 遥控器 / 电源键关机 **[按 Unitree 手册]**，取电池充电。
7. 记录在：`<session>/mcp.jsonl`（每一次工具调用和结果）、`agent-workspace/runtime/frames/`（每一张观测图）、dashboard 的遥测；`g1 ship` 归档。

---

## 10. 和 MA-Teleop 推理流程的对照（你熟悉的那套）

MA-Teleop 真机推理的顺序是：Jetson 上开 `g1_camera_publisher.py` → 服务器上 `./deploy.sh real --input-type zmq_manager`，不按键，等 `Init Done` → 推理机上开 `controller_server` → 服务器上开 `g1_client`。`g1_client` 自己做两步：先发"开始 + 规划器模式"让机器人在规划器下站稳（你说的"先按站立"），确认收到状态后切到 streamed 模式开始流姿势（"再按推理"）。

RAX 的流程里**这两步是同一个东西**，只是搬进了 robot service 的第一次 `g1_hold` 里（第 5 节），并且多了三样：中间要等机器人"安静"才切模式、切模式后 1.5 s 缓到标准站姿（一帧跳过去在 sim 里掀翻过）、以及整个过程被"钥匙"管着。SONIC 这边完全一样：同一个二进制、同样的 `--input-type zmq_manager`、同样的 5556/5557。

两个流程**不能同时跑**：`g1_client` 和 RAX 的 robot service 都会往 5556 发命令，只能有一个。

---

## 11. 和 sim 流程的差别（一句话版）

sim（`3/eng`）和真机用的是**同一个 robot service、同一套工具、同一套安检规矩**，控制代码不用改。差别全在周围：钥匙和吊架在 sim 里是脚本自动做的，真机是你手动做；sim 有真值能自动打分、摔了自动重启，真机没有真值、摔了就是事故；走路标定表和转向安全阈值要在真机重新量；相机深度有噪声；SONIC 在 sim 里走本机回环还能放慢时间，真机实时、不能放慢。完整对照表在 `REAL_ROBOT_DEPLOY.md` 第六节。
