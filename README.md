# 嵌赛 工作区说明

本目录为嵌入式相关竞赛/实验工程集合的根工作区。

## 目录说明

| 路径 | 内容说明 |
|------|----------|
| `.cursor/skills/hardware-solution/` | NextBoard 硬件方案设计 Skill（`SKILL.md` 与 `references/`）；对话中可用 `$hardware-solution` 触发，详见根目录 `AGENTS.md`。 |
| `docs/hardware/` | `$hardware-solution` 产出：`01`～`06`、**BOM**（`07`）、**布线/上电**（`08`）、**投产核对**（`09-pre-order-checklist.md`）等。 |
| `code/` | 固件与脚本。含 `xiaoche_*.ino`（UART 电机板）、**`newboard_5_18.ino`**（AT8236 直驱 + 霍尔 AB）、`mg310_at8236_test.ino`（单电机联调）、`openmv_person_detect.py` 等；`数据手册/` 为历史运行日志。 |
| `ESP32S3_Phone_HTTP_Cmd/` | ESP32-S3 相关工程目录，内含同名 `.ino` 主文件；请用 **Arduino IDE** 或 **Arduino CLI** 打开该文件夹后编译、选择开发板与串口再上传。 |
| `md/` | Agent 会话产出的 Markdown 说明归档；子文件夹名对应该次任务的精炼主题。 |
| 根目录 `作品名称与作品简介.txt` | 作品文字说明（名称与简介），与代码分离存放。 |

## 使用方式

1. **固件开发**：进入 `ESP32S3_Phone_HTTP_Cmd/`，用 Arduino 工具链打开 `.ino` 工程并按板卡要求连接 USB 烧录。  
2. **查阅日志**：在 `code/数据手册/` 下按日期筛选 `log_*.txt` 文本。  
3. **说明文档**：查看 `md/` 下各子目录中的 `.md` 文件。

## OpenMV 人像大致方位（`code/openmv_person_detect.py`）

用 **LAB 肤色块**估计「人像」在画面中的水平位置（无需 Haar、无需 SD）。OpenMV IDE 打开脚本连接相机运行即可；可选 UART 把结果发给 ESP32。

### 用法

1. OpenMV IDE 打开 `code/openmv_person_detect.py`，运行。  
2. 与主控联调：**共地**，**P4（TX）→ 主控 RX**，波特率 `UART_BAUD`（默认 115200）。  
3. **串口协议**：每行一个字符 + 换行 —— **`L`** 目标中心在画面竖直中线**左侧**，**`R`** 在**右侧**，**`C`** 在中线附近（死区内），**`N`** 未检测到色块。

### 可调项（脚本顶部）

| 常量 | 含义 |
|------|------|
| `FRAME_SIZE` | 分辨率，如 `sensor.QVGA`。 |
| `CENTER_DEAD_PX` | 中线左右各放宽多少像素算「正中」输出 `C`；设为 `0` 则只有 `L`/`R`（以半宽为界）。 |
| `SKIN_*` / `MIN_BLOB_WH` | 肤色 LAB 阈值、最小块尺寸，按环境微调。 |
| `USE_UART` / `UART_BAUD` | 是否发串口及波特率。 |

## 新电机驱动板固件（`code/newboard_5_18.ino`）

基于 **`xiaoche_5_3.ino`** 的 WiFi 网页、航向保持、IMU/超声/LCD/OTA 逻辑；**不再使用 UART 电机板**，由 ESP32 直接驱动两颗 **AT8236**（MG310 四线：电机 + 霍尔 AB）。

### 接线（默认引脚，可在源码顶部修改）

| 功能 | GPIO |
|------|------|
| 电机 M1 IN1 / IN2 | 4 / 5 |
| 电机 M3 IN1 / IN2 | 6 / 7 |
| 编码器 M1 霍尔 A / B | 9 / 10 |
| 编码器 M3 霍尔 A / B | 15 / 16 |
| 超声波 TRIG / ECHO | 17 / 18 |

（与正点原子排针一致：未使用 8/11/12/13 等未引出脚；若改线只改 `newboard_5_18.ino` 顶部常量。）

**必须** ESP32 与驱动板 **共地**；VM **12 V** 接驱动板。确认 PCB 上 **IN1≠IN2、OUT1≠OUT2**（勿短接）。

### 烧录与使用

1. Arduino IDE 选择 **ESP32-S3**，打开 `code/newboard_5_18.ino` 编译上传。  
2. 连接 WiFi 或 AP（与 5.3 相同），浏览器打开串口启动日志中的 IP。  
3. 网页 **PWM / 速度模式**、前进后退预设、左转/右转角、`/debug`、`/api/live` 用法与 5.3 一致。  
4. USB 串口 **`XCDBG1` / `XCDBG0`** 切换详细日志（同 5.2+）。  
5. 编码器每 **10 ms** 采样；`ENCODER_PULSES_PER_WHEEL_REV`（默认 520）需按 MG310 实测一圈脉冲校准。

### 单电机联调

`code/mg310_at8236_test.ino`：串口命令 `f`/`g` 全速、`p` 测 GPIO5、`s` 停止等，仅接一路 AT8236 时使用。

---

## 小车固件 USB 串口（`code/xiaoche_5_2.ino` / UART 电机板）

上电后 **默认静默**：只输出少量 `[run]` 运行行、WiFi/HTTP 摘要及错误类信息；电机下发的 `[TX]`、初始化细节、OTA 过程等 **默认不打印**。

在 **USB 串口监视器** 中输入下面内容后按 **回车**（换行）即可切换（区分大小写不敏感）：

| 指令 | 作用 |
|------|------|
| **`XCDBG1`** | 打开详细日志：打印 `[TX]`、WiFi 连接过程、LCD/IMU 初始化、Web/OTA 说明等；并打印一行当前状态快照。 |
| **`XCDBG0`** | 恢复默认静默模式。 |

启动时仍会打印一行固件提示，其中包含上述指令缩写说明。

### `code/xiaoche_5_3.ino`（5.3，UART 电机板）

- 经 **Serial2** 向独立电机板发 `$pwm` / `$spd`；编码器由电机板 `$MTEP` 上报。  
- 网页「全前进 / 全后退」预设、航向保持、左转/右转角等逻辑与 **newboard_5_18** 同源；新板请用 `newboard_5_18.ino`。  
- **IMU**：Pitch/Roll；航向由 **M1/M3 编码器差速** 估计（UART 版为 `$MTEP`，新板为板载 AB 相）。

## 维护约定（摘要）

- 新生成的说明类文档放在 `md/` 下合适子目录；代码类文件放在 `code/`（本仓库历史目录若与约定不一致，以不破坏已有打开路径为前提逐步整理）。  
- `xiaoche` 代码每次生成新版本时，文件名统一为 `xiaoche_month_day.ino`（如 `xiaoche_4_25.ino`，`month` 为当前月份，`day` 为当前日期）。  
- 对话与操作流水见根目录 `log.txt`，本文件只描述工程结构与用法。
