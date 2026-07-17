<img src="assets/yummyimu-product-title.png" alt="YummyIMU 产品图" width="108" style="height: auto; object-fit: contain;" align="right">

# YummyIMU 用户手册

## 1. 产品简介

YummyIMU 是一款面向高稳定姿态测量的双 IMU 融合模块，内置 BMI088 与 BMI270/BMI260/BMI220 双传感器通道，支持恒温控制、自动零漂校准、偏航比例补偿、姿态融合与二维惯性导航。

产品特性：

- 双 IMU 融合输出，抗短时冲击与随机噪声能力更强。
- 双通道恒温闭环控制，自带加热电路，典型工作点 40°C。
- 上电自动加载零漂参数；首次使用可自动完成零漂校准。
- 典型零漂低于 1°/h。
- 支持 `USE`、`DEBUG`、`BINARY` 与 `INS` 四种输出模式。
- 支持串口配置、状态查询、参数保存与自动重启。

## 2. 快速开始

1. 固定模块并保持静止。
2. 上电，等待 LED 进入常亮。
3. 打开串口：`921600, 8N1`。
4. 默认 `USE` 模式输出融合姿态：

```text
Pitch,Roll,Yaw
```

示例：

```text
1.234,-0.567,12.345
```

角度单位均为度。

## 3. LED 状态

| 状态 | 含义 |
| --- | --- |
| 快闪 | 异常，例如 IMU 缺失或温度异常 |
| 慢闪 | 正在加热，尚未到达目标温度 |
| 3 秒亮灭 | 正在零漂校准 |
| 常亮 | 状态正常，正在输出数据 |
| 熄灭 | 正在回复命令或暂未输出 |

## 4. 串口输出

### 4.1 模式与频率

| 模式 | 输出内容 | 最高频率 |
| --- | --- | --- |
| `USE` | 融合姿态，ASCII | 500 Hz |
| `DEBUG` | 双 IMU 与融合姿态，ASCII | 100 Hz |
| `BINARY` | 融合姿态，8 字节二进制包 | 1000 Hz |
| `INS` | 二维位置与速度，ASCII | 100 Hz |

`RATE` 只接受可由 1000 Hz 整数分频的频率：

```text
1,2,4,5,8,10,20,25,40,50,100,125,200,250,500,1000
```

同时必须满足当前模式的最高频率。切换模式时，超出上限的已保存频率会自动降到该模式上限，并回复 `RATE:CLAMPED:<Hz>`。

### 4.2 USE 模式

每行输出融合姿态，顺序为：

```text
Pitch,Roll,Yaw
```

三个字段均为十进制度数，每行以 `\n` 结束。

### 4.3 DEBUG 模式

每行输出两颗真实 IMU 与融合姿态，共 9 个十进制度数：

```text
BMI088_Pitch,BMI088_Roll,BMI088_Yaw,
BMI270_Pitch,BMI270_Roll,BMI270_Yaw,
Virtual_Pitch,Virtual_Roll,Virtual_Yaw
```

以上三行仅用于展示字段顺序，实际数据在同一行连续输出。

### 4.4 BINARY 模式

每个姿态包固定为 8 字节：

| 字节 | 字段 | 格式 |
| --- | --- | --- |
| 0 | 包头 | 固定 `0xA5` |
| 1-2 | Pitch | `int16`，小端 |
| 3-4 | Roll | `int16`，小端 |
| 5-6 | Yaw | `int16`，小端 |
| 7 | 校验和 | 字节 0 至 6 累加和的低 8 位 |

角度解码公式：

```text
角度(°) = int16_le / 100.0
```

角度范围为 `[-180°, 180°)`，分辨率为 `0.01°`。

### 4.5 INS 模式

启动惯导后，每行输出二维位置与速度：

```text
PX,PY,VX,VY
```

| 字段 | 单位 | 含义 |
| --- | --- | --- |
| `PX`、`PY` | m | 相对起点的水平位置 |
| `VX`、`VY` | m/s | 水平速度 |

坐标系由 `INS START` 建立：X 轴为启动时模块 X 轴在水平面上的投影，Y 轴构成右手水平坐标系。系统不计算或输出 Z 轴位置与速度。

尚未设置起点时，每秒输出一次：

```text
INS:WAIT_START
```

惯导数据失效时，每秒输出一次：

```text
ERROR:INS_DATA_INVALID
```

纯惯性位置会随时间累积误差，`INS` 模式适合短时间相对位移测量。需要重新建立起点时可再次发送 `INS START`。

## 5. 串口命令

命令使用 ASCII 文本，以 `\r`、`\n` 或 `\r\n` 结束。命令建议使用大写，参数可用空格、Tab 或逗号分隔。

```text
COMMAND ARG1 ARG2
```

模块只处理完整且有效的命令。普通有效命令会暂停实时输出约 3 秒，并回复：

```text
OK
RECV:<收到的命令>
```

配置类命令成功后会自动保存至 Flash 并重启；`STATUS`、`LIST`、`TDRIFT` 不保存配置。`REBOOT` 只执行重启，不修改 Flash。`INS START` 是例外：它不会暂停实时输出、不会写入 Flash，也不会重启。

## 6. 命令列表

| 命令 | 参数 | 说明 |
| --- | --- | --- |
| `LIST` | 无 | 输出可用命令列表 |
| `REBOOT` | 无 | 立即重新启动模块，不修改 Flash 参数 |
| `STATUS` | 可选：`CONFIG`、`IMU`、`BIAS`、`QUALITY`、`HEAT`、`DIAG`、`INS` | 查询状态 |
| `CAL` | `WAIT_S RECORD_S` | 手动零漂校准；等待 0-600 秒，记录 1-600 秒 |
| `TDRIFT START` | 可选：`RATE_C_PER_MIN REPORT_HZ` | 从当前温度开始连续升温并输出温漂数据 |
| `TDRIFT STATUS` | 无 | 查询连续温漂测试状态 |
| `TDRIFT STOP` | 无 | 停止测试并恢复原目标温度 |
| `TEMP` | `20-85` | 设置目标温度，单位 °C |
| `BAUD` | `115200`、`230400`、`460800`、`921600` | 设置波特率 |
| `RATE` | 见 4.1 节 | 设置当前模式的输出频率 |
| `MODE` | `USE`、`DEBUG`、`BINARY`、`INS` | 设置输出模式 |
| `IMU` | `DUAL`、`BMI088`、`BMI270`、`AUTO` | 设置 IMU 来源 |
| `YAWCAL` | `BMI088_ERR BMI270_ERR` | 设置两颗 IMU 的单圈偏航误差 |
| `INS START` | 无 | 清零二维位置、速度并建立新起点 |

`IMU DUAL` 要求两颗 IMU 均正常；`BMI088` 或 `BMI270` 允许只使用指定传感器；`AUTO` 自动选择当前可用传感器。命令中的 `BMI270` 表示第二 IMU 通道，同时适用于 BMI270、BMI260 与 BMI220。

`YAWCAL` 参数单位为度/圈，范围为 `-30` 至 `30`。单圈误差可按下式获得：

```text
ERR = (传感器累计角度 - 实际累计角度) / 实际圈数
```

### 6.1 连续温漂测试

测试前应使用 `IMU DUAL`，确认两颗 IMU 均在线，并将模块静止放置。发送：

```text
TDRIFT START [RATE_C_PER_MIN] [REPORT_HZ]
```

- `RATE_C_PER_MIN` 为目标温度升高速率，范围 `0.1-60.0 °C/min`，默认 `1.0 °C/min`。
- `REPORT_HZ` 可取 `1`、`2`、`4`、`5`、`8`、`10`，默认 `10 Hz`。
- 测试从两颗 IMU 当前温度中的较高值开始，线性升温至 `70°C`。
- 到达 `70°C` 后继续恒温，待两颗 IMU 均稳定在目标值附近后自动结束。
- 测试期间暂停普通姿态数据，只输出 `TDRIFT` 数据；建议使用 `921600` 波特率。
- 测试不会修改 Flash。完成、停止或异常退出后，模块恢复测试前的目标温度。

例如：

```text
TDRIFT START 1.0 10
```

启动回复：

```text
OK
RECV:TDRIFT START 1.0 10
TDRIFT:START
TDRIFT:CONFIG,START_C=<value>,END_C=70.000,RATE_C_PER_MIN=1.000,REPORT_HZ=10
TDRIFT:COLUMNS,SEQ,ELAPSED_MS,TARGET_C,BMI088_TEMP_C,BMI270_TEMP_C,BMI088_GX_RADPS,BMI088_GY_RADPS,BMI088_GZ_RADPS,BMI270_GX_RADPS,BMI270_GY_RADPS,BMI270_GZ_RADPS,SAMPLES
```

连续数据行采用固定 CSV 列顺序：

```text
TDRIFT:DATA,<SEQ>,<ELAPSED_MS>,<TARGET_C>,<BMI088_TEMP_C>,<BMI270_TEMP_C>,<BMI088_GX_RADPS>,<BMI088_GY_RADPS>,<BMI088_GZ_RADPS>,<BMI270_GX_RADPS>,<BMI270_GY_RADPS>,<BMI270_GZ_RADPS>,<SAMPLES>
```

| 字段 | 含义 |
| --- | --- |
| `SEQ` | 从 0 开始递增的数据序号 |
| `ELAPSED_MS` | 从测试启动开始计算的毫秒时间 |
| `TARGET_C` | 当前线性升温目标，单位 °C |
| `BMI088_TEMP_C`、`BMI270_TEMP_C` | 对应采集窗口内的实际平均温度，单位 °C |
| `BMI088_G*_RADPS`、`BMI270_G*_RADPS` | 未扣除 Flash 零漂的各传感器原生三轴角速度均值，单位 rad/s |
| `SAMPLES` | 本行均值包含的 1kHz 原始样本数量 |

上位机只需处理以 `TDRIFT:DATA,` 开头的行，并分别使用两颗 IMU 的实际温度作为横轴。模块必须保持静止；测试期间的任何转动都会直接进入角速度数据。

测试过程还可能输出：

```text
TDRIFT:HOLD,TARGET_C=70.000
TDRIFT:DONE,ELAPSED_MS=<value>,TOTAL_SAMPLES=<value>
TDRIFT:STOPPED,ELAPSED_MS=<value>,TOTAL_SAMPLES=<value>
TDRIFT:ERROR,REASON=<reason>,ELAPSED_MS=<value>
```

`REASON` 可为 `DUAL_IMU_LOST`、`TEMPERATURE_INVALID` 或 `END_TEMPERATURE_TIMEOUT`，分别表示双 IMU 状态丢失、温度数据异常或 70°C 恒温等待超时。

发送 `TDRIFT STATUS` 可查询阶段、当前目标和实际温度；发送 `TDRIFT STOP` 可随时停止。测试运行时，除 `TDRIFT`、`STATUS`、`LIST` 和 `REBOOT` 外的命令会回复 `ERROR:TDRIFT_BUSY`。

## 7. 惯导操作

### 7.1 启动与复位起点

1. 发送 `MODE INS`。模块保存模式并重启；输出频率高于 100 Hz 时会自动降为 100 Hz。
2. 等待模块工作正常。未设置起点时会输出 `INS:WAIT_START`。
3. 将模块置于希望作为原点的位置，建议保持静止，然后发送 `INS START`。
4. 收到 `INS:STARTED:<计数>` 后，开始解析 `PX,PY,VX,VY`。

`INS START` 的正常回复为：

```text
OK
RECV:INS START
INS:START_QUEUED
INS:STARTED:<IMU_UPDATE_COUNT>
```

可以反复发送 `INS START`。每次执行都会把 X/Y 位置、X/Y 速度以及本地航向原点清零，并从当前位置重新积分。

当前版本采用快速起点初始化，无需进行六面加速度校准。发送 `INS START` 后，模块会在下一帧有效 IMU 数据到达时建立起点并立即进入积分；为减小初始误差，启动时仍建议保持静止。

### 7.2 启动提示

起点建立后可能追加以下提示：

| 输出 | 含义 |
| --- | --- |
| `WARN:INS_START_MOVING` | 设置起点时模块仍在运动，初始误差可能增大 |
| `WARN:INS_ACCEL_BIAS_INVALID` | 加速度零偏估计尚未有效，位移漂移可能增大 |
| `WARN:INS_HEADING_INVALID` | 启动时航向参考无效，局部坐标方向可能不可靠 |
| `ERROR:INS_MODE_REQUIRED` | 当前不是 `INS` 模式，不能执行 `INS START` |

### 7.3 STATUS INS

发送 `STATUS INS` 可查询当前惯导状态。回复首先包含固件版本，例如 `FW:1.4`，随后输出：

```text
INS_STATE:WAIT_START|RUNNING|FAULT
INS_TIME_MS:<value>
INS_STATIONARY:0|1
INS_BIAS_VALID:0|1
INS_FAULT:0|1|2
INS_X:<value>
INS_Y:<value>
INS_VX:<value>
INS_VY:<value>
```

`INS_FAULT` 含义：`0` 为正常，`1` 为融合采样无效，`2` 为惯导数值异常。

## 8. 状态查询

所有 `STATUS` 回复的第一行均为 `FW:<固件版本>`。不带参数的 `STATUS` 返回配置与 IMU 概况。

| 命令 | 主要返回内容 |
| --- | --- |
| `STATUS CONFIG` | 波特率、输出频率、模式、IMU 来源、目标温度、偏航补偿 |
| `STATUS IMU` | 当前传感器、芯片型号、在线状态与温度 |
| `STATUS BIAS` | Flash 中保存的两颗 IMU 三轴零漂 |
| `STATUS QUALITY` | 两颗 IMU 的三轴角速度方差与质量评分 |
| `STATUS HEAT` | 目标温度、两颗 IMU 温度与加热输出 |
| `STATUS DIAG` | 量程饱和计数、IMU 更新计数与超时计数 |
| `STATUS INS` | 惯导状态、时间、位置、速度与故障码 |

## 9. 上电与校准

模块上电后会自动读取已保存参数。

- 有有效零漂参数：加载参数并进入工作状态。
- 无有效零漂参数：等待两颗 IMU 温度与目标值相差不超过 2°C，然后自动进行 30 秒零漂校准。

自动校准期间可能输出：

```text
CAL_WAIT:AUTO_TEMP
CAL_WAIT:T088=39.875,T270=40.012
CAL_WAIT:DONE
CAL:START
CAL_REMAIN:30
CAL:DONE
BMI088_GYRO_SCORE:<0-90>
BMI270_GYRO_SCORE:<0-90>
```

进入 `CAL_REMAIN` 后请保持静止。等待时间和记录时间均按实际秒计时，记录阶段以 1 kHz 采样，`CAL_REMAIN` 按实际经过时间每秒递减。`CAL` 命令中的两个时间参数仅用于本次校准，不作为长期配置保存；校准结果会保存并在重启后生效。

每次自动或手动零漂校准都会同步计算三轴角速度方差和质量评分，并保存到 Flash。发送 `STATUS QUALITY` 可查询：

```text
BMI088_GYRO_VARIANCE_X:<value>
BMI088_GYRO_VARIANCE_Y:<value>
BMI088_GYRO_VARIANCE_Z:<value>
BMI088_GYRO_SCORE:<0-90>
BMI270_GYRO_VARIANCE_X:<value>
BMI270_GYRO_VARIANCE_Y:<value>
BMI270_GYRO_VARIANCE_Z:<value>
BMI270_GYRO_SCORE:<0-90>
```

方差单位为 `(rad/s)^2`。两颗 IMU 的三个轴分别按 30 分评价，单颗最高 90 分，双 IMU 合计最高 180 分。零漂占 20%，方差占 80%；本批实测样本的逐轴平均水平对应约 20 分，数据优于平均水平时分数提高，劣于平均水平时分数快速下降。升级评分标准后，已有 Flash 校准记录会在首次启动时自动重算分数，不需要重新采集零漂。

发送 `REBOOT` 后模块回复以下内容并立即重启：

```text
OK
RECV:REBOOT
REBOOTING
```

## 10. 错误信息

| 输出 | 含义 |
| --- | --- |
| `ERROR:BMI088_MISSING` | BMI088 概要故障：加速度计和陀螺仪均未返回有效 ID |
| `ERROR:BMI270_MISSING` | 第二 IMU 通道概要故障：未返回 BMI270/BMI260/BMI220 的有效 ID |
| `ERROR:BMI088_INIT:<flags>` | BMI088 身份正确，但至少一个内部芯片初始化失败；bit0 为加速度计，bit1 为陀螺仪 |
| `ERROR:<model>_INIT:<flags>` | 第二 IMU 身份正确，但配置文件、电源或量程寄存器初始化失败 |
| `ERROR:BMI088_ID_MISMATCH:A=...,G=...` | BMI088 两个独立片选读到的 ID 与预期不符 |
| `ERROR:BMI270_ID_MISMATCH:<id>` | 第二 IMU 片选读到非 BMI270/BMI260/BMI220 ID |
| `ERROR:NO_ACTIVE_IMU` | 当前没有可用 IMU |
| `ERROR:IMU_INIT_FAILED` | IMU 初始化失败 |
| `ERROR:TEMP_WAIT_TIMEOUT` | 自动校准等待升温超时 |
| `ERROR:CAL_FAILED` | 零漂校准失败 |
| `ERROR:SAVE_FAILED` | 参数保存失败 |
| `ERROR:TDRIFT_REQUIRES_DUAL_IMU` | 温漂测试要求两颗 IMU 均在线并处于 DUAL 模式 |
| `ERROR:TDRIFT_TEMPERATURE_INVALID` | 温漂测试启动时温度数据无效 |
| `ERROR:TDRIFT_START_TEMPERATURE` | 当前温度已达到或超过 70°C，无法开始升温测试 |
| `ERROR:TDRIFT_PARAM` | 温漂测试速率或回传频率超出范围 |
| `ERROR:TDRIFT_ALREADY_ACTIVE` | 温漂测试已经在运行 |
| `ERROR:TDRIFT_NOT_ACTIVE` | 当前没有正在运行的温漂测试 |
| `ERROR:TDRIFT_BUSY` | 温漂测试期间收到了不允许执行的其他命令 |
| `ERROR:STATUS_ARG` | `STATUS` 子命令错误 |
| `ERROR:INS_MODE_REQUIRED` | 当前模式不允许启动惯导 |
| `ERROR:INS_DATA_INVALID` | 惯导输入或计算结果无效 |

IMU 异常时，模块不输出实时数据，只会约每秒输出一次概要错误和结构化诊断行。结构化行使用 `CAUSE=` 标明建议排查方向：

| `CAUSE` | 判断依据与排查方向 |
| --- | --- |
| `NO_RESPONSE` | 连续读取仅得到 `0x00/0xFF`。优先检查供电、焊接、片选、SPI 时钟和 MISO；该结果表示“总线无响应”，不能单凭软件绝对断定虚焊 |
| `WRONG_PART_OR_CS` | 稳定读到非预期 ID。检查器件型号、贴装位号以及片选映射；`DETECTED=` 会在 ID 已知时给出检测到的型号 |
| `CS_SWAPPED` | BMI088 加速度计与陀螺仪的两个片选 ID 恰好对调，优先检查 CS 接线或原理图映射 |
| `UNSTABLE_LINK` | 多次 ID 读取结果发生变化，或有效 ID 与无响应/其他 ID 交替出现。检查焊接接触、供电上升、SPI 信号完整性和频率 |
| `INIT_CONFIG` | 芯片 ID 正确，但配置加载、电源控制或寄存器回读失败。根据 `STAGE`、`REG`、`EXP`、`READ` 等字段定位 |
| `INIT_UNKNOWN` | 初始化返回了未能进一步归类的错误标志，需结合 `STATUS IMU` 和调试器检查 |

`UNSTABLE_LINK` 行中的 `V/N/O/T` 分别表示有效 ID、无响应 ID、其他 ID 的读取次数及 ID 跳变次数。第二 IMU 的 `ID0/ID1/ID` 分别表示复位前、复位后和最终读取到的 ID。发送 `STATUS IMU` 可查询 BMI088 两路独立初始化错误码，以及第二 IMU 的复位前后 ID。

## 11. 上位机解析

- ASCII 模式按行读取，遇到 `\r` 或 `\n` 后解析。
- `USE` 按 3 个浮点数解析，`DEBUG` 按 9 个浮点数解析，`INS` 按 4 个浮点数解析。
- `BINARY` 按固定 8 字节组包，并同时检查 `0xA5` 包头与校验和。
- `OK`、`RECV:`、`FW:`、`KEY:VALUE`、`WARN:`、`ERROR:`、`REBOOTING` 与 `INS:...` 不属于实时数据。
- 温漂曲线只解析 `TDRIFT:DATA,` 行；`TDRIFT:CONFIG`、`TDRIFT:COLUMNS`、`TDRIFT:HOLD`、`TDRIFT:DONE`、`TDRIFT:STOPPED` 和 `TDRIFT:ERROR` 是控制信息。
- 在 `BINARY` 模式下发送命令时，命令回复仍为 ASCII；上位机应在命令窗口内暂停二进制帧解析。
- 配置类命令成功后模块会重启，上位机应重新等待串口输出。
