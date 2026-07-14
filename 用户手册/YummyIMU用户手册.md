<img src="assets/yummyimu-product-title.png" alt="YummyIMU 产品图" width="108" style="height: auto; object-fit: contain;" align="right">

# YummyIMU 用户手册

## 1. 产品简介

YummyIMU 是一款面向高稳定姿态测量的双 IMU 融合模块，内置 BMI088 与 BMI270/BMI220 双传感器通道，支持恒温控制、自动零漂校准、偏航比例补偿、姿态融合与二维惯性导航。

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

配置类命令成功后会自动保存至 Flash 并重启；`STATUS`、`LIST` 不保存配置。`INS START` 是例外：它不会暂停实时输出、不会写入 Flash，也不会重启。

## 6. 命令列表

| 命令 | 参数 | 说明 |
| --- | --- | --- |
| `LIST` | 无 | 输出可用命令列表 |
| `STATUS` | 可选：`CONFIG`、`IMU`、`BIAS`、`HEAT`、`DIAG`、`INS` | 查询状态 |
| `CAL` | `WAIT_S RECORD_S` | 手动零漂校准；等待 0-600 秒，记录 1-600 秒 |
| `TEMP` | `20-85` | 设置目标温度，单位 °C |
| `BAUD` | `115200`、`230400`、`460800`、`921600` | 设置波特率 |
| `RATE` | 见 4.1 节 | 设置当前模式的输出频率 |
| `MODE` | `USE`、`DEBUG`、`BINARY`、`INS` | 设置输出模式 |
| `IMU` | `DUAL`、`BMI088`、`BMI270`、`AUTO` | 设置 IMU 来源 |
| `YAWCAL` | `BMI088_ERR BMI270_ERR` | 设置两颗 IMU 的单圈偏航误差 |
| `INS START` | 无 | 清零二维位置、速度并建立新起点 |

`IMU DUAL` 要求两颗 IMU 均正常；`BMI088` 或 `BMI270` 允许只使用指定传感器；`AUTO` 自动选择当前可用传感器。命令中的 `BMI270` 同时适用于 BMI270 与 BMI220 通道。

`YAWCAL` 参数单位为度/圈，范围为 `-30` 至 `30`。单圈误差可按下式获得：

```text
ERR = (传感器累计角度 - 实际累计角度) / 实际圈数
```

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

发送 `STATUS INS` 可查询当前惯导状态。回复首先包含固件版本，例如 `FW:1.3`，随后输出：

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
```

进入 `CAL_REMAIN` 后请保持静止。`CAL` 命令中的等待时间和记录时间仅用于本次校准，不作为长期配置保存；校准结果会保存并在重启后生效。

## 10. 错误信息

| 输出 | 含义 |
| --- | --- |
| `ERROR:BMI088_MISSING` | BMI088 未检测到 |
| `ERROR:BMI270_MISSING` | BMI270/BMI220 未检测到 |
| `ERROR:NO_ACTIVE_IMU` | 当前没有可用 IMU |
| `ERROR:IMU_INIT_FAILED` | IMU 初始化失败 |
| `ERROR:TEMP_WAIT_TIMEOUT` | 自动校准等待升温超时 |
| `ERROR:CAL_FAILED` | 零漂校准失败 |
| `ERROR:SAVE_FAILED` | 参数保存失败 |
| `ERROR:STATUS_ARG` | `STATUS` 子命令错误 |
| `ERROR:INS_MODE_REQUIRED` | 当前模式不允许启动惯导 |
| `ERROR:INS_DATA_INVALID` | 惯导输入或计算结果无效 |

IMU 异常时，模块不输出实时数据，只会约每秒输出一次 `ERROR:` 行。

## 11. 上位机解析

- ASCII 模式按行读取，遇到 `\r` 或 `\n` 后解析。
- `USE` 按 3 个浮点数解析，`DEBUG` 按 9 个浮点数解析，`INS` 按 4 个浮点数解析。
- `BINARY` 按固定 8 字节组包，并同时检查 `0xA5` 包头与校验和。
- `OK`、`RECV:`、`FW:`、`KEY:VALUE`、`WARN:`、`ERROR:` 与 `INS:...` 不属于实时数据。
- 在 `BINARY` 模式下发送命令时，命令回复仍为 ASCII；上位机应在命令窗口内暂停二进制帧解析。
- 配置类命令成功后模块会重启，上位机应重新等待串口输出。
