<img src="assets/yummyimu-product.png" alt="YummyIMU 产品图" width="280" align="right">

# YummyIMU 用户手册

## 1. 产品简介

YummyIMU 是一款面向高稳定姿态测量的双 IMU 融合模块，内置 BMI088 与 BMI270/BMI220 双传感器通道，支持恒温控制、自动零漂校准、偏航比例补偿和串口数据输出。

产品特性：

- 双 IMU 融合输出，抗短时冲击与随机噪声能力更强。
- 恒温闭环控制，自带加热电路，典型工作点 40°C。
- 上电自动加载零漂参数；首次使用可自动完成零漂校准。
- 典型零漂低于 1°/h。
- 支持 `USE` 正常输出模式与 `DEBUG` 调试输出模式。
- 支持串口配置、状态查询、参数保存与自动重启。

## 2. 快速开始

1. 固定模块，保持静止。
2. 上电，等待 LED 进入常亮。
3. 打开串口：`921600, 8N1`。
4. 正常使用时读取 `USE` 模式输出：

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

实时姿态数据为 ASCII 文本，逗号分隔，每行以 `\n` 结束。

### USE 模式

正式使用模式，只输出融合姿态：

```text
Pitch,Roll,Yaw
```

### DEBUG 模式

调试模式，输出两颗真实 IMU 与融合姿态：

```text
BMI088_Pitch,BMI088_Roll,BMI088_Yaw,BMI270_Pitch,BMI270_Roll,BMI270_Yaw,Virtual_Pitch,Virtual_Roll,Virtual_Yaw
```

输出频率可通过 `RATE` 设置，范围 `1-500 Hz`。

## 5. 串口命令

命令为 ASCII 文本，以换行结束。参数可用空格、Tab 或逗号分隔。

```text
COMMAND ARG1 ARG2
```

模块收到有效命令后，会暂停姿态输出约 3 秒，并回复：

```text
OK
RECV:<收到的命令>
```

配置类命令成功后会自动保存并重启；查询类命令不会重启。

## 6. 命令列表

| 命令 | 参数 | 说明 |
| --- | --- | --- |
| `LIST` | 无 | 查询可用命令 |
| `STATUS` | 可选：`CONFIG`、`IMU`、`BIAS`、`HEAT`、`DIAG` | 查询状态 |
| `CAL` | `WAIT_S RECORD_S` | 手动零漂校准 |
| `TEMP` | `20-85` | 设置目标温度，单位 °C |
| `BAUD` | `115200`、`230400`、`460800`、`921600` | 设置波特率 |
| `RATE` | `1-500` | 设置输出频率，单位 Hz |
| `MODE` | `USE`、`DEBUG` | 设置输出模式 |
| `IMU` | `DUAL`、`BMI088`、`BMI270`、`AUTO` | 设置 IMU 来源 |
| `YAWCAL` | `BMI088_ERR BMI270_ERR` | 设置偏航比例补偿 |

## 7. 上电与校准

模块上电后会自动读取已保存参数。

- 有有效零漂参数：直接进入工作状态。
- 无有效零漂参数：等待温度接近目标值后，自动进行 30 秒零漂校准。

自动校准期间可能输出：

```text
CAL_WAIT:AUTO_TEMP
CAL_WAIT:T088=39.875,T270=40.012
CAL_WAIT:DONE
CAL:START
CAL_REMAIN:30
CAL:DONE
```

进入 `CAL_REMAIN` 后请保持静止。

## 8. 错误信息

| 输出 | 含义 |
| --- | --- |
| `ERROR:BMI088_MISSING` | BMI088 未检测到 |
| `ERROR:BMI270_MISSING` | BMI270/BMI220 未检测到 |
| `ERROR:NO_ACTIVE_IMU` | 当前没有可用 IMU |
| `ERROR:IMU_INIT_FAILED` | IMU 初始化失败 |
| `ERROR:TEMP_WAIT_TIMEOUT` | 自动校准等待升温超时 |
| `ERROR:CAL_FAILED` | 零漂校准失败 |
| `ERROR:SAVE_FAILED` | 参数保存失败 |
| `ERROR:STATUS_ARG` | STATUS 子命令错误 |

IMU 异常时，模块不输出实时姿态数据，只会约每秒输出一次 `ERROR:` 行。

## 9. 上位机解析建议

- 按行读取，遇到 `\n` 后再解析。
- 实时姿态行只包含数字、负号、小数点和逗号。
- `USE` 模式按 3 个浮点数解析。
- `DEBUG` 模式按 9 个浮点数解析。
- `OK`、`RECV:`、`KEY:VALUE`、`ERROR:` 不属于实时姿态数据。
- 配置命令成功后模块会重启，上位机应重新等待串口输出。
