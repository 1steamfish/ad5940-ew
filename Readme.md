# 解释

## ADC 码转电压/电流换算 API

以下工具函数在 `ad5940lib/ad5940.c` 中实现，并在 `ad5940lib/ad5940.h` 中声明。

### `AD5940_ADCCode2Volt`

```c
float AD5940_ADCCode2Volt(uint32_t code, uint32_t ADCPga, float VRef1p82);
```

将 16 位原始 ADC 结果码转换为差分电压（单位：V）。

- `code`：ADC 原始结果（偏移二进制，`0x8000` 对应 0 V 差分）  
- `ADCPga`：ADC PGA 增益设置（`ADCPGA_1`、`ADCPGA_1P5`、`ADCPGA_2`、`ADCPGA_4`、`ADCPGA_9`）  
- `VRef1p82`：实际 1.82 V 参考电压（V）  
- 返回值：差分电压（V）

**示例（ADCPolling）：**
```c
float diff_volt = AD5940_ADCCode2Volt(rd, ADCPGA_1P5, 1.82f);
printf("差分电压: %.4fV, 单端电压: %.4fV\n", diff_volt, diff_volt + 1.11f);
```

### `AD5940_LPRtia2Ohm`

```c
float AD5940_LPRtia2Ohm(uint32_t LpTiaRtia);
```

将 LPTIA 内部 RTIA 选择常量（`LPTIARTIA_*`，见 `ad5940.h`）转换为标称电阻值（单位：Ω）。  
`LPTIARTIA_OPEN` 返回 `0.0f`。对于高精度测量，应使用 `AD5940_LPRtiaCal()` 标定后的值。

### `AD5940_HSRtia2Ohm`

```c
float AD5940_HSRtia2Ohm(uint32_t HsTiaRtia);
```

将 HSTIA 内部 RTIA 选择常量（`HSTIARTIA_*`，见 `ad5940.h`）转换为标称电阻值（单位：Ω）。  
`HSTIARTIA_OPEN` 返回 `0.0f`。对于高精度测量，应使用 `AD5940_HSRtiaCal()` 标定后的值。

### `AD5940_ADCCode2Current`

```c
float AD5940_ADCCode2Current(uint32_t code, uint32_t ADCPga, float VRef1p82, float Rtia);
```

通过 TIA（跨阻放大器）的反馈电阻将 ADC 结果码转换为电流（单位：A）。  
内部调用 `AD5940_ADCCode2Volt()` 获取差分电压，再除以 `Rtia` 得到电流：  
`I = V_diff / Rtia`

- `Rtia`：跨阻放大器反馈电阻（Ω）。可用 `AD5940_LPRtia2Ohm()` / `AD5940_HSRtia2Ohm()` 获取标称值，或传入标定值。  
- `Rtia == 0.0f` 时返回 `0.0f`。

**示例（LPTIA 电流测量）：**
```c
/* LPTIA 配置：PGA=1.5, RTIA=10kΩ */
float rtia = AD5940_LPRtia2Ohm(LPTIARTIA_10K);          /* 10000.0 Ω */
float current = AD5940_ADCCode2Current(adc_code, ADCPGA_1P5, 1.82f, rtia);
printf("电流: %.4f µA\n", current * 1e6f);
```

---


这个文件 AD5940_ADCPolling.c 是一个“ADC 轮询/查询（Polling）模式”的示例：把 AD5940 的 ADC 配好以后，不用 FIFO/DFT 之类的复杂链路，而是持续运行 ADC 转换，并通过检查“数据就绪”中断标志来读出最新的滤波结果，再把结果换算成电压打印出来。

它在做的事情按流程分解是：

- 复位与初始化  
  - `AD5940_HWReset()`：硬件复位 AFE  
  - `AD5940_Initialize()`：初始化 AFE 寄存器到可用状态

- ADC PGA 校准  
  - `AD5940_PGA_Calibration()` 内部调用 `AD5940_ADCPGACal()`，对 ADC PGA 做 offset/gain 校准，避免增益/零点误差影响测量。

- 配置功耗/带宽与测量通道  
  - `AD5940_AFEPwrBW(AFEPWR_LP, AFEBW_250KHZ)`：低功耗模式、250kHz 带宽  
  - `AD5940_AFECtrlS(AFECTRL_DACREFPWR|AFECTRL_HSDACPWR, bTRUE)`：打开 DAC 参考相关电源（示例要测量内部参考/偏置）  
  - `adc_base.ADCMuxP = ADCMUXP_VREF1P8DAC`、`adc_base.ADCMuxN = ADCMUXN_VSET1P1`：把 ADC 正端接到 1.8V DAC 参考，负端接到 1.1V（所以实际测的是两者差分，再加回 1.11 形成单端意义的电压）

- 配置数字滤波链路（SINC3 → SINC2+Notch）  
  - `adc_filter.ADCSinc3Osr = ADCSINC3OSR_4`  
  - `adc_filter.ADCSinc2Osr = ADCSINC2OSR_1333`  
  - `adc_filter.ADCRate = ADCRATE_800KHZ`（假设 ADC 时钟 16MHz 时，对应 800kSPS 的配置）  
  - `adc_filter.Sinc2NotchEnable = bTRUE`：启用 SINC2+Notch 模块  
  最终输出速率在注释里也算了：$800k / 4 / 1333 \approx 150$ Hz。

- 启动 ADC，并“轮询”数据就绪再读数  
  - `AD5940_INTCCfg(... AFEINTSRC_ALLINT, bTRUE)`：把中断源使能到 INTC1（这里不是用真正 MCU 中断处理函数，而是为了能查询标志位）  
  - `AD5940_ADCPowerCtrlS(bTRUE)`、`AD5940_ADCConvtCtrlS(bTRUE)`：上电并开始转换  
  - `while(1)` 循环中：
    - `AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_SINC2RDY)`：查询 SINC2 结果就绪标志  
    - 就绪后清标志、读 `AD5940_ReadAfeResult(AFERESULT_SINC2)`  
    - 每累计约 150 个样本（约 1 秒）打印一次：  
      - `diff_volt = AD5940_ADCCode2Volt(rd, ADCPGA_GAIN_SEL, 1.82)` 把 ADC 码转成差分电压  
      - 打印差分电压和“加上 1.11V 偏置后的电压”

一句话总结：它演示了“测内部参考(VREF1P8DAC 相对 VSET1P1) → 经过 SINC 滤波 → 通过查询 SINC2RDY 标志读结果 → 转电压并周期性 printf”的最小 ADC 工作闭环。

## ADCMeanFIFO

这个文件 AD5940_ADCMeanFIFO.c 演示的是：用 AD5940 的 **Statistic(统计) 模块对 ADC 数据做“均值(mean)”计算**，然后把“均值结果”写入 **FIFO**，MCU 通过 **FIFO 阈值中断**一次性读出一批均值结果并打印。

和你刚看的 “Polling 直接读 SINC2” 不同，它的关键点是 **SINC2+Notch 输出 → Statistic 求均值 → FIFO 存均值 → MCU 中断读 FIFO**。

按代码流程拆开看：

- 复位/初始化与模拟前端配置  
  - `AD5940_HWReset()`、`AD5940_Initialize()`：复位并初始化 AFE  
  - `AD5940_AFEPwrBW(AFEPWR_LP, AFEBW_250KHZ)`：低功耗、250kHz 带宽

- ADC 基本配置：选择要测的输入  
  - `adc_base.ADCMuxP = ADCMUXP_AVDD_2`：正端接到 AVDD/2（电源的一半）  
  - `adc_base.ADCMuxN = ADCMUXN_VSET1P1`：负端接 1.1V  
  - `adc_base.ADCPga = ADCPGA_1`：PGA=1  
  也就是说测的是 `(AVDD/2) - 1.1V` 的差分量（示例用途）。

- 数字滤波配置（和前一个例子类似）  
  - SINC3 OSR=4，SINC2 OSR=1333，ADCRate=800kSPS  
  - 这决定了送给后级的数据速率大约是 $800k/4/1333 \approx 150$ Hz

- 统计模块（本文件的核心）  
  - `stat_cfg.StatEnable = bTRUE`：打开统计模块  
  - `stat_cfg.StatSample = STATSAMPLE_128`：**每累计 128 个点计算一次 mean**  
  所以 mean 的更新频率大约是 $150/128 \approx 1.17$ Hz（大概每 0.85 秒一个均值）。

- FIFO 配置：FIFO 源选择 “MEAN”  
  - `fifo_cfg.FIFOSrc = FIFOSRC_MEAN`：FIFO 里装的是 **统计模块的 mean 结果**，不是原始 ADC/SINC 输出  
  - `fifo_cfg.FIFOThresh = 2`：FIFO 里累计到 2 个数据就触发阈值中断

- 中断与主循环读取  
  - 使能 `AFEINTSRC_DATAFIFOTHRESH`（在 INTC0），并清标志  
  - `while(1)` 里等 `AD5940_GetMCUIntFlag()`（表示 ISR 已经把 MCU 中断标志置位）  
  - 读 `AD5940_FIFOGetCnt()` 得到 FIFO 里有多少数据，再 `AD5940_FIFORd()` 批量读到 `ADCBuff`  
  - `printf("Get %d data, ADC Code[0]:%d\n", ...)` 打印读到的均值码

- 末尾 NOTE 很重要：均值码的格式  
  - 注释写了：**mean 结果已经去掉了 32768**（也就是从偏移编码转换成有符号意义）  
  - 电压换算（按注释）：若均值结果为 $n$，则  
    $$V = \frac{n}{32768}\cdot V_{ref}$$


## ADCNotchTest

这个文件 AD5940_ADCNotchTest.c 是一个 **Notch（陷波）滤波器的自检/回归测试程序**：它会遍历多组 ADC 采样率与 SINC2/SINC3 OSR 组合，自动配置 **SINC2+Notch**，用 **Sequencer** 触发一次采样，把结果从 FIFO 读出来，然后判断“经过 Notch 后的输出是否满足预期（接近 0）”，最后打印整体测试是否通过。

核心目的：验证在不同采样率/OSR 下，Notch 的延时线（DL）配置与输出行为是“可用且正确”的（尤其是 50/60Hz 陷波可用性）。

它做的事情分块看：

- `ad5940_notch_test(adc_rate, sinc3osr, sinc2osr)`：对某一组参数跑一次测试
  - 复位 + `AD5940_Initialize()`  
  - `AD5940_CLKCfg()`：根据 `adc_rate` 选择 16MHz/32MHz（`ADCRATE_800KHZ` vs `ADCRATE_1P6MHZ`）并设置系统时钟分频
  - 配 ADC MUX：`VREF1P8DAC` 对 `VSET1P1`，PGA=1.5，并开 DAC 参考电源
  - 配滤波：`BpNotch = bFALSE`（不旁路 notch，说明就是要测试 notch）、`Sinc2NotchEnable = bTRUE`

- `ad5940_sequencer_init(...)`：搭建一次“唤醒-采样-等待-停止”的 sequencer 流程，并顺便配置 FIFO/AGPIO
  - FIFO 源：`FIFOSRC_SINC2NOTCH`（注意：这里 FIFO 里装的是 **SINC2+Notch 的输出**）
  - 计算 Notch 是否支持 50/60Hz：`AD5940_Notch50HzAvailable()` / `AD5940_Notch60HzAvailable()`  
    - 如果可用，会打印对应 DL 以及大致的陷波点频率（代码里用公式估算）
  - `AD5940_ClksCalculate(... DATATYPE_NOTCH ...)` 算出需要 `SEQ_WAIT(WaitClks)` 等多久才能拿到 1 个 notch 输出点
  - 用 `AD5940_SEQGen...` 生成 sequencer 命令：开 ADC、等待稳定、开始转换+notch、再等待、然后停止
  - 配 AGPIO：把某些引脚设成 SYNC/INT/TRIG（主要用于调试/示波观察时序）

- 测试判定逻辑（`ad5940_notch_test` 的 while 循环里）
  - 等 `AD5940_GetMCUIntFlag()`（说明 FIFO 阈值中断到了）
  - 读 `AD5940_FIFOGetCnt()`，期望 **刚好 1 个样本**
  - 读出 `rd`，转电压：`ADCCode2Volt(...) + 1.11`，再做 `volt -= 1.82`，取绝对值
    - 直观理解：它把测得的“VREF1P8DAC 相对 VSET1P1 的差分”换算回接近 1.82V 的量，然后减去 1.82V 看误差
  - 阈值判断：误差 > 0.0005V（0.5mV）就判 FAIL，否则 PASS

- `AD5940_Main()`：跑全参数遍历
  - `adc_rate` 遍历 0..1（两档速率）
  - `sinc3` 遍历到 `ADCSINC3OSR_2`
  - `sinc2` 遍历到 `ADCSINC2OSR_1333`
  - 任一组合失败则最终 `FAILED`，否则 `SUCCEED`

一句话总结：它不是“演示如何连续采样”，而是一个自动化的 **Notch 滤波链路 + Sequencer + FIFO** 的组合测试，用来验证各个采样参数下 Notch 能工作且结果误差在规定阈值内。

如果你想把这个测试改成“只测某一组你关心的 OSR/速率”或者“把 50/60Hz 的目标频点和 DL 结果打印得更直观”，我也可以直接帮你改代码。

---

## FPGA Verilog 实现说明

`fpga/` 目录下包含了对 `AD5940_ADCPolling.c` 的 FPGA Verilog 等效实现，可在任何支持 Verilog-2012 的 FPGA 工具链或仿真器上使用。

### 文件说明

| 文件 | 说明 |
|------|------|
| `fpga/ad5940_spi_master.v` | AD5940 两阶段 SPI 主控器（Mode 0，支持 16/32-bit 寄存器宽度，可参数化时钟分频 `CLK_DIV`） |
| `fpga/ad5940_adc_polling.v` | 顶层 ADC 轮询逻辑：复位 → 写 22 个初始化寄存器 → 轮询 `SINC2RDY` → 读 `SINC2DAT` → 输出 `adc_valid`/`adc_data` |
| `fpga/tb_ad5940_adc_polling.v` | 仿真 Testbench，含行为级 SPI Slave 模型，5/5 样本全部通过测试 |

**顶层信号接口（`ad5940_adc_polling`）**

```
clk        输入    系统时钟（示例 50 MHz）
rst_n      输入    低有效复位
adc_valid  输出    ADC 结果就绪单周期脉冲
adc_data   输出    16-bit ADC 原始码（0x8000 = 差分零点）
state_out  输出    内部状态机编号（调试用）
afe_rst_n  输出    AD5940 硬件复位（低有效）
spi_cs_n   输出    SPI 片选（低有效）
spi_clk    输出    SPI 时钟
spi_mosi   输出    SPI 主→从数据
spi_miso   输入    SPI 从→主数据
```

**电压换算（与 C 代码一致）**

```
diff_volt = (adc_data / 32768.0 - 1.0) / 1.5 * 1.82   [V]
volt      = diff_volt + 1.11                             [V]
```

### 如何仿真

需要安装 [Icarus Verilog](https://bleyer.org/icarus/)：

```bash
# 编译
iverilog -g2012 -o sim.vvp \
    fpga/ad5940_spi_master.v \
    fpga/ad5940_adc_polling.v \
    fpga/tb_ad5940_adc_polling.v

# 运行仿真
vvp sim.vvp
```

预期输出：

```
[TB] 5 / 5 samples PASSED
[TB] ALL TESTS PASSED
```

---

## 如何新建分支并将本 PR 的代码合并进去

> **场景**：Copilot 已将 FPGA Verilog 代码推送到分支
> `copilot/implement-adc-polling-function`，你想在自己的工作分支中使用这些代码。

### 方法一：基于 Copilot 分支新建你的工作分支

```bash
# 1. 克隆仓库（如已克隆可跳过）
git clone https://github.com/1steamfish/ad5940-ew.git
cd ad5940-ew

# 2. 拉取最新远程分支信息
git fetch origin

# 3. 基于 Copilot 分支创建你自己的新分支
git checkout -b my-fpga-work origin/copilot/implement-adc-polling-function

# 现在你的分支已经包含了所有 FPGA 代码，可以在此基础上继续开发
```

### 方法二：将 Copilot 分支合并到你已有的分支

```bash
# 1. 切换到你的目标分支（例如 main 或你自己的开发分支）
git checkout main          # 或替换为你的分支名

# 2. 拉取最新内容
git pull origin main

# 3. 将 Copilot 分支合并进来
git merge origin/copilot/implement-adc-polling-function

# 4. 如有冲突，手动解决后执行
git add .
git commit -m "Merge FPGA ADC polling implementation"

# 5. 推送到远程
git push origin main       # 或替换为你的分支名
```

### 方法三：通过 GitHub 网页合并 Pull Request

1. 打开本仓库的 [Pull Requests](https://github.com/1steamfish/ad5940-ew/pulls) 页面
2. 找到包含 `fpga/` 目录文件的 PR
3. 点击 **"Merge pull request"** → **"Confirm merge"**
4. 合并后在本地执行 `git pull origin main` 同步最新代码

### 合并后目录结构

```
ad5940-ew/
├── fpga/
│   ├── ad5940_spi_master.v       ← SPI 主控器
│   ├── ad5940_adc_polling.v      ← ADC 轮询顶层
│   ├── tb_ad5940_adc_polling.v   ← 仿真 Testbench
│   └── .gitignore                ← 排除仿真编译产物
├── AD5940_ADCPolling.c
├── AD5940_ADCMeanFIFO.c
├── AD5940_ADCNotchTest.c
└── ...
```
