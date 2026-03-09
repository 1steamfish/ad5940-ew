# 解释

## ADCPolling

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