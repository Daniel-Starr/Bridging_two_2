# Bridging_two_2 — 双板多路 UART 桥接（DMA 版）

基于 STM32U575VGT6 的双板 UART 透传桥接系统，与 Bridging_two 功能相同，但使用 DMA 传输实现更高吞吐量和可靠性。

## 硬件平台

- **MCU:** STM32U575VGT6 (Cortex-M33, LQFP100)
- **开发环境:** STM32CubeIDE / Keil MDK-ARM

## 外设使用

| 外设 | 波特率 | 用途 |
|------|--------|------|
| USART1 | 115200 | PC 串口通道 1 |
| UART4 | 9600 | PC 串口通道 2 |
| LPUART1 | 115200 | PC 串口通道 3 |
| USART3 | **921600** | 板间通信链路（高速） |

## 功能说明

功能与 Bridging_two（轮询版）完全一致：Board1 将多路 PC 串口数据封装帧协议后通过 USART3 发送给 Board2，Board2 追加 `*` 后回传。

## 相比轮询版的改进

- **DMA 接收：** 使用 `HAL_UARTEx_ReceiveToIdle_DMA` 实现空闲中断 + DMA 接收
- **板间链路提速：** USART3 波特率从 115200 提升至 921600
- **环形缓冲区：** 2048 字节 RX 队列 + 32 深度 TX 队列
- **中断安全：** 使用 PRIMASK 临界区保护环形缓冲区操作
- **TX DMA：** 发送端使用 DMA + busy 标志 + 队列排空机制
- **溢出诊断：** 内置溢出计数器用于调试
- **错误恢复：** DMA 异常后自动重启

## 项目结构

```
Bridging_two_2/
├── Bridging01/    # Board1 工程（主板）
└── Bridging02/    # Board2 工程（从板）
```
