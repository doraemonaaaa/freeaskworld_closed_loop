##### Baseline选择目的

选择 GPT-4o 驱动的 零样本 VLN 智能体 (VLN-Zero) 作为对比基准：尝试确立一个由最先进的基础模型在**不进行特定任务微调或主动询问机制**的情况下所能提供的性能下界。

VLN-Zero 基准代表了现代多模态大语言模型（MLLM）的“纯推理”能力。与依赖大规模数据集训练的传统监督学习方法不同，本基准运行在一个 “感知-思考-行动”的闭环中，完全依赖模型的通用知识来理解视觉观测和拓扑地图。这个基准使我们能够：

1、隔离推理能力： 将导航性能与训练数据的偏差解耦。

2、基准化基础模型： 评估通用目的模型 (GPT-4o) 在特定具身导航任务上的表现。

3、凸显差距： 展示被动指令跟随（即本基准）与 FreeAskWorld 所提出的主动澄清机制之间的能力局限与差异。



##### Baseline内在逻辑架构

1、感知与编码 (Perception \& Encoding)：

智能体接收以第一人称视角的 RGB 图像和模拟的鸟瞰图 (Top-down Map)。这些图像被编码为 Base64 格式并嵌入到提示词 (Prompt) 中。

2、结构化推理 (Structured Reasoning - CoT)：

提示词强制模型输出结构化的“思维链”，要求生成以下内容：

地图推理 (Map Reasoning)： 分析目标点相对于智能体的基数方向 (Cardinal Direction)。

视觉推理 (Camera Reasoning)： 分析当前视野内的可通行空间及障碍物。

导航推理 (Navigation Reasoning)： 结合地图与视觉数据的综合逻辑判断。

动作决策 (Action Decision)： 输出离散的动作 ID（前进、左转、右转、停止）

3、动作执行 (Action Execution)：

离散动作 ID 被解析并转换为 ROS SimulatorCommand 消息，在仿真中触发预定义的运动（例如：前进 0.5米 或 旋转 30°）



##### 量化指标与性能

1、核心思维链长度 (Core Thinking Chain Length)： 

该基准展现了高密度的推理过程。平均而言，显式的思维链（包含地图+视觉+导航推理）每步由大约 70 到 90 个 Token 组成。这种显式的结构化推理保证了可解释性，但也引入了显著的上下文开销。

2、Token 消耗与效率 (Token Consumption)： 由于每一步都包含高分辨率图像编码和详细的系统提示词，Token 使用量巨大。

平均提示词 Token (Prompt Tokens)： 每步约 1,950 - 2,050 Token。

平均生成 Token (Completion Tokens)： 每步约 70 - 90 Token。

每步总上下文： 约 2,100 Token。



