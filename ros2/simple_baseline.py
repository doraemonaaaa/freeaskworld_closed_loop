

class SimpleBaseline():
    """
    这是你要的基类。
    它封装了 ROS 的循环，强制执行 'Input -> Inference -> Output' 的逻辑。
    用户只需要重写 InputData 和 Inference。
    """
    def __init__(self):
        super().__init__('simple_baseline')

    # def _control_loop(self):
    #     """
    #     [模板方法] 这是闭环控制的核心驱动。
    #     不要修改这个方法，它定义了数据流。
    #     """
    #     # 1. 检查数据是否就绪
    #     if self.latest_rgb is None or self.latest_depth is None:
    #         self.get_logger().warn("Waiting for sensor inputs...", throttle_duration_sec=2)
    #         return

    #     # 2. 数据预处理 (调用用户的逻辑)
    #     # 这里把 raw data 传给 InputData，用户可以决定取哪些
    #     model_inputs = self.InputData(
    #         rgb=self.latest_rgb,
    #         depth=self.latest_depth,
    #         pose=self.latest_pose,
    #         rot=self.latest_rot
    #     )

    #     # 3. 模型推理 (调用用户的逻辑)
    #     # 输出应该是 NavigationCommand 消息
    #     nav_cmd = self.Inference(model_inputs)

    #     # 4. 执行输出
    #     self.send_command(nav_cmd)

    # # -------------------------------------------------
    # # 以下两个方法是给子类重写的接口 (Abstract Methods)
    # # -------------------------------------------------

    def InputData(self, **kwargs):
        """
        [用户接口] 数据预处理
        Args:
            kwargs: 包含 'rgb', 'depth', 'pose', 'rot' (均为 numpy 数组)
        Returns:
            dict/tensor: 喂给模型的数据结构
        """
        raise NotImplementedError("Please implement InputData in your subclass")

    def Inference(self, inputs):
        """
        [用户接口] 策略逻辑
        Args:
            inputs: 来自 InputData 的返回值
        Returns:
            NavigationCommand: 要发送给仿真器的指令消息
        """
        raise NotImplementedError("Please implement Inference in your subclass")