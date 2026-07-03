import os
import cv2
import time
import json
import rclpy
import numpy as np
import math
import re
import numpy as np
from pathlib import Path
from dotenv import load_dotenv
import tempfile
import threading
from scipy.spatial.transform import Rotation as R
import shutil

# 引入 AgentFlow 依赖
from agentflow.agents.solver_embodied import construct_solver_embodied

# 引入 ROS 消息
from simulator_messages.msg import SimulatorCommand  # 自定义消息
from .vln_connector import VLNConnector
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32
from std_msgs.msg import String

from .events import event_manager
from .agent_costmap import AgentCostMapNode

class AgentBaseline(VLNConnector):
    def __init__(self):
        super().__init__()  # 初始化 ROS Node + RGBD Subscriber

        # 临时目录
        self._temp_dir = tempfile.TemporaryDirectory()
        self.get_logger().info(f"Temporary directory created: {self._temp_dir.name}")
        self._lock = threading.Lock()  # 推理锁
        self._inference_thread = None

        # --- 清理上次日志 ---
        self.log_path = Path("tmp/llm_raw_text.log")
        if self.log_path.exists():
            self.log_path.unlink()  # 删除文件
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f"LLM log file cleared: {self.log_path}")

        load_dotenv(dotenv_path="agentflow/.env")
        self.get_logger().info(
            f"OpenAI Key Loaded: {'OPENAI_API_KEY' in os.environ}"
        )

        self.llm_engine_name = "gpt-4o"
        # gpt-4o-mini
        # gpt-4o
        # gemini-2.5-flash
        # qwen2.5-vl-72b-instruct

        self.solver = construct_solver_embodied(
            llm_engine_name=self.llm_engine_name,
            enabled_tools=[
                "Base_Generator_Tool",
                "GroundedSAM2_Tool"
            ],
            tool_engine=[self.llm_engine_name],
            model_engine=[self.llm_engine_name, self.llm_engine_name, self.llm_engine_name, self.llm_engine_name],
            output_types="direct",
            max_time=300,
            max_steps=1,
            enable_multimodal=True,
            is_enable_memory=True,
            is_use_verifier=True,
            auto_write_memory=False
        )

        # prompts
        self.system_prompt = (
'''
# Task Context
## Task Skills
- Understand specific task instruction, and by that information to find the target.
- There are many same store in the scene, only choose the right store described by the instruction, if you want more information, just ask way from people.
- Navigate in the physical environment to reach the target within 3 meters then stop.
- Avoid obstacles unless you would get trapped.
- Finish your task as quick as possible.
by these information, adjust your strategy to navigate. 
## Observations You Will Receive
1. Ego-centric RGB image.
2. Ego-centric Depth image (Color-coded). 
- Warm colors (yellow/red) are far.
- Cool colors (blue/purple) are close.
3. Decision Value Map:
The Decision Value Map is a cost-based navigation field that encodes obstacles, history, and goal intention into a single spatial representation.
- Ego Icon: Circle with Arrow showing current position & heading.
- Cold Colors (Blue/Dark): Low cost path. 
- Warm Colors (Red/Yellow): High cost.
- Black Color: Obstacles. Occupied cells from the SLAM map are treated as maximum cost and are strictly forbidden.
- Dots: Small dots mark the robot’s previous positions, providing an explicit trace of where it has been.
This mainly shows history trajectory information to you, history trajectory have higher cost, but it is not assert that you can't move to history traj, it all depending on your own situation.
## Task Instruction Principle
- Give step-by-step directions using landmarks and relative movements (forward, left, right).
- Be concise, clear, and follow safe paths (prefer warm, avoid cold/black regions).
# End of Task Context
'''
        )

        self.task_prompt = None
        self.data_prompt = None

        self.temp_dir = Path("tmp/agent_baseline")
        if self.temp_dir.exists():
            shutil.rmtree(self.temp_dir)
        self.temp_dir.mkdir(parents=True, exist_ok=True)

        self.step_counter = 0
        self.last_infer_step = -1  # 防止同一帧重复推理

        self.get_logger().info("AgentBaseline Initialized")

        # map 
        self.raw_map_data = None
        self.raw_map_info = None
        self.raw_map_sub = self.create_subscription(
            OccupancyGrid,
            '/rtabmap/map',
            self.raw_map_callback,
            10
        )

        self.decision_map_sub = self.create_subscription(
            OccupancyGrid,
            "/agent/decision_costmap",  # 对应你之前写的那个节点的发布话题
            self.decision_map_callback,
            10
        )
        
        self.latest_map_img = None
        self.decision_map_data = None

        self.llm_pub = self.create_publisher(String, "/agent/llm_output", 10)
        self.goal_yaw_pub = self.create_publisher(Float32, "/agent/goal_yaw", 10)

        # Interaction
        self.is_interaction_triggered = False
        self.prev_task = None

        # Action Controller
        self.action_queue = []  # 存储多步动作 [(method, params), ...]
        self.action_interval = 0.5  # 秒
        self.action_timer = self.create_timer(self.action_interval, self.execute_action_queue)

        # events
        event_manager.register("task_received", self.handle_task_received)

    # =====================================================
    # control logic（one step）
    # =====================================================
    def control_once_async(self):
        # 如果上一轮推理还在执行，不启动新推理
        if self._inference_thread is not None and self._inference_thread.is_alive():
            return

        # 通过 get_observation 获取最新观测
        obs = self.get_observation()
        if obs is None:
            return  # 数据不完整，直接返回

        rgb_snapshot = obs["rgb"].copy() if obs["rgb"] is not None else None
        depth_snapshot = obs["depth_vis"].copy() if obs["depth_vis"] is not None else None
        latest_pose_snapshot = obs["latest_pose"]  # 如果需要，也可以 deepcopy

        # 非阻塞调用 Inference
        self._inference_thread = threading.Thread(
            target=self.Inference,
            kwargs={
                "rgb": rgb_snapshot,
                "depth_vis": depth_snapshot,
                "latest_pose": latest_pose_snapshot
            }
        )
        self._inference_thread.start()

    def InputData(self, **kwargs):
        rgb = kwargs.get("rgb")
        depth_vis = kwargs.get("depth_vis")

        img_items = []  # 改用 list of dict

        if rgb is None:
            self.get_logger().warn("No RGB input", throttle_duration_sec=2.0)
            return []

        # RGB - 当前视角
        rgb_path = self.temp_dir / "rgb_current.jpg"
        cv2.imwrite(str(rgb_path), rgb)
        img_items.append({
            "path": str(rgb_path),
            "description": "Current ego-centric RGB view of the environment"
        })

        # Depth visualization（如果有）
        if depth_vis is not None:
            depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_TURBO)
            depth_path = self.temp_dir / "depth_current.jpg"
            cv2.imwrite(str(depth_path), depth_colored)
            img_items.append({
                "path": str(depth_path),
                "description": "Current depth visualization (blue=close, red/yellow=far)"
            })
        else:
            self.get_logger().warn("No Depth visualization", throttle_duration_sec=2.0)

        # Decision / fused map（如果有）
        if self.latest_map_img is not None:
            map_path = self.temp_dir / "decision_map.jpg"
            cv2.imwrite(str(map_path), self.latest_map_img)
            img_items.append({
                "path": str(map_path),
                "description": "Top-down decision costmap: blue=low cost (good path), red=high cost, black=obstacle, gray=unknown"
            })
        else:
            self.get_logger().warn("No decision map available", throttle_duration_sec=2.0)

        return img_items   # 返回 List[Dict] 而不是 List[str]

    # =====================================================
    # Inference Adapter
    # =====================================================
    def Inference(self, **args):
        """
        通用 LLM 推理接口（可接收任意输入 via **args）
        线程安全，返回 SimulatorCommand
        """
        latest_pose = args.get("latest_pose", None)
        rgb = args.get("rgb")
        depth_vis = args.get("depth_vis")  # depth 可以留着以后用
        if rgb is None:
            self.get_logger().warn("No RGB input for inference, skipping", throttle_duration_sec=2.0)
            return None
        image_paths = self.InputData(rgb=rgb, depth_vis=depth_vis)

        if self.task_prompt is not None:
            try:
                prompt = self.system_prompt + self.task_prompt
                interaction_data = ""
                pose_data = {
                    "position": {"x_forward": latest_pose[0], "y_left": latest_pose[1]},
                    "orientation": {"yaw_left_positive": latest_pose[2]}
                }
                if self.is_interaction_triggered:
                    interaction_data = f"[Received New Interaction Data]: {self.prev_task}\n[Current Pose]: {json.dumps(pose_data)}"
                else:
                    interaction_data = f"[Current Pose]: {json.dumps(pose_data)}"
                if self.solver is not None and self.solver.is_enable_memory:
                    subgoal = self._extract_current_subgoal(self.solver.latest_verification_result)
                    costmap_summary = self._summarize_costmap()
                    retrieval_query = (
                        f"[Task]: {self.latest_task}\n"
                        f"[Current Subgoal]: {subgoal}\n"
                        f"[Planner Output]: {self.solver.planner_latest_output}\n"
                        f"[Costmap]: {costmap_summary}\n"
                        f"{interaction_data}\n"
                        f"[Last Verifier]: {self.solver.latest_verification_result}"
                    )
                    self.solver.memory.refresh_retrieval_context(retrieval_query)
                output = self.solver.solve(
                    prompt,                          
                    image_paths=image_paths
                )

                raw_text = output.get("direct_output", "")
                self._parse_llm_to_command(raw_text)
                self.get_logger().info(f"[LLM] Thinking... input={image_paths[-1]}")

                # verify the results with snapshots
                verifier_image_paths = []
                if self.latest_map_img is not None:
                    verifier_cost_path = self.temp_dir / f"verifier_beforecommand_cost_map.jpg"
                    cv2.imwrite(str(verifier_cost_path), self.latest_map_img)
                    verifier_image_paths.append({
                        "path": str(verifier_cost_path),
                        "description": f"Verifier snapshot top-down decision costmap before command"
                    })
                for i in range(1, 3):
                    time.sleep(1)  # 模拟等待过程
                    # 保存当前 RGB snapshot
                    verifier_path = self.temp_dir / f"verifier_{i}.jpg"
                    cv2.imwrite(str(verifier_path), rgb)
                    verifier_image_paths.append({
                        "path": str(verifier_path),
                        "description": f"Verifier snapshot #{i} of {i} seconds RGB view after command"
                })
                    
                if self.latest_map_img is not None:
                    verifier_cost_path = self.temp_dir / f"verifier_aftercommand_cost_map.jpg"
                    cv2.imwrite(str(verifier_cost_path), self.latest_map_img)
                    verifier_image_paths.append({
                        "path": str(verifier_cost_path),
                        "description": f"Verifier snapshot top-down decision costmap after command"
                    })
                self.solver.write_verify_data(                 
                    image_paths=verifier_image_paths,       
                    interaction_memory=interaction_data,
                    task_context=self.latest_task,
                    raw_planner_output=self.solver.planner_latest_output
                )
                return None
            
            except Exception as e:
                self.get_logger().error(f"Inference Error: {e}")
                return None
        else:
            self.get_logger().warning(f"Waiting for task", throttle_duration_sec=2.0)
        
    def destroy_node(self):
        super().destroy_node()
        self._temp_dir.cleanup()
        self.get_logger().info("Temporary directory cleaned up.")

    def handle_task_received(self, task:str):
        if task is None:
            self.get_logger().warning(f"Task is empty")
            return
        if self.prev_task != task:
            self.is_interaction_triggered = True
            if self.solver is not None and self.solver.is_enable_memory:
                self.solver.memory.reset()
        print(f"🎯 任务来了")
        self.latest_task = task
        self.task_prompt = f'''
# Specific task instruction
{task}
# End of Specific task instruction
'''
        self.prev_task = task

    def raw_map_callback(self, msg: OccupancyGrid):
        """保存原始地图数据"""
        h, w = msg.info.height, msg.info.width
        # 原始地图数据：-1(未知), 0(空闲), 100(障碍)
        self.raw_map_data = np.array(msg.data, dtype=np.int8).reshape(h, w)
        self.raw_map_info = msg.info
        
        # 尝试融合更新
        self._fuse_and_update_map_img()

    def decision_map_callback(self, msg: OccupancyGrid):
        """保存决策地图数据"""
        h, w = msg.info.height, msg.info.width
        # 决策地图数据：0-100 (Cost)
        self.decision_map_data = np.array(msg.data, dtype=np.int8).reshape(h, w)
        
        # 尝试融合更新
        self._fuse_and_update_map_img()

    def _fuse_and_update_map_img(self):
        if self.raw_map_data is None or self.decision_map_data is None:
            return
        if self.raw_map_data.shape != self.decision_map_data.shape:
            return

        # cost → heatmap (low cost = blue, high cost = red)
        cost_float = self.decision_map_data.astype(np.float32) / 100.0
        cost_float = np.clip(cost_float, 0.0, 1.0)

        heatmap_gray = (cost_float * 255).astype(np.uint8)
        fused_img = cv2.applyColorMap(heatmap_gray, cv2.COLORMAP_JET)

        # obstacles & unknowns
        obstacle_mask = (self.raw_map_data == 100)
        fused_img[obstacle_mask] = [0, 0, 0]

        unknown_mask = (self.raw_map_data == -1)
        fused_img[unknown_mask] = [128, 128, 128]

        # === 新增：垂直翻转，使地图顶部显示在图像顶部 ===
        fused_img = cv2.flip(fused_img, 0)  # 0 表示垂直翻转（上下翻转）

        self.latest_map_img = fused_img

    def _parse_llm_to_command(self, output_text: str):
        log_path = Path("tmp/llm_raw_text.log")
        with open(log_path, "a", encoding="utf-8") as f:
            f.write(output_text + "\n" + "-"*80 + "\n")
            
        msg = String()
        msg.data = output_text
        self.llm_pub.publish(msg)

        self.action_queue = None
        self.action_queue = self.solver.parse_commands(output_text)
        self.get_logger().info(f"Queued {len(self.action_queue)} actions for execution")
    
    def execute_action_queue(self):
        if not self.action_queue:
            return None

        # pop 队列第一个动作
        method, params = self.action_queue.pop(0)

        # 构建 SimulatorCommand
        cmd = SimulatorCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "agent"
        cmd.method = method
        cmd.method_params = params

        self.get_logger().info(f"Executing queued action: {method}({params})")

        # stop 特殊处理
        if method.lower() == "stop":
            self.get_logger().info("🏁 Stop received")
            self._stop_event.set()

        # 发布 ROS
        self.publish_simulator_command(cmd)
        return cmd

    def _extract_current_subgoal(self, verifier_text: str) -> str:
        if not verifier_text:
            return "Unknown"
        # Try to find the line marked as Current
        for line in verifier_text.splitlines():
            if "<Current>" in line or "Current" in line:
                cleaned = re.sub(r"\s*<.*?>\s*", " ", line).strip()
                return cleaned
        return "Unknown"

    def _summarize_costmap(self) -> str:
        if self.decision_map_data is None or self.raw_map_data is None:
            return "No costmap data"
        try:
            cost = self.decision_map_data.astype(np.float32)
            obstacle_ratio = float(np.mean(self.raw_map_data == 100))
            unknown_ratio = float(np.mean(self.raw_map_data == -1))
            summary = {
                "min": float(np.min(cost)),
                "mean": float(np.mean(cost)),
                "max": float(np.max(cost)),
                "obstacle_ratio": round(obstacle_ratio, 3),
                "unknown_ratio": round(unknown_ratio, 3)
            }
            return json.dumps(summary, ensure_ascii=False)
        except Exception as e:
            return f"Costmap summary error: {e}"

# =====================================================
# Main Loop
# =====================================================
def main(args=None):
    rclpy.init(args=args)
    node = AgentBaseline()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)  # 持续刷新订阅数据
            node.control_once_async()               # 非阻塞推理

            if node._stop_event.is_set():
                node.get_logger().info("🔁 Baseline reseting...")
                break
    except KeyboardInterrupt:
        pass
    finally:
        if node._inference_thread is not None:
            node.get_logger().info("Waiting for inference thread to finish...")
            node._inference_thread.join(timeout=2.0)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
