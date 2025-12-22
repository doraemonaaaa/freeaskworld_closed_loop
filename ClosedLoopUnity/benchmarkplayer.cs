using Simulator.ClosedLoop;
using NativeWebSocket;
using System;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using Unity.Transforms;
using UnityEngine;
using UnityEngine.AI;

namespace Simulator.Benchmark
{
    [RequireComponent(typeof(Collider))]
    [RequireComponent(typeof(RGBDSender))]
    [RequireComponent(typeof(JsonSender))]
    public class BenchmarkPlayer : VLNPlayer
    {
        [Header("传感器设置")]
        public RGBDCamera rgbdCamera;
        public RGBDSender rgbdSender;
        public JsonSender jsonSender;
        public VideoRecorder videoRecorder;

        [Header("闭环仿真参数")]
        [SerializeField] private bool autoStep = true;
        [SerializeField] private bool isStep = false; // 是否收到服务器响应
        private int stepCount => BenchmarkManager.Instance.stepCount;
        public bool isSaveVideo = false;

        [Header("Baseline设定")]
        public BenchmarkPlayerType playerType; // 玩家类型
        private Coroutine stepCoroutine; // 用于控制步进的协程

        [Header("运动控制")]
        private Rigidbody rb;
        [SerializeField] private Vector3 _localPositionOffset;
        [SerializeField] private Quaternion _localRotationOffset;
        [SerializeField] private Vector3 targetPosition;
        [SerializeField] private Quaternion targetRotation;
        [SerializeField] private bool hasTarget = false;

        [SerializeField] private bool isStopCMD = false; // 是否停止运动



        protected override void Awake()
        {
            base.Awake();
            interactTrigger.isTrigger = true;
            isStep = false;
        }

        protected override void Start()
        {
            base.Start();

            if (playerType == BenchmarkPlayerType.VLN)
            {
                BenchmarkWebSocketEvents.OnWebSocketMessageReceived -= OnMessageReceived;
                BenchmarkWebSocketEvents.OnWebSocketMessageReceived += OnMessageReceived;
                BenchmarkWebSocketEvents.OnWebSocketReconnected -= OnReconnected;
                BenchmarkWebSocketEvents.OnWebSocketReconnected += OnReconnected;
                EventCenter.Instance.AddListener("OnWebSocketMessageReceived", BenchmarkWebSocketEvents.OnWebSocketMessageReceived);
                EventCenter.Instance.AddListener("OnWebSocketReconnected", BenchmarkWebSocketEvents.OnWebSocketReconnected);
                isStep = false;
                TriggerStep();
                if(isSaveVideo)
                    videoRecorder.StartRecording(VLNDataGenerator.Instance.videoSaveFullFolder, VLNDataGenerator.Instance.videoSaveName);
            }
        }

        protected override void Update()
        {
            base.Update();
            if(playerType == BenchmarkPlayerType.VLN)
            {
                ApplyOffsetStepToAgent(_localPositionOffset, _localRotationOffset);
                // Real-time RGBD streaming - DISABLED FOR TESTING
                // if (rgbdSender != null && rgbdCamera != null)
                // {
                //     rgbdSender.SendRGBDImage(rgbdCamera.texture1, rgbdCamera.texture0, rgbdCamera.Resolution.x, rgbdCamera.Resolution.y);
                // }
            }
            else if (playerType == BenchmarkPlayerType.Human)
            {
                BenchmarkManager.Instance.StepTime();
                ManualControlToAgent(); // 使用手动控制
                if (Input.GetKeyDown(KeyCode.T))
                {
                    TryStartConversationWithNearestHuman();
                }
                if (Input.GetKeyDown(KeyCode.P))
                {
                    isStopCMD = true;
                }

            }
            else
            {

            }

            if (isStopCMD)
            {
                BenchmarkManager.Instance.StopBenchmark();
                Debug.Log("===============收到停止命令=============");
                isStopCMD = false;
            }
        }

        private void OnDestroy()
        {
            //Time.timeScale = 1;
            if (playerType == BenchmarkPlayerType.VLN)
            {
                BenchmarkWebSocketEvents.OnWebSocketMessageReceived -= OnMessageReceived;
                BenchmarkWebSocketEvents.OnWebSocketReconnected -= OnReconnected;
                if (EventCenter.Instance != null)
                {
                    EventCenter.Instance.RemoveListener("OnWebSocketMessageReceived", BenchmarkWebSocketEvents.OnWebSocketMessageReceived);
                    EventCenter.Instance.RemoveListener("OnWebSocketReconnected", BenchmarkWebSocketEvents.OnWebSocketReconnected);
                }
            }
        }

        public override void OnStopBenchmark()
        {
            if (playerType == BenchmarkPlayerType.VLN)
            {
                if (isSaveVideo)
                    videoRecorder.StopRecording();
                //Time.timeScale = 1;
            }
        }

        public void TriggerStep()
        {
            if (stepCoroutine != null) return;

            stepCoroutine = StartCoroutine(StepRoutine());
        }

        private void OnMessageReceived(Type type, System.Object obj)
        {
            if(type == typeof(Step))
            {
                Step step = obj as Step;
                isStep = step.IsStep;
            }
            else if(type == typeof(NavigationCommand))
            {
                NavigationCommand navigation_cmd = obj as NavigationCommand;
                navigation_cmd.ToVectorAndQuaternion(out _localPositionOffset, out _localRotationOffset);
                isStopCMD = navigation_cmd.IsStop;
                hasTarget = false;
            }
            //else if (type == typeof(Skip))
            //{
            //    Skip skip = obj as Skip;
            //    if (skip.IsSkip)
            //    {
            //        _localPositionOffset = Vector3.zero;
            //        _localRotationOffset = Quaternion.identity;
            //        hasTarget = false;
            //        isStep = true;
            //    }
            //}
        }

        private void OnReconnected()
        {
            StopCoroutine(stepCoroutine);
            stepCoroutine = null;
            TriggerStep();
        }

        IEnumerator StepRoutine()
        {
            // 防止提前触发
            isStep = false;
            //Time.timeScale = 0;

            Debug.Log("VLN Model Step Count: " + stepCount);

            yield return new WaitUntil(() =>
                WebSocketManager.Instance != null &&
                WebSocketManager.Instance.connection != null &&
                WebSocketManager.Instance.connection.websocket != null &&
                WebSocketManager.Instance.connection.websocket.State == WebSocketState.Open
            );

            SendInputData();
            yield return null;

            // 等待服务器响应动作数据
            yield return StartCoroutine(WaitForStepSignal());

            yield return new WaitForSeconds(BenchmarkManager.Instance.stepSimTime);

            BenchmarkManager.Instance.Step();

            // 标记协程执行完成
            stepCoroutine = null;

            if (autoStep)
            {
                TriggerStep();
            }
        }

        private void SendInputData()
        {
            // 第一步需要初始化etp nav
            if(stepCount == 0)
            {
                string init_json_type = "Init";
                jsonSender.SendJson(init_json_type, null);
            }
            TransformData transform_data = new TransformData(transform.position, transform.rotation);
            string human_instruction;
            float simTime =
                            BenchmarkManager.Instance.stepCount *
                            BenchmarkManager.Instance.stepSimTime;
            BenchmarkManager.Instance.GetBenchmarkHumanInstruction(out human_instruction);
            rgbdSender.SendRGBDImage(rgbdCamera.texture1, rgbdCamera.texture0, rgbdCamera.Resolution.x, rgbdCamera.Resolution.y);  // 发送图像数据
            jsonSender.SendJson("TransformData", transform_data);  // 发送位置数据
            jsonSender.SendJson("Instruction", human_instruction);  // 发送指令信息
            jsonSender.SendJson(
                "SimulationTime",
                new { time = simTime }
            );
            jsonSender.SendJson("Step", new Step { IsStep = true });
        }

        private IEnumerator WaitForStepSignal()
        {
            yield return new WaitUntil(() => isStep);
            isStep = false; // 自动重置
        }

        #region 运动控制
        private void ApplyOffsetStepPhysically(Vector3 positionOffset, Quaternion rotationOffset, float t)
        {
            if (rb == null)
            {
                Debug.LogWarning("Rigidbody not found! Movement won't handle collisions.");
                return;
            }

            Quaternion currentRot = rb.rotation;
            Vector3 currentPos = rb.position;

            Quaternion targetRot = currentRot * rotationOffset;
            Vector3 rotatedOffset = targetRot * positionOffset;
            Vector3 targetPos = currentPos + rotatedOffset;

            Quaternion newRot = Quaternion.Slerp(currentRot, targetRot, t);
            newRot = Quaternion.Normalize(newRot); // ✅ 保证单位四元数

            Vector3 newPos = Vector3.Lerp(currentPos, targetPos, t);

            rb.MoveRotation(newRot);
            rb.MovePosition(newPos);
        }

        private void ApplyOffsetStepToAgent(Vector3 positionOffset, Quaternion rotationOffset)
        {
            if (Time.timeScale == 0 || agent == null)
            {
                agent?.ResetPath();
                return;
            }

            // 设置目标
            if (!hasTarget)
            {
                targetPosition = transform.position + transform.rotation * positionOffset;
                targetRotation = Quaternion.Normalize(transform.rotation * rotationOffset);
                hasTarget = true;
            }

            // 禁用自动旋转
            agent.updateRotation = false;

            // 应用旋转（可以也只在第一次应用）
            transform.rotation = targetRotation;

            // 计算向量方向
            Vector3 direction = targetPosition - transform.position;
            float distance = direction.magnitude;

            if (distance <= 0.01f)
            {
                // 到达目标位置
                hasTarget = false;
                return;
            }

            // 根据 agent.speed 推进
            float step = agent.speed * Time.deltaTime;
            Vector3 move = direction.normalized * Mathf.Min(step, distance);
            agent.Move(move);
        }

        #endregion

    }
}
