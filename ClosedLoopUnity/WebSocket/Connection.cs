using NativeWebSocket;
using Reallusion.Import;
using System;
using System.Collections;
using System.Threading.Tasks;
using UnityEngine;

public enum ConnectionMode
{
    Localhost,
    RemoteServer
}

public class Connection : MonoBehaviour
{
    public ConnectionMode connectionMode = ConnectionMode.Localhost;
    public string protocol = "ws";        // 支持 ws 或 wss，默认 ws
    public string host = "localhost";     // 地址，默认本地
    public int port = 8766;                // 端口，默认8766

    public bool isReconnectionEnable = true;
    private bool isReconnecting = false;
    private int reconnectAttempts = 0;
    private const int maxReconnectAttempts = 10;
    private const float reconnectDelay = 1f; // 秒

    private bool isQuitting = false;


    public WebSocket websocket;
    [SerializeField] private WebSocketState state;

    private WebSocketOpenEventHandler onOpenHandler;
    private WebSocketErrorEventHandler onErrorHandler;
    private WebSocketCloseEventHandler onCloseHandler;
    private WebSocketMessageEventHandler onMessageHandler;

    private WebSocketState lastState = WebSocketState.Closed;  // 记录上一次状态，避免重复打印

    private void Update()
    {
#if !UNITY_WEBGL || UNITY_EDITOR
        if (websocket != null)
            websocket.DispatchMessageQueue();
#endif

        state = GetConnectionState();

        if (state != lastState)
        {
            Debug.Log($"WebSocket 状态变更: {lastState} -> {state}");
            lastState = state;
        }
    }

    public async Task StartConnectionAsync()
    {
        await InitWebSocket();
    }

    private async Task InitWebSocket()
    {
        string url = "";
        // Always include port when provided (>0) to avoid silently defaulting to localhost or 443
        if (connectionMode == ConnectionMode.Localhost || connectionMode == ConnectionMode.RemoteServer)
        {
            if (port > 0)
            {
                url = $"{protocol}://{host}:{port}";
            }
            else
            {
                url = $"{protocol}://{host}"; // allow host to already contain port or path
            }
        }
        websocket = new WebSocket(url);

        onOpenHandler = () =>
        {
            if (isReconnecting)
            {
                isReconnecting = false;
                reconnectAttempts = 0; // 成功后重置重连计数
                EventCenter.Instance.TriggerEvent("OnWebSocketReconnected");
                Debug.Log("✅ 重连成功！");
            }
            Debug.Log($"连接已打开: {url}");
        };

        onErrorHandler = (e) =>
        {
            _ = TryReconnectAsync();
            Debug.LogError("错误: " + e);
        };

        onCloseHandler = (e) =>
        {
            _ = TryReconnectAsync();
            Debug.LogWarning("连接关闭");
        };

        onMessageHandler = (bytes) =>
        {
            string message = System.Text.Encoding.UTF8.GetString(bytes);
            // Only log if not an ack message to reduce noise
            if (!message.Contains("\"type\": \"ack\"") && !message.Contains("\"type\":\"ack\""))
            {
                Debug.Log("收到消息: " + message);
            }

            // 调用你的反序列化方法
            object obj = JsonReceiver.DeserializeJsonFromWebSocket(bytes);
            if (obj != null)
            {
                Debug.Log($"反序列化对象类型: {obj.GetType().Name}");
            }

            Type jsonType = JsonReceiver.GetJsonType(bytes);
            if (jsonType != null)
            {
                EventCenter.Instance.TriggerEvent("OnWebSocketMessageReceived", jsonType, obj);  // 广播事件，传递收到了什么类型的数据
            }
        };

        websocket.OnOpen -= onOpenHandler;
        websocket.OnError -= onErrorHandler;
        websocket.OnClose -= onCloseHandler;
        websocket.OnMessage -= onMessageHandler;

        websocket.OnOpen += onOpenHandler;
        websocket.OnError += onErrorHandler;
        websocket.OnClose += onCloseHandler;
        websocket.OnMessage += onMessageHandler;

        try
        {
            Debug.Log($"开始尝试连接到{url} (mode={connectionMode}, protocol={protocol}, host={host}, port={port})...");
            await websocket.Connect();
        }
        catch (System.Exception ex)
        {
            Debug.LogError("连接异常: " + ex.Message);
        }
    }

    private async Task TryReconnectAsync()
    {
        if (!isReconnectionEnable || isReconnecting || isQuitting)
            return;

        isReconnecting = true;
        reconnectAttempts++;

        if (reconnectAttempts > maxReconnectAttempts)
        {
            Debug.LogError("❌ 达到最大重连次数，放弃重连。");
            isReconnecting = false;
            return;
        }

        Debug.Log($"🔄 尝试第 {reconnectAttempts} 次重连...");

        await Task.Delay((int)(reconnectDelay * 1000));

        try
        {
            await CloseWebSocket();
            await InitWebSocket();
        }
        catch (Exception ex)
        {
            Debug.LogWarning("重连失败: " + ex.Message);
            _ = TryReconnectAsync(); // 失败后递归尝试
        }
        finally
        {
            isReconnecting = false;
        }
    }



    private async Task CloseWebSocket()
    {
        if (websocket != null)
        {
            websocket.OnOpen -= onOpenHandler;
            websocket.OnError -= onErrorHandler;
            websocket.OnClose -= onCloseHandler;
            websocket.OnMessage -= onMessageHandler;

            if (websocket.State == WebSocketState.Open)
                await websocket.Close();

            websocket = null;
        }
    }
    private async void OnApplicationQuit()
    {
        isQuitting = true;
        if (websocket != null)
            await websocket.Close();
    }

    private void OnDestroy()
    {
        isQuitting = true;
        if (websocket != null)
        {
            websocket.OnOpen -= onOpenHandler;
            websocket.OnError -= onErrorHandler;
            websocket.OnClose -= onCloseHandler;
            websocket.OnMessage -= onMessageHandler;
            websocket = null;
        }
    }

    public WebSocketState GetConnectionState()
    {
        if (websocket == null)
            return WebSocketState.Closed;

        return websocket.State;
    }
}
