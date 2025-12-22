using UnityEngine;
using Newtonsoft.Json;
using NativeWebSocket;
using Simulator.ClosedLoop;
using System.Collections.Generic;

public class JsonSender : MonoBehaviour
{
    /// <summary>
    /// 发送一个封装过的 JSON 对象（包含 type 和 content 字段）
    /// </summary>
    /// <param name="type">数据类型标识，例如 "depth", "rgb", "command"</param>
    /// <param name="jsonObject">实际的内容对象，可以是 Dictionary、匿名对象等</param>
    public void SendJson(string json_type, object jsonObject)
    {
        if (WebSocketManager.Instance == null ||
            WebSocketManager.Instance.connection == null ||
            WebSocketManager.Instance.connection.websocket == null ||
            WebSocketManager.Instance.connection.websocket.State != WebSocketState.Open)
        {
            Debug.LogWarning("JsonSender: WebSocket 未连接，发送失败。");
            return;
        }

        var payload = new
        {
            type = "json",
            json_type = json_type,
            content = jsonObject // 允许为 null
        };

        string jsonStr = JsonConvert.SerializeObject(payload);
        WebSocketManager.Instance.connection.websocket.SendText(jsonStr);
        Debug.Log("JsonSender: 已发送 JSON：" + jsonStr);
    }
}
