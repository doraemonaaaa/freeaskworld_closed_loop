using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using System;
using System.Text;
using UnityEngine;

public static class JsonReceiver
{
    // 先定义接收最外层消息结构
    public class ReceivedMessage
    {
        public string type;
        public string json_type;

        [JsonProperty("content")]
        public JToken contentRaw;
    }

    public static object DeserializeJsonFromWebSocket(byte[] data)
    {
        string msg = Encoding.UTF8.GetString(data);

        if (!msg.TrimStart().StartsWith("{"))
        {
            Debug.Log($"收到非 JSON 格式消息: {msg}");
            return null;
        }

        try
        {
            // ?л??
            var baseMsg = JsonConvert.DeserializeObject<ReceivedMessage>(msg);

            if (baseMsg.type == "json")
            {
                // ??json_type?
                Type targetType = Type.GetType(baseMsg.json_type);
                if (targetType != null)
                {
                    // 先把 JToken 转成字符串
                    string contentJson = baseMsg.contentRaw.ToString();
                    // 再反序列化content字符串为对应类型对象
                    var contentObj = JsonConvert.DeserializeObject(contentJson, targetType);
                    //Debug.Log($"成功解析为类型 {targetType.Name}: {contentObj}");

                    // 返回反序列化对象，方便调用者使用
                    return contentObj;
                }
                else
                {
                    Debug.LogWarning($"δ???: {baseMsg.json_type}");
                }
            }
            else if (baseMsg.type == "ack")
            {
                // Ignore ack messages to reduce log noise
                return null;
            }
            else
            {
                Debug.Log($"? json ?: {baseMsg.type}");
            }
        }
        catch (Exception ex)
        {
            Debug.LogError("消息解析失败: " + ex.Message);
        }

        return null;
    }

    public static Type GetJsonType(byte[] data)
    {
        string msg = Encoding.UTF8.GetString(data);

        if (!msg.TrimStart().StartsWith("{"))
        {
            Debug.Log($"GetJsonType: 非 JSON 格式消息: {msg}");
            return null;
        }

        try
        {
            var baseMsg = JsonConvert.DeserializeObject<ReceivedMessage>(msg);
            if (baseMsg.type == "json")
            {
                Type targetType = Type.GetType(baseMsg.json_type);
                if (targetType != null)
                {
                    //Debug.Log($"成功解析为类型 {targetType.Name}");
                    return targetType;
                }
            }
        }
        catch (Exception ex)
        {
            Debug.LogError("类型解析失败: " + ex.Message);
        }
        return null;
    }

}
