using UnityEngine;

public class WebSocketTest : MonoBehaviour
{
    public JsonSender jsonSender;
    public RGBDSender rgbdSender;

    public int testWidth = 640;
    public int testHeight = 480;

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.R))
        {
            // 创建随机的 RGB 和 Depth 图像
            Texture2D colorTex = GenerateRandomColorTexture(testWidth, testHeight);
            Texture2D depthTex = GenerateRandomDepthTexture(testWidth, testHeight);

            // 发送图像
            rgbdSender.SendRGBDImage(colorTex, depthTex, testWidth, testHeight);
            Debug.Log("📤 Sent RGBD image to server");
        }
        else if (Input.GetKeyDown(KeyCode.T))
        {
            jsonSender.SendJson("test", new
            {
                Position = "(0, 0, 1)",
                message = "This is a test message"
            });
            Debug.Log("📤 Sent JSON message to server");
        }
    }

    private Texture2D GenerateRandomColorTexture(int width, int height)
    {
        Texture2D tex = new Texture2D(width, height, TextureFormat.RGB24, false);
        Color[] colors = new Color[width * height];
        for (int i = 0; i < colors.Length; i++)
        {
            colors[i] = new Color(Random.value, Random.value, Random.value); // RGB 随机
        }
        tex.SetPixels(colors);
        tex.Apply();
        return tex;
    }

    private Texture2D GenerateRandomDepthTexture(int width, int height)
    {
        Texture2D tex = new Texture2D(width, height, TextureFormat.RGB24, false);
        Color[] colors = new Color[width * height];
        for (int i = 0; i < colors.Length; i++)
        {
            float depthValue = Random.Range(0f, 1f); // 模拟深度值（0~1）
            colors[i] = new Color(depthValue, depthValue, depthValue); // 单通道灰度
        }
        tex.SetPixels(colors);
        tex.Apply();
        return tex;
    }
}
