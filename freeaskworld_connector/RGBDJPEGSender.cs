using Enviro;
using Simulator;
using System;
using System.Collections;
using UnityEngine;
using static Unity.Entities.Build.DotsGlobalSettings;

/// <summary>
/// �ɹ��ص�С��������ڽ� RGBD �������ͨ�� WebRTCManager ���͡�
/// </summary>
public class RGBDJPEGSender : MonoBehaviour
{
    [SerializeField] private bool isEnvironEnabled = true;
    [SerializeField] private RGBDCamera rgbdCamera;
    [SerializeField] private bool autoSend = true;
    [SerializeField] private float sendInterval = 0.1f; // 10 FPS
    [SerializeField] private bool logSend;
    [SerializeField] private bool limitResolution = false;
    [SerializeField] private int maxWidth = 640;
    [SerializeField] private int maxHeight = 480;
    [SerializeField, Range(1, 100)] private int colorJpegQuality = 70;

    private Camera _colorCamera;
    private Camera _depthCamera;

    private Coroutine sendRoutine;

    private void Awake()
    {
        if (rgbdCamera == null)
        {
            rgbdCamera = GetComponent<RGBDCamera>();
        }

        //if (rgbdCamera != null)
        //{
        //    rgbdCamera.onSensorUpdated += OnRgbdSensorUpdated;
        //}
    }

    private void Start()
    {
        ResolveCameras();
    }

    private void OnEnable()
    {
        if (WebRTCManager.Instance != null)
        {
            WebRTCManager.Instance.OnChannelOpen += StartStream;
            WebRTCManager.Instance.OnChannelClose += StopStream;

            if (autoSend && WebRTCManager.Instance.IsChannelOpen)
            {
                StartStream();
            }
        }
    }

    private void OnDestroy()
    {
        //if (rgbdCamera != null)
        //{
        //    rgbdCamera.onSensorUpdated -= OnRgbdSensorUpdated;
        //}
    }

    private void OnDisable()
    {
        StopStream();
        if (WebRTCManager.Instance != null)
        {
            WebRTCManager.Instance.OnChannelOpen -= StartStream;
            WebRTCManager.Instance.OnChannelClose -= StopStream;
        }
    }

    public void StartStream()
    {
        if (
            sendRoutine != null ||
            WebRTCManager.Instance == null ||
            rgbdCamera == null ||
            !WebRTCManager.Instance.IsChannelOpen
        )
        {
            return;
        }

        sendRoutine = StartCoroutine(SendLoop());
    }

    public void StopStream()
    {
        if (sendRoutine != null)
        {
            StopCoroutine(sendRoutine);
            sendRoutine = null;
        }
    }

    public void SendOnce()
    {
        if (
            WebRTCManager.Instance == null ||
            rgbdCamera == null ||
            !WebRTCManager.Instance.IsChannelOpen
        )
        {
            return;
        }

        int budgetBytes = WebRTCManager.Instance != null ? WebRTCManager.Instance.MaxMessageSize : 2 * 1024 * 1024;
        bool clampedBudget = false;
        if (budgetBytes <= 0)
        {
            budgetBytes = 8 * 1024 * 1024; // fallback if Inspector had 0
            clampedBudget = true;
        }
        var stream = BuildRGBDStreamWithinBudget(
            rgbdCamera.texture1,
            rgbdCamera.texture0,
            budgetBytes,
            out var stats
        );
        if (stream != null)
        {
            var envelope = new WebRTCManager.DataEnvelope
            {
                type = "rgbd_stream",
                payload = stream,
            };
            WebRTCManager.Instance.SendJson(envelope);
            if (logSend)
            {
                Debug.Log(
                    $"[RGBDStreamSender] srcColor={stats.srcColorW}x{stats.srcColorH} srcDepth={stats.srcDepthW}x{stats.srcDepthH} -> sent={stream.width}x{stream.height} scale={stats.scale:F3} size={stats.jsonBytes}B cap={stats.hardCap} dropped={stats.dropped} clampedBudget={clampedBudget}"
                );
            }
        }
    }

    //private void OnRgbdSensorUpdated()
    //{
    //    if (WebRTCManager.Instance != null && rgbdCamera != null)
    //    {
    //        SendOnce();
    //    }
    //}


    private IEnumerator SendLoop()
    {
        var wait = new WaitForSeconds(sendInterval);
        while (true)
        {
            SendOnce();
            yield return wait;
        }
    }

    private RGBDStream BuildRGBDStreamWithinBudget(Texture2D colorTex, Texture2D depthTex, int maxBytes, out SenderStats stats)
    {
        stats = new SenderStats();
        if (colorTex == null || depthTex == null)
        {
            return null;
        }

        // Keep a bit of headroom for envelope/headers
        int hardCap = Mathf.Max(1024, maxBytes - 4096);

        float scale = 1f;
        // Respect inspector cap first
        var targetMaxW = Mathf.Max(1, maxWidth);
        var targetMaxH = Mathf.Max(1, maxHeight);
        if (limitResolution && (colorTex.width > targetMaxW || colorTex.height > targetMaxH))
        {
            scale = Mathf.Min((float)targetMaxW / colorTex.width, (float)targetMaxH / colorTex.height);
        }

        // Iteratively downscale if payload would exceed channel budget
        for (int i = 0; i < 5; i++)
        {
            var stream = BuildRGBDStream(colorTex, depthTex, scale, out int jsonBytes, out int outW, out int outH);
            if (stream == null)
            {
                return null;
            }

            // Rough size check using serialized JSON length
            if (jsonBytes <= hardCap)
            {
                stats.srcColorW = colorTex.width;
                stats.srcColorH = colorTex.height;
                stats.srcDepthW = depthTex.width;
                stats.srcDepthH = depthTex.height;
                stats.outW = outW;
                stats.outH = outH;
                stats.scale = scale;
                stats.jsonBytes = jsonBytes;
                stats.hardCap = hardCap;
                stats.dropped = false;
                return stream;
            }

            scale *= 0.75f; // reduce resolution and retry
        }

        stats.srcColorW = colorTex.width;
        stats.srcColorH = colorTex.height;
        stats.srcDepthW = depthTex.width;
        stats.srcDepthH = depthTex.height;
        stats.outW = 0;
        stats.outH = 0;
        stats.scale = scale;
        stats.jsonBytes = -1;
        stats.hardCap = hardCap;
        stats.dropped = true;
        Debug.LogWarning($"[RGBDStreamSender] Dropping frame: payload would exceed {hardCap} bytes");
        return null;
    }

    private RGBDStream BuildRGBDStream(Texture2D colorTex, Texture2D depthTex, float scale, out int jsonBytes, out int outW, out int outH)
    {
        jsonBytes = 0;
        outW = 0;
        outH = 0;
        if (colorTex == null || depthTex == null)
        {
            return null;
        }

        var targetMaxW = Mathf.Max(1, maxWidth);
        var targetMaxH = Mathf.Max(1, maxHeight);
        var colorQuality = Mathf.Clamp(colorJpegQuality, 1, 100);

        outW = Mathf.Max(1, Mathf.RoundToInt(colorTex.width * scale));
        outH = Mathf.Max(1, Mathf.RoundToInt(colorTex.height * scale));

        // Downscale color if requested
        var scaledColor = scale < 0.999f ? DownscaleTexture(colorTex, outW, outH) : colorTex;

        // JPEG encode color (PNG is too large for DataChannel)
        var colorJpg = scaledColor.EncodeToJPG(colorQuality);

        // Encode depth as raw float32 meters (resampled to match color resolution)
        var depthBytes = EncodeDepthToFloatRaw(depthTex, outW, outH);

        var stream = new RGBDStream
        {
            width = outW,
            height = outH,
            color = Convert.ToBase64String(colorJpg),
            depth = Convert.ToBase64String(depthBytes),
            depth_format = "float32_m",
            depth_scale = 1.0f,
            timestamp = Time.timeAsDouble
        };

        // Precompute JSON size for budgeting
        string json = JsonUtility.ToJson(stream);
        jsonBytes = System.Text.Encoding.UTF8.GetByteCount(json);

        return stream;
    }

    private Texture2D DownscaleIfNeeded(Texture2D src, int maxW, int maxH)
    {
        if (!limitResolution || (src.width <= maxW && src.height <= maxH))
        {
            return src;
        }

        float scale = Mathf.Min((float)maxW / src.width, (float)maxH / src.height);
        int newW = Mathf.Max(1, Mathf.RoundToInt(src.width * scale));
        int newH = Mathf.Max(1, Mathf.RoundToInt(src.height * scale));

        var rt = RenderTexture.GetTemporary(newW, newH, 0, RenderTextureFormat.Default, RenderTextureReadWrite.Linear);
        var prev = RenderTexture.active;
        Graphics.Blit(src, rt);

        var dst = new Texture2D(newW, newH, src.format, false);
        RenderTexture.active = rt;
        dst.ReadPixels(new Rect(0, 0, newW, newH), 0, 0);
        dst.Apply();

        RenderTexture.active = prev;
        RenderTexture.ReleaseTemporary(rt);
        return dst;
    }

    private Texture2D DownscaleTexture(Texture2D src, int newW, int newH)
    {
        var rt = RenderTexture.GetTemporary(newW, newH, 0, RenderTextureFormat.Default, RenderTextureReadWrite.Linear);
        var prev = RenderTexture.active;
        Graphics.Blit(src, rt);

        var dst = new Texture2D(newW, newH, src.format, false);
        RenderTexture.active = rt;
        dst.ReadPixels(new Rect(0, 0, newW, newH), 0, 0);
        dst.Apply();

        RenderTexture.active = prev;
        RenderTexture.ReleaseTemporary(rt);
        return dst;
    }

    private bool ResolveCameras()
    {
        if (rgbdCamera == null)
        {
            Debug.LogError("[RGBDJPEGSender] No RGBD camera assigned.");
            return false;
        }

        // Depth camera is the main camera on RGBDCameraSensor
        _depthCamera = rgbdCamera.GetComponent<Camera>();

        // Color camera is a child camera created dynamically by RGBDCameraSensor.Init()
        var cameras = rgbdCamera.GetComponentsInChildren<Camera>(true);
        _colorCamera = null;

        foreach (var cam in cameras)
        {
            if (cam != null && cam != _depthCamera)
            {
                _colorCamera = cam;
                Debug.Log($"[RGBDJPEGSender] Found color camera: {cam.name}");
                break;
            }
        }

        if (_colorCamera == null)
        {
            Debug.LogWarning("[RGBDJPEGSender] Color camera not found in children. RGBDCameraSensor may not be initialized yet.");
            return false;
        }

        if (_depthCamera == null)
        {
            Debug.LogWarning("[RGBDJPEGSender] Depth camera not found.");
            return false;
        }

        Debug.Log($"[RGBDJPEGSender] Cameras resolved - Color: {_colorCamera.name}, Depth: {_depthCamera.name}");
        if (isEnvironEnabled)
        {
            EnviroManager.instance.AddAdditionalCamera(_colorCamera);
            _colorCamera.clearFlags = CameraClearFlags.Skybox;
        }
        return true;
    }

    /// <summary>
    /// �������ͼ�ĺ�ɫͨ����Ϊ������ȣ���λ���ף�������Ϊ float32 ԭʼ�ֽ����С�
    /// ���ն˰� float32 ��ԭ��ֱ�ӵõ��׵�λ��ȡ�
    /// </summary>
    private byte[] EncodeDepthToFloatRaw(Texture2D depthTex, int targetW, int targetH)
    {
        var depthFloats = ReadDepthFloats(depthTex);

        if (depthTex.width != targetW || depthTex.height != targetH)
        {
            depthFloats = ResampleDepthNearest(depthFloats, depthTex.width, depthTex.height, targetW, targetH);
        }

        return FloatArrayToBytes(depthFloats);
    }

    private float[] ReadDepthFloats(Texture2D depthTex)
    {
        var pixels = depthTex.GetPixels();
        int count = pixels.Length;
        var depthFloats = new float[count];

        for (int i = 0; i < count; i++)
        {
            float d = pixels[i].r;
            if (d <= 0f || float.IsNaN(d) || float.IsInfinity(d))
            {
                depthFloats[i] = 0f;
            }
            else
            {
                depthFloats[i] = d;
            }
        }

        return depthFloats;
    }

    private float[] ResampleDepthNearest(float[] src, int srcW, int srcH, int dstW, int dstH)
    {
        var dst = new float[dstW * dstH];
        float xRatio = srcW / (float)dstW;
        float yRatio = srcH / (float)dstH;

        for (int y = 0; y < dstH; y++)
        {
            int srcY = Mathf.Clamp(Mathf.FloorToInt((y + 0.5f) * yRatio), 0, srcH - 1);
            int srcRow = srcY * srcW;
            int dstRow = y * dstW;
            for (int x = 0; x < dstW; x++)
            {
                int srcX = Mathf.Clamp(Mathf.FloorToInt((x + 0.5f) * xRatio), 0, srcW - 1);
                dst[dstRow + x] = src[srcRow + srcX];
            }
        }

        return dst;
    }

    private byte[] FloatArrayToBytes(float[] depthFloats)
    {
        var bytes = new byte[depthFloats.Length * sizeof(float)];
        Buffer.BlockCopy(depthFloats, 0, bytes, 0, bytes.Length);
        return bytes;
    }

    private class SenderStats
    {
        public int srcColorW;
        public int srcColorH;
        public int srcDepthW;
        public int srcDepthH;
        public int outW;
        public int outH;
        public float scale;
        public int jsonBytes;
        public int hardCap;
        public bool dropped;
    }

    [Serializable]
    private class RGBDStream
    {
        public int width;
        public int height;
        public string color;
        public string depth;
        public string depth_format; // ���� "float32_m"
        public float depth_scale;   // ���� 1.0�����ն˰���ʹ��
        public double timestamp;
    }
}
