using Simulator.ClosedLoop;
using NativeWebSocket;
using Newtonsoft.Json;
using UnityEngine;
using UnitySensors.Sensor.Camera;
using Unity.WebRTC;

public class RGBDSender : MonoBehaviour
{
    private VideoStreamTrack _colorTrack;
    private VideoStreamTrack _depthTrack;
    private RenderTexture _colorRT;
    private RenderTexture _depthRT;
    private int _frameCount = 0;

    private void OnDestroy()
    {
        _colorTrack?.Dispose();
        _depthTrack?.Dispose();
        if (_colorRT != null) _colorRT.Release();
        if (_depthRT != null) _depthRT.Release();
    }

    public void SendRGBDImage(Texture2D colorTex, Texture2D depthTex, int resolutionx, int resolutiony)
    {
        if (WebRTCManager.Instance == null)
        {
            Debug.LogWarning("RGBDSender: WebRTCManager instance not found.");
            return;
        }

        // Only create tracks when signaling channel is open (offer/answer can be exchanged)
        var ws = WebSocketManager.Instance?.connection?.websocket;
        bool signalingReady = ws != null && ws.State == WebSocketState.Open;

        if ((_colorRT == null || _colorRT.width != resolutionx || _colorRT.height != resolutiony) && signalingReady)
        {
            Debug.Log($"RGBDSender: Creating color track {resolutionx}x{resolutiony}");
            if (_colorRT != null) _colorRT.Release();
            _colorRT = new RenderTexture(resolutionx, resolutiony, 0, RenderTextureFormat.BGRA32);
            _colorRT.Create();

            _colorTrack?.Dispose();
            _colorTrack = new VideoStreamTrack(_colorRT);
            
            // CRITICAL: Set encoder parameters for higher frame rate
            var senders = WebRTCManager.Instance.GetSenders();
            WebRTCManager.Instance.AddTrack(_colorTrack);
            
            Debug.Log("RGBDSender: Color track added to WebRTCManager");
        }

        if ((_depthRT == null || _depthRT.width != resolutionx || _depthRT.height != resolutiony) && signalingReady)
        {
            Debug.Log($"RGBDSender: Creating depth track {resolutionx}x{resolutiony}");
            if (_depthRT != null) _depthRT.Release();
            _depthRT = new RenderTexture(resolutionx, resolutiony, 0, RenderTextureFormat.BGRA32);
            _depthRT.Create();

            _depthTrack?.Dispose();
            _depthTrack = new VideoStreamTrack(_depthRT);
            WebRTCManager.Instance.AddTrack(_depthTrack);
            Debug.Log("RGBDSender: Depth track added to WebRTCManager");
        }

        // If signaling not ready yet, wait for it to create tracks
        if (_colorRT == null || _depthRT == null)
        {
            return;
        }

        Graphics.Blit(colorTex, _colorRT);
        Graphics.Blit(depthTex, _depthRT);
        
        // Force RenderTexture to be marked as updated
        _colorRT.IncrementUpdateCount();
        _depthRT.IncrementUpdateCount();

        _frameCount++;
        if (_frameCount % 60 == 0)
        {
            Debug.Log($"RGBDSender: Sent {_frameCount} RGBD frames to WebRTC tracks");
        }
    }
}
