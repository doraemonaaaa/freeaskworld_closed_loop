using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using Newtonsoft.Json;
using Unity.WebRTC;
using UnityEngine;
using UnityEngine.Networking;

[DisallowMultipleComponent]
public class WebRTCManager : SingletonMono<WebRTCManager>
{
    protected override bool isDontDestroyOnLoad => false;
    [Header("Signaling Server")]
    [SerializeField] private string _signalingServerUrl = "http://localhost:8766";
    [SerializeField] private bool _autoConnect = true;
    [SerializeField] private float _reconnectDelay = 5f;
    [SerializeField] private int _maxReconnectAttempts = 3;

    [Header("WebRTC Configuration")]
    [SerializeField] private string[] _iceServers = { "stun:stun.l.google.com:19302" };

    [Header("Data Channel")]
    [SerializeField] private string _dataChannelLabel = "control";
    [SerializeField] private bool _logDebug = true;
    [SerializeField] private int _maxMessageSize = 10485760;  // 10 MB, must match server config

    public event Action OnConnected;
    public event Action OnDisconnected;
    public event Action<string> OnConnectionError;
    public event Action<string> OnTextMessage;
    public event Action OnChannelOpen;
    public event Action OnChannelClose;

    private RTCPeerConnection _peerConnection;
    private RTCDataChannel _dataChannel;
    private Coroutine _updateCoroutine;
    private string _sessionId;
    private int _reconnectAttempts;
    private bool _isConnecting;
    private List<IceCandidateMessage> _pendingCandidates = new List<IceCandidateMessage>();
    private readonly List<VideoStreamTrack> _videoTracks = new List<VideoStreamTrack>();
    private readonly Dictionary<VideoStreamTrack, RTCRtpSender> _videoTrackSenders = new Dictionary<VideoStreamTrack, RTCRtpSender>();

    public bool IsChannelOpen => IsChannelReady();
    public bool IsConnected => !string.IsNullOrEmpty(_sessionId) && IsChannelReady();
    public string SessionId => _sessionId;
    public string ServerUrl => _signalingServerUrl;
    public int MaxMessageSize => _maxMessageSize;

    private void OnValidate()
    {
        // Prevent an accidentally zero/negative value from the Inspector from shrinking payload budget to 1 KB
        _maxMessageSize = Mathf.Max(_maxMessageSize, 1024 * 1024); // at least 1 MB
    }

    private void Start()
    {
        _updateCoroutine = StartCoroutine(WebRTC.Update());

        if (_autoConnect)
        {
            Connect();
        }
    }

    protected override void OnDestroy()
    {
        base.OnDestroy();
        Disconnect(isActiveAndEnabled);
        if (_updateCoroutine != null)
        {
            StopCoroutine(_updateCoroutine);
            _updateCoroutine = null;
        }
    }

    /// <summary>
    /// Set the signaling server URL at runtime.
    /// </summary>
    public void SetServerUrl(string url)
    {
        _signalingServerUrl = url.TrimEnd('/');
    }

    /// <summary>
    /// Connect to the signaling server and establish WebRTC connection.
    /// </summary>
    public void Connect()
    {
        if (_isConnecting || IsConnected)
        {
            Log("Already connecting or connected");
            return;
        }

        _isConnecting = true;
        _pendingCandidates.Clear();

        SetupPeerConnection();
        StartCoroutine(ConnectCoroutine());
    }

    /// <summary>
    /// Disconnect and cleanup.
    /// </summary>
    public void Disconnect(bool allowCoroutine = true)
    {
        StopAllCoroutines();

        if (!string.IsNullOrEmpty(_sessionId))
        {
            if (allowCoroutine && isActiveAndEnabled)
            {
                StartCoroutine(CloseSessionCoroutine());
            }
        }

        ClosePeerConnection();
        _sessionId = null;
        _isConnecting = false;
        OnDisconnected?.Invoke();
    }

    private IEnumerator ConnectCoroutine()
    {
        Log($"Connecting to {_signalingServerUrl}...");

        // Create offer
        var offerOp = _peerConnection.CreateOffer();
        yield return offerOp;

        if (offerOp.IsError)
        {
            LogError($"CreateOffer error: {offerOp.Error.message}");
            _isConnecting = false;
            OnConnectionError?.Invoke(offerOp.Error.message);
            yield break;
        }

        var desc = offerOp.Desc;
        desc.sdp = OverrideMaxMessageSize(desc.sdp, _maxMessageSize);
        var localOp = _peerConnection.SetLocalDescription(ref desc);
        yield return localOp;

        if (localOp.IsError)
        {
            LogError($"SetLocalDescription error: {localOp.Error.message}");
            _isConnecting = false;
            OnConnectionError?.Invoke(localOp.Error.message);
            yield break;
        }

        // Send offer to signaling server
        yield return SendOfferCoroutine(desc.sdp);
    }

    private IEnumerator SendOfferCoroutine(string sdp)
    {
        string url = $"{_signalingServerUrl}/offer";
        var requestData = new { sdp = sdp };
        string json = JsonConvert.SerializeObject(requestData);

        Log($"Sending offer to {url}");

        using (var request = new UnityWebRequest(url, "POST"))
        {
            byte[] bodyRaw = Encoding.UTF8.GetBytes(json);
            request.uploadHandler = new UploadHandlerRaw(bodyRaw);
            request.downloadHandler = new DownloadHandlerBuffer();
            request.SetRequestHeader("Content-Type", "application/json");
            request.timeout = 30;

            yield return request.SendWebRequest();

            if (request.result != UnityWebRequest.Result.Success)
            {
                LogError($"Offer request failed: {request.error}");
                _isConnecting = false;
                OnConnectionError?.Invoke(request.error);
                HandleReconnect();
                yield break;
            }

            string responseText = request.downloadHandler.text;
            Log($"Received answer");

            AnswerResponse response = null;
            string parseError = null;
            try
            {
                response = JsonConvert.DeserializeObject<AnswerResponse>(responseText);
            }
            catch (Exception ex)
            {
                parseError = ex.Message;
            }

            if (response == null)
            {
                LogError($"Failed to parse answer: {parseError ?? "null response"}");
                _isConnecting = false;
                OnConnectionError?.Invoke(parseError ?? "Invalid answer response");
                yield break;
            }

            _sessionId = response.session_id;

            // Apply remote answer (also override max-message-size to match our offer)
            var remoteDesc = new RTCSessionDescription
            {
                type = RTCSdpType.Answer,
                sdp = OverrideMaxMessageSize(response.sdp, _maxMessageSize)
            };

            var remoteOp = _peerConnection.SetRemoteDescription(ref remoteDesc);
            yield return remoteOp;

            if (remoteOp.IsError)
            {
                LogError($"SetRemoteDescription error: {remoteOp.Error.message}");
                _isConnecting = false;
                OnConnectionError?.Invoke(remoteOp.Error.message);
                yield break;
            }

            _isConnecting = false;
            _reconnectAttempts = 0;
            Log($"Session established: {_sessionId}");

            // Send any pending ICE candidates
            foreach (var candidate in _pendingCandidates)
            {
                StartCoroutine(SendIceCandidateCoroutine(candidate));
            }
            _pendingCandidates.Clear();

            OnConnected?.Invoke();
        }
    }

    private IEnumerator SendIceCandidateCoroutine(IceCandidateMessage candidate)
    {
        if (string.IsNullOrEmpty(_sessionId))
        {
            yield break;
        }

        string url = $"{_signalingServerUrl}/ice/{_sessionId}";
        string json = JsonConvert.SerializeObject(candidate);

        using (var request = new UnityWebRequest(url, "POST"))
        {
            byte[] bodyRaw = Encoding.UTF8.GetBytes(json);
            request.uploadHandler = new UploadHandlerRaw(bodyRaw);
            request.downloadHandler = new DownloadHandlerBuffer();
            request.SetRequestHeader("Content-Type", "application/json");
            request.timeout = 10;

            yield return request.SendWebRequest();

            if (request.result != UnityWebRequest.Result.Success)
            {
                LogWarning($"ICE candidate send failed: {request.error}");
            }
        }
    }

    private IEnumerator CloseSessionCoroutine()
    {
        if (string.IsNullOrEmpty(_sessionId))
        {
            yield break;
        }

        string url = $"{_signalingServerUrl}/session/{_sessionId}";
        using (var request = UnityWebRequest.Delete(url))
        {
            request.timeout = 5;
            yield return request.SendWebRequest();
        }
    }

    private void HandleReconnect()
    {
        if (_reconnectAttempts < _maxReconnectAttempts)
        {
            _reconnectAttempts++;
            Log($"Reconnecting (attempt {_reconnectAttempts}/{_maxReconnectAttempts})...");
            Invoke(nameof(Connect), _reconnectDelay);
        }
    }

    private void SetupPeerConnection()
    {
        ClosePeerConnection();

        var iceList = new List<RTCIceServer>();
        if (_iceServers != null && _iceServers.Length > 0)
        {
            iceList.Add(new RTCIceServer { urls = _iceServers });
        }

        var configuration = new RTCConfiguration
        {
            iceServers = iceList.ToArray()
        };

        _peerConnection = new RTCPeerConnection(ref configuration);
        _peerConnection.OnIceCandidate = HandleIceCandidate;
        _peerConnection.OnDataChannel = channel =>
        {
            _dataChannel = channel;
            RegisterDataChannelCallbacks();
        };
        _peerConnection.OnConnectionStateChange = state =>
        {
            Log($"Connection state: {state}");

            if (state == RTCPeerConnectionState.Connected)
            {
                Log("WebRTC connected!");
            }
            else if (state == RTCPeerConnectionState.Disconnected ||
                     state == RTCPeerConnectionState.Failed ||
                     state == RTCPeerConnectionState.Closed)
            {
                if (state == RTCPeerConnectionState.Failed)
                {
                    OnDisconnected?.Invoke();
                    HandleReconnect();
                }
            }
        };

        // Create DataChannel with explicit options for reliable, ordered delivery
        var dataChannelInit = new RTCDataChannelInit
        {
            ordered = true  // Ensure ordered delivery for large messages
        };
        _dataChannel = _peerConnection.CreateDataChannel(_dataChannelLabel, dataChannelInit);
        RegisterDataChannelCallbacks();
        AttachRegisteredVideoTracks();
    }

    private void HandleIceCandidate(RTCIceCandidate candidate)
    {
        if (candidate == null)
        {
            return;
        }

        var message = new IceCandidateMessage
        {
            candidate = candidate.Candidate,
            sdpMid = candidate.SdpMid,
            sdpMLineIndex = candidate.SdpMLineIndex ?? -1
        };

        if (string.IsNullOrEmpty(_sessionId))
        {
            // Buffer until session is established
            _pendingCandidates.Add(message);
        }
        else
        {
            StartCoroutine(SendIceCandidateCoroutine(message));
        }
    }

    private void ClosePeerConnection()
    {
        foreach (var sender in _videoTrackSenders.Values)
        {
            sender?.Dispose();
        }
        _videoTrackSenders.Clear();

        if (_dataChannel != null)
        {
            _dataChannel.Close();
            _dataChannel.Dispose();
            _dataChannel = null;
        }

        if (_peerConnection != null)
        {
            _peerConnection.Close();
            _peerConnection.Dispose();
            _peerConnection = null;
        }
    }

    private void RegisterDataChannelCallbacks()
    {
        if (_dataChannel == null)
        {
            return;
        }

        _dataChannel.OnOpen = () =>
        {
            if (_logDebug)
            {
                Debug.Log("[WebRTC] Data channel opened");
            }
            OnChannelOpen?.Invoke();
        };

        _dataChannel.OnClose = () =>
        {
            if (_logDebug)
            {
                Debug.Log("[WebRTC] Data channel closed");
            }
            OnChannelClose?.Invoke();
        };

        _dataChannel.OnMessage = bytes =>
        {
            string text = Encoding.UTF8.GetString(bytes);
            Log($"Received: {text}");
            OnTextMessage?.Invoke(text);
        };
    }

    public void RegisterVideoTrack(VideoStreamTrack track)
    {
        if (track == null || _videoTracks.Contains(track))
        {
            return;
        }

        _videoTracks.Add(track);

        if (_peerConnection != null)
        {
            AttachVideoTrack(track);
        }
    }

    public void UnregisterVideoTrack(VideoStreamTrack track)
    {
        if (track == null)
        {
            return;
        }

        if (_videoTrackSenders.TryGetValue(track, out var sender))
        {
            if (_peerConnection != null && sender != null)
            {
                _peerConnection.RemoveTrack(sender);
            }
            _videoTrackSenders.Remove(track);
        }

        _videoTracks.Remove(track);
    }

    private void AttachVideoTrack(VideoStreamTrack track)
    {
        if (track == null || _peerConnection == null)
        {
            return;
        }

        try
        {
            var sender = _peerConnection.AddTrack(track);
            _videoTrackSenders[track] = sender;
        }
        catch (Exception ex)
        {
            LogWarning($"Failed to attach video track: {ex.Message}");
        }
    }

    private void AttachRegisteredVideoTracks()
    {
        foreach (var track in _videoTracks)
        {
            AttachVideoTrack(track);
        }
    }

    public void SendControlMessage(string messageType, object payload)
    {
        // Keep messageType as-is (TitleCase expected by Python backend)
        var envelope = new DataEnvelope
        {
            type = "json",
            json_type = messageType,
            content = payload,
            payload = null
        };

        SendJson(envelope);
    }

    public void SendMetaData(object meta)
    {
        SendControlMessage("metadata", meta);
    }

    public void SendJson(object message)
    {
        if (!IsChannelReady())
        {
            return;
        }

        string json = JsonConvert.SerializeObject(message);
        _dataChannel.Send(Encoding.UTF8.GetBytes(json));
    }

    private bool IsChannelReady()
    {
        return _dataChannel != null && _dataChannel.ReadyState == RTCDataChannelState.Open;
    }

    private void Log(string message)
    {
        if (_logDebug)
        {
            Debug.Log($"[WebRTC] {message}");
        }
    }

    private void LogWarning(string message)
    {
        Debug.LogWarning($"[WebRTC] {message}");
    }

    private void LogError(string message)
    {
        Debug.LogError($"[WebRTC] {message}");
    }

    private string OverrideMaxMessageSize(string sdp, int maxBytes = 2097152)
    {
        var lines = sdp.Split(new[] { "\r\n", "\n" }, StringSplitOptions.None);
        bool replaced = false;
        for (int i = 0; i < lines.Length; i++)
        {
            if (lines[i].StartsWith("a=max-message-size:"))
            {
                lines[i] = $"a=max-message-size:{maxBytes}";
                replaced = true;
            }
        }
        if (!replaced)
        {
            var list = new List<string>(lines);
            for (int i = 0; i < list.Count; i++)
            {
                if (list[i].StartsWith("m=application"))
                {
                    list.Insert(i + 1, $"a=max-message-size:{maxBytes}");
                    replaced = true;
                    break;
                }
            }
            if (!replaced)
            {
                list.Add($"a=max-message-size:{maxBytes}");
            }
            lines = list.ToArray();
        }
        return string.Join("\r\n", lines);
    }

    [Serializable]
    public class IceCandidateMessage
    {
        public string candidate;
        public string sdpMid;
        public int sdpMLineIndex;
    }

    [Serializable]
    private class AnswerResponse
    {
        public string session_id;
        public string sdp;
        public string type;
    }

    [Serializable]
    public class DataEnvelope
    {
        public string type;
        public object payload;
        public string json_type;
        public object content;
    }
}
