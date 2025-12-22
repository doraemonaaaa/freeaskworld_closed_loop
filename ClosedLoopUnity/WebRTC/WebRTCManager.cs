using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.WebRTC;
using Simulator.ClosedLoop;
using Newtonsoft.Json;

namespace Simulator.ClosedLoop
{
    public class WebRTCManager : SingletonMono<WebRTCManager>
    {
        protected override bool isDontDestroyOnLoad => true;

        private RTCPeerConnection _pc;
        private List<RTCRtpSender> _senders = new List<RTCRtpSender>();
        
        // Configuration
        private RTCConfiguration _config = new RTCConfiguration
        {
            iceServers = new[] { new RTCIceServer { urls = new[] { "stun:stun.l.google.com:19302" } } },
            iceTransportPolicy = RTCIceTransportPolicy.All,
            iceCandidatePoolSize = 10
        };

        protected override void Awake()
        {
            base.Awake();
            // Initialize WebRTC - commented out as API might differ or be automatic
            // Unity.WebRTC.WebRTC.Initialize(EncoderType.Software);
        }

        private void Start()
        {
            // WebRTC.Update is an infinite coroutine that handles frame updates. 
            // It should be started once and runs until stopped.
            StartCoroutine(Unity.WebRTC.WebRTC.Update());
            // Listen for signaling messages from WebSocket
            EventCenter.Instance.AddListener<Type, object>("OnWebSocketMessageReceived", OnMessageReceived);
        }

        private void OnDestroy()
        {
            if (EventCenter.Instance != null)
                EventCenter.Instance.RemoveListener<Type, object>("OnWebSocketMessageReceived", OnMessageReceived);
            
            _senders.Clear();

            if (_pc != null)
            {
                _pc.Close();
                _pc.Dispose();
                _pc = null;
            }

            // Unity.WebRTC.WebRTC.Dispose();
        }

        public void AddTrack(VideoStreamTrack track)
        {
            if (_pc == null)
            {
                CreatePeerConnection();
            }
            
            // Check if track already exists
            foreach (var s in _senders)
            {
                if (s.Track == track) return;
            }
            
            Debug.Log($"AddTrack called: track.Kind={track.Kind}, current senders count={_senders.Count}");
            var sender = _pc.AddTrack(track);
            _senders.Add(sender);
            
            // Configure encoder for higher frame rate and quality
            var parameters = sender.GetParameters();
            foreach (var encoding in parameters.encodings)
            {
                encoding.maxFramerate = 30;  // Target 30fps
                encoding.maxBitrate = 2000000;  // 2Mbps
            }
            sender.SetParameters(parameters);
            
            Debug.Log($"Track added successfully. Total tracks: {_senders.Count}, maxFramerate=30");
            
            // If connected but tracks were added late, trigger renegotiation
            if (_pc.ConnectionState == RTCPeerConnectionState.Connected && _senders.Count > 0)
            {
                Debug.Log("Triggering renegotiation for late-added track...");
                StartCoroutine(CreateOffer());
            }
        }
        
        public List<RTCRtpSender> GetSenders()
        {
            return _senders;
        }

        private void CreatePeerConnection()
        {
            if (_pc != null) return;

            _pc = new RTCPeerConnection(ref _config);
            
            _pc.OnIceCandidate = candidate => {
                SendSignalingMessage(new WebRTCSignalingMessage
                {
                    type = "candidate",
                    candidate = candidate.Candidate,
                    sdpMid = candidate.SdpMid,
                    sdpMLineIndex = candidate.SdpMLineIndex ?? 0
                });
            };

            _pc.OnIceConnectionChange = state => {
                Debug.Log($"WebRTC ICE State: {state}");
                // Don't restart on Disconnected - it's often temporary
                // Only log Failed state for debugging
                if (state == RTCIceConnectionState.Failed)
                {
                    Debug.LogError("ICE Failed - check network/firewall on localhost");
                }
            };

            _pc.OnConnectionStateChange = state => {
                Debug.Log($"WebRTC Connection State: {state}");
            };

            _pc.OnNegotiationNeeded = () => {
                // Prevent glare: only negotiate if stable
                if (_pc.SignalingState != RTCSignalingState.Stable)
                {
                    Debug.Log($"Skip negotiation - signaling state: {_pc.SignalingState}");
                    return;
                }
                // Prevent redundant negotiations
                if (_pc.ConnectionState == RTCPeerConnectionState.Connected || 
                    _pc.ConnectionState == RTCPeerConnectionState.Connecting)
                {
                    Debug.Log($"Skip negotiation - already connecting/connected");
                    return;
                }
                Debug.Log($"Starting negotiation... (tracks count: {_senders.Count})");
                StartCoroutine(CreateOffer());
            };
        }

        private IEnumerator CreateOffer()
        {
            var op = _pc.CreateOffer();
            yield return op;

            if (op.IsError)
            {
                Debug.LogError($"WebRTC CreateOffer Error: {op.Error}");
                yield break;
            }

            var desc = op.Desc;
            var op2 = _pc.SetLocalDescription(ref desc);
            yield return op2;

            if (op2.IsError)
            {
                Debug.LogError($"WebRTC SetLocalDescription Error: {op2.Error}");
                yield break;
            }

            SendSignalingMessage(new WebRTCSignalingMessage
            {
                type = "offer",
                sdp = desc.sdp
            });
        }

        private void OnMessageReceived(Type type, object data)
        {
            if (type == typeof(WebRTCSignalingMessage))
            {
                var msg = (WebRTCSignalingMessage)data;
                StartCoroutine(HandleSignalingMessage(msg));
            }
        }

        private IEnumerator HandleSignalingMessage(WebRTCSignalingMessage msg)
        {
            if (_pc == null) CreatePeerConnection();

            if (msg.type == "offer")
            {
                var desc = new RTCSessionDescription { type = RTCSdpType.Offer, sdp = msg.sdp };
                var op = _pc.SetRemoteDescription(ref desc);
                yield return op;
                if (op.IsError) Debug.LogError($"SetRemoteDescription Error: {op.Error}");

                var op2 = _pc.CreateAnswer();
                yield return op2;
                if (op2.IsError) Debug.LogError($"CreateAnswer Error: {op2.Error}");

                var answer = op2.Desc;
                var op3 = _pc.SetLocalDescription(ref answer);
                yield return op3;

                SendSignalingMessage(new WebRTCSignalingMessage { type = "answer", sdp = answer.sdp });
            }
            else if (msg.type == "answer")
            {
                var desc = new RTCSessionDescription { type = RTCSdpType.Answer, sdp = msg.sdp };
                var op = _pc.SetRemoteDescription(ref desc);
                yield return op;
                if (op.IsError) Debug.LogError($"SetRemoteDescription Error: {op.Error}");
            }
            else if (msg.type == "candidate")
            {
                var candidate = new RTCIceCandidate(new RTCIceCandidateInit
                {
                    candidate = msg.candidate,
                    sdpMid = msg.sdpMid,
                    sdpMLineIndex = msg.sdpMLineIndex
                });
                _pc.AddIceCandidate(candidate);
            }
        }

        private void SendSignalingMessage(WebRTCSignalingMessage msg)
        {
            var payload = new
            {
                type = "json",
                json_type = typeof(WebRTCSignalingMessage).FullName,
                content = msg
            };
            
            string json = JsonConvert.SerializeObject(payload);
            if (WebSocketManager.Instance != null && WebSocketManager.Instance.connection != null && WebSocketManager.Instance.connection.websocket != null)
            {
                WebSocketManager.Instance.connection.websocket.SendText(json);
            }
        }
    }
}
