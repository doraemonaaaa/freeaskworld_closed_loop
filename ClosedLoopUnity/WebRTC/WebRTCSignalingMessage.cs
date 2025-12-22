using System;

namespace Simulator.ClosedLoop
{
    [Serializable]
    public class WebRTCSignalingMessage
    {
        public string type; // "offer", "answer", "candidate"
        public string sdp;
        public string candidate;
        public string sdpMid;
        public int sdpMLineIndex;
    }
}
