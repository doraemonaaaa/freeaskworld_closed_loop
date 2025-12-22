using UnityEngine;
using NativeWebSocket;


namespace Simulator.ClosedLoop
{
    [RequireComponent(typeof(Connection))]
    public class WebSocketManager : SingletonMono<WebSocketManager>
    {
        protected override bool isDontDestroyOnLoad => false;
        public Connection connection;

        protected override void Awake()
        {
            base.Awake();
            connection = GetComponent<Connection>();
        }

        private WebSocket WebSocket => connection?.websocket;

        private bool IsConnected => WebSocket != null && WebSocket.State == WebSocketState.Open;

    }
}
