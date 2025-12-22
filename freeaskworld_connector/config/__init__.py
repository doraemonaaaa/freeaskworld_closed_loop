"""Configuration module for FreeAskWorld connector.

This module loads configuration from .env file in this directory
and provides easy access to all settings.

Usage:
    from closed_loop.freeaskworld_connector.config import config
    
    framerate = config.WEBRTC_FRAMERATE
    interval = config.MIN_RESPONSE_INTERVAL
"""

import os
from pathlib import Path
from typing import Any, List, Dict
import json

from dotenv import load_dotenv


class Config:
    """Configuration container with all settings."""
    
    def __init__(self):
        """Load configuration from .env file in this directory."""
        config_dir = Path(__file__).parent
        env_file = config_dir / ".env"
        
        # Load .env file from config directory
        if env_file.exists():
            load_dotenv(dotenv_path=env_file)
            print(f"✓ Loaded configuration from: {env_file}")
        else:
            print(f"⚠ No .env file found at: {env_file}")
            print(f"  Using default values. Copy config/.env.example to config/.env to customize.")
        
        # Inference Control
        self.MIN_RESPONSE_INTERVAL = float(os.environ.get("MIN_RESPONSE_INTERVAL", "2.0"))
        
        # WebRTC Video
        self.WEBRTC_FRAMERATE = int(os.environ.get("WEBRTC_FRAMERATE", "30"))
        self.WEBRTC_BITRATE = int(os.environ.get("WEBRTC_BITRATE", "2000000"))
        
        # Preview
        self.JPEG_UPDATE_INTERVAL = float(os.environ.get("JPEG_UPDATE_INTERVAL", "0.1"))
        self.PREVIEW_PORT = int(os.environ.get("PREVIEW_PORT", "8080"))
        self.PREVIEW_HOST = os.environ.get("PREVIEW_HOST", "0.0.0.0")
        self.PREVIEW_AUTO_OPEN = os.environ.get("PREVIEW_AUTO_OPEN", "0") == "1"
        self.PREVIEW_URL = os.environ.get("PREVIEW_URL", None)
        
        # WebSocket Server
        self.WS_HOST = os.environ.get("WS_HOST", "0.0.0.0")
        self.WS_PORT = int(os.environ.get("WS_PORT", "8766"))
        self.WS_PING_INTERVAL = int(os.environ.get("WS_PING_INTERVAL", "20"))
        self.WS_PING_TIMEOUT = int(os.environ.get("WS_PING_TIMEOUT", "30"))
        self.WS_MAX_SIZE = int(os.environ.get("WS_MAX_SIZE", "30485760"))
        
        # WebRTC ICE
        self.ICE_TRANSPORT_POLICY = os.environ.get("ICE_TRANSPORT_POLICY", "all").lower()
        self.ICE_SERVERS = self._parse_ice_servers()
        
        # AgentFlow
        self.LLM_ENGINE_NAME = os.environ.get("LLM_ENGINE_NAME", "gpt-4o")
        self.FAST_MODE = os.environ.get("FAST_MODE", "true").lower() == "true"
        
        # Debugging
        self.LOG_LEVEL = os.environ.get("LOG_LEVEL", "INFO").upper()
        self.PERSIST_RGBD_DIR = os.environ.get("PERSIST_RGBD_DIR", None)
    
    def _parse_ice_servers(self) -> List[Dict[str, Any]]:
        """Parse ICE_SERVERS from environment."""
        env_value = os.environ.get("ICE_SERVERS")
        if not env_value:
            return [{"urls": "stun:stun.l.google.com:19302"}]
        
        try:
            servers_raw = json.loads(env_value)
            if isinstance(servers_raw, dict):
                return [servers_raw]
            if isinstance(servers_raw, list):
                return servers_raw
        except json.JSONDecodeError:
            pass
        
        # Fallback
        return [{"urls": "stun:stun.l.google.com:19302"}]
    
    def print_summary(self):
        """Print configuration summary."""
        print("\n" + "=" * 60)
        print("📋 Configuration Summary")
        print("=" * 60)
        print(f"  Inference Interval:  {self.MIN_RESPONSE_INTERVAL}s")
        print(f"  Video Framerate:     {self.WEBRTC_FRAMERATE} fps")
        print(f"  Video Bitrate:       {self.WEBRTC_BITRATE / 1_000_000:.1f} Mbps")
        print(f"  Preview Update:      {self.JPEG_UPDATE_INTERVAL}s ({1/self.JPEG_UPDATE_INTERVAL:.0f} fps)")
        print(f"  Preview Port:        {self.PREVIEW_PORT}")
        print(f"  WebSocket Port:      {self.WS_PORT}")
        print(f"  ICE Policy:          {self.ICE_TRANSPORT_POLICY}")
        print(f"  LLM Engine:          {self.LLM_ENGINE_NAME}")
        print(f"  Fast Mode:           {self.FAST_MODE}")
        print(f"  Log Level:           {self.LOG_LEVEL}")
        print("=" * 60 + "\n")


# Global configuration instance
config = Config()


__all__ = ["config", "Config"]
