import asyncio
import json
import threading
from typing import Optional, List, Dict
from fastapi import WebSocket

from .serial_controller import SerialController

class AppState:
    def __init__(self):
        self.controller = SerialController()
        self.ws_clients: List[WebSocket] = []
        self.is_running = False
        self.is_monitoring = False
        self.current_controller_type = "LQR"
        self.control_thread: Optional[threading.Thread] = None
        self.monitor_thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        self.loop: Optional[asyncio.AbstractEventLoop] = None

        self.data_log: Dict[str, list] = {
            "time": [], "pos": [], "angle": [],
            "vel_pos": [], "vel_angle": [], "action": [],
        }
        self.run_start_time = 0.0
        self.calibration_pulses = 0.0
        self.calibration_cm = 0.0


state = AppState()


async def _ws_broadcast_async(data: dict):
    if not state.ws_clients:
        return
    msg = json.dumps(data)
    disconnected = []
    for ws in state.ws_clients:
        try:
            await ws.send_text(msg)
        except Exception:
            disconnected.append(ws)
    for ws in disconnected:
        if ws in state.ws_clients:
            state.ws_clients.remove(ws)


def ws_broadcast(data: dict):
    if state.loop is not None:
        asyncio.run_coroutine_threadsafe(_ws_broadcast_async(data), state.loop)
