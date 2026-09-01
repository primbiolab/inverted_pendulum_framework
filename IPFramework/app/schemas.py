from typing import Optional, List
from pydantic import BaseModel

class ConnectRequest(BaseModel):
    port: str
    baudrate: int = 115200

class GainsRequest(BaseModel):
    gains: List[float]

class ControllerRequest(BaseModel):
    controller: str = "LQR"
    gains: Optional[List[float]] = None

class MoveRequest(BaseModel):
    direction: str
    voltage: float = 5.0
