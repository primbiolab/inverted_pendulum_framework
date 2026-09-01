from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent.parent
FRONTEND_DIR = SCRIPT_DIR.parent / "frontend"

MOTOR_PPR = 2400
SHAFT_R = 1.2
G = 9.81
MP = 0.097
LP = 0.2
JP = 0.00517333
