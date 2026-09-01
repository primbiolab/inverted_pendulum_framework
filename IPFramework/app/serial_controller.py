import math
import serial
import time
import threading

class SerialController:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None

        self.MOTOR_PPR = 2400
        self.SHAFT_R = 1.2
        self.g = 9.81
        self.mp = 0.097
        self.lp = 0.2
        self.mplp = self.mp * self.lp
        self.Jp = 0.00517333
        self.INERTIA_EQ = self.Jp + self.mplp * self.lp
        self.MGL = self.mplp * self.g
        self.desired_energy = 2 * self.MGL

        self.K = [1600, 140, -13, -7.5]
        self.k_swingup = 1.5
        self.theta_threshold = math.radians(12)
        self.angle_setpoint = math.pi
        self.pos_limit_pulses = 5000
        self.startup_kick_voltage = 2.2
        self.startup_kick_max_steps = 12
        self.startup_kick_steps = 0
        self.startup_window_steps = 200
        self.startup_w_threshold = 0.08

        self.rail_left_pulses = 0
        self.rail_right_pulses = 0
        self.rail_center_pulses = 0
        self.position_tolerance = 100

        self.state = {
            "pos_cm": 0.0,
            "angle_rad": 0.0,
            "vel_cm_s": 0.0,
            "w_rad_s": 0.0,
            "raw_pulses": 0,
            "raw_angle_deg": 0.0,
        }
        self.current_voltage = 0.0
        self._lock = threading.Lock()

    # ── Serial connection ─────────────────────────────────────────

    def connect(self):
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        time.sleep(2)
        print(f"[SerialController] Conectado a {self.port}")

    def close(self):
        try:
            if self.ser and self.ser.is_open:
                self.send_voltage(0)
                time.sleep(0.05)
                self.ser.close()
                print("[SerialController] Puerto cerrado.")
        except Exception as e:
            print(f"[SerialController] Error cerrando puerto: {e}")
        finally:
            self.ser = None

    def is_connected(self):
        return self.ser is not None and self.ser.is_open

    # ── State reading ─────────────────────────────────────────────

    def read_state(self) -> bool:
        if not self.is_connected():
            return False
        try:
            with self._lock:
                self.ser.reset_input_buffer()
                self.ser.write(b'R\n')
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        except Exception as e:
            print(f"[SerialController] Error de lectura: {e}")
            return False

        if not line:
            return False

        try:
            parts = line.split(',')
            if len(parts) != 4:
                return False

            raw_pos = float(parts[0])
            raw_angle_deg = float(parts[1])
            raw_vel = float(parts[2])
            raw_angular_speed = float(parts[3])

            self.state["raw_pulses"] = raw_pos
            self.state["raw_angle_deg"] = raw_angle_deg
            self.state["pos_cm"] = (2.0 * math.pi * raw_pos / self.MOTOR_PPR) * self.SHAFT_R
            self.state["vel_cm_s"] = (2.0 * math.pi * raw_vel / self.MOTOR_PPR) * self.SHAFT_R

            theta_cpp = math.radians(-(raw_angle_deg - 180.0))
            theta_cpp = math.fmod(theta_cpp, 2.0 * math.pi)
            if theta_cpp < 0:
                theta_cpp += 2.0 * math.pi

            self.state["angle_rad"] = theta_cpp
            self.state["w_rad_s"] = math.radians(-raw_angular_speed)
            return True

        except ValueError:
            return False

    def read_state_monitor(self) -> bool:
        return self.read_state()

    # ── Command sending ───────────────────────────────────────────

    def send_voltage(self, voltage: float):
        self.current_voltage = voltage
        msg = f"0{voltage:.2f}\n"
        try:
            with self._lock:
                if self.is_connected():
                    self.ser.write(msg.encode('utf-8'))
        except Exception as e:
            print(f"[SerialController] Error enviando voltaje: {e}")

    def reset_encoder(self):
        try:
            with self._lock:
                if self.is_connected():
                    self.ser.write(b"Z\n")
                    time.sleep(0.05)
            print("[SerialController] Encoder reseteado a 0.")
        except Exception as e:
            print(f"[SerialController] Error reseteando encoder: {e}")

    def send_stop_motor(self):
        self.send_voltage(0)

    def send_center(self):
        """Mueve el carro al centro vía comando H (Arduino position control)."""
        self.move_to_center()

    # ── Calibration ─────────────────────────────────────────────

    def _send_and_ack(self, cmd: str, expected_prefix: str, timeout: float = 1.5) -> str:
        if not self.is_connected():
            return ""
        with self._lock:
            self.ser.reset_input_buffer()
            self.ser.write(cmd.encode('utf-8'))
            deadline = time.time() + timeout
            while time.time() < deadline:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith(expected_prefix):
                        return line
                time.sleep(0.005)
        return ""

    def set_left_limit(self) -> int:
        resp = self._send_and_ack("E\n", "E_OK")
        if resp and ',' in resp:
            val = int(resp.split(',')[1])
            self.rail_left_pulses = val
            return val
        return 0

    def set_right_limit(self) -> int:
        resp = self._send_and_ack("D\n", "D_OK")
        if resp and ',' in resp:
            val = int(resp.split(',')[1])
            self.rail_right_pulses = val
            return val
        return 0

    def compute_center(self) -> dict:
        resp = self._send_and_ack("O\n", "O_OK")
        result = {"center": 0, "left": 0, "right": 0}
        if resp and ',' in resp:
            parts = resp.split(',')
            if len(parts) >= 4:
                result["center"] = int(parts[1])
                result["left"] = int(parts[2])
                result["right"] = int(parts[3])
                self.rail_center_pulses = result["center"]
                self.rail_left_pulses = result["left"]
                self.rail_right_pulses = result["right"]
                span = abs(self.rail_right_pulses - self.rail_left_pulses)
                if span > 100:
                    self.pos_limit_pulses = span // 2
        return result

    def move_to_center(self, timeout: float = 10.0) -> bool:
        resp = self._send_and_ack("H\n", "H_OK")
        if not resp.startswith("H_OK"):
            return False
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.read_state()
            pos_error = abs(self.state["raw_pulses"] - self.rail_center_pulses)
            if pos_error <= self.position_tolerance:
                return True
            time.sleep(0.05)
        self.read_state()
        return self.is_at_center()

    def is_at_center(self, tolerance: int = None) -> bool:
        if tolerance is None:
            tolerance = self.position_tolerance
        return abs(self.state["raw_pulses"] - self.rail_center_pulses) <= tolerance

    def is_out_of_limits(self) -> bool:
        return abs(self.state["raw_pulses"]) > self.pos_limit_pulses

    def apply_calibration(self) -> bool:
        if not self.is_at_center():
            raise RuntimeError(
                f"El carro no está en el centro (error: "
                f"{abs(self.state['raw_pulses'] - self.rail_center_pulses)} pulsos). "
                f"Presione 'Mover a Centro' primero."
            )
        resp = self._send_and_ack("A\n", "A_OK")
        if resp.startswith("A_OK"):
            self.rail_center_pulses = 0
            self.rail_left_pulses = -self.pos_limit_pulses
            self.rail_right_pulses = self.pos_limit_pulses
            return True
        raise RuntimeError("No se recibió respuesta A_OK del Arduino al aplicar la calibración")

    def pulses_to_cm(self, pulses: float) -> float:
        return (2.0 * math.pi * pulses / self.MOTOR_PPR) * self.SHAFT_R

    # ── LQR + Swing-up control ──────────────────────────────────

    def avoidStall(self, u: float) -> float:
        MAX_STALL_U = 90.0
        if abs(u) < MAX_STALL_U:
            return (2.0 + MAX_STALL_U) if u > 0 else (-2.0 - MAX_STALL_U)
        return u

    def compute_control(self) -> float:
        theta = self.state["angle_rad"]
        w = self.state["w_rad_s"]
        x = self.state["pos_cm"]
        v = self.state["vel_cm_s"]
        raw_pulses = self.state["raw_pulses"]

        if self.startup_window_steps > 0:
            self.startup_window_steps -= 1
            near_bottom = (theta > (2 * math.pi - 0.35) or theta < 0.35)
            near_rest = abs(w) < self.startup_w_threshold
            if near_bottom and near_rest and self.startup_kick_steps < self.startup_kick_max_steps:
                self.startup_kick_steps += 1
                kick_dir = -1.0 if raw_pulses > 0 else 1.0
                return kick_dir * self.startup_kick_voltage

        if abs(raw_pulses) > self.pos_limit_pulses:
            return 12.0 if raw_pulses <= 0 else -12.0

        if abs(self.angle_setpoint - theta) < self.theta_threshold:
            u_lqr = (
                self.K[0] * (self.angle_setpoint - theta)
                - self.K[1] * w
                + self.K[2] * (0 - x)
                - self.K[3] * v
            )
            u_pwm = self.avoidStall(u_lqr)
            u_pwm = max(min(u_pwm, 255.0), -255.0)
            vol_u = u_pwm * (12.0 / 255.0)
            return vol_u

        current_energy = (
            0.5 * self.INERTIA_EQ * (w ** 2)
            + self.MGL * (1 - math.cos(theta))
        )
        if (theta > (2 * math.pi - 0.28) or theta < 0.28) and current_energy < 0.85:
            accel = -300 * self.k_swingup * abs(current_energy - self.desired_energy) * w
            u_pwm = self.avoidStall(accel)
            u_pwm = max(min(u_pwm, 255.0), -255.0)
            vol_u = u_pwm * (12.0 / 255.0)
            return vol_u

        return 0.0

    def reset_startup(self):
        self.startup_kick_steps = 0
        self.startup_window_steps = 200
