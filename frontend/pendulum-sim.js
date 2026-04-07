// Simulador físico del péndulo invertido
class PendulumSimulator {
  constructor() {
    // Parámetros físicos
    this.M = 1.0; // masa del carro (kg)
    this.m = 0.1; // masa del péndulo (kg)
    this.l = 0.5; // longitud (m)
    this.g = 9.81; // gravedad (m/s²)
    this.b = 0.0; // fricción (N/m/s)
    this.trackHalfRange = 2.0; // rango de la pista (m)

    // Estado inicial
    this.x = 0.0;
    this.xDot = 0.0;
    this.theta = 0.0; // pequeña inclinación inicial
    this.thetaDot = 0.0;
    this.t = 0.0;

    // Control
    this.controlFunc = null;
    this.controlType = "LQR";

    // Ganancia LQR (simplificada)
    this.K = [-1.5, -2.5, 35, 8]; // [x, x_dot, theta, theta_dot]
  }

  setControlType(type) {
    this.controlType = type;
    if (type === "LQR") {
      this.K = [-1.5, -2.5, 35, 8];
    } else if (type === "LQR + Swim up") {
      this.K = [-1.8, -3.0, 40, 10];
    } else if (type === "SAC") {
      this.K = [-1.2, -2.0, 30, 6];
    } else if (type === "DDPG") {
      this.K = [-1.6, -2.8, 38, 9];
    }
  }

  computeControl(state, t) {
    const [x, xDot, theta, thetaDot] = state;

    if (this.controlType === "LQR + Swim up" && Math.abs(theta) > 0.5) {
      // Estrategia de "swim up" - energía creciente
      const energyTarget = 2 * this.m * this.g * this.l;
      const energy =
        0.5 * this.m * Math.pow(this.l * thetaDot, 2) +
        this.m * this.g * this.l * (1 - Math.cos(theta));
      const energyError = energyTarget - energy;
      return Math.min(10, Math.max(-10, 2 * energyError * Math.cos(theta)));
    }

    // Control LQR estándar
    let force = 0;
    force += this.K[0] * x;
    force += this.K[1] * xDot;
    force += this.K[2] * theta;
    force += this.K[3] * thetaDot;

    return Math.min(15, Math.max(-15, force));
  }

  derivatives(state, t, F) {
    const [x, xDot, theta, thetaDot] = state;
    const M = this.M;
    const m = this.m;
    const l = this.l;
    const g = this.g;
    const b = this.b;

    const sinTheta = Math.sin(theta);
    const cosTheta = Math.cos(theta);

    const denom = l * (4 / 3 - (m * cosTheta * cosTheta) / (M + m));
    const thetaNum =
      g * sinTheta +
      cosTheta *
        ((-F - m * l * thetaDot * thetaDot * sinTheta + b * xDot) / (M + m));
    const thetaDDot = thetaNum / denom;
    const xDDot =
      (F +
        m * l * (thetaDot * thetaDot * sinTheta - thetaDDot * cosTheta) -
        b * xDot) /
      (M + m);

    return [xDot, xDDot, thetaDot, thetaDDot];
  }

  rk4Step(dt) {
    const state0 = [this.x, this.xDot, this.theta, this.thetaDot];
    const t0 = this.t;

    let F0 = 0;
    if (this.controlFunc) {
      F0 = this.controlFunc(state0, t0);
    } else {
      F0 = this.computeControl(state0, t0);
    }

    const k1 = this.derivatives(state0, t0, F0);
    const s1 = state0.map((val, i) => val + 0.5 * dt * k1[i]);
    const F1 = this.controlFunc
      ? this.controlFunc(s1, t0 + 0.5 * dt)
      : this.computeControl(s1, t0 + 0.5 * dt);
    const k2 = this.derivatives(s1, t0 + 0.5 * dt, F1);

    const s2 = state0.map((val, i) => val + 0.5 * dt * k2[i]);
    const F2 = this.controlFunc
      ? this.controlFunc(s2, t0 + 0.5 * dt)
      : this.computeControl(s2, t0 + 0.5 * dt);
    const k3 = this.derivatives(s2, t0 + 0.5 * dt, F2);

    const s3 = state0.map((val, i) => val + dt * k3[i]);
    const F3 = this.controlFunc
      ? this.controlFunc(s3, t0 + dt)
      : this.computeControl(s3, t0 + dt);
    const k4 = this.derivatives(s3, t0 + dt, F3);

    const newState = state0.map(
      (val, i) => val + (dt / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]),
    );

    [this.x, this.xDot, this.theta, this.thetaDot] = newState;
    this.t += dt;

    // Normalizar theta a [-π, π]
    while (this.theta > Math.PI) this.theta -= 2 * Math.PI;
    while (this.theta < -Math.PI) this.theta += 2 * Math.PI;
  }

  next(dt) {
    // Sub-pasos para estabilidad
    const maxSubstep = 0.02;
    const steps = Math.max(1, Math.ceil(dt / maxSubstep));
    const subDt = dt / steps;

    for (let i = 0; i < steps; i++) {
      this.rk4Step(subDt);
    }

    // Normalizar posición
    let xNorm = this.x / this.trackHalfRange;
    xNorm = Math.max(-1, Math.min(1, xNorm));

    return {
      cartPos: xNorm,
      cartVel: this.xDot,
      theta: this.theta,
      thetaDot: this.thetaDot,
    };
  }

  reset() {
    this.x = 0;
    this.xDot = 0;
    this.theta = 0.05;
    this.thetaDot = 0;
    this.t = 0;
  }
}

// Dibujador del péndulo (versión corregida)
class PendulumDrawer {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.fixedWidth = 700;
    this.fixedHeight = 400;

    // Configurar tamaño fijo para evitar pixelado
    this.setFixedSize();

    // Escuchar cambios de tamaño de ventana
    window.addEventListener("resize", () => this.handleResize());
  }

  setFixedSize() {
    // Mantener dimensiones fijas para evitar pixelado
    this.canvas.width = this.fixedWidth;
    this.canvas.height = this.fixedHeight;
    this.canvas.style.width = "100%";
    this.canvas.style.height = "auto";
    this.canvas.style.maxWidth = `${this.fixedWidth}px`;
    this.canvas.style.aspectRatio = `${this.fixedWidth}/${this.fixedHeight}`;
  }

  handleResize() {
    // Solo reajustar el estilo, no las dimensiones internas
    if (this.canvas.width !== this.fixedWidth) {
      this.canvas.width = this.fixedWidth;
      this.canvas.height = this.fixedHeight;
    }
    // Forzar redibujo
    if (this.lastState) {
      this.draw(this.lastState.cartPos, this.lastState.theta);
    }
  }

  draw(cartPos, theta) {
    // Guardar el último estado para redibujar si es necesario
    this.lastState = { cartPos, theta };

    const w = this.canvas.width;
    const h = this.canvas.height;
    const ctx = this.ctx;

    // Configurar el canvas para alta calidad
    ctx.imageSmoothingEnabled = true;
    ctx.imageSmoothingQuality = "high";

    // Limpiar canvas con color de fondo
    ctx.fillStyle = "#282a36";
    ctx.fillRect(0, 0, w, h);

    // Configurar colores (Dracula)
    const colors = {
      muted: "#6272a4",
      panel: "#21222c",
      currentLine: "#44475a",
      fg: "#f8f8f2",
      accent: "#bd93f9",
      orange: "#ffb86c",
    };

    // Margen y pista
    const margin = 60;
    const trackLeft = margin;
    const trackRight = w - margin;
    const trackWidth = trackRight - trackLeft;

    // Posición del carro (limitada al track)
    let xPix = trackLeft + ((cartPos + 1) / 2) * trackWidth;
    xPix = Math.max(trackLeft + 20, Math.min(trackRight - 20, xPix));

    // Dimensiones del carro
    const cartW = 80;
    const cartH = 35;
    const cartX = xPix - cartW / 2;
    const cartY = h * 0.6;

    // Dibujar pista con efecto de rieles
    ctx.save();
    ctx.shadowBlur = 0;

    // Línea principal de la pista
    ctx.beginPath();
    ctx.strokeStyle = colors.muted;
    ctx.lineWidth = 3;
    ctx.moveTo(trackLeft, cartY + cartH + 15);
    ctx.lineTo(trackRight, cartY + cartH + 15);
    ctx.stroke();

    // Rieles decorativos
    ctx.beginPath();
    ctx.strokeStyle = colors.currentLine;
    ctx.lineWidth = 1.5;
    ctx.setLineDash([5, 5]);
    ctx.moveTo(trackLeft, cartY + cartH + 10);
    ctx.lineTo(trackRight, cartY + cartH + 10);
    ctx.stroke();
    ctx.setLineDash([]);

    // Sombra del carro
    ctx.fillStyle = "rgba(0, 0, 0, 0.3)";
    ctx.fillRect(cartX + 2, cartY + 2, cartW, cartH);

    // Dibujar carro principal
    ctx.fillStyle = colors.panel;
    ctx.strokeStyle = colors.currentLine;
    ctx.lineWidth = 2;
    ctx.fillRect(cartX, cartY, cartW, cartH);
    ctx.strokeRect(cartX, cartY, cartW, cartH);

    // Detalles del carro (ventanas/líneas)
    ctx.beginPath();
    ctx.strokeStyle = colors.muted;
    ctx.lineWidth = 1;
    ctx.moveTo(cartX + 10, cartY + 5);
    ctx.lineTo(cartX + cartW - 10, cartY + 5);
    ctx.stroke();

    // Dibujar ruedas con efecto 3D
    const wheelRadius = 10;

    // Sombra de ruedas
    ctx.fillStyle = "rgba(0, 0, 0, 0.3)";
    ctx.beginPath();
    ctx.arc(
      cartX + cartW * 0.22 + 1,
      cartY + cartH + wheelRadius + 1,
      wheelRadius,
      0,
      2 * Math.PI,
    );
    ctx.fill();
    ctx.beginPath();
    ctx.arc(
      cartX + cartW * 0.78 + 1,
      cartY + cartH + wheelRadius + 1,
      wheelRadius,
      0,
      2 * Math.PI,
    );
    ctx.fill();

    // Ruedas
    ctx.fillStyle = colors.muted;
    ctx.strokeStyle = colors.currentLine;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(
      cartX + cartW * 0.22,
      cartY + cartH + wheelRadius,
      wheelRadius,
      0,
      2 * Math.PI,
    );
    ctx.fill();
    ctx.stroke();

    ctx.beginPath();
    ctx.arc(
      cartX + cartW * 0.78,
      cartY + cartH + wheelRadius,
      wheelRadius,
      0,
      2 * Math.PI,
    );
    ctx.fill();
    ctx.stroke();

    // Centros de las ruedas
    ctx.fillStyle = colors.panel;
    ctx.beginPath();
    ctx.arc(
      cartX + cartW * 0.22,
      cartY + cartH + wheelRadius,
      4,
      0,
      2 * Math.PI,
    );
    ctx.fill();
    ctx.beginPath();
    ctx.arc(
      cartX + cartW * 0.78,
      cartY + cartH + wheelRadius,
      4,
      0,
      2 * Math.PI,
    );
    ctx.fill();

    // Pivote (punto de rotación)
    const pivotX = xPix;
    const pivotY = cartY - 8;

    // Longitud de la varilla
    const rodLength = Math.min(h * 0.3, 180);

    // Posición de la masa (bob)
    const bobX = pivotX + rodLength * Math.sin(theta);
    const bobY = pivotY - rodLength * Math.cos(theta);

    // Sombra de la varilla y masa
    ctx.shadowColor = "rgba(0, 0, 0, 0.2)";
    ctx.shadowBlur = 5;

    // Dibujar varilla con gradiente
    const gradient = ctx.createLinearGradient(pivotX, pivotY, bobX, bobY);
    gradient.addColorStop(0, colors.accent);
    gradient.addColorStop(1, colors.fg);

    ctx.beginPath();
    ctx.strokeStyle = gradient;
    ctx.lineWidth = 4;
    ctx.moveTo(pivotX, pivotY);
    ctx.lineTo(bobX, bobY);
    ctx.stroke();

    // Dibujar pivote (eje de rotación)
    ctx.shadowBlur = 3;
    ctx.fillStyle = colors.accent;
    ctx.beginPath();
    ctx.arc(pivotX, pivotY, 6, 0, 2 * Math.PI);
    ctx.fill();

    // Anillo exterior del pivote
    ctx.strokeStyle = colors.fg;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(pivotX, pivotY, 8, 0, 2 * Math.PI);
    ctx.stroke();

    // Dibujar masa (bob) con gradiente radial
    const bobRadius = Math.max(12, Math.min(2, rodLength * 0.15));
    const radialGradient = ctx.createRadialGradient(
      bobX - 3,
      bobY - 3,
      3,
      bobX,
      bobY,
      bobRadius,
    );
    radialGradient.addColorStop(0, colors.orange);
    radialGradient.addColorStop(1, "#ff9f4a");

    ctx.fillStyle = radialGradient;
    ctx.beginPath();
    ctx.arc(bobX, bobY, bobRadius, 0, 2 * Math.PI);
    ctx.fill();

    // Borde de la masa
    ctx.strokeStyle = colors.currentLine;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(bobX, bobY, bobRadius, 0, 2 * Math.PI);
    ctx.stroke();

    // Reflejo en la masa
    ctx.fillStyle = "rgba(255, 255, 255, 0.3)";
    ctx.beginPath();
    ctx.arc(bobX - 3, bobY - 3, 4, 0, 2 * Math.PI);
    ctx.fill();

    ctx.restore();

    // Información del ángulo en el canvas
    ctx.font = 'bold 14px "Segoe UI", monospace';
    ctx.fillStyle = colors.muted;
    ctx.shadowBlur = 0;
    ctx.fillText(`Ángulo: ${((theta * 180) / Math.PI).toFixed(1)}°`, 20, 40);

    // Indicador de estabilidad
    if (Math.abs(theta) < 0.1) {
      ctx.fillStyle = colors.green;
      ctx.fillText("✓ Estable", w - 100, 40);
    } else if (Math.abs(theta) > 0.5) {
      ctx.fillStyle = colors.red;
      ctx.fillText("⚠ Inestable", w - 100, 40);
    }
  }
}
