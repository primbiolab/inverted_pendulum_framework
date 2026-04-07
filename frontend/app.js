// Aplicación principal
class PendulumApp {
  constructor() {
    this.simulator = new PendulumSimulator();
    this.isRunning = false;
    this.animationId = null;
    this.lastTimestamp = 0;

    this.initElements();
    this.initEventListeners();
    this.initCanvas();

    // Actualizar estadísticas cada frame
    this.updateStats();
  }

  initElements() {
    // Sidebar
    this.sidebar = document.getElementById("sidebar");
    this.toggleBtn = document.getElementById("toggleBtn");

    // Navegación
    this.navBtns = document.querySelectorAll(".nav-btn[data-page]");
    this.pages = {
      home: document.getElementById("homePage"),
      pendulum: document.getElementById("pendulumPage"),
      train: document.getElementById("trainPage"),
      graphs: document.getElementById("graphsPage"),
    };

    // Controles del péndulo
    this.controlType = document.getElementById("controlType");
    this.comPort = document.getElementById("comPort");
    this.runBtn = document.getElementById("runBtn");
    this.stopBtn = document.getElementById("stopBtn");
    this.trainBtn = document.getElementById("trainBtn");
    this.exitBtn = document.getElementById("exitBtn");

    // Estadísticas
    this.positionVal = document.getElementById("positionVal");
    this.velocityVal = document.getElementById("velocityVal");
    this.angleVal = document.getElementById("angleVal");
    this.angularVelVal = document.getElementById("angularVelVal");
  }

  initEventListeners() {
    // Toggle sidebar
    this.toggleBtn.addEventListener("click", () => this.toggleSidebar());

    // Navegación
    this.navBtns.forEach((btn) => {
      btn.addEventListener("click", (e) => this.navigateTo(btn.dataset.page));
    });

    // Controles del péndulo
    this.controlType.addEventListener("change", (e) => {
      this.simulator.setControlType(e.target.value);
      console.log(`Control type changed to: ${e.target.value}`);
    });

    this.comPort.addEventListener("change", (e) => {
      console.log(`COM port selected: ${e.target.value}`);
    });

    this.runBtn.addEventListener("click", () => this.startSimulation());
    this.stopBtn.addEventListener("click", () => this.stopSimulation());

    this.trainBtn.addEventListener("click", () => {
      console.log("Training started");
      alert("Entrenamiento iniciado (simulación)");
    });

    this.exitBtn.addEventListener("click", () => {
      if (confirm("¿Estás seguro de que quieres salir?")) {
        this.stopSimulation();
        window.close();
      }
    });
  }

  initCanvas() {
    this.canvas = document.getElementById("pendulumCanvas");
    this.drawer = new PendulumDrawer(this.canvas);

    // Inicializar gráficas si existe el canvas
    const graphsCanvas = document.getElementById("graphsCanvas");
    if (graphsCanvas) {
      this.initGraphs(graphsCanvas);
    }
  }

  initGraphs(canvas) {
    const ctx = canvas.getContext("2d");
    canvas.width = 800;
    canvas.height = 400;

    // Dibujar gráfica de ejemplo
    ctx.fillStyle = "#282a36";
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    ctx.strokeStyle = "#bd93f9";
    ctx.lineWidth = 2;
    ctx.beginPath();

    for (let i = 0; i <= 100; i++) {
      const x = i * 8;
      const y = 200 + 50 * Math.sin(i * 0.1);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();

    ctx.fillStyle = "#f8f8f2";
    ctx.font = "14px monospace";
    ctx.fillText("Gráficas de entrenamiento", 20, 30);
    ctx.fillStyle = "#6272a4";
    ctx.fillText("Loss", 20, 370);
    ctx.fillText("Epochs", canvas.width - 60, 390);
  }

  toggleSidebar() {
    this.sidebar.classList.toggle("collapsed");
    const isExpanded = !this.sidebar.classList.contains("collapsed");

    // Actualizar textos de los botones
    document.querySelectorAll(".nav-text").forEach((text) => {
      text.style.display = isExpanded ? "inline" : "none";
    });
  }

  navigateTo(pageId) {
    // Ocultar todas las páginas
    Object.values(this.pages).forEach((page) => {
      if (page) page.classList.add("hidden");
    });

    // Mostrar página seleccionada
    const page = this.pages[pageId];
    if (page) page.classList.remove("hidden");

    // Actualizar botón activo
    this.navBtns.forEach((btn) => {
      btn.classList.remove("active");
      if (btn.dataset.page === pageId) {
        btn.classList.add("active");
      }
    });

    // Detener simulación al salir de la página del péndulo
    if (pageId !== "pendulum" && this.isRunning) {
      this.stopSimulation();
    }

    // Redibujar canvas al cambiar de página
    if (pageId === "pendulum" && this.simulator) {
      setTimeout(() => {
        const state = this.simulator.next(0);
        this.drawer.draw(state.cartPos, state.theta);
      }, 100);
    }
  }

  startSimulation() {
    if (this.isRunning) return;

    this.isRunning = true;
    this.simulator.reset();
    this.simulator.setControlType(this.controlType.value);

    console.log("Simulación iniciada");
    this.lastTimestamp = performance.now();
    this.simulationLoop();
  }

  stopSimulation() {
    this.isRunning = false;
    if (this.animationId) {
      cancelAnimationFrame(this.animationId);
      this.animationId = null;
    }
    console.log("Simulación detenida");
  }

  simulationLoop() {
    if (!this.isRunning) return;

    const now = performance.now();
    let dt = Math.min(0.033, (now - this.lastTimestamp) / 1000);

    if (dt > 0.001) {
      // Actualizar simulación
      const state = this.simulator.next(dt);

      // Actualizar visualización
      this.drawer.draw(state.cartPos, state.theta);

      // Actualizar estadísticas
      this.updateStatsDisplay(state);

      this.lastTimestamp = now;
    }

    this.animationId = requestAnimationFrame(() => this.simulationLoop());
  }

  updateStats() {
    // Actualizar estadísticas cada frame desde la simulación
    if (this.isRunning) return;

    // Si no está corriendo, mostrar estado actual
    const state = this.simulator.next(0);
    this.updateStatsDisplay(state);

    requestAnimationFrame(() => this.updateStats());
  }

  updateStatsDisplay(state) {
    if (this.positionVal) {
      this.positionVal.textContent = state.cartPos.toFixed(3);
      this.velocityVal.textContent = state.cartVel.toFixed(3);
      this.angleVal.textContent =
        ((state.theta * 180) / Math.PI).toFixed(1) + "°";
      this.angularVelVal.textContent = state.thetaDot.toFixed(3);
    }
  }
}

// Inicializar la aplicación cuando el DOM esté listo
document.addEventListener("DOMContentLoaded", () => {
  const app = new PendulumApp();
  console.log("Aplicación iniciada");
});
