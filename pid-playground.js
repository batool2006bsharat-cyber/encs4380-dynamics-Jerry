
"use strict";

const $ = id => document.getElementById(id);

const DT = 0.001;
const DURATION = 20;
const UMAX = 20;

let currentRun = null;
let pendingCheck = null;

const defaults = {
  plant: "first",
  setpoint: 1,
  K: 1,
  tau: 1,
  zeta: 0.5,
  wn: 2,
  kp: 9,
  ki: 0,
  kd: 0
};

// =====================================
// INPUTS AND SETTINGS
// =====================================

function readSettings() {
  const p = { plant: $("plant").value };

  for (const id of [
    "setpoint", "K", "tau", "zeta",
    "wn", "kp", "ki", "kd"
  ]) {
    if ($(id).value.trim() === "") {
      throw Error("Please fill in every numeric field.");
    }

    p[id] = Number($(id).value);
  }

  return p;
}

function validate(p) {
  if (Object.entries(p).some(
    ([k, v]) => k !== "plant" && !Number.isFinite(v)
  )) {
    throw Error("All values must be finite numbers.");
  }

  if (
    p.K <= 0 ||
    p.setpoint <= 0 ||
    (p.plant === "first" && p.tau <= 0) ||
    (p.plant === "second" &&
      (p.wn <= 0 || p.zeta < 0)) ||
    [p.kp, p.ki, p.kd].some(x => x < 0)
  ) {
    throw Error(
      "K and setpoint must be positive; " +
      "time constant/frequency must be positive; " +
      "damping and gains cannot be negative."
    );
  }
}

function changePlant() {
  const first = $("plant").value === "first";

  document.querySelectorAll(".first-only").forEach(el => {
    el.hidden = !first;
  });

  document.querySelectorAll(".second-only").forEach(el => {
    el.hidden = first;
  });

  $("plantEquation").textContent = first
    ? "K / (τs + 1)"
    : "Kωn² / (s² + 2ζωns + ωn²)";
}

function updateGains() {
  for (const id of ["kp", "ki", "kd"]) {
    $(id + "Value").textContent =
      Number($(id).value).toFixed(2);
  }
}

// =====================================
// CLOSED-LOOP PID SIMULATION
// =====================================

function simulate(p) {
  const time = [];
  const output = [];
  const control = [];

  let y = 0;
  let velocity = 0;
  let integral = 0;
  let oldY = 0;

  const steps = Math.round(DURATION / DT);

  for (let i = 0; i <= steps; i++) {
    const t = i * DT;
    const error = p.setpoint - y;

    // Derivative on measurement
    const rate = i === 0
      ? 0
      : (y - oldY) / DT;

    const pd = p.kp * error - p.kd * rate;

    // Conditional anti-windup
    const testIntegral = integral + error * DT;
    const candidate = pd + p.ki * testIntegral;

    if (
      p.ki > 0 &&
      (
        (candidate <= UMAX && candidate >= -UMAX) ||
        (candidate > UMAX && error < 0) ||
        (candidate < -UMAX && error > 0)
      )
    ) {
      integral = testIntegral;
    }

    const u = Math.min(
      UMAX,
      Math.max(-UMAX, pd + p.ki * integral)
    );

    time.push(t);
    output.push(y);
    control.push(u);

    oldY = y;

    if (p.plant === "first") {
      // First-order model
      // G(s) = K / (tau*s + 1)

      y += DT * (p.K * u - y) / p.tau;

    } else {
      // Second-order model

      const acceleration =
        p.K * p.wn * p.wn * u -
        2 * p.zeta * p.wn * velocity -
        p.wn * p.wn * y;

      velocity += DT * acceleration;
      y += DT * velocity;
    }

    if (
      !Number.isFinite(y) ||
      !Number.isFinite(velocity) ||
      Math.abs(y) > 1e7
    ) {
      throw Error(
        "Simulation diverged for these gains. " +
        "Reduce gains and retry."
      );
    }
  }

  return { time, output, control };
}

// =====================================
// PERFORMANCE METRICS
// =====================================

function thresholdCross(t, y, value) {
  for (let i = 1; i < y.length; i++) {
    if (
      y[i - 1] < value &&
      y[i] >= value
    ) {
      const fraction =
        (value - y[i - 1]) /
        (y[i] - y[i - 1]);

      return t[i - 1] +
        fraction * (t[i] - t[i - 1]);
    }
  }

  return null;
}

function metrics(data, target) {
  const { time, output } = data;
  const n = output.length;

  let peak = -Infinity;
  for (const value of output) {
    if (value > peak) peak = value;
  }

  // Estimate the actual final value from the last 5% of the response.
  // This matters for P-only control: the output can settle below the setpoint.
  const tailStart = Math.floor(n * 0.95);
  const tail = output.slice(tailStart);
  const finalValue =
    tail.reduce((sum, value) => sum + value, 0) / tail.length;

  // Rise time is measured from 10% to 90% of the final output,
  // not 10% to 90% of the requested setpoint.
  const ten = thresholdCross(time, output, 0.1 * finalValue);
  const ninety = thresholdCross(time, output, 0.9 * finalValue);
  const rise =
    ten === null || ninety === null ? null : ninety - ten;

  // Settling time is the last time the response leaves the +/-2% band
  // around its actual final value. Separately check that the tail is stable,
  // so a sustained oscillation is not incorrectly marked as settled.
  const tolerance = 0.02 * Math.max(
    Math.abs(finalValue),
    Math.abs(target) * 1e-6
  );
  const tailStable = tail.every(
    value => Math.abs(value - finalValue) <= tolerance
  );

  let lastOutside = -1;
  for (let i = 0; i < n; i++) {
    if (Math.abs(output[i] - finalValue) > tolerance) {
      lastOutside = i;
    }
  }

  const settle = !tailStable
    ? null
    : lastOutside < 0
      ? 0
      : lastOutside === n - 1
        ? null
        : time[lastOutside + 1];

  // Steady-state error is always referenced to the requested setpoint.
  const finalError =
    100 * Math.abs(target - finalValue) / Math.abs(target);

  return {
    rise,
    overshoot: Math.max(0, 100 * (peak - target) / Math.abs(target)),
    settle,
    finalError,
    finalValue,
    tailStable
  };
}

function displayMetrics(m) {
  $("riseTime").textContent =
    m.rise === null
      ? "Not reached"
      : m.rise.toFixed(3) + " s";

  $("overshoot").textContent =
    m.overshoot.toFixed(2) + "%";

  $("settlingTime").textContent =
    m.settle === null
      ? "Not settled"
      : m.settle.toFixed(3) + " s";

  $("steadyError").textContent =
    m.settle === null
      ? "Unsettled"
      : m.finalError.toFixed(2) + "%";
}

// =====================================
// CANVAS CHARTS
// =====================================

function drawChart(id, time, series) {
  const canvas = $(id);
  const ctx = canvas.getContext("2d");

  const w = Math.max(canvas.clientWidth, 280);
  const h = Math.max(canvas.clientHeight, 220);
  const dpr = window.devicePixelRatio || 1;

  canvas.width = Math.round(w * dpr);
  canvas.height = Math.round(h * dpr);

  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);

  const left = 53;
  const right = 18;
  const top = 43;
  const bottom = 40;

  const plotW = w - left - right;
  const plotH = h - top - bottom;

  let min = Infinity;
  let max = -Infinity;

  for (const s of series) {
    for (const value of s.values) {
      if (value < min) min = value;
      if (value > max) max = value;
    }
  }

  const padding = Math.max(
    0.1,
    (max - min) * 0.09
  );

  min -= padding;
  max += padding;

  const endTime = time[time.length - 1];

  const x = value =>
    left + (value / endTime) * plotW;

  const y = value =>
    top + plotH -
    (value - min) / (max - min) * plotH;

  ctx.font = "11px Segoe UI, Arial";

  // Horizontal grid
  for (let i = 0; i <= 4; i++) {
    const value = min + (max - min) * i / 4;
    const py = y(value);

    ctx.beginPath();
    ctx.strokeStyle = "#30405a";
    ctx.lineWidth = 1;
    ctx.moveTo(left, py);
    ctx.lineTo(w - right, py);
    ctx.stroke();

    ctx.fillStyle = "#a8b8d1";
    ctx.fillText(value.toFixed(2), 4, py + 4);
  }

  // Time labels
  for (let i = 0; i <= 5; i++) {
    const t = endTime * i / 5;

    ctx.fillStyle = "#a8b8d1";
    ctx.fillText(
      t.toFixed(1),
      x(t) - 10,
      h - 15
    );
  }

  ctx.fillText(
    "Time (s)",
    Math.max(left, w / 2 - 20),
    h - 2
  );

  // Curves and legend
  let legendX = left;

  for (const s of series) {
    ctx.strokeStyle = s.color;
    ctx.lineWidth = 2.5;
    ctx.setLineDash(s.dashed ? [6, 5] : []);

    ctx.beginPath();

    const skip = Math.max(
      1,
      Math.floor(time.length / 1600)
    );

    for (let i = 0; i < time.length; i += skip) {
      if (i === 0) {
        ctx.moveTo(x(time[i]), y(s.values[i]));
      } else {
        ctx.lineTo(x(time[i]), y(s.values[i]));
      }
    }

    const last = time.length - 1;
    ctx.lineTo(
      x(time[last]),
      y(s.values[last])
    );

    ctx.stroke();
    ctx.setLineDash([]);

    ctx.beginPath();
    ctx.moveTo(legendX, 15);
    ctx.lineTo(legendX + 13, 15);
    ctx.stroke();

    ctx.fillStyle = "#eef4ff";
    ctx.font = "11px Segoe UI, Arial";
    ctx.fillText(s.name, legendX + 18, 19);

    legendX +=
      ctx.measureText(s.name).width + 40;
  }
}

// =====================================
// GRAPH TIME RANGE
// =====================================

function render() {
  if (!currentRun) return;

  const data = currentRun.data;
  const limit = Number($("timeRange").value);

  const end = Math.min(
    data.time.length,
    Math.floor(limit / DT) + 1
  );

  const time = data.time.slice(0, end);

  drawChart("responseChart", time, [
    {
      name: "Output",
      values: data.output.slice(0, end),
      color: "#729dff"
    },
    {
      name: "Setpoint",
      values: time.map(
        () => currentRun.settings.setpoint
      ),
      color: "#43dbc1",
      dashed: true
    }
  ]);

  drawChart("controlChart", time, [
    {
      name: "u(t)",
      values: data.control.slice(0, end),
      color: "#729dff"
    },
    {
      name: "Upper +20",
      values: time.map(() => UMAX),
      color: "#ff7188",
      dashed: true
    },
    {
      name: "Lower -20",
      values: time.map(() => -UMAX),
      color: "#ffb86b",
      dashed: true
    }
  ]);
}

// =====================================
// RUN SIMULATION
// =====================================

function samePlant(a, b) {
  return (
    a.plant === b.plant &&
    a.K === b.K &&
    a.setpoint === b.setpoint &&
    (
      a.plant === "first"
        ? a.tau === b.tau
        : a.zeta === b.zeta && a.wn === b.wn
    )
  );
}

function run() {
  $("errorMessage").textContent = "";

  try {
    const settings = readSettings();
    validate(settings);

    const data = simulate(settings);
    const resultMetrics = metrics(
      data,
      settings.setpoint
    );

    const before = currentRun;
    const choice = $("prediction").value;

    currentRun = {
      settings,
      data,
      m: resultMetrics
    };

    pendingCheck =
      before &&
      choice &&
      samePlant(before.settings, settings)
        ? {
            before,
            after: currentRun,
            choice
          }
        : null;

    render();
    displayMetrics(resultMetrics);

    if (!before) {
      $("predictionResult").textContent =
        "Baseline ready. Change gains and " +
        "predict before running again.";
    } else if (pendingCheck) {
      $("predictionResult").textContent =
        "New run complete. Click Check prediction " +
        "to see the comparison.";
    } else if (choice) {
      $("predictionResult").textContent =
        "Plant/setpoint changed: prediction not scored. " +
        "Use the same plant and setpoint.";
    } else {
      $("predictionResult").textContent =
        "Run complete. Select a prediction " +
        "before the NEXT run.";
    }

    $("prediction").value = "";

  } catch (error) {
    $("errorMessage").textContent = error.message;
  }
}

// =====================================
// PREDICT THEN CHECK
// =====================================

function formatTime(value) {
  return value === null
    ? "Not reached"
    : value.toFixed(3) + " s";
}

function checkPrediction() {
  const result = $("predictionResult");

  if (!pendingCheck) {
    result.textContent =
      "Choose the prediction before clicking " +
      "Run simulation, then check it.";
    return;
  }

  const { before, after, choice } = pendingCheck;
  const a = before.m;
  const b = after.m;

  const timeTol = 0.001;
  const percentTol = 0.01;

  let correct = null;
  let detail = "";

  switch (choice) {
    case "faster":
      correct =
        a.rise !== null && b.rise !== null
          ? b.rise < a.rise - timeTol
          : null;

      detail =
        `Rise time: ${formatTime(a.rise)} → ` +
        formatTime(b.rise);
      break;

    case "slower":
      correct =
        a.rise !== null && b.rise !== null
          ? b.rise > a.rise + timeTol
          : null;

      detail =
        `Rise time: ${formatTime(a.rise)} → ` +
        formatTime(b.rise);
      break;

    case "overshoot":
      correct =
        b.overshoot > a.overshoot + percentTol;

      detail =
        `Overshoot: ${a.overshoot.toFixed(2)}% → ` +
        `${b.overshoot.toFixed(2)}%`;
      break;

    case "lessOvershoot":
      correct =
        b.overshoot < a.overshoot - percentTol;

      detail =
        `Overshoot: ${a.overshoot.toFixed(2)}% → ` +
        `${b.overshoot.toFixed(2)}%`;
      break;

    case "error":
      correct =
        b.settle === null || a.settle === null
          ? null
          : b.finalError < a.finalError - percentTol;

      detail =
        `Final error: ${a.finalError.toFixed(2)}% → ` +
        `${b.finalError.toFixed(2)}%`;
      break;

    case "same":
      correct =
        (
          (a.rise === null && b.rise === null) ||
          (
            a.rise !== null &&
            b.rise !== null &&
            Math.abs(a.rise - b.rise) <= timeTol
          )
        ) &&
        Math.abs(a.overshoot - b.overshoot) <= percentTol &&
        Math.abs(a.finalError - b.finalError) <= percentTol;

      detail =
        "Compared rise time, overshoot, and final error.";
      break;
  }

  result.textContent =
    (
      correct === null
        ? "Inconclusive: a required metric was not reached or settled."
        : correct
          ? "✓ Prediction supported."
          : "✗ Prediction not supported."
    ) + " " + detail;

  pendingCheck = null;
}

// =====================================
// VALIDATION TESTS
// =====================================

function runValidation() {
  const base = { ...defaults };

  try {
    // Test 1: P only
    const data1 = simulate(base);
    const m1 = metrics(data1, 1);

    const finalY =
      base.K * base.kp /
      (1 + base.K * base.kp);

    const measuredTau = thresholdCross(
      data1.time,
      data1.output,
      finalY * (1 - Math.exp(-1))
    );

    const ok1 =
      measuredTau !== null &&
      Math.abs(measuredTau - 0.1) < 0.005 &&
      Math.abs(m1.finalError - 10) < 0.2;

    // Test 2: PI critical damping
    const pi = {
      ...base,
      kp: 1,
      ki: 1
    };

    const wn = Math.sqrt(
      pi.K * pi.ki / pi.tau
    );

    const zeta =
      (1 + pi.K * pi.kp) /
      (2 * Math.sqrt(pi.tau * pi.K * pi.ki));

    const m2 = metrics(simulate(pi), 1);

    const ok2 =
      Math.abs(wn - 1) < 1e-8 &&
      Math.abs(zeta - 1) < 1e-8 &&
      m2.finalError < 0.5;

    // Test 3: Stable PI example
    const m3 = metrics(
      simulate({
        ...base,
        kp: 2,
        ki: 2
      }),
      1
    );

    const ok3 = m3.finalError < 0.5;

    const cases = [
      [
        ok1,
        "P-only Kp = 9",
        `Measured τ ≈ ${
          measuredTau === null
            ? "N/A"
            : measuredTau.toFixed(3)
        } s; final error ${m1.finalError.toFixed(2)}%. ` +
        "Targets: 0.100 s and 10%."
      ],
      [
        ok2,
        "PI Kp = 1, Ki = 1",
        `ωn = ${wn.toFixed(2)} rad/s; ` +
        `ζ = ${zeta.toFixed(2)}; ` +
        `final error ${m2.finalError.toFixed(2)}%.`
      ],
      [
        ok3,
        "Another stable PI case",
        "Kp = 2, Ki = 2; simulated final error " +
        `${m3.finalError.toFixed(2)}%. ` +
        "This tests one stable example, " +
        "not every possible PI gain."
      ]
    ];

    const container = $("validationResults");
    container.replaceChildren();

    for (const [pass, title, detail] of cases) {
      const row = document.createElement("div");
      row.className = "validation-row";

      const status = document.createElement("strong");
      status.className = pass ? "pass" : "fail";
      status.textContent =
        (pass ? "✓ PASS" : "✗ FAIL") + " — " + title;

      const info = document.createElement("div");
      info.textContent = detail;

      row.append(status, info);
      container.append(row);
    }

  } catch (error) {
    $("validationResults").textContent =
      "Validation error: " + error.message;
  }
}

// =====================================
// RESET DEFAULT VALUES
// =====================================

function reset() {
  for (const [id, value] of Object.entries(defaults)) {
    $(id).value = value;
  }

  $("prediction").value = "";
  $("timeRange").value = "5";

  currentRun = null;
  pendingCheck = null;

  $("validationResults").textContent =
    "Tests have not been run yet.";

  changePlant();
  updateGains();
  run();
}

// =====================================
// EVENTS AND INITIALIZATION
// =====================================

$("plant").addEventListener("change", () => {
  changePlant();
  run();
});

for (const id of ["kp", "ki", "kd"]) {
  $(id).addEventListener("input", updateGains);
}

$("runBtn").addEventListener("click", run);
$("resetBtn").addEventListener("click", reset);
$("checkBtn").addEventListener("click", checkPrediction);
$("validateBtn").addEventListener("click", runValidation);

$("timeRange").addEventListener("change", render);

window.addEventListener("resize", render);

// Run automatically when the page opens
changePlant();
updateGains();
run();