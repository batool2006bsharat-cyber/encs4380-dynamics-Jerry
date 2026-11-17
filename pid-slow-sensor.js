"use strict";

// =====================================================
// ENCS4380 · Homework 2 · Module P2
// A slow sensor in the feedback loop
//
//   Plant  : G(s) = K / (tau*s + 1)
//   Sensor : H(s) = 1 / (tau_s*s + 1)   (in the feedback path)
//   PID    : acts on e = r - y_m  (what the SENSOR reports)
// =====================================================

const $ = id => document.getElementById(id);

const DT = 0.001;
const DURATION = 20;
const UMAX = 20;

// Sensor time constants used in the sweep table and overlay chart
const SWEEP = [0, 0.1, 0.25, 0.5, 0.75, 1, 1.25, 1.5];
const OVERLAY = [0, 0.25, 0.5, 1, 1.5];
const OVERLAY_COLORS = [
  "#729dff", "#43dbc1", "#ffd36b", "#ffb86b", "#ff7188"
];

const STATUS_TEXT = {
  settled: "Settled",
  decaying: "Decaying oscillation",
  oscillating: "Sustained oscillation",
  slow: "Not settled in 20 s"
};

let currentRun = null;
let pendingCheck = null;

const defaults = {
  setpoint: 1,
  K: 1,
  tau: 1,
  taus: 0.1,
  kp: 3,
  ki: 8,
  kd: 0
};

const NUMERIC_IDS = ["setpoint", "K", "tau", "taus", "kp", "ki", "kd"];

// =====================================
// INPUTS AND SETTINGS
// =====================================

function readSettings() {
  const p = {};

  for (const id of NUMERIC_IDS) {
    if ($(id).value.trim() === "") {
      throw Error("Please fill in every numeric field.");
    }
    p[id] = Number($(id).value);
  }

  return p;
}

function validate(p) {
  if (Object.values(p).some(v => !Number.isFinite(v))) {
    throw Error("All values must be finite numbers.");
  }

  if (
    p.K <= 0 ||
    p.setpoint <= 0 ||
    p.tau <= 0 ||
    p.taus < 0 ||
    [p.kp, p.ki, p.kd].some(x => x < 0)
  ) {
    throw Error(
      "K, setpoint and τ must be positive; " +
      "sensor τs and the gains cannot be negative."
    );
  }
}

function updateSliders() {
  for (const id of ["kp", "ki", "kd"]) {
    $(id + "Value").textContent = Number($(id).value).toFixed(2);
  }
  $("tausValue").textContent = Number($("taus").value).toFixed(2) + " s";
}

// =====================================
// CLOSED-LOOP SIMULATION WITH SLOW SENSOR
// =====================================

function simulate(p) {
  const time = [];
  const output = [];    // true plant output y(t)
  const measured = [];  // what the sensor reports y_m(t)
  const control = [];

  let y = 0;
  let ym = 0;
  let integral = 0;
  let oldYm = 0;

  const steps = Math.round(DURATION / DT);

  for (let i = 0; i <= steps; i++) {
    const t = i * DT;

    // The controller only knows the sensor reading
    const error = p.setpoint - ym;

    // Derivative on measurement (the measured signal)
    const rate = i === 0 ? 0 : (ym - oldYm) / DT;

    const pd = p.kp * error - p.kd * rate;

    // Conditional anti-windup (same rule as P1)
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

    const u = Math.min(UMAX, Math.max(-UMAX, pd + p.ki * integral));

    time.push(t);
    output.push(y);
    measured.push(ym);
    control.push(u);

    oldYm = ym;

    // Euler step: plant  tau*dy/dt = K*u - y
    const yOld = y;
    y += DT * (p.K * u - y) / p.tau;

    // Euler step: sensor tau_s*dym/dt = y - ym
    // tau_s = 0 means an ideal (instant) sensor
    ym = p.taus > 0
      ? ym + DT * (yOld - ym) / p.taus
      : y;

    if (!Number.isFinite(y) || !Number.isFinite(ym) || Math.abs(y) > 1e7) {
      throw Error("Simulation diverged for these values. Reduce gains and retry.");
    }
  }

  return { time, output, measured, control };
}

// =====================================
// LINEAR STABILITY ANALYSIS (Routh–Hurwitz)
// =====================================
//
// Characteristic equation with PID and the sensor lag:
//   tau*tau_s s^3 + (tau + tau_s + K Kd) s^2 + (1 + K Kp) s + K Ki = 0
// Stable (3rd order)  <=>  a2*a1 > a3*a0
//   (tau + tau_s + K Kd)(1 + K Kp) > tau*tau_s*K*Ki
// Solving for tau_s gives the critical sensor time constant:
//   tau_s,crit = (tau + K Kd)(1 + K Kp) / (tau K Ki - (1 + K Kp))
// At the boundary the loop oscillates at w^2 = a0 / a2.

function linearAnalysis(p) {
  const a1 = 1 + p.K * p.kp;
  const lead = p.tau + p.K * p.kd;

  let critical = Infinity;

  if (p.ki > 0) {
    const denom = p.tau * p.K * p.ki - a1;
    if (denom > 0) {
      critical = lead * a1 / denom;
    }
  }

  const period = Number.isFinite(critical)
    ? 2 * Math.PI / Math.sqrt(p.K * p.ki / (lead + critical))
    : null;

  return {
    critical,
    stable: p.taus < critical,
    period
  };
}

// =====================================
// PERFORMANCE METRICS
// =====================================

function thresholdCross(t, y, value) {
  for (let i = 1; i < y.length; i++) {
    if (y[i - 1] < value && y[i] >= value) {
      const fraction = (value - y[i - 1]) / (y[i] - y[i - 1]);
      return t[i - 1] + fraction * (t[i] - t[i - 1]);
    }
  }
  return null;
}

function peakOvershoot(values, target) {
  let peak = -Infinity;
  for (const v of values) {
    if (v > peak) peak = v;
  }
  return Math.max(0, 100 * (peak - target) / target);
}

function metrics(data, target) {
  const { time, output, measured } = data;
  const n = output.length;

  let lastOutside = -1;
  for (let i = 0; i < n; i++) {
    if (Math.abs(output[i] - target) > 0.02 * target) {
      lastOutside = i;
    }
  }

  const ten = thresholdCross(time, output, 0.1 * target);
  const ninety = thresholdCross(time, output, 0.9 * target);

  const settle =
    lastOutside === n - 1
      ? null
      : lastOutside < 0 ? 0 : time[lastOutside + 1];

  // Classify an unsettled response by comparing the swing
  // (peak-to-peak) in the 3rd and 4th quarters of the run
  let status = "settled";

  if (settle === null) {
    const swing = (from, to) => {
      let lo = Infinity;
      let hi = -Infinity;
      for (let i = from; i < to; i++) {
        if (output[i] < lo) lo = output[i];
        if (output[i] > hi) hi = output[i];
      }
      return hi - lo;
    };

    const third = swing(Math.floor(n * 0.5), Math.floor(n * 0.75));
    const fourth = swing(Math.floor(n * 0.75), n);

    if (fourth <= 0.04 * target) status = "slow";
    else if (fourth < 0.7 * third) status = "decaying";
    else status = "oscillating";
  }

  return {
    rise: ten === null || ninety === null ? null : ninety - ten,
    overshoot: peakOvershoot(output, target),
    measuredOvershoot: peakOvershoot(measured, target),
    settle,
    finalError: 100 * Math.abs(target - output[n - 1]) / target,
    status
  };
}

function formatTime(value) {
  return value === null ? "Not settled" : value.toFixed(3) + " s";
}

function displayMetrics(m) {
  $("overshootTrue").textContent = m.overshoot.toFixed(2) + "%";
  $("overshootSensor").textContent = m.measuredOvershoot.toFixed(2) + "%";
  $("settlingTime").textContent = formatTime(m.settle);
  $("steadyError").textContent =
    m.settle === null ? "Unsettled" : m.finalError.toFixed(2) + "%";
}

function displayStability(p, lin, m) {
  $("criticalTaus").textContent = Number.isFinite(lin.critical)
    ? lin.critical.toFixed(3) + " s"
    : "None (always stable)";

  $("tausRatio").textContent = Number.isFinite(lin.critical)
    ? (100 * p.taus / lin.critical).toFixed(0) + "% of critical"
    : "—";

  $("oscPeriod").textContent = lin.period === null
    ? "—"
    : lin.period.toFixed(2) + " s";

  const verdict = $("linearVerdict");
  verdict.textContent = lin.stable ? "Stable" : "Unstable → oscillates";
  verdict.className = "badge " + (lin.stable ? "ok" : "bad");

  const sim = $("simVerdict");
  sim.textContent = STATUS_TEXT[m.status];
  sim.className = "badge " + (
    m.status === "settled" ? "ok" : m.status === "oscillating" ? "bad" : "warn"
  );
}

// =====================================
// tau_s SWEEP (move to oscillation)
// =====================================

function sweep(p) {
  return SWEEP.map(taus => {
    const settings = { ...p, taus };
    const data = simulate(settings);
    return {
      taus,
      data,
      m: metrics(data, p.setpoint),
      lin: linearAnalysis(settings)
    };
  });
}

function displaySweep(rows, current) {
  const body = $("sweepBody");
  body.replaceChildren();

  for (const r of rows) {
    const tr = document.createElement("tr");
    if (Math.abs(r.taus - current) < 1e-9) tr.className = "current";

    const cells = [
      r.taus.toFixed(2) + " s",
      r.m.overshoot.toFixed(1) + "%",
      r.m.measuredOvershoot.toFixed(1) + "%",
      formatTime(r.m.settle),
      STATUS_TEXT[r.m.status],
      Math.abs(r.taus - r.lin.critical) < 1e-9
        ? "Marginal (critical)"
        : r.lin.stable ? "Stable" : "Unstable"
    ];

    cells.forEach((text, k) => {
      const td = document.createElement("td");
      td.textContent = text;
      if (k >= 4) {
        const good = k === 4 ? r.m.status === "settled" : r.lin.stable;
        td.className = good ? "pass" : "fail";
      }
      tr.append(td);
    });

    body.append(tr);
  }
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
  const top = 48;
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

  const padding = Math.max(0.1, (max - min) * 0.09);
  min -= padding;
  max += padding;

  const endTime = time[time.length - 1];
  const x = value => left + (value / endTime) * plotW;
  const y = value => top + plotH - (value - min) / (max - min) * plotH;

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
    ctx.fillText(t.toFixed(1), x(t) - 10, h - 15);
  }

  ctx.fillText("Time (s)", Math.max(left, w / 2 - 20), h - 2);

  // Curves and legend (legend wraps on narrow screens)
  let legendX = left;
  let legendY = 15;
  const skip = Math.max(1, Math.floor(time.length / 1600));

  for (const s of series) {
    ctx.strokeStyle = s.color;
    ctx.lineWidth = s.width || 2.5;
    ctx.setLineDash(s.dashed ? [6, 5] : []);

    ctx.beginPath();
    for (let i = 0; i < time.length; i += skip) {
      if (i === 0) ctx.moveTo(x(time[i]), y(s.values[i]));
      else ctx.lineTo(x(time[i]), y(s.values[i]));
    }
    const last = time.length - 1;
    ctx.lineTo(x(time[last]), y(s.values[last]));
    ctx.stroke();
    ctx.setLineDash([]);

    const textW = ctx.measureText(s.name).width;
    if (legendX > left && legendX + 18 + textW > w - right) {
      legendX = left;
      legendY += 16;
    }

    ctx.beginPath();
    ctx.lineWidth = 2.5;
    ctx.moveTo(legendX, legendY);
    ctx.lineTo(legendX + 13, legendY);
    ctx.stroke();

    ctx.fillStyle = "#eef4ff";
    ctx.fillText(s.name, legendX + 18, legendY + 4);

    legendX += textW + 40;
  }
}

// =====================================
// GRAPH TIME RANGE
// =====================================

function render() {
  if (!currentRun) return;

  const { data, settings, sweepRows } = currentRun;
  const limit = Number($("timeRange").value);
  const end = Math.min(data.time.length, Math.floor(limit / DT) + 1);
  const time = data.time.slice(0, end);
  const setpointLine = time.map(() => settings.setpoint);

  drawChart("responseChart", time, [
    {
      name: "True output y(t)",
      values: data.output.slice(0, end),
      color: "#729dff"
    },
    {
      name: "Sensor reading ym(t)",
      values: data.measured.slice(0, end),
      color: "#ffb86b"
    },
    {
      name: "Setpoint",
      values: setpointLine,
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

  const sweepLimit = Number($("sweepRange").value);
  const sweepEnd = Math.min(data.time.length, Math.floor(sweepLimit / DT) + 1);
  const sweepTime = data.time.slice(0, sweepEnd);

  const overlay = OVERLAY.map((taus, k) => {
    const row = sweepRows.find(r => r.taus === taus);
    return {
      name: "τs = " + taus + " s",
      values: row.data.output.slice(0, sweepEnd),
      color: OVERLAY_COLORS[k],
      width: 2
    };
  });

  overlay.push({
    name: "Setpoint",
    values: sweepTime.map(() => settings.setpoint),
    color: "#a8b8d1",
    dashed: true,
    width: 1.5
  });

  drawChart("sweepChart", sweepTime, overlay);
}

// =====================================
// RUN SIMULATION
// =====================================

function samePlant(a, b) {
  return a.K === b.K && a.tau === b.tau && a.setpoint === b.setpoint;
}

function run() {
  $("errorMessage").textContent = "";

  try {
    const settings = readSettings();
    validate(settings);

    const data = simulate(settings);
    const m = metrics(data, settings.setpoint);
    const lin = linearAnalysis(settings);
    const sweepRows = sweep(settings);

    const before = currentRun;
    const choice = $("prediction").value;

    currentRun = { settings, data, m, lin, sweepRows };

    pendingCheck =
      before && choice && samePlant(before.settings, settings)
        ? { before, after: currentRun, choice }
        : null;

    render();
    displayMetrics(m);
    displayStability(settings, lin, m);
    displaySweep(sweepRows, settings.taus);

    if (!before) {
      $("predictionResult").textContent =
        "Baseline ready. Change τs or the gains and " +
        "predict before running again.";
    } else if (pendingCheck) {
      $("predictionResult").textContent =
        "New run complete. Click Check prediction to see the comparison.";
    } else if (choice) {
      $("predictionResult").textContent =
        "Plant/setpoint changed: prediction not scored. " +
        "Keep K, τ and the setpoint the same.";
    } else {
      $("predictionResult").textContent =
        "Run complete. Select a prediction before the NEXT run.";
    }

    $("prediction").value = "";

  } catch (error) {
    $("errorMessage").textContent = error.message;
  }
}

// =====================================
// PREDICT THEN CHECK
// =====================================

function checkPrediction() {
  const result = $("predictionResult");

  if (!pendingCheck) {
    result.textContent =
      "Choose the prediction before clicking Run simulation, then check it.";
    return;
  }

  const { before, after, choice } = pendingCheck;
  const a = before.m;
  const b = after.m;
  const tol = 0.01;

  let correct = null;
  let detail = "";

  const overshootDetail =
    `Overshoot of the true output: ${a.overshoot.toFixed(2)}% → ` +
    `${b.overshoot.toFixed(2)}%`;

  const settleDetail =
    `Settling time: ${formatTime(a.settle)} → ${formatTime(b.settle)}`;

  switch (choice) {
    case "moreOvershoot":
      correct = b.overshoot > a.overshoot + tol;
      detail = overshootDetail;
      break;

    case "lessOvershoot":
      correct = b.overshoot < a.overshoot - tol;
      detail = overshootDetail;
      break;

    case "slowerSettle":
      if (a.settle === null && b.settle === null) correct = null;
      else if (b.settle === null) correct = true;
      else if (a.settle === null) correct = false;
      else correct = b.settle > a.settle + 0.001;
      detail = settleDetail;
      break;

    case "fasterSettle":
      if (a.settle === null && b.settle === null) correct = null;
      else if (a.settle === null) correct = true;
      else if (b.settle === null) correct = false;
      else correct = b.settle < a.settle - 0.001;
      detail = settleDetail;
      break;

    case "oscillate":
      correct = b.status === "oscillating";
      detail = `Simulated behaviour: ${STATUS_TEXT[a.status]} → ` +
        STATUS_TEXT[b.status];
      break;

    case "same":
      correct =
        Math.abs(a.overshoot - b.overshoot) <= tol &&
        a.status === b.status &&
        (
          (a.settle === null && b.settle === null) ||
          (a.settle !== null && b.settle !== null &&
            Math.abs(a.settle - b.settle) <= 0.001)
        );
      detail = "Compared overshoot, settling time and behaviour.";
      break;
  }

  const sensorNote =
    `\nSensor τs: ${before.settings.taus.toFixed(2)} s → ` +
    `${after.settings.taus.toFixed(2)} s ` +
    `(critical ≈ ${Number.isFinite(after.lin.critical)
      ? after.lin.critical.toFixed(3) + " s"
      : "none"}).`;

  result.textContent =
    (
      correct === null
        ? "Inconclusive: neither run settled, so settling time cannot be compared."
        : correct ? "✓ Prediction supported." : "✗ Prediction not supported."
    ) + " " + detail + sensorNote;

  pendingCheck = null;
}

// =====================================
// VALIDATION TESTS
// =====================================

function runValidation() {
  const base = { ...defaults };

  try {
    // Test 1: an ideal sensor (tau_s = 0) reproduces P1
    const p1 = { ...base, taus: 0, kp: 9, ki: 0, kd: 0 };
    const d1 = simulate(p1);
    const m1 = metrics(d1, 1);
    const tauCl = thresholdCross(d1.time, d1.output, 0.9 * (1 - Math.exp(-1)));
    const ok1 =
      tauCl !== null &&
      Math.abs(tauCl - 0.1) < 0.005 &&
      Math.abs(m1.finalError - 10) < 0.2;

    // Test 2: the sensor has DC gain 1, so a slow sensor does not change
    // the steady state of P-only control (still 10% error)
    const d2 = simulate({ ...p1, taus: 0.5 });
    const n2 = d2.output.length - 1;
    const yEnd = d2.output[n2];
    const ymEnd = d2.measured[n2];
    const ok2 =
      Math.abs(yEnd - ymEnd) < 1e-3 &&
      Math.abs(yEnd - 0.9) < 2e-3;

    // Test 3: overshoot grows as tau_s grows (default PI gains)
    const os = [0, 0.25, 0.5].map(taus =>
      metrics(simulate({ ...base, taus }), 1).overshoot
    );
    const ok3 = os[0] < os[1] && os[1] < os[2];

    // Test 4: Routh prediction matches the simulation
    const crit = linearAnalysis(base).critical;
    const below = metrics(simulate({ ...base, taus: 0.7 * crit }), 1);
    const above = metrics(simulate({ ...base, taus: 1.3 * crit }), 1);
    const ok4 =
      below.status !== "oscillating" &&
      above.status === "oscillating";

    const cases = [
      [
        ok1,
        "Ideal sensor (τs = 0) reproduces P1",
        `P-only Kp = 9: measured τ ≈ ${tauCl === null ? "N/A" : tauCl.toFixed(3)} s, ` +
        `final error ${m1.finalError.toFixed(2)}%. Targets: 0.100 s and 10%.`
      ],
      [
        ok2,
        "Sensor lag does not change the steady state",
        `Kp = 9, τs = 0.5 s: final y = ${yEnd.toFixed(4)}, ` +
        `final ym = ${ymEnd.toFixed(4)}. Target: both 0.9000.`
      ],
      [
        ok3,
        "Overshoot grows with τs",
        `Kp = 3, Ki = 8: τs = 0, 0.25, 0.5 s gives ` +
        os.map(v => v.toFixed(1) + "%").join(", ") + "."
      ],
      [
        ok4,
        "Routh critical τs matches the simulation",
        `Critical τs = ${crit.toFixed(3)} s. ` +
        `At 0.7×: ${STATUS_TEXT[below.status]}. ` +
        `At 1.3×: ${STATUS_TEXT[above.status]}.`
      ]
    ];

    const container = $("validationResults");
    container.replaceChildren();

    for (const [pass, title, detail] of cases) {
      const row = document.createElement("div");
      row.className = "validation-row";

      const status = document.createElement("strong");
      status.className = pass ? "pass" : "fail";
      status.textContent = (pass ? "✓ PASS" : "✗ FAIL") + " — " + title;

      const info = document.createElement("div");
      info.textContent = detail;

      row.append(status, info);
      container.append(row);
    }

  } catch (error) {
    $("validationResults").textContent = "Validation error: " + error.message;
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
  $("sweepRange").value = "10";

  currentRun = null;
  pendingCheck = null;

  $("validationResults").textContent = "Tests have not been run yet.";

  updateSliders();
  run();
}

// =====================================
// EVENTS AND INITIALIZATION
// =====================================

for (const id of ["kp", "ki", "kd", "taus"]) {
  $(id).addEventListener("input", updateSliders);
}

$("runBtn").addEventListener("click", run);
$("resetBtn").addEventListener("click", reset);
$("checkBtn").addEventListener("click", checkPrediction);
$("validateBtn").addEventListener("click", runValidation);

$("timeRange").addEventListener("change", render);
$("sweepRange").addEventListener("change", render);

window.addEventListener("resize", render);

// Run automatically when the page opens
updateSliders();
run();
