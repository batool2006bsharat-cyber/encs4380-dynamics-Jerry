"use strict";

/* 
   Colors and plot style (same palette as the rest of the toolkit)
   */

const ROSE = "#e2548f";
const LILAC = "#8f6fdc";
const MINT = "#4fb89a";
const ORANGE = "#e08a4f";
const GREY = "#b9a3b3";
const RED = "#b3263f";

function layoutFor(title, xTitle, yTitle) {
    return {
        title: { text: title, font: { size: 14 } },
        xaxis: { title: { text: xTitle }, gridcolor: "#f5dce8", zerolinecolor: "#e9c9da" },
        yaxis: { title: { text: yTitle }, gridcolor: "#f5dce8", zerolinecolor: "#e9c9da" },
        paper_bgcolor: "rgba(0,0,0,0)",
        plot_bgcolor: "#fffafc",
        font: { color: "#4a2740", family: "Segoe UI, Arial, sans-serif", size: 12 },
        legend: { orientation: "h", y: -0.28 },
        margin: { t: 40, r: 20, b: 70, l: 58 },
        hovermode: "closest"
    };
}
const plotConfig = { responsive: true, displayModeBar: false };

const $ = id => document.getElementById(id);


/*
   State
   */

const state = {
    plant: "first", K: 1, tau: 1, zeta: 0.5, wn: 2,
    Kp: 5, Ki: 5, Kd: 0.2,
    Ts: 0.02, uMax: 8, r: 1,
    aw: true, showCont: true, showParts: false,
    cpr: 360, stepPct: 50
};

/* The Micromouse mode has its own units (rpm and PWM %), so each mode keeps its own values.
   Starting values for the motor: a typical small Micromouse motor, until you measure yours. */
const MODE_KEYS = ["K", "tau", "Kp", "Ki", "Kd", "uMax", "r", "Ts"];
const buckets = {
    generic: null,
    motor: { K: 5, tau: 0.1, Kp: 0.2, Ki: 4.1, Kd: 0, uMax: 100, r: 300, Ts: 0.01 }
};
function modeOf(plant) { return plant === "motor" ? "motor" : "generic"; }
function isMotor() { return state.plant === "motor"; }
function pickModeValues() {
    const o = {};
    MODE_KEYS.forEach(k => { o[k] = state[k]; });
    return o;
}
function setPlant(newPlant) {
    const from = modeOf(state.plant), to = modeOf(newPlant);
    if (from !== to) {
        buckets[from] = pickModeValues();
        if (buckets[to]) Object.assign(state, buckets[to]);
    }
    state.plant = newPlant;
}

const U_MIN = 0;     // the actuator cannot go below 0 (like a PWM duty cycle of 0 %)


/* 
   Plant model, stepped in time with RK4
   First order:  τ y' + y = K u   (also the Micromouse motor: y in rpm, u in % PWM)
   Second order: y'' + 2ζωn y' + ωn² y = K ωn² u
   x = [y, y']
    */

function plantDeriv(p, x, u) {
    if (p.plant !== "second") {
        return [(p.K * u - x[0]) / p.tau, 0];
    }
    return [x[1], p.wn * p.wn * (p.K * u - x[0]) - 2 * p.zeta * p.wn * x[1]];
}

function rk4Step(p, x, u, h) {
    const k1 = plantDeriv(p, x, u);
    const k2 = plantDeriv(p, [x[0] + h / 2 * k1[0], x[1] + h / 2 * k1[1]], u);
    const k3 = plantDeriv(p, [x[0] + h / 2 * k2[0], x[1] + h / 2 * k2[1]], u);
    const k4 = plantDeriv(p, [x[0] + h * k3[0], x[1] + h * k3[1]], u);
    return [
        x[0] + h / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]),
        x[1] + h / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])
    ];
}

/* Fastest time scale of the plant, used to pick the RK4 step */
function plantTimeScale(p) {
    return p.plant !== "second" ? p.tau : 1 / p.wn;
}

/* How long to simulate: long enough for the slowest plant to settle */
function simTime(p) {
    if (p.plant === "motor") return Math.min(10, Math.max(1, 10 * p.tau));
    const slow = p.plant === "first" ? p.tau : 1 / (Math.min(p.zeta, 1) * p.wn);
    return Math.min(40, Math.max(5, 8 * slow));
}


/*
   The digital PID loop
   Every Ts the controller samples y, computes u[k] with the
   homework formula, clamps it to [U_MIN, uMax] and holds it.
   Between samples the plant is stepped with RK4 (several small
   steps per sample). This is exactly the code shown below.
   */

function simulate(p, Ts, tEnd) {
    const substeps = Math.max(1, Math.ceil(Ts / (plantTimeScale(p) / 50)));
    const h = Ts / substeps;
    const samples = Math.ceil(tEnd / Ts);

    let x = [0, 0];
    let sumE = 0;
    let prevE = 0;
    let firstRun = true;

    const out = { t: [0], y: [0], tk: [], yk: [], uk: [], pk: [], ik: [], dk: [], atLimit: 0 };

    for (let k = 0; k < samples; k++) {

        const y = x[0];                      // read the sensor once
        const e = p.r - y;

        if (firstRun) { prevE = e; firstRun = false; }   // no derivative spike on the first sample
        const dE = (e - prevE) / Ts;

        if (p.aw) {
            // Anti-windup (conditional integration): only add e to the sum
            // if that would not push u further past a limit it is already at.
            const uTry = p.Kp * e + p.Ki * Ts * (sumE + e) + p.Kd * dE;
            const stuckHigh = uTry > p.uMax && e > 0;
            const stuckLow = uTry < U_MIN && e < 0;
            if (!stuckHigh && !stuckLow) sumE += e;
        } else {
            sumE += e;
        }

        const P = p.Kp * e;
        const I = p.Ki * Ts * sumE;
        const D = p.Kd * dE;
        let u = P + I + D;
        if (u >= p.uMax || u <= U_MIN) out.atLimit += Ts;
        u = Math.min(p.uMax, Math.max(U_MIN, u));

        prevE = e;

        out.tk.push(k * Ts); out.yk.push(y); out.uk.push(u);
        out.pk.push(P); out.ik.push(I); out.dk.push(D);

        // hold u for one sample period while the plant moves on
        for (let i = 0; i < substeps; i++) {
            x = rk4Step(p, x, u, h);
            out.t.push(k * Ts + (i + 1) * h);
            out.y.push(x[0]);
        }
        if (!isFinite(x[0]) || Math.abs(x[0]) > 1e6) break;
    }
    return out;
}

/* Metrics read off the simulated curve */
function readMetrics(t, y, r) {
    const n = y.length;
    const tail = y.slice(Math.floor(n * 0.9));
    const yf = tail.reduce((a, b) => a + b, 0) / tail.length;
    const spread = Math.max(...tail) - Math.min(...tail);
    const yMax = Math.max(...y);
    const settled = isFinite(yMax) && Math.abs(yMax) < 1e5 && spread < 0.02 * Math.abs(r);

    const cross = level => {
        for (let i = 1; i < n; i++) {
            if (y[i] >= level) return t[i - 1] + (level - y[i - 1]) / (y[i] - y[i - 1]) * (t[i] - t[i - 1]);
        }
        return null;
    };

    if (!settled) return { settled: false, yf: NaN, tr: null, os: null, ts: null, sse: null };

    const t10 = cross(0.1 * yf), t90 = cross(0.9 * yf);
    const os = Math.max(0, (yMax - yf) / yf * 100);
    let last = -1;
    for (let i = 0; i < n; i++) if (Math.abs(y[i] - yf) > 0.02 * Math.abs(yf)) last = i;
    const ts = last < 0 ? 0 : (last + 1 < n ? t[last + 1] : null);
    return { settled: true, yf, tr: (t10 !== null && t90 !== null) ? t90 - t10 : null, os, ts, sse: r - yf };
}


/* 
   Sampling-time slider: logarithmic, snapped to whole milliseconds
   so the simulation and the Arduino code use exactly the same Ts.
   */

const TS_MIN = 0.001, TS_MAX = 1;
function sliderToTs(v) {
    const ts = TS_MIN * Math.pow(TS_MAX / TS_MIN, v / 1000);
    return Math.max(1, Math.round(ts * 1000)) / 1000;
}
function tsToSlider(ts) {
    return Math.round(1000 * Math.log(ts / TS_MIN) / Math.log(TS_MAX / TS_MIN));
}


/* 
   Controls
    */

const sliders = [
    ["sK", "K", "oK", v => isMotor() ? v.toFixed(1) + " rpm/%" : v.toFixed(1)],
    ["sTau", "tau", "oTau", v => isMotor() ? v.toFixed(3) + " s" : v.toFixed(1) + " s"],
    ["sZeta", "zeta", "oZeta", v => v.toFixed(2)],
    ["sWn", "wn", "oWn", v => v.toFixed(1) + " rad/s"],
    ["sKp", "Kp", "oKp", v => isMotor() ? v.toFixed(2) : v.toFixed(1)],
    ["sKi", "Ki", "oKi", v => isMotor() ? v.toFixed(2) : v.toFixed(1)],
    ["sKd", "Kd", "oKd", v => isMotor() ? v.toFixed(3) : v.toFixed(2)],
    ["sUmax", "uMax", "oUmax", v => isMotor() ? v.toFixed(0) + " %" : v.toFixed(1)],
    ["sR", "r", "oR", v => isMotor() ? v.toFixed(0) + " rpm" : v.toFixed(1)]
];

/* Slider ranges [min, max, step] for each mode */
const RANGES = {
    generic: { sK: [0.2, 5, 0.1], sTau: [0.1, 5, 0.1], sKp: [0, 20, 0.1], sKi: [0, 20, 0.1], sKd: [0, 2, 0.01],
               sUmax: [0.5, 10, 0.1], sR: [0.2, 3, 0.1] },
    motor:   { sK: [0.5, 20, 0.1], sTau: [0.01, 1, 0.005], sKp: [0, 2, 0.01], sKi: [0, 20, 0.05], sKd: [0, 0.05, 0.001],
               sUmax: [10, 100, 1], sR: [50, 1000, 10] }
};
const KEY_OF = { sK: "K", sTau: "tau", sKp: "Kp", sKi: "Ki", sKd: "Kd", sUmax: "uMax", sR: "r" };

function applyRanges() {
    const R = RANGES[modeOf(state.plant)];
    for (const id in R) {
        const [lo, hi, step] = R[id];
        const v = state[KEY_OF[id]];
        $(id).min = Math.min(lo, v);            // widen the slider if a measured value falls outside it
        $(id).max = Math.max(hi, v);
        $(id).step = step;
    }
    const m = isMotor();
    $("lblK").textContent = m ? "Motor gain K (rpm per 1 % PWM)" : "Plant gain K";
    $("lblTau").textContent = m ? "Motor time constant τ" : "Time constant τ";
    $("lblUmax").innerHTML = m ? "PWM limit" : "Control limit u<sub>max</sub>";
    $("lblR").textContent = m ? "Speed setpoint r" : "Setpoint step r";
}

function syncControls() {
    applyRanges();
    sliders.forEach(([sid, key, oid, f]) => { $(sid).value = state[key]; $(oid).textContent = f(state[key]); });
    $("sTs").value = tsToSlider(state.Ts);
    $("oTs").textContent = Math.round(state.Ts * 1000) + " ms";
    $("plantFirst").setAttribute("aria-pressed", state.plant === "first");
    $("plantSecond").setAttribute("aria-pressed", state.plant === "second");
    $("plantMotor").setAttribute("aria-pressed", state.plant === "motor");
    $("motorNote").hidden = state.plant !== "motor";
    $("rowTau").hidden = state.plant === "second";
    $("rowZeta").hidden = state.plant !== "second";
    $("rowWn").hidden = state.plant !== "second";
    $("cAw").checked = state.aw;
    $("cCont").checked = state.showCont;
    $("cParts").checked = state.showParts;
}

sliders.forEach(([sid, key]) => {
    $(sid).addEventListener("input", e => {
        stopExperiment();
        state[key] = parseFloat(e.target.value);
        update();
    });
});
$("sTs").addEventListener("input", e => {
    stopExperiment();
    state.Ts = sliderToTs(parseFloat(e.target.value));
    update();
});
$("plantFirst").addEventListener("click", () => { stopExperiment(); setPlant("first"); update(); });
$("plantSecond").addEventListener("click", () => { stopExperiment(); setPlant("second"); update(); });
$("plantMotor").addEventListener("click", () => { stopExperiment(); setPlant("motor"); update(); });
$("cAw").addEventListener("change", e => { stopExperiment(); state.aw = e.target.checked; update(); });
$("cCont").addEventListener("change", e => { state.showCont = e.target.checked; update(); });
$("cParts").addEventListener("change", e => { state.showParts = e.target.checked; update(); });


/* 
   Drawing
   */

const CONT_TS = 0.001;     // "continuous" reference: the same loop sampled every 1 ms

function fmtS(v) { return v === null || v === undefined ? "—" : v.toFixed(3) + " s"; }
function fmtPct(v) { return v === null || v === undefined ? "—" : v.toFixed(1) + " %"; }

function update(skipSweep) {
    syncControls();

    const p = state;
    const tEnd = simTime(p);
    const dig = simulate(p, p.Ts, tEnd);
    const cont = simulate(p, CONT_TS, tEnd);
    const mD = readMetrics(dig.t, dig.y, p.r);
    const mC = readMetrics(cont.t, cont.y, p.r);

    drawOutput(p, dig, cont, tEnd);
    drawControl(p, dig, cont, tEnd);
    drawMetrics(p, mD, mC, dig, cont);
    drawCode(p);
    if (!skipSweep) scheduleSweep();
}

function drawOutput(p, dig, cont, tEnd) {
    const data = [
        { x: [0, tEnd], y: [p.r, p.r], mode: "lines", name: "Setpoint r",
          line: { color: LILAC, width: 2, dash: "dash" }, hoverinfo: "skip" }
    ];
    if (p.showCont) {
        data.push({ x: cont.t, y: cont.y, mode: "lines", name: "Continuous PID",
                    line: { color: GREY, width: 2, dash: "dot" } });
    }
    data.push({ x: dig.t, y: dig.y, mode: "lines", name: "Plant output y(t)",
                line: { color: ROSE, width: 3 },
                hovertemplate: "t = %{x:.3f} s<br>y = %{y:.4f}<extra></extra>" });
    if (dig.tk.length <= 160) {
        data.push({ x: dig.tk, y: dig.yk, mode: "markers", name: "Samples y[k] the controller sees",
                    marker: { color: MINT, size: 7 },
                    hovertemplate: "sample at t = %{x:.3f} s<br>y[k] = %{y:.4f}<extra></extra>" });
    }
    const L = isMotor() ? layoutFor("Wheel speed and setpoint", "time t (s)", "speed (rpm)")
                        : layoutFor("Output and setpoint", "time t (s)", "output y");
    L.xaxis.range = [0, tEnd];
    const yAll = dig.y.filter(isFinite);
    const top = Math.max(p.r * 1.25, Math.min(Math.max(...yAll) * 1.05, p.r * 4));
    const bottom = Math.min(0, Math.max(Math.min(...yAll) * 1.05, -p.r * 2));
    L.yaxis.range = [bottom, top];
    Plotly.react("plotY", data, L, plotConfig);
}

function drawControl(p, dig, cont, tEnd) {
    const data = [];
    if (p.showCont) {
        data.push({ x: cont.tk, y: cont.uk, mode: "lines", name: "Continuous u(t)",
                    line: { color: GREY, width: 2, dash: "dot" } });
    }
    // stair step: each value is held until the next sample ("hv" shape)
    const tk = dig.tk.concat([dig.tk.length * p.Ts]);
    const uk = dig.uk.concat([dig.uk[dig.uk.length - 1]]);
    data.push({ x: tk, y: uk, mode: "lines", name: "u[k] sent to the actuator", line: { color: ROSE, width: 3, shape: "hv" },
                hovertemplate: "t = %{x:.3f} s<br>u = %{y:.4f}<extra></extra>" });
    if (p.showParts) {
        data.push({ x: dig.tk, y: dig.pk, mode: "lines", name: "P part", line: { color: LILAC, width: 1.5, shape: "hv" } });
        data.push({ x: dig.tk, y: dig.ik, mode: "lines", name: "I part", line: { color: MINT, width: 1.5, shape: "hv" } });
        data.push({ x: dig.tk, y: dig.dk, mode: "lines", name: "D part", line: { color: ORANGE, width: 1.5, shape: "hv" } });
    }
    data.push({ x: [0, tEnd], y: [p.uMax, p.uMax], mode: "lines", name: "Limit u_max",
                line: { color: ORANGE, width: 2, dash: "dash" }, hoverinfo: "skip" });

    const L = isMotor() ? layoutFor("PWM command u[k]: one value per sample, held in between", "time t (s)", "PWM (%)")
                        : layoutFor("Control signal u[k]: one value per sample, held in between", "time t (s)", "control u");
    L.xaxis.range = [0, tEnd];
    const lo = p.showParts ? Math.max(-p.uMax * 3, Math.min(0, ...dig.pk, ...dig.ik, ...dig.dk)) : -0.05 * p.uMax;
    const hi = p.showParts ? Math.min(p.uMax * 3, Math.max(p.uMax, ...dig.pk, ...dig.ik, ...dig.dk)) * 1.1 : p.uMax * 1.15;
    L.yaxis.range = [lo, hi];
    Plotly.react("plotU", data, L, plotConfig);
}

function drawMetrics(p, mD, mC, dig, cont) {
    $("hDig").textContent = "Digital (Ts = " + Math.round(p.Ts * 1000) + " ms)";
    const cell = (m, f) => m.settled ? f(m) : "does not settle";
    const rows = [
        ["Rise time (10% to 90%)", m => fmtS(m.tr)],
        ["Overshoot", m => fmtPct(m.os)],
        ["Settling time (2% band)", m => fmtS(m.ts)],
        ["Steady-state error r − y(∞)", m => (m.sse === null ? "—" : (Math.abs(m.sse) < 0.002 * Math.abs(p.r) ? "≈ 0" : m.sse.toFixed(3)))]
    ];
    let html = rows.map(([name, f]) =>
        "<tr><td>" + name + "</td><td>" + cell(mD, f) + "</td><td>" + cell(mC, f) + "</td></tr>").join("");
    html += "<tr><td>Time with u stuck at a limit</td><td>" + dig.atLimit.toFixed(2) + " s</td><td>" +
            cont.atLimit.toFixed(2) + " s</td></tr>";
    $("metricBody").innerHTML = html;

    // How many samples fit into the continuous loop's rise time? Rule of thumb: at least 10.
    const v = $("verdict");
    if (!mD.settled) {
        v.className = "verdict bad";
        v.textContent = "At Ts = " + Math.round(p.Ts * 1000) + " ms the loop never settles: the controller reacts too late to what the plant is doing.";
    } else if (mC.settled && mC.tr) {
        const perRise = mC.tr / p.Ts;
        const extra = mD.os - mC.os;
        if (perRise >= 10) {
            v.className = "verdict good";
            v.textContent = perRise.toFixed(0) + " samples fit in the rise time (" + mC.tr.toFixed(2) +
                " s). That is at least 10, so the digital loop behaves almost like the continuous one.";
        } else {
            v.className = "verdict " + (extra > 5 ? "bad" : "meh");
            v.textContent = "Only " + perRise.toFixed(1) + " samples fit in the rise time (" + mC.tr.toFixed(2) +
                " s). Rule of thumb: keep at least 10. Overshoot is " + mD.os.toFixed(1) + " % here against " +
                mC.os.toFixed(1) + " % for the continuous loop.";
        }
    } else {
        v.className = "verdict meh";
        v.textContent = "The continuous loop itself does not settle with these gains, so try smaller gains first.";
    }

    // Can the setpoint be reached at all?
    const w = $("reachWarn");
    const uNeeded = p.r / p.K;
    if (uNeeded > p.uMax) {
        w.hidden = false;
        w.textContent = isMotor()
            ? "Holding " + p.r.toFixed(0) + " rpm needs PWM = r/K = " + uNeeded.toFixed(0) + " %, above the limit " +
              p.uMax.toFixed(0) + " %. The wheel can never reach that speed."
            : "Holding y = " + p.r.toFixed(1) + " needs u = r/K = " + uNeeded.toFixed(2) +
              ", above the limit " + p.uMax.toFixed(1) + ". The output can never reach the setpoint.";
    } else {
        w.hidden = true;
    }

    // Design check with the first-order formulas from the lecture (ignores Kd and sampling)
    const d = $("designNote");
    if (p.plant === "second") { d.hidden = true; return; }
    d.hidden = false;
    let text;
    if (p.Ki > 0) {
        const zeta = (1 + p.K * p.Kp) / (2 * Math.sqrt(p.tau * p.K * p.Ki));
        const wn = Math.sqrt(p.K * p.Ki / p.tau);
        text = "PI design check (lecture formulas, ignoring Kd and sampling): ζ = " + zeta.toFixed(2) +
               ", ωn = " + wn.toFixed(1) + " rad/s. ";
        if (isMotor()) {
            // the homework's target for the Micromouse speed loop
            const inRange = zeta >= 0.6 && zeta <= 0.8;
            d.className = "verdict " + (inRange ? "good" : "meh");
            text += inRange ? "Inside the target range 0.6 ≤ ζ ≤ 0.8." : "The target for the Micromouse speed loop is 0.6 ≤ ζ ≤ 0.8.";
        } else {
            d.className = "verdict good";
            text += zeta < 1 ? "ζ < 1: the closed-loop poles are complex, so expect some ringing."
                             : "ζ ≥ 1: the closed-loop poles are real, so the loop does not ring (the PI zero can still add a little overshoot).";
        }
    } else {
        d.className = "verdict meh";
        text = "P only: closed-loop τ = τ/(1 + KKp) = " + (p.tau / (1 + p.K * p.Kp)).toFixed(3) +
               " s and steady-state error 1/(1 + KKp) = " + (100 / (1 + p.K * p.Kp)).toFixed(1) + " %. Add Ki to remove the error.";
    }
    if (isMotor()) {
        const peak = Math.max(...dig.uk);
        text += " Peak PWM: " + peak.toFixed(0) + " % " + (peak < p.uMax - 1e-9 ? "(stays inside 0–100 %)." : "(hits the limit).");
    }
    d.textContent = text;
}


/* =========================================================
   Performance against Ts (computed a moment after the sliders stop)
   ========================================================= */

let sweepTimer = null;
function scheduleSweep() {
    clearTimeout(sweepTimer);
    sweepTimer = setTimeout(drawSweep, 120);
}

function drawSweep() {
    const p = state;
    const tEnd = simTime(p);
    const xs = [], os = [], ts = [], badX = [], badY = [];
    for (let i = 0; i < 40; i++) {
        const Ts = Math.round(1000 * 0.002 * Math.pow(500, i / 39)) / 1000;
        if (xs.length && Ts === xs[xs.length - 1]) continue;
        const s = simulate(p, Ts, tEnd);
        const m = readMetrics(s.t, s.y, p.r);
        xs.push(Ts);
        if (m.settled) { os.push(m.os); ts.push(m.ts); }
        else { os.push(null); ts.push(null); badX.push(Ts); }
    }
    const osMax = Math.max(20, ...os.filter(v => v !== null)) * 1.15;
    badY.push(...badX.map(() => osMax * 0.95));

    const data = [
        { x: xs, y: os, mode: "lines+markers", name: "Overshoot (%)", line: { color: ROSE, width: 3 },
          marker: { size: 6 }, hovertemplate: "Ts = %{x:.3f} s<br>overshoot = %{y:.1f} %<extra></extra>" },
        { x: xs, y: ts, mode: "lines+markers", name: "Settling time (s)", yaxis: "y2",
          line: { color: LILAC, width: 2.5 }, marker: { size: 6 },
          hovertemplate: "Ts = %{x:.3f} s<br>settling = %{y:.2f} s<extra></extra>" },
        { x: badX, y: badY, mode: "markers", name: "Never settles", marker: { color: RED, size: 11, symbol: "x" },
          hovertemplate: "Ts = %{x:.3f} s<br>never settles<extra></extra>" }
    ];
    const L = layoutFor("Overshoot and settling time against Tₛ", "sampling time Ts (log scale)", "overshoot (%)");
    L.xaxis.type = "log";
    L.xaxis.tickvals = [0.002, 0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1];
    L.xaxis.ticktext = ["2 ms", "5 ms", "10 ms", "20 ms", "50 ms", "100 ms", "200 ms", "500 ms", "1 s"];
    L.yaxis.range = [0, osMax];
    L.yaxis2 = { title: { text: "settling time (s)" }, overlaying: "y", side: "right", rangemode: "tozero",
                 showgrid: false, zeroline: false };
    L.margin.r = 60;
    L.shapes = [{ type: "line", xref: "x", yref: "paper", x0: p.Ts, x1: p.Ts, y0: 0, y1: 1,
                  line: { color: ORANGE, width: 2, dash: "dash" } }];
    Plotly.react("plotSweep", data, L, plotConfig);
}


/* =========================================================
   Arduino code for the current gains
   ========================================================= */

function arduinoCode(p) {
    const ms = Math.round(p.Ts * 1000);
    const f = v => v.toFixed(4);
    const lines = isMotor() ? motorHeader(p, ms, f) : [
        "// Digital PID generated by the ENCS4380 PID Control Toolkit (Module P3)",
        "// u[k] = Kp*e[k] + Ki*Ts*sum(e[0..k]) + Kd*(e[k] - e[k-1])/Ts",
        "",
        "const float Kp = " + f(p.Kp) + ";",
        "const float Ki = " + f(p.Ki) + ";",
        "const float Kd = " + f(p.Kd) + ";",
        "",
        "const unsigned long TS_MS = " + ms + ";      // sampling time in milliseconds",
        "const float Ts = TS_MS / 1000.0;            // the same, in seconds",
        "",
        "const float U_MIN = 0.0;                    // actuator limits (same as the plot)",
        "const float U_MAX = " + f(p.uMax) + ";",
        "",
        "const int SENSOR_PIN = A0;",
        "const int PWM_PIN = 9;",
        "",
        "float setpoint = " + f(p.r) + ";",
        "float sumE = 0;                             // running sum of the error",
        "float prevE = 0;                            // error at the previous sample",
        "bool firstRun = true;",
        "unsigned long lastTime = 0;",
        "",
        "// Convert the ADC reading to the same units as the setpoint.",
        "// Replace the scale factor with your own sensor's calibration.",
        "float readOutput() {",
        "  return analogRead(SENSOR_PIN) * (5.0 / 1023.0);",
        "}",
        "",
        "// Map u from [U_MIN, U_MAX] to a PWM duty cycle of 0..255.",
        "void writeControl(float u) {",
        "  int pwm = (int)(255.0 * (u - U_MIN) / (U_MAX - U_MIN));",
        "  analogWrite(PWM_PIN, constrain(pwm, 0, 255));",
        "}",
        "",
        "void setup() {",
        "  pinMode(PWM_PIN, OUTPUT);",
        "  lastTime = millis();",
        "}",
        ""
    ];
    lines.push(
        "void loop() {",
        "  if (millis() - lastTime < TS_MS) return;   // wait for the next sample",
        "  lastTime += TS_MS;",
        "",
        "  float y = readOutput();                   // sample the sensor once",
        "  float e = setpoint - y;",
        "",
        "  if (firstRun) {                           // avoid a derivative spike on the first sample",
        "    prevE = e;",
        "    firstRun = false;",
        "  }",
        "  float dE = (e - prevE) / Ts;",
        ""
    );
    if (p.aw) {
        lines.push(
            "  // Anti-windup (conditional integration): add e to the sum only if",
            "  // that does not push u further past a limit it has already hit.",
            "  float uTry = Kp * e + Ki * Ts * (sumE + e) + Kd * dE;",
            "  bool stuckHigh = (uTry > U_MAX) && (e > 0);",
            "  bool stuckLow  = (uTry < U_MIN) && (e < 0);",
            "  if (!stuckHigh && !stuckLow) {",
            "    sumE += e;",
            "  }"
        );
    } else {
        lines.push(
            "  sumE += e;                                // no anti-windup: always integrate"
        );
    }
    lines.push(
        "",
        "  float u = Kp * e + Ki * Ts * sumE + Kd * dE;",
        "  u = constrain(u, U_MIN, U_MAX);           // the actuator cannot do more",
        "  writeControl(u);                          // held until the next sample",
        "",
        "  prevE = e;"
    );
    if (isMotor()) {
        lines.push(
            "",
            "  if (LOG) {                                // time, speed, PWM: compare with the simulation",
            "    Serial.print(millis()); Serial.print(',');",
            "    Serial.print(y); Serial.print(',');",
            "    Serial.println(u);",
            "  }"
        );
    }
    lines.push("}");
    return alignComments(lines);
}

/* Header for the Micromouse speed loop: encoder in, PWM % out. The loop part is shared. */
function motorHeader(p, ms, f) {
    return [
        "// Micromouse speed loop for one drive motor",
        "// generated by the ENCS4380 PID Control Toolkit (Module P3)",
        "// y = wheel speed in rpm from the encoder, u = PWM command in % (0..100)",
        "// u[k] = Kp*e[k] + Ki*Ts*sum(e[0..k]) + Kd*(e[k] - e[k-1])/Ts",
        "",
        "const float Kp = " + f(p.Kp) + ";                  // % PWM per rpm",
        "const float Ki = " + f(p.Ki) + ";",
        "const float Kd = " + f(p.Kd) + ";",
        "",
        "const unsigned long TS_MS = " + ms + ";      // sampling time in milliseconds",
        "const float Ts = TS_MS / 1000.0;            // the same, in seconds",
        "",
        "const float U_MIN = 0.0;                    // PWM limits in %",
        "const float U_MAX = " + f(p.uMax) + ";",
        "",
        "const int ENC_A = 2;                        // encoder channel A (an interrupt pin: 2 or 3 on an Uno)",
        "const int ENC_B = 4;                        // encoder channel B",
        "const int PWM_PIN = 9;                      // motor driver PWM input",
        "const int DIR_PIN = 8;                      // motor driver direction input",
        "const float COUNTS_PER_REV = " + state.cpr.toFixed(1) + ";        // rising edges of A per wheel revolution",
        "const bool LOG = true;                      // print t,speed,PWM every sample",
        "",
        "float setpoint = " + f(p.r) + ";                // wheel speed setpoint in rpm",
        "float sumE = 0;                             // running sum of the error",
        "float prevE = 0;                            // error at the previous sample",
        "bool firstRun = true;",
        "unsigned long lastTime = 0;",
        "volatile long encCount = 0;                 // changed inside the interrupt",
        "",
        "// Runs on every rising edge of channel A. If the speed comes out negative, swap ENC_A and ENC_B.",
        "void onEncoderA() {",
        "  if (digitalRead(ENC_B)) encCount++;",
        "  else encCount--;",
        "}",
        "",
        "// Wheel speed in rpm from the counts seen during the last Ts.",
        "// One count in one Ts is " + (60 / (state.cpr * p.Ts)).toFixed(1) + " rpm, so the reading moves in steps of that size:",
        "// a longer Ts or more counts per revolution gives a finer speed reading.",
        "float readOutput() {",
        "  noInterrupts();",
        "  long counts = encCount;",
        "  encCount = 0;",
        "  interrupts();",
        "  return (counts / COUNTS_PER_REV) * (60.0 / Ts);",
        "}",
        "",
        "// u is PWM in %; analogWrite wants 0..255.",
        "void writeControl(float u) {",
        "  analogWrite(PWM_PIN, (int)(u * 255.0 / 100.0 + 0.5));",
        "}",
        "",
        "void setup() {",
        "  pinMode(ENC_A, INPUT_PULLUP);",
        "  pinMode(ENC_B, INPUT_PULLUP);",
        "  attachInterrupt(digitalPinToInterrupt(ENC_A), onEncoderA, RISING);",
        "  pinMode(PWM_PIN, OUTPUT);",
        "  pinMode(DIR_PIN, OUTPUT);",
        "  digitalWrite(DIR_PIN, HIGH);                // forward",
        "  Serial.begin(115200);",
        "  lastTime = millis();",
        "}",
        ""
    ];
}

/* Step test sketch: 0.5 s at 0 %, then a PWM step, speed printed every 10 ms */
function stepTestCode() {
    const lines = [
        "// Step test for one Micromouse drive motor (ENCS4380 PID Control Toolkit, Module P3)",
        "// Prints \"t_ms,rpm\" every 10 ms: 0.5 s at 0 % PWM, then a " + state.stepPct + " % step for 3 s.",
        "// Paste everything it prints into the P3 page to get K and tau.",
        "",
        "const int ENC_A = 2;                        // encoder channel A (an interrupt pin: 2 or 3 on an Uno)",
        "const int ENC_B = 4;                        // encoder channel B",
        "const int PWM_PIN = 9;                      // motor driver PWM input",
        "const int DIR_PIN = 8;                      // motor driver direction input",
        "const float COUNTS_PER_REV = " + state.cpr.toFixed(1) + ";        // rising edges of A per wheel revolution",
        "const float STEP_PERCENT = " + state.stepPct.toFixed(1) + ";          // size of the PWM step",
        "const unsigned long TS_MS = 10;             // one speed sample every 10 ms",
        "",
        "volatile long encCount = 0;",
        "",
        "void onEncoderA() {                         // if the speed comes out negative, swap ENC_A and ENC_B",
        "  if (digitalRead(ENC_B)) encCount++;",
        "  else encCount--;",
        "}",
        "",
        "void setup() {",
        "  pinMode(ENC_A, INPUT_PULLUP);",
        "  pinMode(ENC_B, INPUT_PULLUP);",
        "  attachInterrupt(digitalPinToInterrupt(ENC_A), onEncoderA, RISING);",
        "  pinMode(PWM_PIN, OUTPUT);",
        "  pinMode(DIR_PIN, OUTPUT);",
        "  digitalWrite(DIR_PIN, HIGH);                // forward",
        "  analogWrite(PWM_PIN, 0);",
        "",
        "  Serial.begin(115200);",
        "  delay(3000);                                // time to open the Serial Monitor",
        "  Serial.println(\"t_ms,rpm\");",
        "",
        "  noInterrupts(); encCount = 0; interrupts();",
        "  unsigned long last = millis();",
        "  for (int k = -50; k <= 300; k++) {",
        "    while (millis() - last < TS_MS) { }       // wait for the next sample",
        "    last += TS_MS;",
        "    noInterrupts();",
        "    long counts = encCount;                   // counts during the last 10 ms",
        "    encCount = 0;",
        "    interrupts();",
        "    float rpm = (counts / COUNTS_PER_REV) * (60000.0 / TS_MS);",
        "    Serial.print((long)k * (long)TS_MS);     // time in ms (negative before the step)",
        "    Serial.print(',');",
        "    Serial.println(rpm);",
        "    if (k == 0) {                             // the step starts at t = 0",
        "      analogWrite(PWM_PIN, (int)(STEP_PERCENT * 255.0 / 100.0 + 0.5));",
        "    }",
        "  }",
        "  analogWrite(PWM_PIN, 0);                    // stop the motor",
        "  Serial.println(\"done\");",
        "}",
        "",
        "void loop() { }"
    ];
    return alignComments(lines);
}

/* line up the end-of-line comments in one column */
function alignComments(lines) {
    return lines.map(line => {
        const i = line.indexOf("//");
        if (i <= 0 || line.slice(0, i).trim() === "") return line;
        const codePart = line.slice(0, i).trimEnd();
        return codePart.padEnd(Math.max(44, codePart.length + 2)) + line.slice(i);
    }).join("\n");
}

function escapeHtml(s) { return s.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;"); }

function codeToHtml(text) {
    // grey out comments
    return text.split("\n").map(line => {
        const i = line.indexOf("//");
        if (i < 0) return escapeHtml(line);
        return escapeHtml(line.slice(0, i)) + '<span class="c">' + escapeHtml(line.slice(i)) + "</span>";
    }).join("\n");
}

function drawCode(p) {
    $("code").innerHTML = codeToHtml(arduinoCode(p));
    $("stepCode").innerHTML = codeToHtml(stepTestCode());
    $("hCode").textContent = isMotor() ? "Micromouse speed-loop code for these gains" : "Arduino code for these gains";
    $("codeNote").innerHTML = isMotor()
        ? "This is the same loop the simulation above runs, line for line, with the speed read from the encoder " +
          "and the PWM written in %. Check the pin numbers and the counts per revolution for your mouse."
        : "This is the same loop the simulation above runs, line for line. Change a slider and the code updates. " +
          "Set <code>readOutput()</code> and <code>writeControl()</code> for your own sensor and actuator.";
}

function copyText(text, msgId) {
    const done = () => {
        $(msgId).textContent = "Copied ✓ ";
        setTimeout(() => { $(msgId).textContent = ""; }, 2000);
    };
    if (navigator.clipboard && window.isSecureContext) {
        navigator.clipboard.writeText(text).then(done, () => fallbackCopy(text, done, msgId));
    } else {
        fallbackCopy(text, done, msgId);
    }
}
$("copyBtn").addEventListener("click", () => copyText(arduinoCode(state), "copyMsg"));
$("copyStepBtn").addEventListener("click", () => copyText(stepTestCode(), "copyStepMsg"));

function fallbackCopy(text, done, msgId) {
    const area = document.createElement("textarea");
    area.value = text;
    area.style.position = "fixed";
    area.style.opacity = "0";
    document.body.appendChild(area);
    area.select();
    try { document.execCommand("copy"); done(); }
    catch (err) { $(msgId).textContent = "Select the code and copy it by hand."; }
    document.body.removeChild(area);
}


/* 
   Micromouse: step test inputs, K and τ from the log, PI suggestion
   */

$("inCpr").addEventListener("input", e => {
    const v = parseFloat(e.target.value);
    if (v > 0) { state.cpr = v; drawCode(state); }
});
$("inStep").addEventListener("input", e => {
    const v = parseFloat(e.target.value);
    if (v > 0 && v <= 100) { state.stepPct = v; drawCode(state); }
});

/* Read "t_ms,rpm" lines; anything that is not two numbers (the header, "done") is skipped. */
function parseLog(text) {
    const pts = [];
    text.split(/\r?\n/).forEach(line => {
        const nums = line.split(/[,;\s\t]+/).filter(x => x !== "").map(Number);
        if (nums.length >= 2 && nums.every(isFinite)) pts.push([nums[0], nums[1]]);
    });
    return pts;
}

/* First-order fit. K comes from the final value. τ comes from the area between the final value
   and the curve: for y = rise·(1 − e^(−t/τ)) that area is exactly rise·τ. Using the whole curve
   averages out the encoder's whole-count steps, which can make the single 63.2 % crossing early.
   The 63.2 % point (the Module 1 method) is still found and shown as a cross-check.
   Each logged speed is the average over the previous sample period, so for the 63.2 % point it
   is placed half a period earlier (t − Δt/2); the area needs no shift. */
function fitStep(pts, stepPct) {
    if (pts.length < 20) return { error: "Only " + pts.length + " data lines found. Paste the whole Serial Monitor output." };
    const dts = [];
    for (let i = 1; i < pts.length; i++) dts.push(pts[i][0] - pts[i - 1][0]);
    dts.sort((a, b) => a - b);
    const dt = dts[Math.floor(dts.length / 2)];                    // ms
    if (!(dt > 0)) return { error: "The time column must increase from line to line." };

    const before = pts.filter(p => p[0] <= 0).map(p => p[1]);
    const after = pts.filter(p => p[0] > 0).map(p => [(p[0] - dt / 2) / 1000, p[1]]);   // seconds
    if (after.length < 10) return { error: "Not enough samples after t = 0 (the step)." };

    const baseline = before.length ? before.reduce((a, b) => a + b, 0) / before.length : 0;
    const tailN = Math.max(3, Math.floor(after.length * 0.2));
    const tail = after.slice(-tailN).map(p => p[1]);
    const final = tail.reduce((a, b) => a + b, 0) / tail.length;
    const rise = final - baseline;
    if (!(rise > 0)) return { error: "The speed did not go up after the step. If it went negative, swap ENC_A and ENC_B; otherwise check the wiring." };

    // still rising at the end? compare the last 10 % with the 10 % before it
    const n10 = Math.max(2, Math.floor(after.length * 0.1));
    const avg = a => a.reduce((x, y) => x + y, 0) / a.length;
    const last10 = avg(after.slice(-n10).map(p => p[1]));
    const prev10 = avg(after.slice(-2 * n10, -n10).map(p => p[1]));
    const stillRising = (last10 - prev10) > 0.03 * rise;

    const level = baseline + 0.632 * rise;
    let tau = null;
    let prev = [0, baseline];
    for (const p of after) {
        if (p[1] >= level) {
            tau = prev[0] + (level - prev[1]) / (p[1] - prev[1]) * (p[0] - prev[0]);
            break;
        }
        prev = p;
    }
    if (tau === null || !(tau > 0)) return { error: "Could not find the 63.2 % point. Is the log complete?" };
    const tau63 = tau;

    // area between the final value and the measured curve, divided by the rise
    let area = 0;
    pts.filter(p => p[0] > 0).forEach(p => { area += (final - p[1]) * dt / 1000; });
    const tauArea = area / rise;

    return { K: rise / stepPct, tau: tauArea > 0 ? tauArea : tau63, tau63, baseline, final, dt, stillRising, after, stepPct };
}

let lastFit = null;

$("fitBtn").addEventListener("click", () => {
    const fit = fitStep(parseLog($("logIn").value), state.stepPct);
    const out = $("fitOut");
    out.hidden = false;
    if (fit.error) {
        lastFit = null;
        out.innerHTML = "<b>Could not fit:</b> " + escapeHtml(fit.error);
        $("useFit").hidden = true;
        $("fitPlotBox").hidden = true;
        return;
    }
    lastFit = fit;
    out.innerHTML =
        "<b>K = " + fit.K.toFixed(2) + " rpm per 1 % PWM</b> (the speed rose by " + (fit.final - fit.baseline).toFixed(0) +
        " rpm for a " + fit.stepPct + " % step)<br>" +
        "<b>τ = " + fit.tau.toFixed(3) + " s</b> (area between the final speed and the curve, divided by the rise)<br>" +
        "Cross-check: the curve reaches 63.2 % of the rise at t = " + fit.tau63.toFixed(3) + " s. " +
        "The two should be close; a big gap means a noisy encoder or a motor that is not first order.<br>" +
        "Sample spacing in the log: " + fit.dt.toFixed(0) + " ms." +
        (fit.stillRising ? "<br><b>Warning:</b> the speed was still rising at the end of the log. Run the test longer, or K will come out too small." : "");
    $("useFit").hidden = false;

    // measured points and the fitted curve on one plot
    $("fitPlotBox").hidden = false;
    const tEnd = fit.after[fit.after.length - 1][0];
    const ft = [], fy = [];
    for (let i = 0; i <= 200; i++) {
        const t = tEnd * i / 200;
        ft.push(t);
        fy.push(fit.baseline + fit.K * fit.stepPct * (1 - Math.exp(-t / fit.tau)));
    }
    const L = layoutFor("Your step test and the fitted first-order model", "time after the step (s)", "speed (rpm)");
    L.shapes = [{ type: "line", xref: "x", yref: "y", x0: fit.tau63, x1: fit.tau63, y0: 0,
                  y1: fit.baseline + 0.632 * fit.K * fit.stepPct, line: { color: ORANGE, width: 1.5, dash: "dot" } }];
    Plotly.react("plotFit", [
        { x: fit.after.map(p => p[0]), y: fit.after.map(p => p[1]), mode: "markers", name: "Measured speed",
          marker: { color: ROSE, size: 5 } },
        { x: ft, y: fy, mode: "lines", name: "Model K·step·(1 − e^(−t/τ))", line: { color: LILAC, width: 3 } },
        { x: [fit.tau63], y: [fit.baseline + 0.632 * fit.K * fit.stepPct], mode: "markers+text", name: "Measured 63.2 % point",
          text: ["63.2 % at " + fit.tau63.toFixed(3) + " s"], textposition: "bottom right", marker: { color: MINT, size: 10 } }
    ], L, plotConfig);
});

$("useFit").addEventListener("click", () => {
    if (!lastFit) return;
    stopExperiment();
    setPlant("motor");
    state.K = Math.round(lastFit.K * 100) / 100;
    state.tau = Math.round(lastFit.tau * 1000) / 1000;
    update();
    document.querySelector(".instrument").scrollIntoView({ behavior: "smooth", block: "start" });
});

/* PI for zeta = 0.7: Kp = 1/K, then wn = 1/(zeta*tau) and Ki = wn^2 * tau / K = 1/(zeta^2 * tau * K) */
$("suggestBtn").addEventListener("click", () => {
    stopExperiment();
    if (!isMotor()) setPlant("motor");
    const zeta = 0.7;
    state.Kp = Math.round(100 / state.K) / 100;
    state.Ki = Math.round(100 / (zeta * zeta * state.tau * state.K)) / 100;
    state.Kd = 0;
    update();
    $("suggestMsg").textContent = "Kp = " + state.Kp.toFixed(2) + ", Ki = " + state.Ki.toFixed(2) +
        " for K = " + state.K.toFixed(2) + " rpm/% and τ = " + state.tau.toFixed(3) + " s.";
    document.querySelector(".instrument").scrollIntoView({ behavior: "smooth", block: "start" });
});


/* 
   Predict, then check
   */

const base = { plant: "first", K: 1, tau: 1, Kp: 5, Ki: 5, Kd: 0.2, uMax: 8, r: 1, aw: true, showCont: true, showParts: false };

const quizQuestions = [
    {
        question: "Kp = 5, Ki = 5, Kd = 0.2. If the sampling time grows from 20 ms to 250 ms, the overshoot will…",
        options: ["stay the same: the gains did not change", "grow", "shrink"],
        correct: 1,
        explain: "Between samples the controller is blind and keeps sending an old value of u. A longer Ts means the loop reacts later, which acts like a delay in the loop, and delay adds overshoot: here it goes from about 1 % to about 38 %. Watch the pink curve pull away from the dotted continuous one.",
        start: Object.assign({}, base, { Ts: 0.02 }),
        end: { Ts: 0.25 }
    },
    {
        question: "Kp = 2, Ki = 5, and u is limited to 1.2. The integral winds up while u is stuck at the limit. Switching anti-windup on will make the overshoot…",
        options: ["larger", "smaller", "no different"],
        correct: 1,
        explain: "Without anti-windup the error sum keeps growing while u is stuck at its limit. When y finally reaches the setpoint, that big sum keeps u high and the output shoots past (about 19 % here). With anti-windup the sum stops growing while u is stuck, so the overshoot drops to about 2 %. Tick “Show the P, I and D parts” to watch the I part.",
        start: Object.assign({}, base, { Kp: 2, Ki: 5, Kd: 0, uMax: 1.2, Ts: 0.05, aw: false, showParts: true }),
        end: { aw: true }
    },
    {
        question: "A PI controller (Kp = 3, Ki = 3) is sampled slowly, at Ts = 300 ms, but the loop still settles. Its steady-state error is…",
        options: ["zero", "about 10 %", "it grows with Ts"],
        correct: 0,
        explain: "As long as the loop settles, e[k] must be zero in the end, otherwise the sum Σe would keep changing u. So a slow sample rate costs speed and overshoot, but not accuracy. The table shows a steady-state error of ≈ 0.",
        start: Object.assign({}, base, { Kp: 3, Ki: 3, Kd: 0, Ts: 0.02 }),
        end: { Ts: 0.3 }
    },
    {
        question: "Between two samples, what does the control signal u do?",
        options: ["It follows the error smoothly", "It stays constant", "It drops to zero"],
        correct: 1,
        explain: "The Arduino writes one PWM value per loop and nothing changes it until the next sample. That is the stair step in the second plot (a zero-order hold). With Ts = 400 ms the steps are wide enough to see clearly.",
        start: Object.assign({}, base, { Kp: 2, Ki: 2, Kd: 0, Ts: 0.02 }),
        end: { Ts: 0.4 }
    },
    {
        question: "P only, Kp = 9 (the closed-loop time constant is 0.1 s). Which sampling time first makes the loop oscillate without settling?",
        options: ["20 ms", "100 ms", "250 ms"],
        correct: 2,
        explain: "For this loop the sampled pole is a − (1 − a)·K·Kp with a = e^(−Ts/τ). It leaves the unit circle when (1 − e^(−Ts/τ))(1 + K·Kp) > 2, which happens at Ts ≈ 223 ms. At 100 ms it is still fine, at 250 ms it is unstable. Slower sampling can turn a stable design unstable without changing any gain.",
        start: Object.assign({}, base, { Kp: 9, Ki: 0, Kd: 0, uMax: 10, Ts: 0.02 }),
        end: { Ts: 0.25 }
    }
];

let quizIndex = 0, quizPicked = null, quizAnswered = 0, quizRight = 0;
let experimentRaf = null;

function showQuestion() {
    const q = quizQuestions[quizIndex];
    quizPicked = null;
    $("quizCount").textContent = "Question " + (quizIndex + 1) + " of " + quizQuestions.length;
    $("quizScore").textContent = quizAnswered ? quizRight + " of " + quizAnswered + " correct" : "";
    $("quizQuestion").textContent = q.question;
    const box = $("quizOptions");
    box.innerHTML = "";
    q.options.forEach((text, i) => {
        const label = document.createElement("label");
        label.innerHTML = '<input type="radio" name="quizOption" value="' + i + '"><span></span>';
        label.querySelector("span").textContent = text;
        label.querySelector("input").addEventListener("change", () => { quizPicked = i; $("quizLock").disabled = false; });
        box.appendChild(label);
    });
    $("quizLock").disabled = true;
    $("quizLock").hidden = false;
    $("quizRun").hidden = true;
    $("quizNext").hidden = true;
    $("quizExplain").innerHTML = "";
}

$("quizLock").addEventListener("click", () => {
    if (quizPicked === null) return;
    const q = quizQuestions[quizIndex];
    quizAnswered++;
    if (quizPicked === q.correct) quizRight++;
    document.querySelectorAll("#quizOptions label").forEach((label, i) => {
        label.querySelector("input").disabled = true;
        if (i === q.correct) label.classList.add("right");
        else if (i === quizPicked) label.classList.add("wrong");
    });
    $("quizScore").textContent = quizRight + " of " + quizAnswered + " correct";
    $("quizExplain").innerHTML = "<b>" + (quizPicked === q.correct ? "Correct." : "Not quite.") + "</b> " + q.explain;
    $("quizLock").hidden = true;
    $("quizRun").hidden = false;
    $("quizNext").hidden = false;
});

function stopExperiment() {
    if (experimentRaf) { cancelAnimationFrame(experimentRaf); experimentRaf = null; }
}

/* Run the experiment: load the start values, then move to the end values.
   Ts moves smoothly on a log scale; on/off settings switch half way. */
$("quizRun").addEventListener("click", () => {
    const q = quizQuestions[quizIndex];
    stopExperiment();
    if (isMotor()) buckets.motor = pickModeValues();     // keep the motor values for later
    Object.assign(state, q.start);
    update();
    document.querySelector(".instrument").scrollIntoView({ behavior: "smooth", block: "start" });

    const reduce = window.matchMedia("(prefers-reduced-motion: reduce)").matches;
    const dur = reduce ? 1 : 2200, t0 = performance.now() + 500;
    const frame = now => {
        const f = Math.min(1, Math.max(0, (now - t0) / dur));
        if (q.end.Ts !== undefined) {
            const ts = q.start.Ts * Math.pow(q.end.Ts / q.start.Ts, f);
            state.Ts = Math.max(1, Math.round(ts * 1000)) / 1000;
        }
        if (q.end.aw !== undefined && f >= 0.5) state.aw = q.end.aw;
        update(f < 1);
        if (f < 1) experimentRaf = requestAnimationFrame(frame);
        else { experimentRaf = null; state.Ts = q.end.Ts !== undefined ? q.end.Ts : state.Ts; update(); }
    };
    experimentRaf = requestAnimationFrame(frame);
});

$("quizNext").addEventListener("click", () => {
    stopExperiment();
    quizIndex = (quizIndex + 1) % quizQuestions.length;
    showQuestion();
});


/* =========================================================
   Validation checks (homework section 2.1), run through this
   page's own digital loop with Ts = 1 ms
   ========================================================= */

function runValidation() {
    const plant = { plant: "first", K: 1, tau: 1, zeta: 0, wn: 0, Kd: 0, uMax: 1e6, r: 1, aw: false };
    const Ts = 0.001;
    const rows = [];

    // 1. P only, Kp = 9: tau_cl = 0.1 s, e_ss = 10 %
    {
        const p = Object.assign({}, plant, { Kp: 9, Ki: 0 });
        const s = simulate(p, Ts, 3);
        const m = readMetrics(s.t, s.y, 1);
        const i63 = s.y.findIndex(v => v >= 0.632 * m.yf);
        const tauCl = s.t[i63 - 1] + (0.632 * m.yf - s.y[i63 - 1]) / (s.y[i63] - s.y[i63 - 1]) * (s.t[i63] - s.t[i63 - 1]);
        const ok = Math.abs(tauCl - 0.1) < 0.002 && Math.abs(m.sse - 0.1) < 0.001;
        rows.push(["1. P only, Kp = 9: closed-loop τ and steady-state error",
                   "τ/(1 + KKp) = 0.1 s and 1/(1 + KKp) = 10 %",
                   tauCl.toFixed(3) + " s and " + (m.sse * 100).toFixed(1) + " %", ok]);
    }

    // 2. PI, Kp = 1, Ki = 1: wn = 1 rad/s, zeta = 1
    {
        const K = 1, tau = 1, Kp = 1, Ki = 1;
        const wn = Math.sqrt(K * Ki / tau);
        const zeta = (1 + K * Kp) / (2 * Math.sqrt(tau * K * Ki));
        // closed-loop poles: roots of s^2 + (1 + K Kp)/tau s + K Ki/tau = 0
        const b = (1 + K * Kp) / tau, c = K * Ki / tau, disc = b * b / 4 - c;
        const pole = disc >= 0 ? (-b / 2 + Math.sqrt(disc)).toFixed(3) + ", " + (-b / 2 - Math.sqrt(disc)).toFixed(3) : "complex";
        // the simulated loop should then follow y = 1 - e^(-t) (the PI zero cancels one pole)
        const p = Object.assign({}, plant, { Kp, Ki });
        const s = simulate(p, Ts, 8);
        let maxErr = 0;
        for (let i = 0; i < s.t.length; i += 10) maxErr = Math.max(maxErr, Math.abs(s.y[i] - (1 - Math.exp(-s.t[i]))));
        const ok = Math.abs(wn - 1) < 1e-9 && Math.abs(zeta - 1) < 1e-9 && maxErr < 0.005;
        rows.push(["2. PI, Kp = 1, Ki = 1: natural frequency and damping",
                   "ωn = 1 rad/s and ζ = 1",
                   "ωn = " + wn.toFixed(3) + " rad/s, ζ = " + zeta.toFixed(3) + "<br>poles: " + pole +
                   "<br>simulation vs 1 − e<sup>−t</sup>: max difference " + maxErr.toFixed(4), ok]);
    }

    // 3. Any PI controller: zero steady-state error
    {
        const gains = [[1, 1], [3, 3], [9, 2], [0.5, 4], [5, 10]];
        const errs = gains.map(([Kp, Ki]) => {
            const s = simulate(Object.assign({}, plant, { Kp, Ki }), Ts, 30);
            return readMetrics(s.t, s.y, 1).sse;
        });
        const worst = Math.max(...errs.map(Math.abs));
        rows.push(["3. Any PI controller: steady-state error for a step",
                   "0 for every PI gain pair",
                   "largest |e<sub>ss</sub>| over 5 gain pairs: " + worst.toFixed(4), worst < 0.001]);
    }

    // P3 only: a slow sample rate destabilises Kp = 9 where theory says
    {
        const p = Object.assign({}, plant, { Kp: 9, Ki: 0, uMax: 1e6 });
        const tsTheory = -Math.log(1 - 2 / (1 + 9));               // (1 - e^(-Ts/tau))(1 + K Kp) = 2
        const before = readMetrics(...(s => [s.t, s.y])(simulate(p, Math.floor(tsTheory * 1000 - 20) / 1000, 15)), 1);
        const after = readMetrics(...(s => [s.t, s.y])(simulate(p, Math.ceil(tsTheory * 1000 + 20) / 1000, 15)), 1);
        rows.push(["4. P3: P only, Kp = 9, sampled slowly",
                   "settles below Ts = " + (tsTheory * 1000).toFixed(0) + " ms, never settles above it",
                   (before.settled ? "settles" : "never settles") + " at " + Math.floor(tsTheory * 1000 - 20) + " ms, " +
                   (after.settled ? "settles" : "never settles") + " at " + Math.ceil(tsTheory * 1000 + 20) + " ms",
                   before.settled && !after.settled]);
    }

    $("valBody").innerHTML = rows.map(([name, exp, got, pass]) =>
        "<tr><td>" + name + "</td><td>" + exp + "</td><td>" + got + "</td><td class='" + (pass ? "pass" : "fail") + "'>" +
        (pass ? "PASS ✓" : "FAIL ✗") + "</td></tr>").join("");
}


/* 
   Start
   */

update();
showQuestion();
runValidation();
