"use strict";

const $ = (id) => document.getElementById(id);

const COLORS = {
  cyan: "#43d9d3",
  amber: "#ffb65f",
  rose: "#ff718a",
  violet: "#ad91ff",
  text: "#ecf8fa",
  muted: "#8daab5",
  grid: "rgba(164, 203, 217, 0.12)",
  axis: "rgba(164, 203, 217, 0.28)",
  bg: "#071521"
};

const defaults = Object.freeze({
  mode: "micro16",
  speedRpm: 20,
  direction: 1,
  loadTorque: 0.005,
  duration: 0.8,
  accelTime: 0.08,
  pulseEnabled: false,
  pulseTorque: 2.6,
  pulseStart: 0.45,
  pulseDuration: 0.03,
  voltage: 24,
  currentLimit: 3.4,
  resistance: 0.5,
  inductance: 2.0,
  torqueConstant: 0.575,
  holdingTorque: 1.96,
  rotorTeeth: 50,
  inertia: 48,
  damping: 0.05,
  detentTorque: 0.068,
  bandwidth: 700
});

const OFFICIAL_DATASHEET_URL = "https://www.omc-stepperonline.com/e-series-nema-23-bipolar-1-8deg-1-9nm-269oz-in-2-8a-3-2v-57x57x76mm-4-wires-23he30-2804s";

const motorProfiles = Object.freeze({
  project: {
    name: "Supplied project profile",
    values: {
      voltage: 24,
      currentLimit: 3.4,
      resistance: 0.5,
      inductance: 2.0,
      torqueConstant: 0.575,
      holdingTorque: 1.96,
      rotorTeeth: 50,
      inertia: 48,
      damping: 0.05,
      detentTorque: 0.068,
      bandwidth: 700
    }
  },
  stepperonline23HE30: {
    name: "StepperOnline 23HE30-2804S",
    values: {
      voltage: 24,
      currentLimit: 2.8,
      resistance: 1.17,
      inductance: 4.8,
      torqueConstant: 2.0 / 2.8,
      holdingTorque: 2.0,
      rotorTeeth: 50,
      inertia: 48,
      damping: 0.05,
      detentTorque: 0.068,
      bandwidth: 700
    }
  }
});

let datasheetMeta = {
  name: motorProfiles.project.name,
  kind: "project",
  importedFields: []
};

let simulation = null;
let animationFrame = null;
let isPlaying = false;
let playbackIndex = 0;
let playbackAnchor = 0;
let simulationTimeAnchor = 0;

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function readNumber(id) {
  return Number($(id).value);
}

function readParameters() {
  return {
    mode: $("mode").value,
    speedRpm: readNumber("speedRpm"),
    direction: Number($("direction").value),
    loadTorque: readNumber("loadTorque"),
    duration: readNumber("duration"),
    accelTime: readNumber("accelTime"),
    pulseEnabled: $("pulseEnabled").checked,
    pulseTorque: readNumber("pulseTorque"),
    pulseStart: readNumber("pulseStart"),
    pulseDuration: readNumber("pulseDuration"),
    voltage: readNumber("voltage"),
    currentLimit: readNumber("currentLimit"),
    resistance: readNumber("resistance"),
    inductance: readNumber("inductance") * 1e-3,
    torqueConstant: readNumber("torqueConstant"),
    holdingTorque: readNumber("holdingTorque"),
    rotorTeeth: Math.round(readNumber("rotorTeeth")),
    inertia: readNumber("inertia") * 1e-6,
    damping: readNumber("damping"),
    detentTorque: readNumber("detentTorque"),
    bandwidth: readNumber("bandwidth")
  };
}

function validateParameters(p) {
  const values = Object.entries(p).filter(([, value]) => typeof value === "number");
  if (values.some(([, value]) => !Number.isFinite(value))) {
    return "تأكدي أن جميع الحقول العددية تحتوي قيماً صحيحة.";
  }
  if (p.pulseEnabled && p.pulseStart + p.pulseDuration > p.duration) {
    return "نبضة الحمل تنتهي بعد زمن المحاكاة. قلّلي بداية النبضة أو مدتها.";
  }
  if (p.accelTime > p.duration) {
    return "زمن التسارع يجب أن يكون أصغر من زمن المحاكاة.";
  }
  if (p.loadTorque > 1.4 * p.torqueConstant * p.currentLimit && !p.pulseEnabled) {
    return null;
  }
  return null;
}

function electricalIncrement(mode) {
  if (mode === "full") return Math.PI / 2;
  if (mode === "half") return Math.PI / 4;
  return Math.PI / 32;
}

function commandAngle(phiContinuous, increment, direction) {
  const scaled = phiContinuous / increment;
  return direction >= 0
    ? Math.floor(scaled + 1e-10) * increment
    : Math.ceil(scaled - 1e-10) * increment;
}

function simulate(p) {
  const dt = Math.min(50e-6, Math.max(8e-6, (p.inductance / p.resistance) / 40));
  const totalSteps = Math.ceil(p.duration / dt);
  const recordStride = Math.max(1, Math.floor(totalSteps / 1100));
  const omegaTarget = p.direction * p.speedRpm * 2 * Math.PI / 60;
  const increment = electricalIncrement(p.mode);
  const fullStepRad = Math.PI / (2 * p.rotorTeeth);
  const wc = 2 * Math.PI * p.bandwidth;
  const kp = p.inductance * wc;
  const ki = p.resistance * wc;
  const electricalDecay = Math.exp(-p.resistance * dt / p.inductance);

  let ia = 0;
  let ib = 0;
  let theta = 0;
  let omega = 0;
  let thetaCommandContinuous = 0;
  let integratorA = 0;
  let integratorB = 0;
  let previousCommand = 0;
  let voltageSaturationCount = 0;
  let peakCurrent = 0;
  let peakTorque = 0;
  let maxErrorSteps = 0;
  let sumSpeedTail = 0;
  let speedTailCount = 0;

  const data = [];

  for (let k = 0; k <= totalSteps; k += 1) {
    const t = Math.min(k * dt, p.duration);
    const ramp = p.accelTime > 0 ? Math.min(t / p.accelTime, 1) : 1;
    const omegaCommand = omegaTarget * ramp;
    thetaCommandContinuous += omegaCommand * dt;

    const phiContinuous = p.rotorTeeth * thetaCommandContinuous;
    const phiCommand = commandAngle(phiContinuous, increment, p.direction);
    const thetaReference = phiCommand / p.rotorTeeth;
    const iaReference = p.currentLimit * Math.cos(phiCommand);
    const ibReference = p.currentLimit * Math.sin(phiCommand);

    const errorA = iaReference - ia;
    const errorB = ibReference - ib;
    const unsaturatedA = kp * errorA + integratorA;
    const unsaturatedB = kp * errorB + integratorB;
    const va = clamp(unsaturatedA, -p.voltage, p.voltage);
    const vb = clamp(unsaturatedB, -p.voltage, p.voltage);

    if (Math.abs(unsaturatedA) < p.voltage || Math.sign(errorA) !== Math.sign(unsaturatedA)) {
      integratorA = clamp(integratorA + ki * errorA * dt, -p.voltage, p.voltage);
    } else {
      voltageSaturationCount += 1;
    }
    if (Math.abs(unsaturatedB) < p.voltage || Math.sign(errorB) !== Math.sign(unsaturatedB)) {
      integratorB = clamp(integratorB + ki * errorB * dt, -p.voltage, p.voltage);
    } else {
      voltageSaturationCount += 1;
    }

    const thetaElectrical = p.rotorTeeth * theta;
    const iaEquilibrium = (va + p.torqueConstant * omega * Math.sin(thetaElectrical)) / p.resistance;
    const ibEquilibrium = (vb - p.torqueConstant * omega * Math.cos(thetaElectrical)) / p.resistance;
    ia = iaEquilibrium + (ia - iaEquilibrium) * electricalDecay;
    ib = ibEquilibrium + (ib - ibEquilibrium) * electricalDecay;
    const currentCeiling = 1.08 * p.currentLimit;
    ia = clamp(ia, -currentCeiling, currentCeiling);
    ib = clamp(ib, -currentCeiling, currentCeiling);

    const electromagneticTorque = p.torqueConstant * (
      -ia * Math.sin(thetaElectrical) + ib * Math.cos(thetaElectrical)
    );
    const detentTorque = p.detentTorque * Math.sin(2 * thetaElectrical);
    const pulseActive = p.pulseEnabled && t >= p.pulseStart && t < p.pulseStart + p.pulseDuration;
    const loadMagnitude = p.loadTorque + (pulseActive ? p.pulseTorque : 0);
    const loadTorque = p.direction * loadMagnitude;
    const acceleration = (electromagneticTorque - p.damping * omega - detentTorque - loadTorque) / p.inertia;

    omega += acceleration * dt;
    theta += omega * dt;

    const positionError = thetaReference - theta;
    const errorSteps = positionError / fullStepRad;
    peakCurrent = Math.max(peakCurrent, Math.abs(ia), Math.abs(ib));
    peakTorque = Math.max(peakTorque, Math.abs(electromagneticTorque));
    maxErrorSteps = Math.max(maxErrorSteps, Math.abs(errorSteps));
    if (t >= 0.8 * p.duration) {
      sumSpeedTail += omega * 60 / (2 * Math.PI);
      speedTailCount += 1;
    }

    if (k % recordStride === 0 || k === totalSteps) {
      data.push({
        t,
        ia,
        ib,
        iaReference,
        ibReference,
        theta,
        thetaReference,
        omega,
        omegaRpm: omega * 60 / (2 * Math.PI),
        torque: electromagneticTorque,
        loadTorque,
        va,
        vb,
        errorSteps,
        commandChanged: phiCommand !== previousCommand,
        pulseActive
      });
    }
    previousCommand = phiCommand;

    if (![ia, ib, theta, omega].every(Number.isFinite) || Math.abs(omega) > 2e5) {
      throw new Error("النموذج خرج عددياً عن الاستقرار لهذه القيم. خفّضي القيم المتطرفة ثم أعيدي التشغيل.");
    }
  }

  const final = data[data.length - 1];
  const finalErrorSteps = final.errorSteps;
  const lostSteps = Math.abs(finalErrorSteps) >= 0.5 ? Math.max(1, Math.round(Math.abs(finalErrorSteps))) : 0;
  const saturationRatio = voltageSaturationCount / Math.max(1, 2 * totalSteps);
  const averageSpeed = speedTailCount ? sumSpeedTail / speedTailCount : 0;
  const holdingEstimate = p.torqueConstant * p.currentLimit;
  const maxAppliedLoad = p.loadTorque + (p.pulseEnabled ? p.pulseTorque : 0);
  const outsideEnvelope = (
    p.voltage < 12 || p.voltage > 36 ||
    p.currentLimit > 5 || p.speedRpm > 120 ||
    p.resistance < 0.2 || p.resistance > 2 ||
    p.inductance < 0.5e-3 || p.inductance > 10e-3
  );

  return {
    parameters: p,
    data,
    dt,
    fullStepRad,
    fullStepDeg: fullStepRad * 180 / Math.PI,
    metrics: {
      finalPositionDeg: final.theta * 180 / Math.PI,
      finalReferenceDeg: final.thetaReference * 180 / Math.PI,
      finalErrorDeg: (final.thetaReference - final.theta) * 180 / Math.PI,
      finalErrorSteps,
      averageSpeed,
      peakCurrent,
      peakTorque,
      maxErrorSteps,
      lostSteps,
      saturationRatio,
      holdingEstimate,
      maxAppliedLoad,
      outsideEnvelope
    }
  };
}

function formatSigned(value, digits = 2) {
  const safe = Math.abs(value) < 0.5 * 10 ** (-digits) ? 0 : value;
  return `${safe.toFixed(digits)}`;
}

function setStatus(kind, label) {
  const node = $("runStatus");
  node.className = `run-status ${kind}`;
  node.innerHTML = `<span aria-hidden="true"></span>${label}`;
}

function classifyResult(result) {
  const m = result.metrics;
  if (m.lostSteps > 0) {
    return {
      kind: "bad",
      label: "فقدان خطوات محتمل",
      title: "الروتر لم يحافظ على التزامن مع الأمر",
      text: `الخطأ النهائي يساوي ${Math.abs(m.finalErrorSteps).toFixed(2)} خطوة كاملة تقريباً. خفّضي الحمل أو السرعة، أو ارفعي تيار الطور ضمن حدود الماطور.`
    };
  }
  if (m.saturationRatio > 0.08 || m.outsideEnvelope) {
    const reasons = [];
    if (m.saturationRatio > 0.08) reasons.push("الـdriver قضى جزءاً ملحوظاً عند حد الجهد");
    if (m.outsideEnvelope) reasons.push("بعض القيم خارج نطاق المعلمات الافتراضي");
    return {
      kind: "warn",
      label: "نتيجة تحتاج حذر",
      title: "التتبّع مستقر، لكن دقّة التقدير أقل",
      text: `${reasons.join("، ")}. النتيجة مفيدة للمقارنة النوعية، وتحتاج معايرة قبل استخدامها كتنبؤ كمي.`
    };
  }
  return {
    kind: "good",
    label: "تشغيل متزامن تقديرياً",
    title: "الروتر حافظ على التزامن ضمن معيار نصف خطوة",
    text: `الخطأ النهائي ${Math.abs(m.finalErrorSteps).toFixed(3)} خطوة، والحمل الأقصى ${m.maxAppliedLoad.toFixed(3)} N·m مقابل عزم كهرومغناطيسي تقديري متاح يقارب ${m.holdingEstimate.toFixed(3)} N·m.`
  };
}

function updateMetrics(result) {
  const m = result.metrics;
  $("finalPosition").textContent = `${formatSigned(m.finalPositionDeg, 2)}°`;
  $("averageSpeed").textContent = `${formatSigned(m.averageSpeed, 1)} rpm`;
  $("peakTorque").textContent = `${m.peakTorque.toFixed(3)} N·m`;
  $("peakCurrent").textContent = `${m.peakCurrent.toFixed(2)} A`;
  $("finalError").textContent = `${formatSigned(m.finalErrorDeg, 3)}°`;
  $("stepLossText").textContent = m.lostSteps
    ? `تقدير: فقدان ${m.lostSteps} خطوة كاملة تقريباً`
    : `لا يوجد فقدان نهائي وفق معيار ±0.5 خطوة (${Math.abs(m.finalErrorSteps).toFixed(3)} step)`;

  const outcome = classifyResult(result);
  setStatus(outcome.kind, outcome.label);
  const box = $("interpretation");
  box.className = `interpretation ${outcome.kind}`;
  box.innerHTML = `<span class="interpretation-icon" aria-hidden="true">${outcome.kind === "good" ? "✓" : outcome.kind === "warn" ? "!" : "×"}</span><div><b>${outcome.title}</b><p>${outcome.text}</p></div>`;
}

function prepareCanvas(canvas, cssHeight = null) {
  const ratio = Math.min(window.devicePixelRatio || 1, 2);
  const rect = canvas.getBoundingClientRect();
  const width = Math.max(260, rect.width || canvas.clientWidth || 400);
  const height = cssHeight || rect.height || canvas.clientHeight || 220;
  canvas.width = Math.round(width * ratio);
  canvas.height = Math.round(height * ratio);
  const ctx = canvas.getContext("2d");
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  return { ctx, width, height };
}

function niceRange(min, max) {
  if (!Number.isFinite(min) || !Number.isFinite(max)) return [-1, 1];
  if (Math.abs(max - min) < 1e-12) {
    const pad = Math.max(1, Math.abs(max) * 0.15);
    return [min - pad, max + pad];
  }
  const pad = 0.08 * (max - min);
  return [min - pad, max + pad];
}

function drawChart(canvas, data, series, yUnit) {
  const { ctx, width, height } = prepareCanvas(canvas, canvas.clientHeight || 220);
  ctx.clearRect(0, 0, width, height);
  const margin = { top: 15, right: 13, bottom: 31, left: 49 };
  const plotW = width - margin.left - margin.right;
  const plotH = height - margin.top - margin.bottom;
  const tMax = Math.max(1e-6, data[data.length - 1].t);
  const values = [];
  for (const row of data) for (const item of series) values.push(item.value(row));
  let [yMin, yMax] = niceRange(Math.min(...values), Math.max(...values));
  if (yMin > 0 && yMin < 0.15 * yMax) yMin = 0;
  if (yMax < 0 && yMax > 0.15 * yMin) yMax = 0;

  const x = (t) => margin.left + (t / tMax) * plotW;
  const y = (v) => margin.top + (1 - (v - yMin) / (yMax - yMin)) * plotH;

  ctx.strokeStyle = COLORS.grid;
  ctx.lineWidth = 1;
  ctx.fillStyle = COLORS.muted;
  ctx.font = "11px ui-sans-serif, system-ui";
  ctx.textAlign = "right";
  ctx.textBaseline = "middle";
  for (let i = 0; i <= 4; i += 1) {
    const yy = margin.top + (i / 4) * plotH;
    const value = yMax - (i / 4) * (yMax - yMin);
    ctx.beginPath();
    ctx.moveTo(margin.left, yy);
    ctx.lineTo(width - margin.right, yy);
    ctx.stroke();
    ctx.fillText(value.toFixed(Math.abs(yMax - yMin) < 1 ? 2 : 1), margin.left - 7, yy);
  }

  ctx.textAlign = "center";
  ctx.textBaseline = "top";
  for (let i = 0; i <= 4; i += 1) {
    const xx = margin.left + (i / 4) * plotW;
    ctx.fillText(((i / 4) * tMax).toFixed(2), xx, height - margin.bottom + 8);
  }

  if (yMin <= 0 && yMax >= 0) {
    ctx.strokeStyle = COLORS.axis;
    ctx.beginPath();
    ctx.moveTo(margin.left, y(0));
    ctx.lineTo(width - margin.right, y(0));
    ctx.stroke();
  }

  ctx.fillStyle = COLORS.muted;
  ctx.textAlign = "left";
  ctx.textBaseline = "top";
  ctx.fillText(yUnit, 7, 2);
  ctx.textAlign = "right";
  ctx.fillText("t [s]", width - margin.right, height - 17);

  for (const item of series) {
    ctx.strokeStyle = item.color;
    ctx.lineWidth = item.width || 1.8;
    ctx.setLineDash(item.dash || []);
    ctx.beginPath();
    data.forEach((row, index) => {
      const xx = x(row.t);
      const yy = y(item.value(row));
      if (index === 0) ctx.moveTo(xx, yy);
      else ctx.lineTo(xx, yy);
    });
    ctx.stroke();
    ctx.setLineDash([]);
  }
}

function renderCharts(result) {
  const d = result.data;
  drawChart($("currentChart"), d, [
    { value: (r) => r.ia, color: COLORS.cyan },
    { value: (r) => r.ib, color: COLORS.amber }
  ], "A");
  drawChart($("positionChart"), d, [
    { value: (r) => r.thetaReference * 180 / Math.PI, color: COLORS.violet, dash: [6, 4], width: 1.5 },
    { value: (r) => r.theta * 180 / Math.PI, color: COLORS.cyan, width: 2.1 }
  ], "deg");
  drawChart($("speedChart"), d, [
    { value: (r) => r.omegaRpm, color: COLORS.cyan, width: 2 }
  ], "rpm");
  drawChart($("torqueChart"), d, [
    { value: (r) => r.torque, color: COLORS.rose, width: 1.9 },
    { value: (r) => r.loadTorque, color: COLORS.amber, dash: [5, 4], width: 1.5 }
  ], "N·m");
}

function polarPoint(cx, cy, radius, angle) {
  return [cx + radius * Math.cos(angle), cy - radius * Math.sin(angle)];
}

function roundedRect(ctx, x, y, w, h, r) {
  const rr = Math.min(r, w / 2, h / 2);
  ctx.beginPath();
  ctx.moveTo(x + rr, y);
  ctx.arcTo(x + w, y, x + w, y + h, rr);
  ctx.arcTo(x + w, y + h, x, y + h, rr);
  ctx.arcTo(x, y + h, x, y, rr);
  ctx.arcTo(x, y, x + w, y, rr);
  ctx.closePath();
}

function drawArrow(ctx, cx, cy, radius, angle, color, label) {
  const [ex, ey] = polarPoint(cx, cy, radius, angle);
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(ex, ey);
  ctx.stroke();
  const head = 9;
  ctx.beginPath();
  ctx.moveTo(ex, ey);
  ctx.lineTo(ex - head * Math.cos(angle - 0.42), ey + head * Math.sin(angle - 0.42));
  ctx.lineTo(ex - head * Math.cos(angle + 0.42), ey + head * Math.sin(angle + 0.42));
  ctx.closePath();
  ctx.fill();
  if (label) {
    ctx.font = "700 11px ui-sans-serif, system-ui";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(label, ex + 13 * Math.cos(angle), ey - 13 * Math.sin(angle));
  }
}

function drawMotor(sample, p) {
  const canvas = $("motorCanvas");
  const { ctx, width, height } = prepareCanvas(canvas, canvas.clientWidth || 420);
  ctx.clearRect(0, 0, width, height);
  const cx = width / 2;
  const cy = height / 2;
  const base = Math.min(width, height);
  const outer = base * 0.42;
  const rotorOuter = base * 0.245;
  const rotorInner = base * 0.145;

  const gradient = ctx.createRadialGradient(cx, cy, rotorInner, cx, cy, outer * 1.1);
  gradient.addColorStop(0, "rgba(67,217,211,0.07)");
  gradient.addColorStop(0.55, "rgba(8,26,38,0.18)");
  gradient.addColorStop(1, "rgba(1,8,13,0)");
  ctx.fillStyle = gradient;
  ctx.beginPath();
  ctx.arc(cx, cy, outer * 1.14, 0, 2 * Math.PI);
  ctx.fill();

  ctx.strokeStyle = "rgba(164,203,217,0.28)";
  ctx.lineWidth = Math.max(10, base * 0.035);
  ctx.beginPath();
  ctx.arc(cx, cy, outer, 0, 2 * Math.PI);
  ctx.stroke();

  const phaseA = clamp(Math.abs(sample.ia) / Math.max(0.1, p.currentLimit), 0, 1.15);
  const phaseB = clamp(Math.abs(sample.ib) / Math.max(0.1, p.currentLimit), 0, 1.15);
  const poles = [
    { angle: Math.PI / 2, label: "A+", phase: phaseA, current: sample.ia, color: COLORS.cyan },
    { angle: 0, label: "B+", phase: phaseB, current: sample.ib, color: COLORS.amber },
    { angle: -Math.PI / 2, label: "A−", phase: phaseA, current: -sample.ia, color: COLORS.cyan },
    { angle: Math.PI, label: "B−", phase: phaseB, current: -sample.ib, color: COLORS.amber }
  ];

  for (const pole of poles) {
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(-pole.angle + Math.PI / 2);
    const pw = base * 0.145;
    const ph = base * 0.16;
    const py = -outer + base * 0.008;
    const intensity = clamp(pole.phase, 0, 1);
    ctx.shadowColor = pole.color;
    ctx.shadowBlur = 9 + 17 * intensity;
    ctx.fillStyle = pole.current >= 0
      ? `rgba(${pole.color === COLORS.cyan ? "67,217,211" : "255,182,95"},${0.18 + 0.5 * intensity})`
      : `rgba(173,145,255,${0.18 + 0.46 * intensity})`;
    ctx.strokeStyle = pole.color;
    ctx.lineWidth = 1.4;
    roundedRect(ctx, -pw / 2, py, pw, ph, 8);
    ctx.fill();
    ctx.shadowBlur = 0;
    ctx.stroke();
    ctx.restore();

    const [lx, ly] = polarPoint(cx, cy, outer * 1.08, pole.angle);
    ctx.fillStyle = COLORS.text;
    ctx.font = `800 ${Math.max(11, base * 0.026)}px ui-sans-serif, system-ui`;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(pole.label, lx, ly);
  }

  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(-sample.theta);
  const visualTeeth = clamp(Math.round(p.rotorTeeth), 15, 72);
  ctx.beginPath();
  for (let i = 0; i < visualTeeth * 2; i += 1) {
    const angle = (i / (visualTeeth * 2)) * 2 * Math.PI;
    const radius = i % 2 === 0 ? rotorOuter : rotorOuter * 0.90;
    const x = radius * Math.cos(angle);
    const y = radius * Math.sin(angle);
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  }
  ctx.closePath();
  const rotorGrad = ctx.createRadialGradient(0, 0, rotorInner * 0.25, 0, 0, rotorOuter);
  rotorGrad.addColorStop(0, "#2a4d5f");
  rotorGrad.addColorStop(0.7, "#173445");
  rotorGrad.addColorStop(1, "#87a4ae");
  ctx.fillStyle = rotorGrad;
  ctx.strokeStyle = "#bdd1d6";
  ctx.lineWidth = 1.4;
  ctx.fill();
  ctx.stroke();

  ctx.save();
  ctx.beginPath();
  ctx.arc(0, 0, rotorInner, -Math.PI / 2, Math.PI / 2);
  ctx.lineTo(0, -rotorInner);
  ctx.fillStyle = "#ff718a";
  ctx.fill();
  ctx.beginPath();
  ctx.arc(0, 0, rotorInner, Math.PI / 2, 3 * Math.PI / 2);
  ctx.lineTo(0, rotorInner);
  ctx.fillStyle = "#5aa9ff";
  ctx.fill();
  ctx.fillStyle = "white";
  ctx.font = `800 ${Math.max(11, base * 0.029)}px ui-sans-serif, system-ui`;
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText("N", rotorInner * 0.48, 0);
  ctx.fillText("S", -rotorInner * 0.48, 0);
  ctx.restore();

  ctx.beginPath();
  ctx.arc(0, 0, rotorInner * 0.27, 0, 2 * Math.PI);
  ctx.fillStyle = "#06111b";
  ctx.fill();
  ctx.strokeStyle = "rgba(236,248,250,0.55)";
  ctx.stroke();
  ctx.restore();

  const fieldMagnitude = Math.hypot(sample.ia, sample.ib);
  if (fieldMagnitude > 0.03) {
    const fieldAngle = Math.atan2(sample.ib, sample.ia);
    drawArrow(ctx, cx, cy, rotorInner * 0.82, fieldAngle, "rgba(255,255,255,0.82)", "B");
  }
  drawArrow(ctx, cx, cy, rotorOuter * 1.22, sample.thetaReference, COLORS.violet, "θref");

  ctx.fillStyle = COLORS.muted;
  ctx.font = `600 ${Math.max(10, base * 0.022)}px ui-sans-serif, system-ui`;
  ctx.textAlign = "left";
  ctx.textBaseline = "bottom";
  ctx.fillText(`${p.rotorTeeth} rotor teeth · full step ${(90 / p.rotorTeeth).toFixed(3)}°`, 12, height - 10);
}

function drawExternalMotor(sample, p) {
  const canvas = $("externalMotorCanvas");
  const { ctx, width, height } = prepareCanvas(canvas, canvas.clientWidth || 420);
  ctx.clearRect(0, 0, width, height);

  const base = Math.min(width, height);
  const faceSize = base * 0.50;
  const frontX = width * 0.16;
  const frontY = height * 0.28;
  const depthX = base * 0.22;
  const depthY = -base * 0.10;
  const centerX = frontX + faceSize / 2;
  const centerY = frontY + faceSize / 2;

  const backdrop = ctx.createRadialGradient(centerX, centerY, base * 0.06, centerX, centerY, base * 0.58);
  backdrop.addColorStop(0, "rgba(67,217,211,0.11)");
  backdrop.addColorStop(0.6, "rgba(8,28,40,0.12)");
  backdrop.addColorStop(1, "rgba(1,8,13,0)");
  ctx.fillStyle = backdrop;
  ctx.fillRect(0, 0, width, height);

  const topFace = [
    [frontX, frontY],
    [frontX + depthX, frontY + depthY],
    [frontX + faceSize + depthX, frontY + depthY],
    [frontX + faceSize, frontY]
  ];
  const sideFace = [
    [frontX + faceSize, frontY],
    [frontX + faceSize + depthX, frontY + depthY],
    [frontX + faceSize + depthX, frontY + faceSize + depthY],
    [frontX + faceSize, frontY + faceSize]
  ];
  const drawPolygon = (points, fill, stroke) => {
    ctx.beginPath();
    points.forEach(([x, y], index) => index ? ctx.lineTo(x, y) : ctx.moveTo(x, y));
    ctx.closePath();
    ctx.fillStyle = fill;
    ctx.fill();
    ctx.strokeStyle = stroke;
    ctx.lineWidth = 1.2;
    ctx.stroke();
  };

  drawPolygon(topFace, "#294858", "rgba(190,216,223,0.42)");
  drawPolygon(sideFace, "#112d3d", "rgba(190,216,223,0.38)");

  ctx.save();
  ctx.beginPath();
  ctx.moveTo(frontX + faceSize + 4, frontY + 7);
  ctx.lineTo(frontX + faceSize + depthX - 5, frontY + depthY + 7);
  ctx.lineTo(frontX + faceSize + depthX - 5, frontY + faceSize + depthY - 7);
  ctx.lineTo(frontX + faceSize + 4, frontY + faceSize - 7);
  ctx.clip();
  for (let i = 0; i < 7; i += 1) {
    const x = frontX + faceSize + 13 + i * depthX / 7;
    ctx.strokeStyle = i % 2 ? "rgba(67,217,211,0.16)" : "rgba(204,226,232,0.16)";
    ctx.lineWidth = Math.max(2, base * 0.009);
    ctx.beginPath();
    ctx.moveTo(x, frontY + depthY + 4);
    ctx.lineTo(x - depthY, frontY + faceSize + depthY - 4);
    ctx.stroke();
  }
  ctx.restore();

  const faceGradient = ctx.createLinearGradient(frontX, frontY, frontX + faceSize, frontY + faceSize);
  faceGradient.addColorStop(0, "#355565");
  faceGradient.addColorStop(0.52, "#183748");
  faceGradient.addColorStop(1, "#0d2635");
  roundedRect(ctx, frontX, frontY, faceSize, faceSize, base * 0.045);
  ctx.fillStyle = faceGradient;
  ctx.fill();
  ctx.strokeStyle = "rgba(214,235,240,0.55)";
  ctx.lineWidth = 1.4;
  ctx.stroke();

  const boltOffset = faceSize * 0.13;
  for (const [bx, by] of [
    [frontX + boltOffset, frontY + boltOffset],
    [frontX + faceSize - boltOffset, frontY + boltOffset],
    [frontX + boltOffset, frontY + faceSize - boltOffset],
    [frontX + faceSize - boltOffset, frontY + faceSize - boltOffset]
  ]) {
    ctx.beginPath();
    ctx.arc(bx, by, base * 0.018, 0, 2 * Math.PI);
    ctx.fillStyle = "#b8cbd0";
    ctx.fill();
    ctx.strokeStyle = "#07151f";
    ctx.lineWidth = 1;
    ctx.stroke();
  }

  const phaseA = clamp(Math.abs(sample.ia) / Math.max(0.1, p.currentLimit), 0, 1.15);
  const phaseB = clamp(Math.abs(sample.ib) / Math.max(0.1, p.currentLimit), 0, 1.15);
  const indicators = [
    { angle: Math.PI / 2, label: "A+", level: phaseA, current: sample.ia, rgb: "67,217,211" },
    { angle: 0, label: "B+", level: phaseB, current: sample.ib, rgb: "255,182,95" },
    { angle: -Math.PI / 2, label: "A−", level: phaseA, current: -sample.ia, rgb: "67,217,211" },
    { angle: Math.PI, label: "B−", level: phaseB, current: -sample.ib, rgb: "255,182,95" }
  ];
  const indicatorRadius = faceSize * 0.34;
  for (const indicator of indicators) {
    const [x, y] = polarPoint(centerX, centerY, indicatorRadius, indicator.angle);
    const color = indicator.current >= 0 ? indicator.rgb : "173,145,255";
    ctx.save();
    ctx.shadowColor = `rgb(${color})`;
    ctx.shadowBlur = 5 + 15 * indicator.level;
    ctx.beginPath();
    ctx.arc(x, y, base * (0.018 + 0.010 * indicator.level), 0, 2 * Math.PI);
    ctx.fillStyle = `rgba(${color},${0.32 + 0.56 * indicator.level})`;
    ctx.fill();
    ctx.restore();
    ctx.fillStyle = "rgba(236,248,250,0.76)";
    ctx.font = `800 ${Math.max(8, base * 0.022)}px ui-sans-serif, system-ui`;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(indicator.label, x, y - base * 0.042);
  }

  ctx.beginPath();
  ctx.arc(centerX, centerY, faceSize * 0.245, 0, 2 * Math.PI);
  ctx.fillStyle = "#718c96";
  ctx.fill();
  ctx.strokeStyle = "#d2e1e5";
  ctx.lineWidth = 1.5;
  ctx.stroke();

  const shaftX = centerX - base * 0.075;
  const shaftY = centerY + base * 0.038;
  ctx.strokeStyle = "#9fb5bc";
  ctx.lineWidth = base * 0.09;
  ctx.lineCap = "round";
  ctx.beginPath();
  ctx.moveTo(centerX, centerY);
  ctx.lineTo(shaftX, shaftY);
  ctx.stroke();
  ctx.lineCap = "butt";

  ctx.save();
  ctx.translate(shaftX, shaftY);
  ctx.rotate(-sample.theta);
  const shaftGradient = ctx.createRadialGradient(-base * 0.015, -base * 0.018, 1, 0, 0, base * 0.07);
  shaftGradient.addColorStop(0, "#eef7f8");
  shaftGradient.addColorStop(0.55, "#91aab1");
  shaftGradient.addColorStop(1, "#415e68");
  ctx.beginPath();
  ctx.arc(0, 0, base * 0.066, 0, 2 * Math.PI);
  ctx.fillStyle = shaftGradient;
  ctx.fill();
  ctx.strokeStyle = "#dcebed";
  ctx.lineWidth = 1.2;
  ctx.stroke();
  ctx.strokeStyle = COLORS.rose;
  ctx.lineWidth = Math.max(2, base * 0.009);
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(base * 0.052, 0);
  ctx.stroke();
  ctx.beginPath();
  ctx.arc(base * 0.047, 0, base * 0.012, 0, 2 * Math.PI);
  ctx.fillStyle = COLORS.rose;
  ctx.fill();
  ctx.restore();

  const direction = sample.omegaRpm > 0.05 ? "CCW" : sample.omegaRpm < -0.05 ? "CW" : "HOLD";
  ctx.fillStyle = COLORS.text;
  ctx.font = `800 ${Math.max(11, base * 0.032)}px ui-sans-serif, system-ui`;
  ctx.textAlign = "left";
  ctx.textBaseline = "bottom";
  ctx.fillText(`${direction}  ·  ${formatSigned(sample.theta * 180 / Math.PI, 1)}°`, 12, height - 28);
  ctx.fillStyle = COLORS.muted;
  ctx.font = `600 ${Math.max(9, base * 0.022)}px ui-sans-serif, system-ui`;
  ctx.fillText("shaft angle follows calculated θ(t)", 12, height - 10);
}

function updateLiveReadout(sample) {
  $("liveTime").textContent = `${sample.t.toFixed(3)} s`;
  $("liveAngle").textContent = `${formatSigned(sample.theta * 180 / Math.PI, 2)}°`;
  $("liveSpeed").textContent = `${formatSigned(sample.omegaRpm, 1)} rpm`;
  $("liveTorque").textContent = `${formatSigned(sample.torque, 3)} N·m`;
}

function showFrame(index) {
  if (!simulation) return;
  playbackIndex = clamp(Math.round(index), 0, simulation.data.length - 1);
  const sample = simulation.data[playbackIndex];
  drawMotor(sample, simulation.parameters);
  drawExternalMotor(sample, simulation.parameters);
  updateLiveReadout(sample);
  $("timeline").value = String(playbackIndex);
}

function pausePlayback() {
  isPlaying = false;
  $("playPause").textContent = "▶";
  $("playPause").setAttribute("aria-label", "تشغيل الحركة");
  if (animationFrame) cancelAnimationFrame(animationFrame);
  animationFrame = null;
}

function playbackLoop(now) {
  if (!isPlaying || !simulation) return;
  const factor = Number($("playbackSpeed").value);
  const elapsedWall = (now - playbackAnchor) / 1000;
  const targetTime = simulationTimeAnchor + elapsedWall * factor;
  const data = simulation.data;
  while (playbackIndex < data.length - 1 && data[playbackIndex].t < targetTime) playbackIndex += 1;
  showFrame(playbackIndex);
  if (playbackIndex >= data.length - 1) {
    pausePlayback();
    return;
  }
  animationFrame = requestAnimationFrame(playbackLoop);
}

function startPlayback(restartIfFinished = true) {
  if (!simulation) return;
  if (restartIfFinished && playbackIndex >= simulation.data.length - 1) playbackIndex = 0;
  isPlaying = true;
  $("playPause").textContent = "❚❚";
  $("playPause").setAttribute("aria-label", "إيقاف الحركة مؤقتاً");
  playbackAnchor = performance.now();
  simulationTimeAnchor = simulation.data[playbackIndex].t;
  animationFrame = requestAnimationFrame(playbackLoop);
}

function runFromForm() {
  const form = $("simForm");
  if (!form.reportValidity()) return;
  const p = readParameters();
  const error = validateParameters(p);
  const errorBox = $("formError");
  if (error) {
    errorBox.textContent = error;
    errorBox.hidden = false;
    return;
  }
  errorBox.hidden = true;
  pausePlayback();
  $("runButton").disabled = true;
  $("runButton").innerHTML = "جاري الحساب…";
  setStatus("neutral", "جاري حل المعادلات");

  window.setTimeout(() => {
    try {
      finishSimulation(p, true);
    } catch (simulationError) {
      errorBox.textContent = simulationError.message || "تعذّر تشغيل النموذج بهذه القيم.";
      errorBox.hidden = false;
      setStatus("bad", "تعذّر التشغيل");
    } finally {
      $("runButton").disabled = false;
      $("runButton").innerHTML = '<span class="run-icon" aria-hidden="true">▶</span>شغّلي المحاكاة';
    }
  }, 30);
}

function finishSimulation(p, autoplay) {
  pausePlayback();
  simulation = simulate(p);
  updateMetrics(simulation);
  renderCharts(simulation);
  $("timeline").min = "0";
  $("timeline").max = String(simulation.data.length - 1);
  $("timeline").value = "0";
  $("timeline").disabled = false;
  $("playPause").disabled = false;
  $("downloadCsv").disabled = false;
  playbackIndex = 0;
  showFrame(0);
  if (autoplay) startPlayback(false);
  return simulation;
}

function renderDatasheetStatus(kind, title, message, fields = [], sourceUrl = null) {
  const box = $("datasheetStatus");
  box.className = `datasheet-status ${kind}`;
  box.replaceChildren();

  const heading = document.createElement("strong");
  heading.textContent = title;
  box.appendChild(heading);

  const paragraph = document.createElement("p");
  paragraph.textContent = message;
  box.appendChild(paragraph);

  if (fields.length) {
    const list = document.createElement("ul");
    for (const field of fields) {
      const item = document.createElement("li");
      item.textContent = field;
      list.appendChild(item);
    }
    box.appendChild(list);
  }

  if (sourceUrl) {
    const link = document.createElement("a");
    link.href = sourceUrl;
    link.target = "_blank";
    link.rel = "noopener noreferrer";
    link.textContent = "Official manufacturer source · مصدر المصنّع الرسمي ↗";
    box.appendChild(link);
  }
}

function renderProjectDatasheetStatus() {
  renderDatasheetStatus(
    "neutral",
    "Supplied project profile · البروفايل الأساسي",
    "قيم النموذج الأساسية وليست قياسات مختبرية لهذا الماطور. ارفعي CSV/JSON للتعبئة التلقائية، أو PDF/صورة كمرجع للمراجعة.",
    ["I 3.4 A", "R 0.5 Ω", "L 2.0 mH", "Kt 0.575 N·m/A", "step 1.8°"]
  );
}

function applyMotorValues(values) {
  for (const [id, value] of Object.entries(values)) {
    const input = $(id);
    if (input && Number.isFinite(value)) input.value = String(Number(value.toPrecision(8)));
  }
  updateDerivedValues();
}

function applyMotorProfile(profileKey, rerun = true) {
  if (profileKey === "custom") {
    datasheetMeta = { name: "Custom datasheet", kind: "custom", importedFields: [] };
    renderDatasheetStatus(
      "neutral",
      "Custom datasheet · ملف مخصص",
      "ارفعي ملف CSV/JSON أو أدخلي القيم يدوياً. الملفات تبقى داخل المتصفح ولا تُرفع من هذه الصفحة."
    );
    return;
  }

  const profile = motorProfiles[profileKey];
  if (!profile) return;
  applyMotorValues(profile.values);
  datasheetMeta = { name: profile.name, kind: profileKey, importedFields: Object.keys(profile.values) };

  if (profileKey === "stepperonline23HE30") {
    renderDatasheetStatus(
      "good",
      "Verified motor profile · بروفايل موثّق",
      "الموثّق من المصنّع: زاوية الخطوة، عزم التثبيت، التيار، المقاومة والمحاثة. Kt مشتق من 2.0÷2.8؛ أما J والتخميد وDetent والـbandwidth فتبقى تقديرات، و24V إعداد Driver افتراضي إلى أن تحدديه.",
      ["1.8° step", "2.0 N·m holding", "2.8 A rated", "1.17 Ω", "4.8 mH", "Kt ≈ 0.714 N·m/A"],
      OFFICIAL_DATASHEET_URL
    );
  } else {
    renderProjectDatasheetStatus();
  }

  if (rerun) runFromForm();
}

function parseDelimitedLine(line, delimiter) {
  const cells = [];
  let value = "";
  let quoted = false;
  for (let i = 0; i < line.length; i += 1) {
    const char = line[i];
    if (char === '"') {
      if (quoted && line[i + 1] === '"') {
        value += '"';
        i += 1;
      } else {
        quoted = !quoted;
      }
    } else if (char === delimiter && !quoted) {
      cells.push(value.trim());
      value = "";
    } else {
      value += char;
    }
  }
  cells.push(value.trim());
  return cells;
}

function parseDatasheetCsv(text) {
  const lines = text.replace(/^\uFEFF/, "").split(/\r?\n/).filter((line) => line.trim());
  if (lines[0]?.toLowerCase().startsWith("sep=")) lines.shift();
  if (lines.length < 2) throw new Error("ملف CSV يحتاج صف عناوين وصف قيم واحد على الأقل.");
  const commaCount = (lines[0].match(/,/g) || []).length;
  const semicolonCount = (lines[0].match(/;/g) || []).length;
  const delimiter = semicolonCount > commaCount ? ";" : ",";
  const headers = parseDelimitedLine(lines[0], delimiter);
  const values = parseDelimitedLine(lines[1], delimiter);
  return Object.fromEntries(headers.map((header, index) => [header, values[index] ?? ""]));
}

function normalizeImportKey(key) {
  return String(key).toLowerCase().replace(/[µμ]/g, "micro").replace(/[^a-z0-9]/g, "");
}

function normalizeDatasheetObject(input) {
  if (!input || typeof input !== "object" || Array.isArray(input)) {
    throw new Error("صيغة JSON يجب أن تكون object يحتوي حقول الداتا شيت.");
  }
  const nested = input.parameters && typeof input.parameters === "object" && !Array.isArray(input.parameters)
    ? { ...input, ...input.parameters }
    : input;
  const keyMap = {
    model: "model",
    motorname: "model",
    stepangle: "stepAngleDeg",
    stepangledeg: "stepAngleDeg",
    fullstepangledeg: "stepAngleDeg",
    holdingtorque: "holdingTorqueNm",
    holdingtorquenm: "holdingTorqueNm",
    ratedcurrent: "ratedCurrentA",
    ratedcurrenta: "ratedCurrentA",
    currenta: "ratedCurrentA",
    phaseresistance: "phaseResistanceOhm",
    phaseresistanceohm: "phaseResistanceOhm",
    resistanceohm: "phaseResistanceOhm",
    phaseinductance: "phaseInductanceMh",
    phaseinductancemh: "phaseInductanceMh",
    inductancemh: "phaseInductanceMh",
    rotorinertiamicrokgm2: "rotorInertiaMicroKgm2",
    inertiamicrokgm2: "rotorInertiaMicroKgm2",
    detenttorque: "detentTorqueNm",
    detenttorquenm: "detentTorqueNm",
    damping: "dampingNmsRad",
    dampingnmsrad: "dampingNmsRad",
    supplyvoltage: "supplyVoltageV",
    supplyvoltagev: "supplyVoltageV",
    voltagev: "supplyVoltageV",
    torqueconstant: "torqueConstantNmA",
    torqueconstantnma: "torqueConstantNmA",
    bandwidth: "bandwidthHz",
    bandwidthhz: "bandwidthHz",
    rotorteeth: "rotorTeeth"
  };
  const normalized = {};
  for (const [key, value] of Object.entries(nested)) {
    const mapped = keyMap[normalizeImportKey(key)];
    if (mapped) normalized[mapped] = value;
  }
  return normalized;
}

function numericImportValue(value, label) {
  if (value === undefined || value === null || String(value).trim() === "") return null;
  const normalized = typeof value === "string" && value.includes(",") && !value.includes(".")
    ? value.replace(",", ".")
    : value;
  const number = Number(normalized);
  if (!Number.isFinite(number)) throw new Error(`${label}: القيمة ليست رقماً صالحاً.`);
  return number;
}

function applyImportedDatasheet(rawData, filename) {
  const imported = normalizeDatasheetObject(rawData);
  const specs = [
    ["ratedCurrentA", "currentLimit", "Rated current", 0.2, 8],
    ["phaseResistanceOhm", "resistance", "Phase resistance", 0.1, 5],
    ["phaseInductanceMh", "inductance", "Phase inductance", 0.2, 20],
    ["rotorInertiaMicroKgm2", "inertia", "Rotor inertia", 5, 500],
    ["detentTorqueNm", "detentTorque", "Detent torque", 0, 0.5],
    ["dampingNmsRad", "damping", "Viscous damping", 0, 0.2],
    ["supplyVoltageV", "voltage", "Supply voltage", 6, 60],
    ["torqueConstantNmA", "torqueConstant", "Torque constant", 0.05, 1.5],
    ["holdingTorqueNm", "holdingTorque", "Holding torque", 0.05, 12],
    ["bandwidthHz", "bandwidth", "Current bandwidth", 100, 2500],
    ["rotorTeeth", "rotorTeeth", "Rotor teeth", 15, 100]
  ];
  const pending = [];
  const labels = [];

  for (const [sourceKey, inputId, label, min, max] of specs) {
    const value = numericImportValue(imported[sourceKey], label);
    if (value === null) continue;
    if (value < min || value > max) throw new Error(`${label}: يجب أن تكون القيمة بين ${min} و${max}.`);
    pending.push([inputId, value]);
    labels.push(label);
  }

  const stepAngle = numericImportValue(imported.stepAngleDeg, "Step angle");
  if (stepAngle !== null) {
    if (stepAngle < 0.9 || stepAngle > 6) throw new Error("Step angle: يجب أن تكون بين 0.9° و6°.");
    if (imported.rotorTeeth === undefined || String(imported.rotorTeeth).trim() === "") {
      pending.push(["rotorTeeth", Math.round(90 / stepAngle)]);
      labels.push("Step angle → rotor teeth");
    }
  }

  const hasTorqueConstant = pending.some(([id]) => id === "torqueConstant");
  const holdingTorque = numericImportValue(imported.holdingTorqueNm, "Holding torque");
  const ratedCurrent = numericImportValue(imported.ratedCurrentA, "Rated current");
  if (!hasTorqueConstant && holdingTorque !== null && ratedCurrent !== null && ratedCurrent > 0) {
    const derivedKt = holdingTorque / ratedCurrent;
    if (derivedKt >= 0.05 && derivedKt <= 1.5) {
      pending.push(["torqueConstant", derivedKt]);
      labels.push("Kt derived from holding torque/current");
    }
  }

  if (!pending.length) {
    throw new Error("ما لقيت حقول رقمية معروفة. استخدمي قالب CSV الموجود بالموقع.");
  }

  for (const [inputId, value] of pending) $(inputId).value = String(Number(value.toPrecision(8)));
  updateDerivedValues();
  $("motorProfile").value = "custom";
  const modelName = imported.model ? String(imported.model).trim().slice(0, 80) : filename;
  datasheetMeta = { name: modelName || filename, kind: "imported", importedFields: [...new Set(labels)] };
  const dynamicMissing = ["rotorInertiaMicroKgm2", "dampingNmsRad", "detentTorqueNm", "bandwidthHz"]
    .filter((key) => imported[key] === undefined || String(imported[key]).trim() === "").length;
  renderDatasheetStatus(
    "good",
    `Imported: ${datasheetMeta.name}`,
    `تمت تعبئة ${new Set(labels).size} حقول. القيم غير الموجودة بقيت كما هي${dynamicMissing ? "، ومنها بعض المعلمات الديناميكية التقديرية" : ""}. راجعي الأرقام قبل اعتماد النتيجة.`,
    [...new Set(labels)]
  );
  runFromForm();
}

async function handleDatasheetUpload(event) {
  const file = event.target.files?.[0];
  if (!file) return;
  if (file.size > 10 * 1024 * 1024) {
    renderDatasheetStatus("bad", "File too large · الملف كبير", "الحد الأقصى 10 MB.");
    event.target.value = "";
    return;
  }

  const extension = file.name.split(".").pop()?.toLowerCase() || "";
  try {
    if (extension === "json" || file.type === "application/json") {
      const parsed = JSON.parse(await file.text());
      applyImportedDatasheet(Array.isArray(parsed) ? parsed[0] : parsed, file.name);
    } else if (extension === "csv" || file.type === "text/csv") {
      applyImportedDatasheet(parseDatasheetCsv(await file.text()), file.name);
    } else if (extension === "pdf" || file.type === "application/pdf" || file.type.startsWith("image/")) {
      $("motorProfile").value = "custom";
      datasheetMeta = { name: file.name, kind: "reference", importedFields: [] };
      renderDatasheetStatus(
        "warn",
        `Reference selected: ${file.name}`,
        "تم اختيار الملف محلياً كمرجع فقط. ما أستخرج أرقام PDF/الصورة تلقائياً حتى ما أدخل قراءة خاطئة؛ انقلي القيم للحقول أو استخدمي قالب CSV/JSON. الملف لا يُرفع من الصفحة."
      );
    } else {
      throw new Error("الصيغ المدعومة: CSV، JSON، PDF، أو صورة.");
    }
  } catch (error) {
    renderDatasheetStatus("bad", "Import failed · تعذّر الاستيراد", error.message || "تعذّر قراءة الملف.");
  } finally {
    event.target.value = "";
  }
}

function downloadDatasheetTemplate() {
  const header = [
    "model", "stepAngleDeg", "holdingTorqueNm", "ratedCurrentA", "phaseResistanceOhm",
    "phaseInductanceMh", "rotorInertiaMicroKgm2", "detentTorqueNm", "dampingNmsRad",
    "supplyVoltageV", "torqueConstantNmA", "bandwidthHz"
  ];
  const example = ["My Motor", "1.8", "2.0", "2.8", "1.17", "4.8", "", "", "", "24", "", "700"];
  const blob = new Blob([[header.join(","), example.join(",")].join("\n")], { type: "text/csv;charset=utf-8" });
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = url;
  link.download = "hybrid_stepper_datasheet_template.csv";
  document.body.appendChild(link);
  link.click();
  link.remove();
  URL.revokeObjectURL(url);
}

function setDefaults() {
  for (const [key, value] of Object.entries(defaults)) {
    const node = $(key);
    if (!node) continue;
    if (node.type === "checkbox") node.checked = value;
    else node.value = String(value);
  }
  syncPulseFields();
  updateDerivedValues();
  $("motorProfile").value = "project";
  datasheetMeta = { name: motorProfiles.project.name, kind: "project", importedFields: [] };
  renderProjectDatasheetStatus();
  $("accelTimeOut").textContent = `${Number($("accelTime").value).toFixed(2)} s`;
  runFromForm();
}

function setOverloadPreset() {
  $("pulseEnabled").checked = true;
  $("pulseTorque").value = "2.60";
  $("pulseStart").value = "0.45";
  $("pulseDuration").value = "0.030";
  $("speedRpm").value = "35";
  syncPulseFields();
  runFromForm();
}

function syncPulseFields() {
  const enabled = $("pulseEnabled").checked;
  $("pulseFields").setAttribute("aria-disabled", String(!enabled));
  for (const id of ["pulseTorque", "pulseStart", "pulseDuration"]) $(id).disabled = !enabled;
}

function updateDerivedValues() {
  const teeth = Math.max(1, Math.round(readNumber("rotorTeeth") || 50));
  $("derivedStepAngle").textContent = `${(90 / teeth).toFixed(3)}°`;
}

function downloadCsv() {
  if (!simulation) return;
  const header = ["time_s", "ia_A", "ib_A", "theta_deg", "theta_ref_deg", "speed_rpm", "torque_Nm", "load_torque_Nm", "va_V", "vb_V", "position_error_steps"];
  const lines = [header.join(",")];
  for (const r of simulation.data) {
    lines.push([
      r.t, r.ia, r.ib, r.theta * 180 / Math.PI, r.thetaReference * 180 / Math.PI,
      r.omegaRpm, r.torque, r.loadTorque, r.va, r.vb, r.errorSteps
    ].map((v) => Number(v).toPrecision(10)).join(","));
  }
  const blob = new Blob([lines.join("\n")], { type: "text/csv;charset=utf-8" });
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = url;
  link.download = "hybrid_stepper_results.csv";
  document.body.appendChild(link);
  link.click();
  link.remove();
  URL.revokeObjectURL(url);
}

function resizeVisuals() {
  if (simulation) {
    showFrame(playbackIndex);
    renderCharts(simulation);
  } else {
    const initialSample = { t: 0, ia: 0, ib: 0, theta: 0, thetaReference: 0, omegaRpm: 0, torque: 0 };
    const parameters = readParameters();
    drawMotor(initialSample, parameters);
    drawExternalMotor(initialSample, parameters);
  }
}

function exposeWebTool() {
  const context = typeof document === "undefined" ? undefined : document.modelContext;
  if (!context?.registerTool) return;
  const lifecycle = new AbortController();
  try {
    void Promise.resolve(context.registerTool({
      name: "run_stepper_estimate",
      title: "Run stepper estimate",
      description: "Set the main operating values, run the visible two-phase hybrid stepper estimate, and return its key results.",
      inputSchema: {
        type: "object",
        properties: {
          mode: { type: "string", enum: ["full", "half", "micro16"] },
          speedRpm: { type: "number", minimum: 1, maximum: 300 },
          direction: { type: "integer", enum: [1, -1] },
          loadTorque: { type: "number", minimum: 0, maximum: 4 },
          duration: { type: "number", minimum: 0.2, maximum: 3 },
          voltage: { type: "number", minimum: 6, maximum: 60 },
          currentLimit: { type: "number", minimum: 0.2, maximum: 8 },
          rotorTeeth: { type: "integer", minimum: 15, maximum: 100 }
        },
        additionalProperties: false
      },
      annotations: { readOnlyHint: false, untrustedContentHint: false },
      execute: async (input) => {
        if (!input || typeof input !== "object" || Array.isArray(input)) throw new Error("Input must be an object.");
        const mapping = {
          mode: "mode", speedRpm: "speedRpm", direction: "direction", loadTorque: "loadTorque",
          duration: "duration", voltage: "voltage", currentLimit: "currentLimit", rotorTeeth: "rotorTeeth"
        };
        for (const [key, value] of Object.entries(input)) {
          if (!(key in mapping)) throw new Error(`Unsupported input: ${key}`);
          $(mapping[key]).value = String(value);
        }
        if (!$("simForm").reportValidity()) throw new Error("One or more inputs are outside their permitted range.");
        updateDerivedValues();
        const p = readParameters();
        const error = validateParameters(p);
        if (error) throw new Error(error);
        const result = finishSimulation(p, false);
        return {
          status: classifyResult(result).label,
          finalPositionDeg: Number(result.metrics.finalPositionDeg.toFixed(4)),
          averageSpeedRpm: Number(result.metrics.averageSpeed.toFixed(4)),
          finalErrorDeg: Number(result.metrics.finalErrorDeg.toFixed(4)),
          lostStepsEstimate: result.metrics.lostSteps,
          peakTorqueNm: Number(result.metrics.peakTorque.toFixed(4)),
          peakCurrentA: Number(result.metrics.peakCurrent.toFixed(4))
        };
      }
    }, { signal: lifecycle.signal })).catch(() => {});
  } catch (_) {
    // The page remains fully functional when WebMCP is unavailable.
  }
}

$("simForm").addEventListener("submit", (event) => {
  event.preventDefault();
  runFromForm();
});
$("resetDefaults").addEventListener("click", setDefaults);
$("overloadPreset").addEventListener("click", setOverloadPreset);
$("motorProfile").addEventListener("change", () => applyMotorProfile($("motorProfile").value));
$("datasheetFile").addEventListener("change", handleDatasheetUpload);
$("downloadTemplate").addEventListener("click", downloadDatasheetTemplate);
$("pulseEnabled").addEventListener("change", syncPulseFields);
$("rotorTeeth").addEventListener("input", updateDerivedValues);
$("holdingTorque").addEventListener("change", () => {
  const holdingTorque = readNumber("holdingTorque");
  const ratedCurrent = readNumber("currentLimit");
  const derivedKt = holdingTorque / ratedCurrent;
  if (Number.isFinite(derivedKt) && derivedKt >= 0.05 && derivedKt <= 1.5) {
    $("torqueConstant").value = String(Number(derivedKt.toPrecision(7)));
    $("motorProfile").value = "custom";
    datasheetMeta = { name: "Manual custom profile", kind: "custom", importedFields: ["Holding torque", "Kt derived"] };
    renderDatasheetStatus(
      "neutral",
      "Manual custom profile · بروفايل يدوي",
      `تم اشتقاق Kt = ${derivedKt.toFixed(4)} N·m/A من عزم التثبيت ÷ تيار الطور. شغّلي المحاكاة لتحديث النتائج.`,
      ["Holding torque", "Kt derived"]
    );
  }
});
for (const id of ["voltage", "currentLimit", "resistance", "inductance", "torqueConstant", "rotorTeeth", "inertia", "damping", "detentTorque", "bandwidth"]) {
  $(id).addEventListener("change", () => {
    if ($("motorProfile").value === "custom") return;
    $("motorProfile").value = "custom";
    datasheetMeta = { name: "Manual custom profile", kind: "custom", importedFields: ["Manually edited values"] };
    renderDatasheetStatus(
      "neutral",
      "Manual custom profile · بروفايل يدوي",
      "تم تعديل قيمة من معلمات الماطور يدوياً، لذلك لم يعد البروفايل مطابقاً للمصدر الجاهز. شغّلي المحاكاة بعد إكمال التعديلات.",
      ["Manually edited values"]
    );
  });
}
$("accelTime").addEventListener("input", () => {
  $("accelTimeOut").textContent = `${Number($("accelTime").value).toFixed(2)} s`;
});
$("playPause").addEventListener("click", () => {
  if (isPlaying) pausePlayback();
  else startPlayback(true);
});
$("timeline").addEventListener("input", () => {
  pausePlayback();
  showFrame(Number($("timeline").value));
});
$("downloadCsv").addEventListener("click", downloadCsv);

let resizeTimer = null;
window.addEventListener("resize", () => {
  window.clearTimeout(resizeTimer);
  resizeTimer = window.setTimeout(resizeVisuals, 120);
});

syncPulseFields();
updateDerivedValues();
renderProjectDatasheetStatus();
resizeVisuals();
exposeWebTool();
window.setTimeout(runFromForm, 80);
