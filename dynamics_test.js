// H2-X 오른팔 웹 시뮬레이터 동역학 검증:  node dynamics_test.js
// 기준값(ref_cases.json)은 URDF 원본으로 Python(numpy)에서 독립 계산한 정역학 토크.
"use strict";
const fs = require("fs");
const path = require("path");
const { Sim, Arm, Trajectory, staticPayloadTest, N, util } = require("./dynamics.js");
const model = JSON.parse(fs.readFileSync(path.join(__dirname, "model.json"), "utf8"));
const refs = JSON.parse(fs.readFileSync(path.join(__dirname, "tools", "ref_cases.json"), "utf8"));
const D = Math.PI / 180;
let fails = 0;
const check = (name, ok, detail = "") => { console.log(`${ok ? "PASS" : "FAIL"}  ${name} ${detail}`); if (!ok) fails++; };
const maxAbs = (a) => Math.max(...Array.from(a, Math.abs));

// 1) 정역학: JS RNEA vs Python 독립 계산 (18 케이스: 6자세 × 하중 0/1/2.5 kg · 팔 질량 ×1/×1.3)
{
  let worst = 0;
  for (const c of refs) {
    const arm = new Arm(model);
    arm.setArmMass(arm.armMass0 * c.scale);
    arm.setPayload(c.pay, c.off);
    const t = arm.gravityTorque(c.q);
    worst = Math.max(worst, maxAbs(t.map((v, i) => v - c.tau[i])));
  }
  check("정적 중력토크 = URDF 독립계산", worst < 1e-6, `(18건 최대 오차 ${worst.toExponential(2)} N·m)`);
}
// 2) 질량행렬 대칭·양정치, 에너지 보존 (중력+무제어 자유진동, dt 0.1 ms)
{
  const arm = new Arm(model); arm.setPayload(1.0, [0.12, 0, 0]);
  const q = [-0.5, -0.3, 0.4, 0.8, 0.3, 0.2, -0.3];
  const M = arm.massMatrix(q);
  let asym = 0; for (let i = 0; i < N; i++) for (let j = 0; j < N; j++) asym = Math.max(asym, Math.abs(M[i * N + j] - M[j * N + i]));
  const qd = [0.8, -0.5, 1.0, 0.6, -1.2, 0.9, 0.7];
  // 수치 검증: τ(q, qd, qdd) = M qdd + bias
  const qdd = [0.3, -0.2, 0.5, 0.1, -0.4, 0.2, 0.6];
  const t1 = arm.rnea(q, qd, qdd), b = arm.rnea(q, qd, new Float64Array(N));
  let lin = 0; for (let i = 0; i < N; i++) { let s = b[i]; for (let j = 0; j < N; j++) s += M[i * N + j] * qdd[j]; lin = Math.max(lin, Math.abs(s - t1[i])); }
  check("질량행렬 대칭 · RNEA 선형성", asym < 1e-9 && lin < 1e-9, `(비대칭 ${asym.toExponential(1)}, 선형 오차 ${lin.toExponential(1)})`);
  // 에너지: KE = ½ qdᵀMqd, PE = -Σ m g·c
  const energy = (q, qd) => {
    const M = arm.massMatrix(q); let ke = 0;
    for (let i = 0; i < N; i++) for (let j = 0; j < N; j++) ke += 0.5 * qd[i] * M[i * N + j] * qd[j];
    const F = arm.fk(q); let pe = 0;
    for (let i = 0; i < N; i++) { const c = util.add(F[i].p, util.mv(F[i].R, arm.bodies[i].c)); pe += arm.bodies[i].m * 9.81 * c[2]; }
    return ke + pe;
  };
  let qq = q.slice(), vv = Float64Array.from(qd); const E0 = energy(qq, vv); const dt = 1e-4; let drift = 0;
  for (let k = 0; k < 10000; k++) { // 1 s
    const M2 = arm.massMatrix(qq), bias = arm.rnea(qq, vv, new Float64Array(N));
    const a = util.cholSolve(M2, Array.from(bias, (v) => -v), N);
    for (let i = 0; i < N; i++) { vv[i] += a[i] * dt; qq[i] += vv[i] * dt; }
    if (k % 1000 === 999) drift = Math.max(drift, Math.abs(energy(qq, vv) - E0));
  }
  check("자유진동 1 s 에너지 보존", drift < 0.02 * Math.abs(E0) + 0.05, `(E0 ${E0.toFixed(3)} J, 최대 변화 ${drift.toFixed(4)} J)`);
}
// 3) 궤적 생성기
{
  const tr = new Trajectory([0, 0], [-3, -3], [3, 3]);
  const v = 60 * D, a = 240 * D; tr.setLimits([v, v], [a, a]); tr.setTarget([90 * D, -5 * D]);
  let mv = 0, ma = 0, mn = 1e9, mx = -1e9;
  for (let k = 0; k < 4000; k++) { if (k === 700) tr.setTarget([-40 * D, 20 * D]); tr.step(0.001);
    mv = Math.max(mv, Math.abs(tr.v[0]), Math.abs(tr.v[1])); ma = Math.max(ma, Math.abs(tr.a[0]), Math.abs(tr.a[1]));
    if (k > 700) { mn = Math.min(mn, tr.q[0]); mx = Math.max(mx, tr.q[1]); } }
  check("궤적 |v|≤60°/s, |a|≤240°/s², 과도 없음", mv <= v * (1 + 1e-9) && ma <= a * (1 + 1e-9) && mn >= -40 * D - 1e-9 && mx <= 20 * D + 1e-9,
    `(${(mv / D).toFixed(3)}°/s, ${(ma / D).toFixed(3)}°/s²)`);
}
// 4) 뻗은 자세 0 / 1 kg 정적 시험
const pose = [-90, 0, 0, 88, 0, 0, 0].map((v) => v * D);
{
  const r = staticPayloadTest(model, pose, 1.0, [0.12, 0, 0]);
  const nm = ["sh_pitch", "sh_roll", "sh_yaw", "elbow", "wr_roll", "wr_pitch", "wr_yaw"];
  console.log("   관절      τ0        τ1        Δτ시뮬    Δτ이론    오차      추종오차1kg[°]");
  for (let i = 0; i < N; i++) console.log(`   ${nm[i].padEnd(9)} ${r.tau0[i].toFixed(4).padStart(9)} ${r.tau1[i].toFixed(4).padStart(9)} ${r.dsim[i].toFixed(4).padStart(9)} ${r.theory[i].toFixed(4).padStart(9)} ${r.err[i].toExponential(1).padStart(9)} ${(r.track1[i] / D).toExponential(1).padStart(9)}`);
  check("어깨·팔꿈치 Δτ = 이론", Math.max(...r.err.slice(0, 4).map(Math.abs)) < 1e-3, `(어깨 pitch Δτ ${r.dsim[0].toFixed(4)} vs ${r.theory[0].toFixed(4)})`);
  check("어깨 pitch Δτ ≈ −5.669 (MuJoCo 버전 README 이론값)", Math.abs(r.dsim[0] + 5.6685) < 2e-3);
  check("정적 추종오차 < 0.01°", Math.max(...r.track0.concat(r.track1).map(Math.abs)) < 0.01 * D);
  check("포화 없음", !r.sat0.concat(r.sat1).some(Boolean));
}
// 5) 동적 계단 추종 (1 kg)
{
  const sim = new Sim(model); sim.setPayload(1.0, [0.12, 0, 0], false); sim.setTarget(pose);
  let e = 0, sat = false, vmax = 0;
  for (let k = 0; k < 4000; k++) { sim.step(1); for (let i = 0; i < N; i++) { e = Math.max(e, Math.abs(sim.traj.q[i] - sim.q[i])); vmax = Math.max(vmax, Math.abs(sim.traj.v[i])); } sat = sat || sim.sat.some(Boolean); }
  check("계단 추종: 오차 < 0.5°, 속도 ≤ 60°/s, 포화 없음", e < 0.5 * D && vmax <= 60 * D + 1e-9 && !sat, `(최대 오차 ${(e / D).toFixed(4)}°)`);
}
// 6) 과부하(20 kg @ 0.2 m): 폭주·떨림 없이 처짐, 가동범위 유지, 토크 = 한계
{
  const sim = new Sim(model); sim.setTarget(pose); sim.step(3000);
  sim.setPayload(20, [0.20, 0, 0]); // 1 s 램프 (한계 초과 하중)
  let flips = new Array(N).fill(0), prev = Array.from(sim.tauAct), outRange = 0, finite = true, maxV = 0;
  for (let k = 0; k < 6000; k++) {
    sim.step(1);
    for (let i = 0; i < N; i++) {
      if (Math.sign(sim.tauAct[i]) !== Math.sign(prev[i]) && Math.abs(sim.tauAct[i] - prev[i]) > 0.2 * sim.arm.effort[i]) flips[i]++;
      prev[i] = sim.tauAct[i];
      outRange = Math.max(outRange, sim.q[i] - sim.arm.upper[i], sim.arm.lower[i] - sim.q[i]);
      maxV = Math.max(maxV, Math.abs(sim.qd[i]));
      finite = finite && Number.isFinite(sim.q[i]);
    }
  }
  console.log(`   20 kg@0.2 m: 포화 ${sim.sat.map((s, i) => (s ? sim.names[i].replace("right_", "").replace("_joint", "") : "")).filter(Boolean).join(",") || "없음"}, 최대 속도 ${maxV.toFixed(2)} rad/s, 큰 토크 반전 ${flips.join("/")}`);
  check("과부하에서 발산 없음", finite);
  check("과부하에서 토크 떨림 없음 (한계폭 20% 이상 부호반전 ≤ 5회/관절)", Math.max(...flips) <= 5);
  check("가동범위 초과 < 2°", outRange < 2 * D, `(최대 ${(outRange / D).toFixed(3)}°)`);
  check("포화 관절 τ = 한계", sim.sat.every((s, i) => !s || Math.abs(Math.abs(sim.tauAct[i]) - sim.arm.effort[i]) < 1e-9) && sim.satLatched.some(Boolean));
}
// 7) 팔 질량 ×1.5 → 정적 토크 ×1.5
{
  const s1 = new Sim(model), s2 = new Sim(model); s2.setArmMass(s2.arm.armMass0 * 1.5);
  const a = s1.settle(pose), b = s2.settle(pose);
  const err = maxAbs(Array.from(b.tau, (v, i) => v - 1.5 * a.tau[i]));
  check("팔 질량 ×1.5 → 토크 ×1.5", err < 1e-3, `(어깨 pitch ${a.tau[0].toFixed(3)} → ${b.tau[0].toFixed(3)})`);
}
// 9) 몸 자세(허리 기울임): 오른팔 정적 토크 vs URDF 독립계산
{
  const refsW = JSON.parse(fs.readFileSync(path.join(__dirname, "tools", "ref_waist.json"), "utf8"));
  let worst = 0, worstSim = 0;
  for (const c of refsW) {
    const sim = new Sim(model); sim.setBodyPose(c.waist); sim.setPayload(1.0, [0.12, 0, 0], false);
    const q = c.arm_deg.map((v) => v * D);
    worst = Math.max(worst, maxAbs(sim.arm.gravityTorque(q).map((v, i) => v - c.tau[i])));
    sim.reset(q); const r = sim.settle(q, 2, 0.3);
    worstSim = Math.max(worstSim, maxAbs(Array.from(r.tau, (v, i) => v - c.tau[i])));
  }
  check("허리 기울임 자세 정적 토크 = URDF 독립계산", worst < 1e-6 && worstSim < 2e-3, `(정역학 ${worst.toExponential(1)}, 시뮬 정착 ${worstSim.toExponential(1)} N·m)`);
}
// 10) 충돌: 손·팔이 몸통을 뚫지 않음 (충돌 끄면 뚫리는 목표로 확인)
{
  const deep = (sim) => { // 비제외 쌍 중 최대 침투 깊이
    const F = sim.arm.fk(sim.q); const cs = sim._contacts(F); return cs.reduce((m, c) => Math.max(m, -c.d), 0);
  };
  const targets = [[-20, 19.6, 120, -56, 0, 0, 0], [-60, 15, 90, -50, 60, 30, 60], [-100, 19, 110, -40, 0, 30, 0], [10, 19.6, 0, 0, 0, 0, 0]];
  let worstOn = 0, worstOff = 0, nContact = 0;
  for (const tg of targets) {
    for (const col of [true, false]) {
      const sim = new Sim(model, { collision: col }); sim.setTarget(tg.map((v) => v * D));
      let w = 0;
      for (let k = 0; k < 5000; k++) { sim.step(1); if (k % 10 === 0) { const d = col ? deep(sim) : (() => { sim.cfg.collision = true; const x = deep(sim); sim.cfg.collision = false; return x; })(); w = Math.max(w, d); } if (col && sim.contacts.length) nContact++; }
      if (col) worstOn = Math.max(worstOn, w); else worstOff = Math.max(worstOff, w);
    }
  }
  check("충돌 켬: 최대 침투 < 5 mm (충돌 끄면 같은 목표에서 > 20 mm 뚫림)", worstOn < 0.005 && worstOff > 0.02,
    `(켬 ${(worstOn * 1000).toFixed(2)} mm, 끔 ${(worstOff * 1000).toFixed(1)} mm, 접촉 스텝 ${nContact})`);
}
// 11) 구동부 마찰·반사관성: 정지 토크 불변, 등속 구간에서 구동토크 − 출력축 토크 = 마찰
{
  const a = new Sim(model, { viscous: [0, 0, 0, 0, 0, 0, 0], coulomb: [0, 0, 0, 0, 0, 0, 0], armature: [0, 0, 0, 0, 0, 0, 0] });
  const b = new Sim(model);
  const ra = a.settle(pose, 3, 0.3), rb = b.settle(pose, 3, 0.3);
  const ds = maxAbs(Array.from(ra.tau, (v, i) => v - rb.tau[i]));
  const c = new Sim(model); c.setTarget([0, 0, 0, 60 * D, 0, 0, 0]); // 팔꿈치 60°/s 등속 구간
  let gap = 0, n = 0, fr = 0;
  for (let k = 0; k < 1000; k++) { c.step(1); if (k > 300 && k < 700) { gap += c.tauAct[3] - c.tauOut[3]; fr += c.cfg.viscous[3] * c.qd[3] + c.cfg.coulomb[3] * Math.tanh(c.qd[3] / c.cfg.vStick); n++; } }
  check("구동부 모델: 정지 토크 불변, 등속 시 (구동−출력) = 마찰", ds < 2e-3 && Math.abs(gap / n - fr / n) < 0.05,
    `(정지 차 ${ds.toExponential(1)}, 등속 차 ${(gap / n).toFixed(3)} vs 마찰 ${(fr / n).toFixed(3)} N·m)`);
}
// 8) 성능
{
  const sim = new Sim(model); sim.setTarget(pose); const t0 = Date.now(); sim.step(5000); const ms = Date.now() - t0;
  check("계산 속도 (실시간 대비)", ms < 5000, `(5 s 시뮬 ${ms} ms → ${(5000 / ms).toFixed(1)}×)`);
}
console.log(fails ? `\n실패 ${fails}건` : "\n모두 통과");
process.exit(fails ? 1 : 0);
