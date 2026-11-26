/* H2-X 오른팔 7축 강체 동역학 · 궤적 · 제어 (브라우저/Node 공용, 외부 라이브러리 없음)
 *
 * - 골반 고정, 오른팔 외 관절은 기립 자세(0 rad)로 고정 → 몸통(torso_link) 프레임이 고정 베이스
 * - 링크 질량·무게중심·관성텐서는 URDF 원본 (model.json)
 * - 역동역학: 재귀 뉴턴–오일러(RNEA), 질량행렬: 단위 가속도 RNEA 7회, 적분: 반암시적 오일러 1 ms
 * - 구동 토크 tauAct = 관절(감속기 출력축) 토크, 토크 한계 = URDF effort 로 클립
 * - 가동범위: 범위를 넘을 때만 작동하는 강성·감쇠 벽(제약 토크, 구동 토크와 별도 기록)
 */
(function (root, factory) {
  if (typeof module === "object" && module.exports) module.exports = factory();
  else root.H2XDyn = factory();
})(typeof self !== "undefined" ? self : this, function () {
  "use strict";
  const N = 7;
  const PAYLOAD_R = 0.03;

  // ---------- 3x3 / 3-vector 유틸 (행 우선 배열 길이 9) ----------
  const mm = (A, B) => {
    const C = new Array(9);
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++)
      C[3 * i + j] = A[3 * i] * B[j] + A[3 * i + 1] * B[3 + j] + A[3 * i + 2] * B[6 + j];
    return C;
  };
  const mv = (A, v) => [A[0] * v[0] + A[1] * v[1] + A[2] * v[2], A[3] * v[0] + A[4] * v[1] + A[5] * v[2], A[6] * v[0] + A[7] * v[1] + A[8] * v[2]];
  const tr = (A) => [A[0], A[3], A[6], A[1], A[4], A[7], A[2], A[5], A[8]];
  const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
  const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
  const scl = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  function axisAngle(a, t) {
    const c = Math.cos(t), s = Math.sin(t), v = 1 - c, [x, y, z] = a;
    return [c + x * x * v, x * y * v - z * s, x * z * v + y * s,
            y * x * v + z * s, c + y * y * v, y * z * v - x * s,
            z * x * v - y * s, z * y * v + x * s, c + z * z * v];
  }
  const clamp = (x, lo, hi) => Math.min(hi, Math.max(lo, x));

  // 대칭 양정치 행렬 풀이 (Cholesky)
  function cholSolve(A, b, n) {
    const L = new Float64Array(n * n);
    for (let i = 0; i < n; i++) for (let j = 0; j <= i; j++) {
      let s = A[i * n + j];
      for (let k = 0; k < j; k++) s -= L[i * n + k] * L[j * n + k];
      if (i === j) L[i * n + i] = Math.sqrt(Math.max(s, 1e-14));
      else L[i * n + j] = s / L[j * n + j];
    }
    const y = new Float64Array(n), x = new Float64Array(n);
    for (let i = 0; i < n; i++) { let s = b[i]; for (let k = 0; k < i; k++) s -= L[i * n + k] * y[k]; y[i] = s / L[i * n + i]; }
    for (let i = n - 1; i >= 0; i--) { let s = y[i]; for (let k = i + 1; k < n; k++) s -= L[k * n + i] * x[k]; x[i] = s / L[i * n + i]; }
    return x;
  }
  function invDiag(A, n) { // diag(A^-1)
    const d = new Float64Array(n);
    for (let j = 0; j < n; j++) { const e = new Float64Array(n); e[j] = 1; d[j] = cholSolve(A, e, n)[j]; }
    return d;
  }

  function invFull(A, n) {
    const X = new Float64Array(n * n);
    for (let j = 0; j < n; j++) { const e = new Float64Array(n); e[j] = 1; const c = cholSolve(A, e, n); for (let i = 0; i < n; i++) X[i * n + j] = c[i]; }
    return X;
  }
  /* 관절별 독립 피드백의 관성 스케일 D.
   * 기본은 diag(M). 폐루프는 q̈ = M⁻¹·D·(Kp e + Kd ė) 이므로 λ = eig(D½ M⁻¹ D½)가 클수록
   * 이산시간(1 ms)에서 불안정해진다. λmax ≤ lambdaMax 가 되도록 D를 일괄 축소한다.
   * 어떤 관절이 포화(개루프)돼도 남은 관절의 행렬은 주부분행렬이라 λ가 더 커지지 않는다. */
  function feedbackInertia(M, Minv, lambdaMax) {
    const D = Array.from({ length: N }, (_, i) => M[i * N + i]);
    const A = new Float64Array(N * N);
    for (let i = 0; i < N; i++) for (let j = 0; j < N; j++) A[i * N + j] = Math.sqrt(D[i] * D[j]) * Minv[i * N + j];
    let v = new Float64Array(N).fill(1), lam = 0;
    for (let it = 0; it < 60; it++) {
      const w = new Float64Array(N);
      for (let i = 0; i < N; i++) { let s = 0; for (let j = 0; j < N; j++) s += A[i * N + j] * v[j]; w[i] = s; }
      const nw = Math.hypot(...w); lam = nw / Math.hypot(...v); for (let i = 0; i < N; i++) v[i] = w[i] / nw;
    }
    const s = Math.min(1, lambdaMax / Math.max(lam * 1.02, 1e-12));
    return D.map((d) => d * s);
  }

  // ---------- 궤적 생성기 (Python traj.py와 동일 규칙) ----------
  class Trajectory {
    constructor(q0, lower, upper) {
      this.q = Float64Array.from(q0); this.v = new Float64Array(N); this.a = new Float64Array(N);
      this.target = Float64Array.from(q0); this.lower = lower; this.upper = upper;
      this.vmax = new Float64Array(N).fill(1); this.amax = new Float64Array(N).fill(1);
    }
    setLimits(vmax, amax) { for (let i = 0; i < N; i++) { this.vmax[i] = vmax[i]; this.amax[i] = amax[i]; } }
    setTarget(t) { for (let i = 0; i < N; i++) this.target[i] = clamp(t[i], this.lower[i], this.upper[i]); }
    reset(q) { for (let i = 0; i < N; i++) { this.q[i] = q[i]; this.v[i] = 0; this.a[i] = 0; this.target[i] = q[i]; } }
    step(dt) {
      for (let i = 0; i < N; i++) {
        const e = this.target[i] - this.q[i];
        const eEff = Math.max(Math.abs(e) - Math.abs(this.v[i]) * dt, 0);
        const vStar = Math.sign(e) * Math.min(this.vmax[i], Math.sqrt(2 * this.amax[i] * eEff));
        let a = clamp((vStar - this.v[i]) / dt, -this.amax[i], this.amax[i]);
        let vNew = this.v[i] + a * dt;
        let qNew = this.q[i] + 0.5 * (this.v[i] + vNew) * dt;
        const crossed = e !== 0 && Math.sign(this.target[i] - qNew) !== Math.sign(e);
        const near = Math.abs(e) < 1e-7 && Math.abs(this.v[i]) < this.amax[i] * dt * 1.5;
        if (crossed || near) { qNew = this.target[i]; a = -this.v[i] / dt; vNew = 0; }
        this.q[i] = clamp(qNew, this.lower[i], this.upper[i]); this.v[i] = vNew; this.a[i] = a;
      }
    }
  }

  // ---------- 팔 모델 ----------
  class Arm {
    constructor(model) {
      this.model = model;
      this.J = model.joints;
      this.lower = Float64Array.from(this.J.map((j) => j.lower));
      this.upper = Float64Array.from(this.J.map((j) => j.upper));
      this.effort = Float64Array.from(this.J.map((j) => j.effort));
      this.urdfVel = Float64Array.from(this.J.map((j) => j.velocity));
      this.torsoR = model.torso.R; this.torsoP = model.torso.p;
      this.g = model.gravity.slice();
      this.mass0 = this.J.map((j) => j.mass);
      this.armMass0 = this.mass0.reduce((a, b) => a + b, 0);
      this.armMass = this.armMass0;
      this.payload = { mass: 0, off: [0.12, 0, 0] };
      this.body = this.J.map((j) => ({ m: j.mass, c: j.com.slice(), I: j.inertia.slice() }));
      this._rebuildBodies();
    }
    // 팔 질량 변경: 링크별 질량·관성텐서 같은 비율, 무게중심 유지
    setArmMass(total) {
      if (!(total > 0)) throw new Error("팔 질량은 0보다 커야 합니다");
      this.armMass = total; this._rebuildBodies();
    }
    setPayload(mass, off) {
      if (!(mass >= 0)) throw new Error("하중 질량은 0 이상");
      this.payload = { mass, off: off.slice() }; this._rebuildBodies();
    }
    _rebuildBodies() {
      const k = this.armMass / this.armMass0;
      this.bodies = this.J.map((j) => ({ m: j.mass * k, c: j.com.slice(), I: j.inertia.map((v) => v * k) }));
      // 손 하중: 손 링크(7번)에 강체로 합침 (균질구 r=3 cm, 평행축 정리)
      const pm = this.payload.mass;
      if (pm > 0) {
        const b = this.bodies[N - 1], o = this.payload.off;
        const M = b.m + pm;
        const c = scl(add(scl(b.c, b.m), scl(o, pm)), 1 / M);
        const shift = (I, m, r) => { // 무게중심 관성 → 점 c 기준
          const d = sub(r, c), dd = dot(d, d);
          return I.map((v, idx) => v + m * ((idx % 4 === 0 ? dd : 0) - d[Math.floor(idx / 3)] * d[idx % 3]));
        };
        const Ip = [1, 0, 0, 0, 1, 0, 0, 0, 1].map((v) => v * 0.4 * pm * PAYLOAD_R * PAYLOAD_R);
        const Ib = shift(b.I, b.m, b.c), Ipp = shift(Ip, pm, o);
        this.bodies[N - 1] = { m: M, c, I: Ib.map((v, i) => v + Ipp[i]) };
      }
    }
    // 순기구학: 링크 프레임 R,p (월드), 관절축 z (월드)
    fk(q) {
      const out = [];
      let Rp = this.torsoR, pp = this.torsoP;
      for (let i = 0; i < N; i++) {
        const j = this.J[i];
        const R = mm(mm(Rp, j.R0), axisAngle(j.axis, q[i]));
        const p = add(pp, mv(Rp, j.xyz));
        out.push({ R, p, z: mv(R, j.axis) });
        Rp = R; pp = p;
      }
      return out;
    }
    // RNEA: τ = M(q)qdd + C(q,qd)qd + G(q)   (grav=false면 중력 제외)
    rnea(q, qd, qdd, grav = true, F = null) {
      F = F || this.fk(q);
      const w = [], al = [], ao = [], comW = [], Iw = [];
      let wp = [0, 0, 0], alp = [0, 0, 0], aop = grav ? scl(this.g, -1) : [0, 0, 0], pp = this.torsoP;
      for (let i = 0; i < N; i++) {
        const { R, p, z } = F[i];
        const wi = add(wp, scl(z, qd[i]));
        const ali = add(add(alp, scl(z, qdd[i])), cross(wp, scl(z, qd[i])));
        const r = sub(p, pp);
        const aoi = add(add(aop, cross(alp, r)), cross(wp, cross(wp, r)));
        w.push(wi); al.push(ali); ao.push(aoi);
        comW.push(add(p, mv(R, this.bodies[i].c)));
        Iw.push(mm(mm(R, this.bodies[i].I), tr(R)));
        wp = wi; alp = ali; aop = aoi; pp = p;
      }
      const tau = new Float64Array(N);
      let fc = [0, 0, 0], nc = [0, 0, 0], pc = null;
      for (let i = N - 1; i >= 0; i--) {
        const { p, z } = F[i], b = this.bodies[i];
        const rc = sub(comW[i], p);
        const ac = add(add(ao[i], cross(al[i], rc)), cross(w[i], cross(w[i], rc)));
        const Fi = scl(ac, b.m);
        const Ni = add(mv(Iw[i], al[i]), cross(w[i], mv(Iw[i], w[i])));
        let f = add(Fi, fc);
        let n = add(add(Ni, cross(rc, Fi)), nc);
        if (pc) n = add(n, cross(sub(pc, p), fc));
        tau[i] = dot(z, n);
        fc = f; nc = n; pc = p;
      }
      return tau;
    }
    massMatrix(q, F) {
      const M = new Float64Array(N * N), z = new Float64Array(N);
      for (let j = 0; j < N; j++) {
        const e = new Float64Array(N); e[j] = 1;
        const col = this.rnea(q, z, e, false, F);
        for (let i = 0; i < N; i++) M[i * N + j] = col[i];
      }
      for (let i = 0; i < N; i++) for (let j = 0; j < i; j++) { const s = 0.5 * (M[i * N + j] + M[j * N + i]); M[i * N + j] = M[j * N + i] = s; }
      return M;
    }
    gravityTorque(q) { return this.rnea(q, new Float64Array(N), new Float64Array(N), true); }
    handPoint(q, off) { const F = this.fk(q); return add(F[N - 1].p, mv(F[N - 1].R, off)); }
  }

  // ---------- 시뮬레이터 ----------
  const DEF = { dt: 0.001, vmaxDeg: 60, amaxDeg: 240, wn: 40, zeta: 1, eMaxDeg: 10, edMax: 3, lambdaMax: 3,
                feedforward: true, payloadRamp: 1.0, armature: [0.03, 0.025, 0.025, 0.025, 0.01, 0.01, 0.01], logHz: 100, trailHz: 100, trailSec: 6, damping: 1.0 };

  class Sim {
    constructor(model, cfg = {}) {
      this.cfg = Object.assign({}, DEF, cfg);
      this.arm = new Arm(model);
      this.n = N;
      this.names = model.joints.map((j) => j.name);
      this.ref = new Float64Array(N);
      this.traj = new Trajectory(this.ref, this.arm.lower, this.arm.upper);
      this.payloadTarget = 0; this.payloadMass = 0;
      this.reset();
      this.applyLimits();
    }
    reset(q) {
      q = q || this.ref;
      this.q = Float64Array.from(q); this.qd = new Float64Array(N); this.t = 0;
      this.tauAct = new Float64Array(N); this.tauOut = new Float64Array(N); this.tauRaw = new Float64Array(N); this.tauLim = new Float64Array(N);
      this.sat = new Array(N).fill(false); this.satLatched = new Array(N).fill(false); this.satDur = 0;
      this.limHit = new Array(N).fill(false);
      this.traj.reset(this.q);
      this.clearRecords();
    }
    clearRecords() { this.trail = []; this.log = []; this.nextLog = this.t; this.nextTrail = this.t; this.satLatched.fill(false); }
    applyLimits() {
      const v = new Float64Array(N), a = new Float64Array(N);
      for (let i = 0; i < N; i++) { v[i] = Math.min(this.cfg.vmaxDeg * Math.PI / 180, this.arm.urdfVel[i]); a[i] = this.cfg.amaxDeg * Math.PI / 180; }
      this.traj.setLimits(v, a);
    }
    setTarget(t) { this.traj.setTarget(t); }
    setArmMass(kg) { this.arm.setArmMass(kg); }
    setPayload(kg, off, ramp = true) {
      this.payloadTarget = kg; this.arm.payload.off = off.slice();
      if (!ramp || this.cfg.payloadRamp <= 0) this.payloadMass = kg;
      this.arm.setPayload(this.payloadMass, off);
    }
    step(nsteps = 1) {
      const c = this.cfg, dt = c.dt, kp = c.wn * c.wn, kd = 2 * c.zeta * c.wn, eMax = c.eMaxDeg * Math.PI / 180;
      for (let s = 0; s < nsteps; s++) {
        if (this.payloadMass !== this.payloadTarget) {
          const st = dt * Math.max(this.payloadTarget, this.payloadMass, 0.1) / c.payloadRamp;
          this.payloadMass += clamp(this.payloadTarget - this.payloadMass, -st, st);
          if (Math.abs(this.payloadMass - this.payloadTarget) < 1e-12) this.payloadMass = this.payloadTarget;
          this.arm.setPayload(this.payloadMass, this.arm.payload.off);
        }
        this.traj.step(dt);
        const F = this.arm.fk(this.q);
        const M = this.arm.massMatrix(this.q, F);
        for (let i = 0; i < N; i++) M[i * N + i] += c.armature[i];   // 모터 로터 반사관성 (감속기 입력측)
        const bias = this.arm.rnea(this.q, this.qd, new Float64Array(N), true, F);
        // 피드포워드: 실제 자세에서 '기준 궤적'의 속도·가속도로 계산한 역동역학
        // (실제 속도의 코리올리를 되먹이지 않음 → 과부하로 팔이 처질 때 증폭 없음)
        const ffTraj = this.arm.rnea(this.q, this.traj.v, this.traj.a, true, F);
        const Minv = invFull(M, N);
        const Md = Array.from({ length: N }, (_, i) => Minv[i * N + i]);
        const Dg = feedbackInertia(M, Minv, c.lambdaMax);
        for (let i = 0; i < N; i++) ffTraj[i] += c.armature[i] * this.traj.a[i] + c.damping * this.traj.v[i];
        let anySat = false;
        for (let i = 0; i < N; i++) {
          const e = clamp(this.traj.q[i] - this.q[i], -eMax, eMax);
          const ed = clamp(this.traj.v[i] - this.qd[i], -c.edMax, c.edMax);
          const fb = Dg[i] * (kp * e + kd * ed);
          const raw = c.feedforward ? ffTraj[i] + fb : fb;
          this.tauRaw[i] = raw;
          this.sat[i] = Math.abs(raw) > this.arm.effort[i];
          this.tauAct[i] = clamp(raw, -this.arm.effort[i], this.arm.effort[i]);
          if (this.sat[i]) { anySat = true; this.satLatched[i] = true; }
        }
        this.satDur = anySat ? this.satDur + dt : 0;
        const rhs = new Float64Array(N);
        for (let i = 0; i < N; i++) rhs[i] = this.tauAct[i] - c.damping * this.qd[i] - bias[i];
        const qdd = cholSolve(M, rhs, N);
        const qPrev = Float64Array.from(this.q), qdPrev = Float64Array.from(this.qd), tPrev = this.t;
        for (let i = 0; i < N; i++) this.qd[i] += qdd[i] * dt;
        // 가동범위 = 기계식 스토퍼: 다음 위치가 범위를 넘으면 그 관절 속도를 충격량으로 멈춘다
        // (비탄성, 질량행렬로 결합된 다른 관절에도 반작용 반영 · 투영 가우스–자이델)
        const lam = new Float64Array(N);
        this.limHit.fill(false);
        for (let it = 0; it < 6; it++) {
          for (let i = 0; i < N; i++) {
            const lo = this.arm.lower[i], hi = this.arm.upper[i], qn = this.q[i] + this.qd[i] * dt;
            const upperSide = this.q[i] > (lo + hi) / 2;   // 가까운 쪽 스토퍼
            let vStar = null;
            if (upperSide && (qn > hi || lam[i] < 0)) vStar = Math.max((hi - this.q[i]) / dt, -0.2);
            else if (!upperSide && (qn < lo || lam[i] > 0)) vStar = Math.min((lo - this.q[i]) / dt, 0.2);
            if (vStar === null) continue;
            let d = (vStar - this.qd[i]) / Minv[i * N + i];
            const newLam = upperSide ? Math.min(0, lam[i] + d) : Math.max(0, lam[i] + d);
            d = newLam - lam[i]; lam[i] = newLam;
            if (d !== 0) for (let k = 0; k < N; k++) this.qd[k] += Minv[k * N + i] * d;
          }
        }
        for (let i = 0; i < N; i++) {
          this.tauLim[i] = lam[i] / dt; this.limHit[i] = lam[i] !== 0;
          // 감속기 출력축 토크 = 모터 구동토크 − 로터 반사관성 가속 토크 (정지 시 동일)
          this.tauOut[i] = this.tauAct[i] - c.armature[i] * (this.qd[i] - qdPrev[i]) / dt;
          this.q[i] += this.qd[i] * dt;
        }
        this.t += dt;
        this._record(tPrev, qPrev, qdPrev);
      }
    }
    _record(t, q, qd) {
      const c = this.cfg;
      if (t + 1e-12 >= this.nextTrail) {
        this.nextTrail = t + 1 / c.trailHz;
        this.trail.push({ t, x: Array.from(q, (v, i) => v - this.ref[i]), y: Array.from(this.tauOut), s: this.sat.slice() });
        const keep = c.trailSec * c.trailHz + 1;
        if (this.trail.length > keep * 1.5) this.trail = this.trail.slice(-keep);
      }
      if (t + 1e-12 >= this.nextLog) {
        this.nextLog = t + 1 / c.logHz;
        if (this.log.length < 3600 * c.logHz) this.log.push({ t, tgt: Array.from(this.traj.target), ref: Array.from(this.traj.q), q: Array.from(q), qd: Array.from(qd),
          tau: Array.from(this.tauOut), raw: Array.from(this.tauRaw), sat: this.sat.slice(), arm: this.arm.armMass, pay: this.payloadMass, off: this.arm.payload.off.slice() });
      }
    }
    trailWindow() { const k = this.cfg.trailSec * this.cfg.trailHz + 1; return this.trail.length > k ? this.trail.slice(-k) : this.trail; }
    csv() {
      const d = (r) => (r * 180 / Math.PI);
      const head = ["time_s", "joint", "target_deg", "traj_ref_deg", "actual_deg", "displacement_deg", "velocity_deg_s",
        "output_torque_Nm", "cmd_torque_unclipped_Nm", "torque_limit_Nm", "saturated", "arm_mass_kg", "payload_kg", "payload_off_x_m", "payload_off_y_m", "payload_off_z_m"];
      const rows = [head.join(",")];
      for (const r of this.log) for (let i = 0; i < N; i++) {
        rows.push([r.t.toFixed(4), this.names[i], d(r.tgt[i]).toFixed(4), d(r.ref[i]).toFixed(4), d(r.q[i]).toFixed(4), d(r.q[i] - this.ref[i]).toFixed(4),
          d(r.qd[i]).toFixed(4), r.tau[i].toFixed(5), r.raw[i].toFixed(5), this.arm.effort[i], r.sat[i] ? 1 : 0, r.arm.toFixed(5), r.pay.toFixed(5),
          r.off[0], r.off[1], r.off[2]].join(","));
      }
      return rows.join("\n");
    }
    // 정적 시험: 목표 자세로 이동·정착 후 평균
    settle(qTarget, tSettle = 4, tAvg = 0.5) {
      this.setTarget(qTarget);
      this.step(Math.round(tSettle / this.cfg.dt));
      const n = Math.round(tAvg / this.cfg.dt), tau = new Float64Array(N), q = new Float64Array(N);
      let anySat = new Array(N).fill(false);
      for (let k = 0; k < n; k++) { this.step(1); for (let i = 0; i < N; i++) { tau[i] += this.tauAct[i] / n; q[i] += this.q[i] / n; anySat[i] = anySat[i] || this.sat[i]; } }
      return { tau, q, err: Array.from(q, (v, i) => this.traj.target[i] - v), sat: anySat };
    }
  }

  // 0 kg / m kg 정적 시험 + 이론값(하중 중력 모멘트를 실제 관절축에 투영)
  function staticPayloadTest(model, pose, payload = 1.0, off = [0.12, 0, 0], armMass = null) {
    const res = {};
    for (const m of [0, payload]) {
      const sim = new Sim(model);
      if (armMass) sim.setArmMass(armMass);
      sim.setPayload(m, off, false);
      res[m] = sim.settle(pose);
      res[m].sim = sim;
    }
    const a = res[0], b = res[payload], arm = b.sim.arm;
    const F = arm.fk(b.q);
    const c = add(F[N - 1].p, mv(F[N - 1].R, off));
    const gF = scl(model.gravity, payload);
    const theory = F.map(({ p, z }) => -dot(z, cross(sub(c, p), gF)));
    const dsim = Array.from(b.tau, (v, i) => v - a.tau[i]);
    return { pose, payload, off, tau0: Array.from(a.tau), tau1: Array.from(b.tau), dsim, theory,
      err: dsim.map((v, i) => v - theory[i]), track0: a.err, track1: b.err, sat0: a.sat, sat1: b.sat,
      q0: Array.from(a.q), q1: Array.from(b.q), payloadPoint: c, effort: Array.from(arm.effort) };
  }

  return { Sim, Arm, Trajectory, staticPayloadTest, N, util: { mm, mv, cross, dot, sub, add, axisAngle, cholSolve } };
});
