/* node reducer_test.js — 관절 감속기 배치 · 충격 판정 계산 검증 (모두 자동 판정) */
const assert = require("assert");
const RC = require("./reducer_core.js");
require("./reducer_presets.js");
const model = require("./model.json");
const PRE = globalThis.REDUCER_PRESETS, D2R = Math.PI / 180;
const sc = { pose: [-90, 0, 0, 88, 0, 0, 0].map((v) => v * D2R), payload: 2, off: [0.12, 0, 0],
  impact: { F: 150, T: 0.03, dir: [0, -1, 0], link: 6 }, tPre: 0.05, tPost: 0.5 };
const rec = RC.runRigid(model, sc);
const pk = (j) => rec.tauOut.reduce((a, r) => (Math.abs(r[j]) > Math.abs(a) ? r[j] : a), 0);

// 1) 관절 반력의 축 성분 = 강체 시뮬레이터 출력축 토크
//    쿨롱 마찰 0이면 수치 오차 수준으로 일치. 기본 마찰에서는 시뮬레이터의 마찰 암시적 선형화 몫만 차이 (≤ 2·τc)
{
  const r0 = RC.runRigid(model, Object.assign({}, sc, { cfg: { coulomb: [0, 0, 0, 0, 0, 0, 0] } }));
  for (let j = 0; j < 7; j++) {
    let e0 = 0, e = 0;
    for (let k = 0; k < r0.t.length; k++) e0 = Math.max(e0, Math.abs(r0.wrench[k][j].T - r0.tauOut[k][j]));
    for (let k = 0; k < rec.t.length; k++) e = Math.max(e, Math.abs(rec.wrench[k][j].T - rec.tauOut[k][j]));
    assert(e0 < 1e-9, `관절 ${j}: 마찰 0 반력 토크 차 ${e0}`);
    assert(e < 2 * rec.coulomb[j], `관절 ${j}: 반력 토크 차 ${e}`);
  }
}
// 2) 정지 상태 반력 = 정역학 (충격 전: 자식 쪽 전체 무게가 축방향·반경 성분으로 나뉨, 크기 = 무게)
{
  const arm = new (require("./dynamics.js").Arm)(model); arm.setPayload(2, [0.12, 0, 0]);
  const z = new Float64Array(7), W = RC.jointWrench(arm, sc.pose, z, z, null);
  const mTot = arm.bodies.reduce((a, b) => a + b.m, 0);
  const F0 = Math.hypot(W[0].Fa, W[0].Fr);
  assert(Math.abs(F0 - mTot * 9.81) < 1e-6, `어깨 pitch 반력 ${F0} vs 무게 ${mTot * 9.81}`);
}
// 3) 감속기 예시 제원: 엔진 오류 없음, 감속비 목표 근처, 정적 한계 토크 ≥ URDF effort
const want = { A: [22.5, 111], B: [20, 53.7], C: [16, 24.8] }, info = {};
for (const k of "ABC") {
  info[k] = RC.reducerInfo(PRE[k].D);
  assert(info[k].ok, k + " 엔진 오류");
  assert(Math.abs(info[k].ratio / want[k][0] - 1) < 0.03, `${k} 감속비 ${info[k].ratio}`);
  assert(info[k].staticCap >= want[k][1], `${k} 정적 한계 ${info[k].staticCap}`);
  // 정적 한계 토크에서 판정 여유 = 1 (엔진 capacity와 같은 기준)
  const g = RC.gearCheck(info[k], info[k].staticCap);
  assert(Math.abs(g.ratio - 1) < 0.01, `${k} 한계 토크에서 여유 ${g.ratio}`);
}
// 4) 유연 모델: 백래시 0 + 매우 단단 → 강체와 같음 (피크 3 % 이내, 평균 차 피크의 1 % 이내)
//    한 스텝(1 ms) 안의 최대값을 기록하므로 토크가 급변하는 순간에는 다음 스텝 값 쪽으로 앞서 보일 수 있어 시점별 최대 차 대신 피크·평균으로 봄
for (const j of [0, 1, 3]) {
  const r = RC.runCompliant(rec, j, { K: 1e8, backlash: 0, Jm: rec.armature[j], zetaG: 0.3 });   // 강체 계산과 같은 모터 쪽 관성
  const p = r.tg.reduce((a, v) => (Math.abs(v) > Math.abs(a) ? v : a), 0);
  let m = 0; for (let k = 0; k < rec.t.length; k++) m += Math.abs(r.tg[k] - rec.tauOut[k][j]) / rec.t.length;
  assert(Math.abs(Math.abs(p) / Math.abs(pk(j)) - 1) < 0.03, `관절 ${j}: 피크 ${p} vs 강체 ${pk(j)}`);
  assert(m < 0.01 * Math.abs(pk(j)), `관절 ${j}: 평균 차 ${m}`);
}
// 5) 백래시가 있으면 하중 부호가 바뀌는 관절에서 이 떨어짐이 생기고 피크가 강체보다 크거나 같음
{
  const r = RC.runCompliant(rec, 1, { K: info.B.K, backlash: info.B.backlash, Jm: 0.012, zetaG: 0.05 });
  const p = r.tg.reduce((a, v) => Math.max(a, Math.abs(v)), 0);
  assert(r.sep > 0 && p >= Math.abs(pk(1)) * 0.99, `어깨 roll 이 떨어짐 ${r.sep}, 피크 ${p}`);
}
// 6) 관절 종합: 7관절 모두 유한한 여유와 최약 부품
const map = "ABBBBCC";
for (let j = 0; j < 7; j++) {
  const r = RC.evaluateJoint(rec, j, { info: info[map[j]], Jm: PRE[map[j]].Jm, kScale: 1, zetaG: 0.05 });
  assert(r.parts.every((p) => Number.isFinite(p.k) && p.k > 0), "여유 계산 " + j);
}
console.log("reducer_test: all ok");
// 7) 실시간 감시(Live): 돌아가는 Sim에 붙여도 일괄 계산과 같은 결과 (피크 5 % 이내), 기어 여유 표 보간 = 엔진 직접 계산 (3 % 이내)
{
  const Dyn = require("./dynamics.js");
  const sim = new Dyn.Sim(model, { collision: false, payloadRamp: 0 });
  sim.setPayload(2, [0.12, 0, 0], false); sim.reset(sc.pose); sim.setTarget(sc.pose); sim.step(1000);
  const tabs = {}; for (const k of "ABC") tabs[k] = RC.gearTable(info[k]);
  const joints = Array.from({ length: 7 }, (_, j) => { const k = map[j]; return { info: info[k], tab: tabs[k], K: info[k].K, bl: info[k].backlash, Jm: rec.armature[j] }; });
  const live = new RC.Live(sim, joints, { kScale: 1, zetaG: 0.05 }).attach();
  sim.step(50); sim.applyImpulse([0, -150, 0], 0.03, 6); sim.step(500);
  for (let j = 0; j < 7; j++) {
    const ref = RC.evaluateJoint(rec, j, { info: info[map[j]], Jm: rec.armature[j], kScale: 1, zetaG: 0.05 });
    const a = Math.abs(live.peak[j].tg), b = Math.abs(ref.cPk);
    assert(Math.abs(a / b - 1) < 0.05, `Live 관절 ${j}: 피크 ${a} vs 일괄 ${b}`);
  }
  for (const k of "ABC") for (const T of [0.3, 0.7, 1.0].map((f) => f * info[k].staticCap)) {
    const e = RC.gearAt(tabs[k], T).k, d = RC.gearCheck(info[k], T).ratio;
    assert(Math.abs(e / d - 1) < 0.03, `${k} 표 보간 ${e} vs 직접 ${d}`);
  }
}
console.log("reducer_test: live ok");
