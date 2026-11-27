/* 관절별 유성감속기 기어세트 3D (three.js r128) — 설계실 viewer3d의 검증된 치형(gearGeometry)·배치(layout)·위상 공식을 그대로 사용
 * 단위: 설계실은 mm → 로봇 장면은 m (×0.001). 기어세트 z축 = 관절축.
 * 링(하우징 쪽)은 부모 링크에 고정, 마지막 캐리어(출력)는 자식 링크와 같이 돈다 → 자식 링크 그룹 안에서 링을 −q만큼 되돌려 그림.
 * 색 = 부품별 여유 (썬·유성·링 = 기어 접촉·굽힘, 캐리어 = 유성 베어링, 바깥 고리 = 출력 베어링). */
(function (root) {
  "use strict";
  const MM = 0.001;
  const colorOf = (k) => (!Number.isFinite(k) || k >= 2 ? 0x5fbf86 : k >= 1.5 ? 0xc8c84a : k >= 1 ? 0xe08a2a : 0xff3b2a);

  function build(info, D) {
    const T = root.THREE, G3 = root.PGD3D;
    const L = G3.layout(info.R, D);
    const outer = new T.Group(), rel = new T.Group();   // outer: 관절축 정렬, rel: 링(부모) 기준
    outer.userData.reducer = true; outer.add(rel);
    const mats = {}, parts = {};
    const mat = (key, opacity = 1) => {
      if (!mats[key]) mats[key] = new T.MeshStandardMaterial({ color: 0x5fbf86, metalness: 0.35, roughness: 0.4, emissive: 0x000000, transparent: opacity < 1, opacity, side: T.DoubleSide });
      return mats[key];
    };
    const kin = [];
    let ringBody = null, ringBodyPhase = 0;
    L.S.forEach((x, i) => {
      const g = x.g, st = x.st, b = st.b, at = g.at, zb = x.z0;
      const zs = g.parts[0].z, zp = g.parts[1].z, zr = g.parts[2].z, a = x.a, sid = i + 1;
      const rho0 = (zp % 2) ? Math.PI / zr : 0, tb = Math.tan(st.beta * Math.PI / 180), rr = g.parts[2].d / 2;
      let psi = 0;
      if (L.shared && i > 0) psi = (ringBodyPhase - zb * tb / rr) - rho0;
      else {
        const depth = L.shared ? L.gearEnd : b, key = L.shared ? "R" : `${sid}-2`;
        const ring = new T.Mesh(G3.gearGeometry(g.parts[2], at, depth, -st.beta, true, x.ringOD), mat(key, 0.55));
        ring.position.z = zb; ring.rotation.z = rho0; rel.add(ring);
        if (L.shared) { ringBody = ring; ringBodyPhase = rho0; }
      }
      const sun = new T.Mesh(G3.gearGeometry(g.parts[0], at, b, st.beta, false, x.sunBore), mat(`${sid}-0`));
      sun.position.z = zb; rel.add(sun);
      const pGeo = G3.gearGeometry(g.parts[1], at, b, -st.beta, false, x.planetBore), planets = [];
      for (let j = 0; j < st.n; j++) { const pm = new T.Mesh(pGeo, mat(`${sid}-1`)); pm.position.z = zb; rel.add(pm); planets.push(pm); }
      // 캐리어: 유성 축 반경을 덮는 얇은 원판 2장 (색 = 이 단 유성 베어링 여유)
      const car = new T.Group(), plateR = Math.max(x.plateR || a + g.parts[1].da / 2 * 0.6, a + 1), boreR = Math.max(0.5, (x.sunBore || 0) / 2 + 0.5);
      const plate = new T.RingGeometry(boreR, plateR, 64);
      [zb - 0.8, zb + b + 0.8].forEach((z) => { const m = new T.Mesh(plate, mat(`P${sid}`, 0.8)); m.position.z = z; car.add(m); });
      rel.add(car);
      const phase = Array.from({ length: st.n }, (_, j) => {
        const phi = 2 * Math.PI * j / st.n, pitchS = 2 * Math.PI / zs, delta = ((phi % pitchS) + pitchS) % pitchS;
        return Math.PI - Math.PI / zp + delta * zs / zp;
      });
      kin.push({ sun, planets, car, a, zs, zp, ratio: g.ratio, n: st.n, phase, psi });
    });
    // 출력 베어링 (크로스 롤러): 롤러 피치원 dp 위 토러스
    const ob = new T.Mesh(new T.TorusGeometry(L.dp / 2, Math.max(0.8, L.Dw * 0.6), 10, 72), mat("OB"));
    ob.position.z = (L.zBear[0] + L.zBear[1]) / 2; rel.add(ob);
    // 전체 길이 중앙을 관절 원점에
    const zMid = (Math.min(0, ...L.S.map((x) => x.z0)) + L.zBear[1]) / 2;
    rel.position.z = -zMid;
    outer.scale.setScalar(MM);
    const size = { OD: Math.max(L.ringOD || 0, 2 * (L.raceOut || 0)), L: L.zBear[1] - Math.min(0, ...L.S.map((x) => x.z0)) };

    // q: 관절각(자식 링크 기준), totalRatio로 썬 회전 계산. 링을 −q로 되돌리고 그 안에서 출력 = +q
    function pose(q) {
      rel.rotation.z = -q;
      let sunAng = q * kin.reduce((r, p) => r * p.ratio, 1);
      kin.forEach((p) => {
        const carA = sunAng / p.ratio;
        p.sun.rotation.z = sunAng + p.psi;
        p.car.rotation.z = carA + p.psi;
        p.planets.forEach((pm, j) => {
          const phi = 2 * Math.PI * j / p.n + carA + p.psi;
          pm.position.x = p.a * Math.cos(phi); pm.position.y = p.a * Math.sin(phi);
          pm.rotation.z = p.phase[j] + phi - (sunAng - carA) * p.zs / p.zp;
        });
        sunAng = carA;
      });
    }
    // k: {"1-0": 썬1, "1-1": 유성1, "1-2" 또는 "R": 링, planet: [단별 s0/목표], ob: 출력 베어링}
    function paint(k) {
      for (const [key, m] of Object.entries(mats)) {
        let v;
        if (key === "OB") v = k.ob;
        else if (key[0] === "P") v = k.planet ? k.planet[+key.slice(1) - 1] : Infinity;
        else v = k[key];
        const c = colorOf(v);
        m.color.setHex(c);
        m.emissive.setHex(Number.isFinite(v) && v < 1.5 ? c : 0x000000);
        m.emissiveIntensity = Number.isFinite(v) && v < 1 ? 0.6 : 0.25;
      }
    }
    function dispose() { outer.traverse((o) => { if (o.geometry) o.geometry.dispose(); }); Object.values(mats).forEach((m) => m.dispose()); }
    pose(0);
    return { outer, pose, paint, dispose, size, keys: Object.keys(mats) };
  }
  root.ReducerGears = { build, colorOf };
})(typeof self !== "undefined" ? self : this);
