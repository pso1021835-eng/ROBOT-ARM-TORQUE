/* 유성감속기 설계실 — 3D viewer (Three.js r128, no examples modules, no external assets)
   mode 'gears': gear set only (step 4).  mode 'full': assembled reducer with housing, carriers, bearings, flange.
   Gear outline + phasing are the verified versions from the previous app.js (do not change). */
(function () {
  'use strict';
  const T = () => window.THREE;
  const STEEL = 7.85e-6, ALU = 2.7e-6;
  const ease = t => t < .5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2;
  const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
  const fmt = (x, d = 1) => Number.isFinite(x) ? x.toFixed(d) : '—';

  // ------------------------------------------------------------------ verified gear geometry
  function outline(g, at) {
    const z = g.z, rb = g.db / 2, inv = a => Math.tan(a) - a;
    const inner = g.internal ? g.da / 2 : g.df / 2, outer = g.internal ? g.df / 2 : g.da / 2;
    const half = (Math.PI / 2 + (g.internal ? -2 : 2) * g.x * Math.tan(20 * Math.PI / 180)) / z;
    const w = r => { const rr = Math.max(r, rb * 1.0000001); return half + inv(at) - inv(Math.acos(Math.min(1, rb / rr))); };
    const r0 = Math.max(inner, rb), steps = 7, pts = [];
    for (let k = 0; k < z; k++) {
      const c = 2 * Math.PI * k / z + (g.internal ? Math.PI / z : 0);
      pts.push([inner, c - w(r0)]);
      for (let i = 0; i <= steps; i++) { const r = r0 + (outer - r0) * i / steps; pts.push([r, c - w(r)]); }
      const wa = w(outer); for (let i = 1; i < 4; i++) pts.push([outer, c - wa + 2 * wa * i / 4]);
      for (let i = steps; i >= 0; i--) { const r = r0 + (outer - r0) * i / steps; pts.push([r, c + w(r)]); }
      pts.push([inner, c + w(r0)]);
      const nx = 2 * Math.PI * (k + 1) / z + (g.internal ? Math.PI / z : 0) - w(r0), cur = c + w(r0);
      for (let i = 1; i < 3; i++) pts.push([inner, cur + (nx - cur) * i / 3]);
    }
    return pts.map(([r, a]) => [r * Math.cos(a), r * Math.sin(a)]);
  }
  function ringOuterRadius(g, diameter) {
    return Math.max(g.df / 2 + .5 * g.m, (diameter || g.df + 8 * g.m) / 2);
  }
  // 선삭 질감용 원통 UV: u = 둘레 각, v = 축방향 위치(원통면) 또는 반경(끝면) [mm].
  // 'turned' 거칠기 맵의 가로줄이 원통면에서는 원주 방향 선, 끝면에서는 동심원이 되어 모든 부품이 같은 질감을 가진다.
  function turnedUV(geo) {
    const TH = T(), pos = geo.attributes.position, nm = geo.attributes.normal, uv = new Float32Array(pos.count * 2);
    for (let i = 0; i < pos.count; i++) {
      const x = pos.getX(i), y = pos.getY(i), z = pos.getZ(i), face = nm && Math.abs(nm.getZ(i)) > .7;
      uv[2 * i] = Math.atan2(y, x) / (2 * Math.PI) + .5; uv[2 * i + 1] = face ? Math.hypot(x, y) : z;
    }
    geo.setAttribute('uv', new TH.BufferAttribute(uv, 2));
    return geo;
  }
  // 선삭 거칠기 맵 (이미지 없이 생성): 가로줄 = 가공선. 종합판정·부품 편집이 같은 맵을 쓴다
  function turnedTexture(renderer) {
    const TH = T(), c = document.createElement('canvas'); c.width = c.height = 256; const g = c.getContext('2d');
    let seed = 7; const rnd = () => (seed = (seed * 16807) % 2147483647) / 2147483647;
    for (let y = 0; y < 256; y++) { const v = 150 + (rnd() - .5) * 70 + 25 * Math.sin(y * .9); g.fillStyle = `rgb(${v},${v},${v})`; g.fillRect(0, y, 256, 1); }
    const tx = new TH.CanvasTexture(c); tx.wrapS = tx.wrapT = TH.RepeatWrapping; tx.repeat.set(1, 18); tx.anisotropy = renderer.capabilities.getMaxAnisotropy?.() || 1; return tx;
  }
  // 이 측면(인볼류트·필렛) 매끈하게: 같은 위치의 정점끼리, 면 법선이 35° 이내인 것만 평균 (이끝·이뿌리 모서리는 살린다)
  function smoothSides(geo) {
    const pos = geo.attributes.position, nm = geo.attributes.normal, n = pos.count, key = i => `${Math.round(pos.getX(i) * 1e4)},${Math.round(pos.getY(i) * 1e4)},${Math.round(pos.getZ(i) * 1e4)}`;
    const groups = new Map(), fn = new Float32Array(n * 3);
    for (let i = 0; i < n; i++) { fn[3 * i] = nm.getX(i); fn[3 * i + 1] = nm.getY(i); fn[3 * i + 2] = nm.getZ(i); if (Math.abs(fn[3 * i + 2]) > .5) continue; const k = key(i); (groups.get(k) || groups.set(k, []).get(k)).push(i); }
    const c35 = Math.cos(35 * Math.PI / 180);
    for (const ids of groups.values()) for (const i of ids) {
      let x = 0, y = 0, z = 0;
      for (const j of ids) { const d = fn[3 * i] * fn[3 * j] + fn[3 * i + 1] * fn[3 * j + 1] + fn[3 * i + 2] * fn[3 * j + 2]; if (d >= c35) { x += fn[3 * j]; y += fn[3 * j + 1]; z += fn[3 * j + 2]; } }
      const L = Math.hypot(x, y, z) || 1; nm.setXYZ(i, x / L, y / L, z / L);
    }
    nm.needsUpdate = true;
  }
  // rimDia: bore for external gears (0 = solid), outside diameter for the ring
  function gearGeometry(g, at, b, betaDeg, internal, rimDia) {
    const TH = T(), pts = outline(g, at).map(([x, y]) => new TH.Vector2(x, y));
    let shape;
    if (internal) {
      shape = new TH.Shape(); const R0 = ringOuterRadius(g, rimDia);
      shape.absarc(0, 0, R0, 0, Math.PI * 2, false); shape.holes.push(new TH.Path(pts.slice().reverse()));
    } else {
      shape = new TH.Shape(pts);
      const rh = Math.min((rimDia || 0) / 2, g.df / 2 - .3 * g.m);
      if (rh > .2) { const hole = new TH.Path(); hole.absarc(0, 0, rh, 0, Math.PI * 2, true); shape.holes.push(hole); }
    }
    const geo = new TH.ExtrudeGeometry(shape, { depth: b, bevelEnabled: false, curveSegments: 96, steps: Math.max(2, Math.round(b)) });
    const tb = Math.tan(betaDeg * Math.PI / 180), rp = g.d / 2, pos = geo.attributes.position;
    // ring gear: the outer diameter is a plain straight cylinder — only the teeth follow the helix
    const Rout = internal ? ringOuterRadius(g, rimDia) : Infinity;
    const onOD = i => Math.hypot(pos.getX(i), pos.getY(i)) > Rout - 1e-4;
    for (let i = 0; i < pos.count; i++) { if (onOD(i)) continue; const x = pos.getX(i), y = pos.getY(i), zz = pos.getZ(i), a = zz * tb / rp, c = Math.cos(a), s = Math.sin(a); pos.setXY(i, x * c - y * s, x * s + y * c); }
    geo.computeVertexNormals();
    smoothSides(geo);
    if (internal) { const nm = geo.attributes.normal; for (let i = 0; i < pos.count; i++) if (onOD(i) && Math.abs(nm.getZ(i)) < .5) { const x = pos.getX(i), y = pos.getY(i), r = Math.hypot(x, y); nm.setXYZ(i, x / r, y / r, 0); } nm.needsUpdate = true; }
    else {
      // 썬·유성 보어도 링 외경처럼 반경 방향 법선으로: 면 단위로 각져 보이지 않는 매끈한 원통
      const rh = Math.min((rimDia || 0) / 2, g.df / 2 - .3 * g.m), nm = geo.attributes.normal;
      if (rh > .2) { for (let i = 0; i < pos.count; i++) { const x = pos.getX(i), y = pos.getY(i), r = Math.hypot(x, y); if (r < rh + 1e-4 && Math.abs(nm.getZ(i)) < .5) nm.setXYZ(i, -x / r, -y / r, 0); } nm.needsUpdate = true; }
    }
    return turnedUV(geo);
  }
  // revolved solid from an (r, z) outline (closed), chamfered corners look crisp under the environment map
  function lathe(profile, seg = 96) {
    const TH = T();
    return turnedUV(new TH.LatheGeometry(profile.map(([r, z]) => new TH.Vector2(r, z)), seg).rotateX(Math.PI / 2));
  }
  // annular ring r1..r2, z0..z1 with 0.3 mm chamfers
  function ringSolid(r1, r2, z0, z1, ch = .3) {
    const c = Math.min(ch, (r2 - r1) / 4, (z1 - z0) / 4);
    const p = r1 > .01
      ? [[r1 + c, z0], [r2 - c, z0], [r2, z0 + c], [r2, z1 - c], [r2 - c, z1], [r1 + c, z1], [r1, z1 - c], [r1, z0 + c], [r1 + c, z0]]
      : [[0, z0], [r2 - c, z0], [r2, z0 + c], [r2, z1 - c], [r2 - c, z1], [0, z1]];
    return lathe(p);
  }
  // annular ring with rounded insertion corners: rIn on the bore corners, rOut on the OD corners (others: small chamfer)
  function ringRounded(r1, r2, z0, z1, rIn = 0, rOut = 0, ch = .2) {
    const lim = Math.min((r2 - r1) / 2.2, (z1 - z0) / 2.2), c = Math.min(ch, lim), p = [];
    // walk BL → BR → TR → TL; each corner is an arc (radius > 0) or a chamfer of c
    const arc = (cx, cz, rad, a0) => { for (let k = 0; k <= 6; k++) { const a = a0 + k / 6 * Math.PI / 2; p.push([cx + rad * Math.cos(a), cz + rad * Math.sin(a)]); } };
    const ri = Math.min(rIn, lim), ro = Math.min(rOut, lim);
    if (ri > .01) arc(r1 + ri, z0 + ri, ri, Math.PI); else p.push([r1, z0 + c], [r1 + c, z0]);
    if (ro > .01) arc(r2 - ro, z0 + ro, ro, 1.5 * Math.PI); else p.push([r2 - c, z0], [r2, z0 + c]);
    if (ro > .01) arc(r2 - ro, z1 - ro, ro, 0); else p.push([r2, z1 - c], [r2 - c, z1]);
    if (ri > .01) arc(r1 + ri, z1 - ri, ri, .5 * Math.PI); else p.push([r1 + c, z1], [r1, z1 - c]);
    p.push(p[0]);
    return lathe(p);
  }
  function annulusMass(r1, r2, L, rho) { return Math.PI * (r2 * r2 - r1 * r1) * L * rho; }

  function reverseFaces(geometry) {
    const g = geometry.index ? geometry.toNonIndexed() : geometry;
    if (g !== geometry) geometry.dispose();
    for (const name of ['position', 'normal']) {
      const { array, itemSize } = g.attributes[name];
      for (let i = 0; i < array.length; i += 3 * itemSize) {
        for (let j = 0; j < itemSize; j++) {
          [array[i + itemSize + j], array[i + 2 * itemSize + j]] = [array[i + 2 * itemSize + j], array[i + itemSize + j]];
        }
      }
      if (name === 'normal') for (let i = 0; i < array.length; i++) array[i] *= -1;
    }
    return g;
  }
  function carrierBody(shape) {
    const TH = T(), positions = [], normals = [];
    const add = (source, reversed = false) => {
      const geometry = reversed ? reverseFaces(source) : (source.index ? source.toNonIndexed() : source);
      for (const x of geometry.attributes.position.array) positions.push(x);
      for (const x of geometry.attributes.normal.array) normals.push(x);
      geometry.dispose();
      if (!reversed && geometry !== source) source.dispose();
    };
    const wall = (r0, r1, [z0, z1], x, y, reversed, segments = 72) => add(
      new TH.CylinderGeometry(r1, r0, z1 - z0, segments, 1, true)
        .rotateX(Math.PI / 2).translate(x, y, (z0 + z1) / 2), reversed
    );
    const ro = shape.outerD / 2, ri = shape.innerD / 2;
    const pitch = 2 * Math.PI / shape.pins.length;
    const web = shape.web?.sections.length ? shape.web : null;
    const fillet = web && shape.fillets.length ? shape.pocketR : 0;
    const half = (shape.pocket || 0) / 2;
    // Opening angle at a cylinder of radius radius. Offset follows the
    // quarter-circle tool profile from the straight wall onto the plate.
    function opening(radius, offset) {
      const angle = Math.asin(clamp((half - offset) / radius, -1, 1));
      return clamp(angle, 0, pitch / 2);
    }
    const samples = Math.ceil(240 / shape.pins.length);
    const angles = Array.from({ length: samples + 1 }, (_, i) => i * pitch / samples);
    if (web) for (const r of [ri, ro]) {
      const a = opening(r, fillet);
      angles.push(a, pitch - a);
    }
    const knots = Array.from(new Set(angles)).sort((a,b) => a-b);
    const circle = shape.pins.flatMap(pin => knots.slice(0,-1).map(a => pin.angle + a));
    const polar = (r,t,z) => [r*Math.cos(t), r*Math.sin(t), z];
    const outline = r => circle.map(t => new TH.Vector2(r*Math.cos(t),r*Math.sin(t)));
    function triangle(a,b,c,na,nb,nc) {
      const ab = new TH.Vector3().subVectors(b,a), ac = new TH.Vector3().subVectors(c,a);
      const cross = ab.cross(ac);
      if (cross.lengthSq() < 1e-20) return;
      if (cross.dot(na.clone().add(nb).add(nc)) < 0) { [b,c] = [c,b]; [nb,nc] = [nc,nb]; }
      positions.push(a.x,a.y,a.z,b.x,b.y,b.z,c.x,c.y,c.z);
      normals.push(na.x,na.y,na.z,nb.x,nb.y,nb.z,nc.x,nc.y,nc.z);
    }
    const point = (r,t,z) => new TH.Vector3(...polar(r,t,z));
    const radial = (t,sg) => new TH.Vector3(sg*Math.cos(t),sg*Math.sin(t),0);
    function cylinderStrip(radius, sign, angle, lo, hi, cut) {
      const a = cut ? opening(radius,lo.offset) : 0, b = cut ? opening(radius,hi.offset) : 0;
      let prev;
      for (const knot of knots) {
        const u = angle + clamp(knot,a,pitch-a), v = angle + clamp(knot,b,pitch-b);
        const next = [point(radius,u,lo.z), point(radius,v,hi.z), radial(u,sign), radial(v,sign)];
        if (prev) {
          triangle(prev[0],next[0],next[1],prev[2],next[2],next[3]);
          triangle(prev[0],next[1],prev[1],prev[2],next[3],prev[3]);
        }
        prev = next;
      }
    }
    const rootRadius = offset => clamp((half-offset)/Math.sin(pitch/2), ri, ro);
    function pocketFloor(angle) {
      const outer = opening(ro,fillet), inner = opening(ri,fillet);
      const arc = (r,lim) => {
        const points = [0,...knots.filter(t=>t>0 && t<lim),lim].filter((t,k,arr)=>!k || t-arr[k-1]>1e-9);
        return [...points.slice(1).reverse().map(t=>-t),...points].map(t=>point(r,angle+t,0));
      };
      const corner = rootRadius(fillet), far = opening(corner,fillet);
      const edges = arc(ro,outer);
      if (corner>ri+1e-8) edges.push(point(corner,angle+far,0));
      edges.push(...arc(ri,inner).reverse());
      if (corner>ri+1e-8 && far>1e-8) edges.push(point(corner,angle-far,0));
      return new TH.Shape(edges.map(p=>new TH.Vector2(p.x,p.y)));
    }
    function face(profile, pins, radius, z, high) {
      if (radius > 1e-8) for (const pin of pins) {
        const hole = new TH.Path(); hole.absarc(pin.x,pin.y,radius,0,2*Math.PI,true); profile.holes.push(hole);
      }
      add(new TH.ShapeGeometry(profile,48).translate(0,0,z), !high);
    }
    for (const slab of shape.slabs) {
      const [z0,z1] = slab.z;
      for (const pin of shape.pins) {
        cylinderStrip(slab.outer,1,pin.angle,{z:z0},{z:z1},false);
        cylinderStrip(slab.inner,-1,pin.angle,{z:z0},{z:z1},false);
      }
      for (const high of [false,true]) {
        const z = slab.z[+high], band = slab.drilling.find(h=>Math.abs(h.z[+high]-z)<1e-8);
        const radius = band ? (high ? band.r1 : band.r0) : 0;
        if (web && Math.abs(z-web.z[high ? 0 : 1]) < 1e-8) {
          // Only exposed pocket floors are faces. The plate/web interface is solid.
          for (const pin of shape.pins) face(pocketFloor(pin.angle),[pin],radius,z,high);
        } else {
          const profile = new TH.Shape(outline(slab.outer));
          profile.holes.push(new TH.Path(outline(slab.inner).reverse()));
          face(profile,shape.pins,radius,z,high);
        }
      }
      for (const band of slab.drilling) for (const pin of shape.pins)
        wall(band.r0,band.r1,band.z,pin.x,pin.y,true,96);
    }
    if (web) {
      const [z0,z1] = web.z;
      const levels = [];
      // Use tangent normals on the round and exactly radial normals on the
      // turned OD/ID. Both faces terminate on the same boundary vertices.
      const curve = fillet ? Array.from({length:25}, (_,k) => k*Math.PI/48) : [Math.PI/2];
      // Split at the exact contact with the bore, where the two sides of a web
      // stop meeting each other and start terminating on the inner cylinder.
      const boreOffset = half - ri*Math.sin(pitch/2);
      if (boreOffset > 0 && boreOffset < fillet) {
        curve.push(Math.asin(1 - boreOffset/fillet));
        curve.sort((a,b) => a-b);
      }
      for (const top of [false,true]) {
        for (const t of top ? [...curve].reverse() : curve) {
          const height = fillet*(1-Math.cos(t));
          levels.push({z:top ? z1-height : z0+height, offset:fillet*(1-Math.sin(t)),
            wallNormal:Math.sin(t), faceNormal:(top ? -1 : 1)*Math.cos(t)});
        }
      }
      for (const pin of shape.pins) {
        for (let i=1;i<levels.length;i++) {
          const lo=levels[i-1], hi=levels[i];
          if (hi.z-lo.z<1e-10) continue;
          cylinderStrip(ro,1,pin.angle,lo,hi,true);
          cylinderStrip(ri,-1,pin.angle,lo,hi,true);
          for (const side of [-1,1]) {
            const normalAngle = pin.angle - side*Math.PI/2;
            const sample = level => {
              const r0=rootRadius(level.offset);
              const pos=r=>point(r,pin.angle+side*opening(r,level.offset),level.z);
              const n=radial(normalAngle,level.wallNormal); n.z=level.faceNormal;
              return [pos(r0),pos(ro),n];
            };
            const a=sample(lo), b=sample(hi);
            triangle(a[0],a[1],b[1],a[2],a[2],b[2]);
            triangle(a[0],b[1],b[0],a[2],b[2],b[2]);
          }
        }
      }
    }
    const geometry = new TH.BufferGeometry();
    geometry.setAttribute('position', new TH.Float32BufferAttribute(positions, 3));
    geometry.setAttribute('normal', new TH.Float32BufferAttribute(normals, 3));
    return turnedUV(geometry);
  }

  // ------------------------------------------------------------------ geometry model (dimensions only)
  // optional parts: shown/hidden (on) and dimension lock (lock); strength calculations are unchanged
  const OPTIONS = [
    { key: 'pin', label: '유성핀', parts: ['pin1', 'pin2'] },
    { key: 'pbrg', label: '유성 베어링', parts: ['needle1', 'needle2'] },
    { key: 'obrg', label: '출력 베어링', parts: ['xroller'] }
  ];
  const optOf = id => OPTIONS.find(o => o.parts.includes(id));
  const optOn = (D, key) => D.options?.[key]?.on !== false;
  const partOn = (D, id) => { const o = optOf(id); return !o || optOn(D, o.key); };
  const partLocked = (D, id) => { const o = optOf(id); return !!(o && D.options?.[o.key]?.lock); };
  const compName = (R, D, id) => {
    const shared = D.config.sharedRing && R.ctx.stages.length > 1;
    const m = /^(sun|planet|ring|carrier)(\d?)$/.exec(id);
    if (m) { const n = m[2] || '1'; return { sun: `S${n} 썬기어`, planet: `P${n} 유성기어`, ring: shared ? 'R1-2 링기어 (공유)' : `R${n} 링기어`, carrier: `C${n} 캐리어${n === String(R.ctx.stages.length) ? ' (출력)' : ''}` }[m[1]]; }
    return null;
  };

  function layout(R, D) {
    // 하우징 · 커버 · 출력 플랜지 · 입력/출력 중공축 · 썬 베어링 · 오일 실 · 볼트는 사이트 범위에서 삭제 (사용자 결정 2026-11-26 BOM-1).
    // 외형(외경 · 길이)은 링기어와 출력 베어링으로만 정한다. 아래 값은 0 두께로 남겨 다른 계산이 깨지지 않게 한다.
    const stages = R.ctx.stages, sh = { wall: 0, coverT: 0, boltN: 0, boltD: 0, flangeT: 0 };
    const G = D.geom || {}, auto = {};
    // dimension override: D.geom[part][key] (mm, diameters as diameters); auto values are kept for the input panel
    const ov = (part, key, a) => { (auto[part] ||= {})[key] = a; const v = G[part]?.[key]; return Number.isFinite(v) && v > 0 ? v : a; };
    const S = stages.map(s => {
      const g = s.geo, st = s.st, rim = k => g.rims?.[k]?.diameter;
      const carrier = s.carrier;
      return { s, g, st, carrier, a: carrier.orbit, plateR: carrier.outerD / 2, sunBore: rim(0), planetBore: rim(1), ringOD: rim(2), b: g.parts[1].width };
    });
    // Stack each stage from its assembled gear, carrier, pin and boss extents.
    let z = 0; const zStart = [];
    S.forEach((x, i) => {
      x.z0 = i ? z + Math.max(0, D.geom?.stageGap ?? .6) - x.carrier.axial[0] : 0;
      zStart.push(x.z0);
      z = x.z0 + x.carrier.axial[1];
    });
    const gearEnd = S[S.length - 1].z0 + S[S.length - 1].b;
    // 카탈로그 형번이 선택돼 있으면 내경·외경·폭을 카탈로그 치수로, 아니면 dp ± 1.1·Dw로 추정
    const ob = D.outBearing || {}, cat = ob.d > 0 && ob.D > ob.d && ob.B > 0;
    const dp = ob.dp || (cat ? (ob.d + ob.D) / 2 : 60), Dw = ob.Dw || 4, Bw = ov('xroller', 'width', cat ? ob.B : Dw * 1.5);
    const raceIn = cat ? ob.d / 2 : dp / 2 - Dw * 1.1, raceOut = cat ? ob.D / 2 : dp / 2 + Dw * 1.1;
    const zBear = [z + .6, z + .6 + Bw];
    const zFlange = [zBear[1], zBear[1]];
    const ringOD = Math.max(...S.map(x => x.ringOD || 0));
    // 중공 = 썬 보어로 통과 (입력 중공축 삭제)
    const hollowR = Math.max(.5, Math.min(...S.map(x => (x.sunBore || 0) / 2)));
    const shaftR = hollowR, shaftExt = 0, sealW = 0, sealH = 0;
    const housR = ringOD / 2, flangeR = raceOut, hubR = Math.min(raceIn, S[S.length - 1].plateR), coverBoltR = 0, flangeBoltR = 0;
    // 1단 썬기어 베어링: between input shaft and a boss on the input cover
    const s1on = false, b1w = 0, b1h = 0;
    const zIn = S[0].carrier.axial[0];
    const zB1 = [zIn, zIn];
    // 출력단 중공축 (rides with the output) and 2단 썬기어 베어링 between it and the last sun bore
    const last = S[S.length - 1], sunBoreR = Math.max(1, (last.sunBore || 2 * (hollowR + 3)) / 2);
    const s2on = false, b2h = 0, b2w = 0, oshaftR = 0, oshaftIR = 0;
    const zOS = [last.z0 - (S.length > 1 ? .2 : 0), zFlange[1]];
    const zB2 = [last.z0 + (last.b - b2w) / 2, last.z0 + (last.b + b2w) / 2];
    // 출력단 커버: retainer ring over the outer race on the output face
    const ocT = 0, ocIR = 0;
    const zOC = [zBear[1] + .15, zBear[1] + .15 + ocT];
    return { S, zStart, gearEnd, zEnd: z, ob, cat, dp, Dw, Bw, raceIn, raceOut, zBear, zFlange, ringOD, hollowR, shaftR, shaftExt, sealW, sealH, housR, flangeR, hubR, coverBoltR, flangeBoltR, zIn, sh, auto,
      s1on, b1w, b1h, zB1, s2on, b2h, b2w, zB2, sunBoreR, oshaftR, oshaftIR, zOS, ocT, ocIR, zOC, shared: !!(D.config.sharedRing && S.length > 1) };
  }


  // parts list for BOM (also used for picking names); masses are geometric estimates
  function partsList(R, D) {
    const L = layout(R, D), out = [];
    const fam = i => { const m = D.materials[i]; return m ? { material: { NV: '질화강', Eh: '침탄강', VA: '조질합금강', VC: '탄소강' }[m.family] || m.family, heat: { NV: '질화', Eh: '침탄', VA: '조질', VC: '조질' }[m.family] + (m.surfaceHRC ? ` HRC${m.surfaceHRC}` : '') } : { material: '—', heat: '—' }; };
    const reqSH = D.req.SH, reqSF = D.req.SF;
    const gearVerdict = (stage, part) => {
      const rows = R.rated.rows.filter(r => r.stage === stage && r.part === part); if (!rows.length) return null;
      const s = Math.min(...rows.map(r => Math.min(r.SH / reqSH, r.SF / reqSF)));
      const r = rows.reduce((m, x) => Math.min(x.SH / reqSH, x.SF / reqSF) < Math.min(m.SH / reqSH, m.SF / reqSF) ? x : m);
      return { ok: s >= 1, text: `SH ${fmt(r.SH, 2)} · SF ${fmt(r.SF, 2)}` };
    };
    L.S.forEach((x, i) => {
      const g = x.g, n = x.st.n, st = i + 1, mi = i * 3;
      const gm = (p, bore, w) => annulusMass((bore || 0) / 2, (p.da + p.df) / 4, w, STEEL);
      out.push({ id: `sun${st}`, name: compName(R, D, `sun${st}`), qty: 1, ...fam(mi), mass: gm(g.parts[0], x.sunBore, g.parts[0].width), step: 's3', verdict: gearVerdict(st, 0) });
      out.push({ id: `planet${st}`, name: compName(R, D, `planet${st}`), qty: n, ...fam(mi + 1), mass: gm(g.parts[1], x.planetBore, g.parts[1].width), step: 's3', verdict: gearVerdict(st, 1) });
      if (!L.shared || i === 0) out.push({ id: L.shared ? 'ring' : `ring${st}`, name: compName(R, D, L.shared ? 'ring' : `ring${st}`), qty: 1, ...fam(mi + 2),
        mass: annulusMass((g.parts[2].da + g.parts[2].df) / 4, L.ringOD / 2, L.shared ? L.gearEnd : g.parts[2].width, STEEL), step: 's3',
        verdict: (() => { const v = [gearVerdict(1, 2), L.shared ? gearVerdict(2, 2) : null].filter(Boolean); return v.length ? { ok: v.every(q => q.ok), text: v.map(q => q.text).join(' / ') } : null; })() });
      const cp = R.carrierPin[i];
      out.push({ id: `pin${st}`, name: `${st}단 유성핀`, qty: n, material: x.st.pin.material, heat: '담금질', mass: Math.PI / 4 * x.carrier.diameter ** 2 * x.carrier.pinLength * STEEL, step: 'scar',
        verdict: cp ? (() => { const g = cp.Sbend <= cp.Spress ? cp.modes.bend : cp.modes.press, ok = g.S >= 1; return { ok, text: `${g.name} ${fmt(g.S, 2)}${ok ? '' : ' · 미달'} · ${g.kind} ${fmt(g.T, 1)} N·m` }; })() : null });
      const br = R.bearings[i];
      out.push({ id: `needle${st}`, name: `${st}단 유성베어링 (${x.st.bearing.name})`, qty: n * (x.st.bearing.count || 1), material: '베어링강', heat: '—', mass: annulusMass(x.st.pin.d / 2, (x.planetBore || x.st.pin.d + 3) / 2, x.b, STEEL) * .45, step: 's9',
        verdict: br ? { ok: br.L10h >= D.req.lifeH && br.governing.s0 >= D.req.s0, text: `L10 ${br.L10h >= 1e6 ? fmt(br.L10h / 1e6, 1) + 'M' : fmt(br.L10h / 1e3, 1) + 'k'} h · s0 ${fmt(br.governing.s0, 2)}` } : null });
      out.push({ id: `carrier${st}`, name: compName(R, D, `carrier${st}`), qty: 1, material: x.st.carrier?.material || 'S45C', heat: '조질', step: 'scar',
        mass: x.carrier.materialVolume * STEEL,
        verdict: cp ? { ok: cp.Sweb >= 1, text: Number.isFinite(cp.Sweb) ? `기둥 S ${fmt(cp.Sweb, 2)} · ${cp.modes.web.kind} ${fmt(cp.modes.web.T, 1)} N·m` : '편단 지지' } : null });
    });
    const ob = R.outBearing, roll = ob.roll;
    out.push({ id: 'xroller', name: `출력베어링 (${D.outBearing.name})`, qty: 1, material: '베어링강', heat: '담금질', step: 's9',
      mass: L.cat && D.outBearing.mass > 0 ? D.outBearing.mass : annulusMass(L.raceIn, L.raceOut, L.Bw, STEEL) * .8,
      verdict: { ok: ob.L10h >= D.req.lifeH && ob.governing.s0 >= D.req.s0 && (!roll || roll.governing.s0 >= D.req.s0), text: `s0 ${fmt(roll ? roll.governing.s0 : ob.governing.s0, 2)}` } });
    // 출력 플랜지 · 하우징 · 커버 · 중공축 · 썬 베어링 · 오일 실 · 볼트: 삭제 (BOM-1)
    const shown = out.filter(p => partOn(D, p.id));
    shown.forEach((p, i) => { p.no = i + 1; const o = optOf(p.id); p.option = o?.key || null; p.locked = partLocked(D, p.id); });
    return shown;
  }

  // ------------------------------------------------------------------ environment (code-generated soft boxes)
  function makeEnv(renderer) {
    const TH = T(), env = new TH.Scene();
    // photo-studio HDR: dark room with bright, sharp-edged strip lights → crisp specular streaks on metal
    const room = new TH.Mesh(new TH.BoxGeometry(40, 20, 40), new TH.MeshBasicMaterial({ color: 0x262524, side: TH.BackSide }));
    env.add(room);
    const panel = (w, h, x, y, z, ry, rx, k, tint = [1, 1, 1]) => { const m = new TH.Mesh(new TH.PlaneGeometry(w, h), new TH.MeshBasicMaterial({ color: new TH.Color(k * tint[0], k * tint[1], k * tint[2]), side: TH.DoubleSide })); m.position.set(x, y, z); m.rotation.set(rx || 0, ry || 0, 0); env.add(m); };
    panel(22, 4, 0, 9.5, 0, 0, Math.PI / 2, 14);              // overhead strip (main streak)
    panel(3, 16, -19.5, 2, 8, Math.PI / 2, 0, 12, [1, .97, .92]); // tall key strip left, warm
    panel(3, 16, -19.5, 2, -6, Math.PI / 2, 0, 7);            // second strip left
    panel(10, 12, 19.5, 1, -4, -Math.PI / 2, 0, 3.5, [.92, .96, 1]); // soft fill right, cool
    panel(2, 14, 14, 2, 19.5, 0, 0, 10);                      // front-side kicker
    panel(20, 1.6, 0, 4, -19.5, 0, 0, 9);                     // rim line back
    panel(30, 8, 0, -9.5, 0, 0, -Math.PI / 2, 1.4, [1, .96, .9]); // warm floor bounce
    const pm = new TH.PMREMGenerator(renderer);
    const tex = pm.fromScene(env, .02).texture; pm.dispose();
    env.traverse(o => { o.geometry?.dispose?.(); o.material?.dispose?.(); });
    return tex;
  }

  // ------------------------------------------------------------------ mount
  function mount(host, opt) {
    const TH = T(); if (!TH) return null;
    const { R, D, mode = 'full' } = opt;
    const full = mode === 'full';
    // only: 'sprc' → machined parts only (sun, planet, ring, carrier + its pins); bearings, housing, covers, shafts, seals, bolts left out for now
    const extras = full && opt.only !== 'sprc';
    const W0 = host.clientWidth, H0 = host.clientHeight;
    const mobile = Math.min(W0, window.innerWidth) < 760;
    const renderer = new TH.WebGLRenderer({ antialias: true, alpha: true, preserveDrawingBuffer: false });
    const DPR = mobile ? 1 : Math.min(1.5, window.devicePixelRatio || 1); renderer.setPixelRatio(DPR);
    renderer.setSize(W0, H0);
    renderer.outputEncoding = TH.sRGBEncoding; renderer.toneMapping = TH.ACESFilmicToneMapping; renderer.toneMappingExposure = 1.0; renderer.setClearColor(0x000000, 0);
    renderer.shadowMap.enabled = full && !mobile; renderer.shadowMap.type = TH.PCFSoftShadowMap; renderer.shadowMap.autoUpdate = false;
    renderer.localClippingEnabled = true;
    host.prepend(renderer.domElement);
    const scene = new TH.Scene();
    const prefs = Object.assign({ bg: '#1a130e', shade: 'pbr', light: 1, lightAz: 35 }, D.view || {});
    if (['#17181a', '#e8eaed', '#f3f0ea'].includes(prefs.bg)) prefs.bg = '#1a130e'; // older defaults → warm dark stage
    // studio backdrop: soft radial light falling off to a slightly deeper tone of the chosen colour
    const paintBg = () => {
      // dark stage: a warm amber glow behind the model; light colours keep the old studio falloff
      const c = new TH.Color(prefs.bg), dark = c.r + c.g + c.b < 1.2, hi = c.clone().lerp(new TH.Color(dark ? 0xd6a05a : 0xffffff), dark ? .22 : .55), lo = c.clone().multiplyScalar(dark ? .55 : .9);
      host.style.background = `radial-gradient(${dark ? '70% 60% at 52% 46%' : '120% 90% at 58% 38%'}, #${hi.getHexString()} 0%, #${c.getHexString()} ${dark ? 55 : 48}%, #${lo.getHexString()} 100%)`;
      host.classList.toggle('v3-dark', dark);
    };
    paintBg();
    scene.environment = makeEnv(renderer);
    const camera = new TH.PerspectiveCamera(30, W0 / H0, 1, 6000);
    const hemi = new TH.HemisphereLight(0xfffaf2, 0x6d6a64, .45); scene.add(hemi);
    const key = new TH.DirectionalLight(0xfff6ec, 2.2); key.castShadow = renderer.shadowMap.enabled;
    key.shadow.mapSize.set(2048, 2048); key.shadow.bias = -.0004; key.shadow.radius = 9; scene.add(key); scene.add(key.target);
    const fill = new TH.DirectionalLight(0xe6ecf5, .55); scene.add(fill);
    const rim = new TH.DirectionalLight(0xffffff, .8); scene.add(rim);

    const L = layout(R, D);
    const root = new TH.Group(); scene.add(root);
    const parts = new Map();          // id -> {objs:[], name, center:Vector3(local), explode}
    const movers = [];                // {obj, base:Vector3, dz, layer}
    const meshes = [];                // all pickable meshes
    const mats = [];                  // all part materials (clones)
    const MAT = {
      sun: { color: 0xc9ccd2, metalness: 1, roughness: .28 }, planet: { color: 0xb3b8bf, metalness: 1, roughness: .3 },
      sun1: { color: 0x2f62b4, metalness: 1, roughness: .12, clearcoat: .8, clearcoatRoughness: .06 }, planet1: { color: 0x4a86d8, metalness: 1, roughness: .14, clearcoat: .8, clearcoatRoughness: .06 },
      sun2: { color: 0xc8901c, metalness: 1, roughness: .12, clearcoat: .8, clearcoatRoughness: .06 }, planet2: { color: 0xdaa636, metalness: 1, roughness: .14, clearcoat: .8, clearcoatRoughness: .06 },
      ring: { color: 0xd6d8dc, metalness: 1, roughness: .18 }, pin: { color: 0xf4f5f7, metalness: 1, roughness: .06 },
      carrier: { color: 0x1a1c20, metalness: .9, roughness: .24 },
      housing: { color: 0xdcd8cf, metalness: 0, roughness: .26, envMapIntensity: .9 },
      flange: { color: 0xb8240a, metalness: .6, roughness: .24, envMapIntensity: .75 }, roller: { color: 0xf7f8fa, metalness: 1, roughness: .04 }, race: { color: 0xdadde1, metalness: 1, roughness: .1 },
      bolt: { color: 0x1d1e21, metalness: 1, roughness: .22 }, seal: { color: 0xc4290a, metalness: 0, roughness: .55 }, shaft: { color: 0xdcdee2, metalness: 1, roughness: .12 }
    };
    // procedural surface finish (no image assets): turning marks for lathe parts, fine grain for the rest.
    // Lathe/ring UVs run along the profile, so horizontal stripes become concentric marks on faces and
    // circumferential lines on cylinders — the look of a real turned part.
    const finish = (() => {
      const mk = (draw, rep) => { const c = document.createElement('canvas'); c.width = c.height = 256; const g = c.getContext('2d'); draw(g); const tx = new TH.CanvasTexture(c); tx.wrapS = tx.wrapT = TH.RepeatWrapping; tx.repeat.set(...rep); tx.anisotropy = renderer.capabilities.getMaxAnisotropy?.() || 1; return tx; };
      let seed = 7; const rnd = () => (seed = (seed * 16807) % 2147483647) / 2147483647;
      const turned = mk(g => { for (let y = 0; y < 256; y++) { const v = 150 + (rnd() - .5) * 70 + 25 * Math.sin(y * .9); g.fillStyle = `rgb(${v},${v},${v})`; g.fillRect(0, y, 256, 1); } }, [1, 18]);
      const grain = mk(g => { const im = g.createImageData(256, 256); for (let i = 0; i < im.data.length; i += 4) { const v = 140 + (rnd() - .5) * 90; im.data[i] = im.data[i + 1] = im.data[i + 2] = v; im.data[i + 3] = 255; } g.putImageData(im, 0, 0); }, [.08, .08]);
      return { turned, grain };
    })();
    // 링 외경과 같은 선삭 질감으로 통일 (볼트만 입자 질감)
    const FIN = { ring: 'turned', race: 'turned', shaft: 'turned', flange: 'turned', pin: 'turned', housing: 'turned', carrier: 'turned', bolt: 'grain', sun: 'turned', planet: 'turned', sun1: 'turned', planet1: 'turned', sun2: 'turned', planet2: 'turned', roller: 'turned' };
    const mkMat = k => {
      const m = new TH.MeshPhysicalMaterial(Object.assign({ envMapIntensity: 1.1 }, MAT[k]));
      const f = FIN[k];
      if (f) { m.roughnessMap = finish[f]; m.roughness = Math.min(1, (MAT[k].roughness || .2) * 1.35); }
      m.userData.base = Object.assign({}, MAT[k], f ? { roughness: m.roughness } : {}); mats.push(m); return m;
    };
    function add(obj, partId, parent = root, layer = 0, dz = 0) {
      parent.add(obj);
      obj.traverse(o => { if (o.isMesh) { o.userData.partId = partId; o.castShadow = o.receiveShadow = renderer.shadowMap.enabled; meshes.push(o); } });
      if (!parts.has(partId)) parts.set(partId, { objs: [] });
      parts.get(partId).objs.push(obj);
      if (dz) movers.push({ obj, base: obj.position.clone(), dz, layer });
      return obj;
    }
    const unit = Math.max(14, L.S[0].b * 3);

    // ---- gears + carriers per stage
    const kin = [];
    const separatePlanets = full && opt.only === 'sprc';
    const rings = L.shared ? L.S.slice(0, 1) : L.S;
    const ringRadius = Math.max(...rings.map(x => ringOuterRadius(x.g.parts[2], x.ringOD)));
    let ringBody = null, ringBodyPhase = 0;
    L.S.forEach((x, i) => {
      const g = x.g, st = x.st, b = st.b, at = g.at, zb = x.z0;
      const zs = g.parts[0].z, zp = g.parts[1].z, zr = g.parts[2].z, a = x.a;
      const rho0 = (zp % 2) ? Math.PI / zr : 0;
      const tb = Math.tan(st.beta * Math.PI / 180), rr = g.parts[2].d / 2;
      const sid = i + 1;
      // explode layers: input side negative, output side positive; stage 1 → toward input, stage 2 → toward output
      const dzStage = (i === 0 ? -1 : 1) * unit * (i === 0 ? 1.4 : 1.4);
      let psi = 0, ring;
      if (L.shared && i > 0) { ring = ringBody; psi = (ringBodyPhase - zb * tb / rr) - rho0; }
      else {
        const depth = L.shared ? L.gearEnd : b;
        ring = new TH.Mesh(gearGeometry(g.parts[2], at, depth, -st.beta, true, x.ringOD), mkMat('ring'));
        ring.position.z = zb; ring.rotation.z = rho0;
        add(ring, L.shared ? 'ring' : `ring${sid}`, root, 0, 0);
        if (L.shared) { ringBody = ring; ringBodyPhase = rho0; }
      }
      const sun = new TH.Mesh(gearGeometry(g.parts[0], at, b, st.beta, false, x.sunBore), mkMat(i === 0 ? 'sun1' : 'sun2'));
      sun.position.z = zb;
      // 부품 편집에서 정한 썬 축(허브)을 종합판정에도 같은 형상으로 (축·하우징을 따로 그리는 전체 보기에서는 입력축이 대신함)
      if (!extras) {
        const h = (D.geom || {})['sun' + sid] || {}, rb = Math.max(.01, (x.sunBore || 0) / 2), hub = (ro, z0, z1) => lathe([[rb, z0], [ro, z0], [ro, z1], [rb, z1], [rb, z0]]);
        if (h.hubInOD / 2 > rb && h.hubInL > 0) sun.add(new TH.Mesh(hub(h.hubInOD / 2, -h.hubInL, 0), sun.material));
        if (h.hubOutOD / 2 > rb && h.hubOutL > 0) sun.add(new TH.Mesh(hub(h.hubOutOD / 2, b, b + h.hubOutL), sun.material));
      }
      add(sun, `sun${sid}`, root, 2, i === 0 ? dzStage * 1.9 : 0);
      const pg = []; const pMat = mkMat(i === 0 ? 'planet1' : 'planet2');
      const pGeo = gearGeometry(g.parts[1], at, b, -st.beta, false, x.planetBore);
      for (let j = 0; j < st.n; j++) { const pm = new TH.Mesh(pGeo, pMat); pm.position.z = zb; add(pm, `planet${sid}`, root, 1, dzStage); pg.push(pm); }
      // Carrier holes and pin stock are positioned from the same resolved geometry.
      const car = new TH.Group(); car.userData.stage = i;
      const shape = x.carrier;
      const body = new TH.Mesh(carrierBody(shape).translate(0, 0, zb), mkMat('carrier'));
      // Rounded pockets already carry their machined silhouette in the mesh.
      body.userData.smoothPocket = shape.pocketR > 0;
      car.add(body);
      const pinMat = mkMat('pin'), pinGeo = turnedUV(new TH.CylinderGeometry(shape.diameter / 2, shape.diameter / 2, shape.pinLength, 72).rotateX(Math.PI / 2));
      const pins = new TH.Group(), pinZ = zb + (shape.stock[0] + shape.stock[1]) / 2;
      for (const center of shape.pins) {
        const pin = new TH.Mesh(pinGeo, pinMat);
        pin.position.set(center.x, center.y, pinZ);
        pins.add(pin);
      }
      // needle rollers (instanced) between pin and planet bore
      let needles = null;
      if (extras && partOn(D, `needle${sid}`) && x.planetBore > st.pin.d + .6) {
        const rIn = st.pin.d / 2, rOut = x.planetBore / 2, nd = (rOut - rIn) * .92, rc = (rIn + rOut) / 2;
        const cnt = Math.max(6, Math.floor(2 * Math.PI * rc / (nd * 1.08)));
        needles = new TH.InstancedMesh(new TH.CylinderGeometry(nd / 2, nd / 2, b * .9, 10).rotateX(Math.PI / 2), mkMat('roller'), cnt * st.n);
        const m4 = new TH.Matrix4(); let k = 0;
        for (const pin of shape.pins) {
          for (let q = 0; q < cnt; q++) { const t = 2 * Math.PI * q / cnt; m4.makeTranslation(pin.x + rc * Math.cos(t), pin.y + rc * Math.sin(t), zb + b / 2); needles.setMatrixAt(k++, m4); }
        }
      }
      root.add(car);
      if (needles) needles.frustumCulled = false;
      if (partOn(D, `pin${sid}`)) add(pins, `pin${sid}`, car, 1, 0); if (needles) add(needles, `needle${sid}`, car, 1, 0);
      car.children.filter(c => c.isMesh && !c.userData.partId).forEach(c => { c.userData.partId = `carrier${sid}`; c.castShadow = c.receiveShadow = renderer.shadowMap.enabled; meshes.push(c); });
      if (!parts.has(`carrier${sid}`)) parts.set(`carrier${sid}`, { objs: [] }); parts.get(`carrier${sid}`).objs.push(car);
      movers.push({ obj: car, base: car.position.clone(), dz: dzStage, layer: 1 });
      const phase = Array.from({ length: st.n }, (_, j) => {
        const phi = 2 * Math.PI * j / st.n, pitchS = 2 * Math.PI / zs, delta = ((phi % pitchS) + pitchS) % pitchS;
        return Math.PI - Math.PI / zp + delta * zs / zp;
      });
      // Travel is measured from the assembled orbit. Clear the pin end and
      // the ring end face before moving the planet across either cylinder.
      const direction = Math.sign(dzStage);
      const clearance = Math.max(2, ringRadius * .09);
      const ringEnd = L.shared ? L.gearEnd : zb + b;
      const pull = direction < 0
        ? Math.max(b - shape.axial[0], b + zb - (L.shared ? 0 : zb))
        : Math.max(shape.axial[1], ringEnd - zb);
      const axialTravel = direction * (pull + clearance);
      const planetRadius = g.parts[1].da / 2;
      const radialTravel = Math.max(ringRadius - a, shape.diameter / 2) + planetRadius + clearance;
      kin.push({ axialTravel, radialTravel, planetRadius, zb, b, sun, planets: pg, car, a, zs, zp, ratio: g.ratio, n: st.n, phase, psi, dzStage, ownRing: !(L.shared && i > 0), ring });
    });

    // ---- housing, cover, input shaft, output bearing, flange, seals, bolts (full mode)
    // cross roller: inner race rides the output, outer race fixed, rollers on a half-speed cage
    const drawXroller = (outG, fixG, cageG) => {
      const raceMat = mkMat('race');
      // 카탈로그 형번: 단순 내륜·외륜 링 (내륜 내경·외륜 외경 삽입부 R 표현, 볼트 탭 취부는 후순위)
      const secW = L.raceOut - L.raceIn, Dw = L.cat ? Math.min(L.Dw, secW * .3, L.Bw * .7) : L.Dw, gap = L.cat ? Math.min(Dw * .35, secW * .1) : L.Dw * .35;
      const rI = L.cat ? (L.ob.rIn ?? L.ob.r ?? .5) : 0, rO = L.cat ? (L.ob.rOut ?? L.ob.r ?? .5) : 0;
      add(new TH.Mesh(L.cat ? ringRounded(L.raceIn, L.dp / 2 - gap, L.zBear[0], L.zBear[1], rI, 0) : ringSolid(L.raceIn, L.dp / 2 - gap, L.zBear[0], L.zBear[1], .3), raceMat), 'xroller', outG, 3, unit * 2.6);
      add(new TH.Mesh(L.cat ? ringRounded(L.dp / 2 + gap, L.raceOut, L.zBear[0], L.zBear[1], 0, rO) : ringSolid(L.dp / 2 + gap, L.raceOut, L.zBear[0], L.zBear[1], .3), raceMat), 'xroller', fixG, 3, unit * 2.6);
      const Z = Math.max(8, Math.round(L.ob.Z || (L.cat ? Math.PI * L.dp / (Dw * 1.3) : 40))), Lw = L.cat ? Dw * .9 : L.ob.Lw || Dw * .9;
      const rollers = new TH.InstancedMesh(new TH.CylinderGeometry(Dw / 2, Dw / 2, Lw, 16), mkMat('roller'), Z);
      const m4 = new TH.Matrix4(), q = new TH.Quaternion(), e = new TH.Euler(), sc = new TH.Vector3(1, 1, 1), p = new TH.Vector3();
      for (let j = 0; j < Z; j++) {
        const ang = 2 * Math.PI * j / Z, s = j % 2 ? 1 : -1;
        // roller axis in the (radial, axial) plane at ±45°
        const axis = new TH.Vector3(Math.cos(ang) * Math.SQRT1_2, Math.sin(ang) * Math.SQRT1_2, s * Math.SQRT1_2).normalize();
        q.setFromUnitVectors(new TH.Vector3(0, 1, 0), axis);
        p.set(L.dp / 2 * Math.cos(ang), L.dp / 2 * Math.sin(ang), (L.zBear[0] + L.zBear[1]) / 2);
        m4.compose(p, q, sc); rollers.setMatrixAt(j, m4);
      }
      rollers.frustumCulled = false;
      add(rollers, 'xroller', cageG, 3, unit * 2.6);
    };
    let outRot = null, cage = null, inRot = null;
    // 3D 종합판정(가공 부품만)에서도 카탈로그에서 고른 출력 베어링은 구매 부품으로 표시
    if (full && !extras && L.cat && partOn(D, 'xroller')) { outRot = new TH.Group(); root.add(outRot); cage = new TH.Group(); root.add(cage); drawXroller(outRot, root, cage); }
    if (extras) {   // 하우징 · 커버 · 중공축 · 썬 베어링 · 오일 실 · 플랜지 · 볼트 삭제 (BOM-1) — 출력 베어링만
      outRot = new TH.Group(); root.add(outRot); cage = new TH.Group(); root.add(cage);
      if (partOn(D, 'xroller')) drawXroller(outRot, root, cage);
    }

    // ---- crisp feature edges (thin graphite lines) on non-gear solids
    const edgeMats = [];
    const EDGE = /^(housing|cover|ocover|oflange|carrier\d|xroller|inshaft|oshaft|pin\d|brgS\d|seal)$/;
    meshes.slice().forEach(ms => {
      if (ms.isInstancedMesh || ms.userData.smoothPocket || !EDGE.test(ms.userData.partId || '')) return;
      const n = ms.geometry.attributes.position?.count || 0; if (!n || n > 120000) return;
      const dark = /^(carrier|bolt)/.test(ms.userData.partId);
      const lm = new TH.LineBasicMaterial({ color: dark ? 0x6b6f76 : 0x2a2724, transparent: true, opacity: dark ? .5 : .42, toneMapped: false });
      lm.userData.op = lm.opacity; edgeMats.push(lm);
      const ln = new TH.LineSegments(new TH.EdgesGeometry(ms.geometry, 28), lm); ln.raycast = () => { }; ln.userData.edge = true; ms.add(ln);
    });
    // ---- centre, ground, camera
    const box = new TH.Box3().setFromObject(root), ctr = box.getCenter(new TH.Vector3()), size = box.getSize(new TH.Vector3());
    root.position.sub(ctr);
    const maxR = Math.max(size.x, size.y) / 2, len = size.z;
    const ground = new TH.Mesh(new TH.PlaneGeometry(maxR * 30, maxR * 30), new TH.ShadowMaterial({ opacity: .4 }));
    ground.rotation.x = -Math.PI / 2; ground.position.y = -maxR * 1.35 - 2; ground.receiveShadow = true; ground.visible = renderer.shadowMap.enabled; scene.add(ground);
    // soft contact shadow (works without shadow maps, e.g. on phones): radial gradient texture under the model
    const blobTex = (() => { const c = document.createElement('canvas'); c.width = c.height = 256; const g = c.getContext('2d'), gr = g.createRadialGradient(128, 128, 0, 128, 128, 128);
      gr.addColorStop(0, 'rgba(214,160,90,.26)'); gr.addColorStop(.5, 'rgba(214,160,90,.09)'); gr.addColorStop(1, 'rgba(214,160,90,0)'); g.fillStyle = gr; g.fillRect(0, 0, 256, 256); const tx = new TH.CanvasTexture(c); tx.encoding = TH.sRGBEncoding; return tx; })();
    const blob = new TH.Mesh(new TH.PlaneGeometry(1, 1), new TH.MeshBasicMaterial({ map: blobTex, transparent: true, depthWrite: false, toneMapped: false }));
    blob.rotation.x = -Math.PI / 2; blob.position.y = ground.position.y + .05; blob.scale.set(maxR * 7, len * 2.4 + maxR * 4, 1); blob.renderOrder = -1; scene.add(blob);
    // floor grid: fine amber lines fading with distance, a lit patch under the model
    const gridSize = Math.ceil(maxR * 14 / 10) * 10, grid = new TH.GridHelper(gridSize, gridSize / 5, 0xd6a05a, 0xd6a05a);
    grid.material.transparent = true; grid.material.opacity = .13; grid.material.depthWrite = false; grid.material.toneMapped = false; grid.position.y = ground.position.y + .02; grid.renderOrder = -2; grid.raycast = () => { }; scene.add(grid); grid.visible = prefs.grid !== false;
    scene.fog = new TH.Fog(new TH.Color(prefs.bg).multiplyScalar(.7).getHex(), Math.max(maxR * 2, len) * 5, Math.max(maxR * 2, len) * 14);
    // construction axis: thin dash-dot centre line through the whole assembly (stretches with the explode)
    const axisMat = new TH.LineDashedMaterial({ color: 0xd6a05a, dashSize: 3, gapSize: 1.6, transparent: true, opacity: .5, toneMapped: false });
    const axisGeo = new TH.BufferGeometry().setFromPoints([new TH.Vector3(0, 0, box.min.z - ctr.z - unit * 6), new TH.Vector3(0, 0, box.max.z - ctr.z + unit * 6)]);
    const axisLine = new TH.Line(axisGeo, axisMat); axisLine.computeLineDistances(); axisLine.raycast = () => { }; scene.add(axisLine);
    // world: reducer axis = z, y up. Part centres (local to root) for focus / leader anchors
    root.updateMatrixWorld(true);
    parts.forEach((p, id) => { const bb = new TH.Box3(); p.objs.forEach(o => bb.expandByObject(o)); p.center = bb.getCenter(new TH.Vector3()); p.radius = bb.getSize(new TH.Vector3()).length() / 2; });
    const S0 = Math.max(maxR * 2, len);
    const expandedRadius = Math.max(maxR, ...kin.map(p => p.a + p.radialTravel + p.planetRadius));
    const expandedExtent = Math.max(...kin.map(p => {
      const z = p.zb + p.dzStage + p.axialTravel - ctr.z;
      return Math.hypot(p.a + p.radialTravel + p.planetRadius, Math.max(Math.abs(z), Math.abs(z + p.b)));
    }));
    const explodeZoom = separatePlanets ? Math.max(.75, expandedExtent * 2 / S0 - 1) : .75;
    const fitDist = () => S0 * 2.7 / Math.min(1, (host.clientWidth / Math.max(1, host.clientHeight)) / 1.25);
    const view = { target: new TH.Vector3(), az: Math.PI / 4, el: Math.atan(1 / Math.SQRT2), dist: fitDist() };
    const home = { az: view.az, el: view.el, dist: view.dist, target: new TH.Vector3() };
    if (opt.initialView) { const iv = opt.initialView; view.az = iv.az; view.el = iv.el; view.dist = iv.dist; view.target.fromArray(iv.target); }
    const goal = { target: view.target.clone(), az: view.az, el: view.el, dist: view.dist };
    const placeLights = () => {
      const a = (prefs.lightAz || 35) * Math.PI / 180, I = prefs.light ?? 1;
      key.position.set(S0 * 2 * Math.cos(a), S0 * 2.6, S0 * 2 * Math.sin(a) + S0); key.intensity = 2.2 * I;
      key.shadow.camera.left = key.shadow.camera.bottom = -S0 * 1.4; key.shadow.camera.right = key.shadow.camera.top = S0 * 1.4; key.shadow.camera.near = 1; key.shadow.camera.far = S0 * 10; key.shadow.camera.updateProjectionMatrix();
      fill.position.set(-S0 * 2, S0, S0); fill.intensity = .45 * I; rim.position.set(0, S0, -S0 * 3); rim.intensity = .8 * I; hemi.intensity = .25 * I;
      mats.forEach(m => m.envMapIntensity = (m.userData.base.envMapIntensity ?? 1.1) * (.5 + .5 * I));
    };
    placeLights();
    const place = () => {
      const t = view.target, ce = Math.cos(view.el);
      camera.position.set(t.x + view.dist * ce * Math.sin(view.az), t.y + view.dist * Math.sin(view.el), t.z + view.dist * ce * Math.cos(view.az));
      camera.lookAt(t);
    };
    place();

    // ---- state: explode, section, opacity, visibility, shading
    const st = { explodeTarget: 0, explode: 0, playing: true, theta: 0, section: false, secAngle: 0, hover: null, selected: null, issues: [] };
    if (opt.initialView) { st.explodeTarget = st.explode = opt.initialView.explode || 0; st.theta = opt.initialView.theta || 0; st.playing = opt.initialView.playing !== false; st.secAngle = opt.initialView.secAngle || 0; }
    const plane = new TH.Plane(new TH.Vector3(1, 0, 0), 0);
    const caps = [];
    function applyShade() {
      mats.forEach(m => {
        const b = m.userData.base;
        m.wireframe = prefs.shade === 'wire';
        if (prefs.shade === 'matte') { m.metalness = 0; m.roughness = .85; m.clearcoat = 0; }
        else { m.metalness = b.metalness; m.roughness = b.roughness; m.clearcoat = b.clearcoat || 0; }
        m.needsUpdate = true;
      });
    }
    function setSection(on, angDeg) {
      st.section = on; if (angDeg != null) st.secAngle = angDeg;
      mats.forEach(m => { m.clippingPlanes = on ? [plane] : null; m.clipShadows = on; m.side = on ? TH.DoubleSide : TH.FrontSide; m.needsUpdate = true; });
      edgeMats.forEach(m => { m.clippingPlanes = on ? [plane] : null; m.needsUpdate = true; });
      caps.splice(0).forEach(c => { c.parent?.remove(c); c.material.dispose(); });
      if (on) meshes.forEach(ms => {
        if (ms.isInstancedMesh || !ms.visible) return;
        const cm = new TH.MeshBasicMaterial({ color: 0xc8320e, side: TH.BackSide, clippingPlanes: [plane], toneMapped: false });
        const c = new TH.Mesh(ms.geometry, cm); c.raycast = () => { }; c.renderOrder = 1; ms.add(c); caps.push(c);
      });
    }
    function setOpacity(id, v, persist = true) {
      const list = id ? meshesOf(id) : meshes;
      list.forEach(ms => { const m = ms.material; m.opacity = v; m.transparent = v < .999; m.depthWrite = v >= .95; m.needsUpdate = true;
        ms.children.forEach(c => { if (c.userData.edge) c.material.opacity = c.material.userData.op * v; }); });
      if (persist && full) {
        prefs.opacity ??= {};
        for (const key of id ? [id] : parts.keys()) prefs.opacity[key] = v;
        savePrefs();
        opt.onOpacity?.();
      }
    }
    function meshesOf(id) { return meshes.filter(m => m.userData.partId === id); }
    function setVisible(id, v) { meshesOf(id).forEach(m => m.visible = v); }
    function isolate(id) { meshes.forEach(m => m.visible = m.userData.partId === id); }
    function showAll() { meshes.forEach(m => m.visible = true); }
    applyShade();
    if (full) {
      for (const key of parts.keys()) {
        const opacity = prefs.opacity?.[key] ?? (/^(ring|carrier)/.test(key) ? .28 : 1);
        setOpacity(key, opacity, false);
      }
    }
    if (opt.initialView?.section) setSection(true);

    // ---- highlight
    const HL = { hover: new TH.Color(0x1a1612), sel: new TH.Color(0x5c2208) };
    function paint() {
      meshes.forEach(ms => { const m = ms.material; if (!m.emissive) return; m.emissive.setRGB(0, 0, 0); });
      if (st.hover) meshesOf(st.hover).forEach(ms => ms.material.emissive?.copy(HL.hover));
      if (st.selected) meshesOf(st.selected).forEach(ms => ms.material.emissive?.copy(HL.sel));
    }
    // shared materials across a part are fine; planets share one material per stage (by design: highlight together)

    // ---- focus
    function focus(id) {
      const p = parts.get(id); if (!p) return;
      const wc = p.center.clone().applyMatrix4(root.matrixWorld);
      goal.target.copy(wc); goal.dist = Math.max(S0 * 1.1, p.radius * 4.2); goal.az = view.az; goal.el = view.el;
    }
    function resetView() { st.zoomed = false; goal.target.copy(home.target); goal.az = home.az; goal.el = home.el; goal.dist = home.dist = fitDist() * (1 + explodeZoom * st.explodeTarget); }

    // ---- pointer: orbit with inertia, pan (shift / middle), wheel zoom, pick, context menus
    const cv = renderer.domElement, ray = new TH.Raycaster(), ndc = new TH.Vector2();
    let drag = null, vel = { az: 0, el: 0 }, lastHover = 0;
    const pick = (cx, cy) => {
      const r = cv.getBoundingClientRect(); ndc.set((cx - r.left) / r.width * 2 - 1, -(cy - r.top) / r.height * 2 + 1);
      ray.setFromCamera(ndc, camera);
      const hit = ray.intersectObjects(meshes.filter(m => m.visible), false).find(h => !st.section || plane.distanceToPoint(h.point) >= -1e-6);
      return hit ? hit.object.userData.partId : null;
    };
    cv.addEventListener('contextmenu', e => e.preventDefault());
    // 두 손가락: 벌리고 오므리면 확대·축소 (모바일)
    const touches = new Map(); let pinch0 = 0, pinchD = 0;
    const pinchDist = () => { const [a, b] = [...touches.values()]; return Math.hypot(a.x - b.x, a.y - b.y); };
    cv.addEventListener('pointerdown', e => { if (e.pointerType === 'touch') { touches.set(e.pointerId, { x: e.clientX, y: e.clientY }); if (touches.size === 2) { pinch0 = pinchDist(); pinchD = goal.dist; drag = null; } } }, true);
    cv.addEventListener('pointermove', e => { if (!touches.has(e.pointerId)) return; touches.set(e.pointerId, { x: e.clientX, y: e.clientY }); if (touches.size === 2 && pinch0 > 0) { goal.dist = clamp(pinchD * pinch0 / pinchDist(), S0 * .5, S0 * 12); st.zoomed = true; e.stopImmediatePropagation(); } }, true);
    const untouch = e => { touches.delete(e.pointerId); if (touches.size < 2) pinch0 = 0; };
    cv.addEventListener('pointerup', untouch, true); cv.addEventListener('pointercancel', untouch, true);
    cv.addEventListener('pointerdown', e => {
      closeMenu();
      if (e.button === 2 || touches.size > 1) return;
      drag = { x: e.clientX, y: e.clientY, sx: e.clientX, sy: e.clientY, pan: e.shiftKey || e.button === 1, t: performance.now() };
      cv.setPointerCapture(e.pointerId); vel.az = vel.el = 0;
    });
    cv.addEventListener('pointermove', e => {
      if (drag) {
        const dx = e.clientX - drag.x, dy = e.clientY - drag.y; drag.x = e.clientX; drag.y = e.clientY;
        if (drag.pan) {
          const k = view.dist * .0012, right = new TH.Vector3().setFromMatrixColumn(camera.matrix, 0), up = new TH.Vector3().setFromMatrixColumn(camera.matrix, 1);
          goal.target.addScaledVector(right, -dx * k).addScaledVector(up, dy * k); view.target.copy(goal.target);
        } else { vel.az = -dx * .006; vel.el = dy * .006; goal.az += vel.az; goal.el = clamp(goal.el + vel.el, -1.45, 1.45); view.az = goal.az; view.el = goal.el; }
        return;
      }
      const now = performance.now(); if (now - lastHover < 40) return; lastHover = now;
      const id = pick(e.clientX, e.clientY); if (id !== st.hover) { st.hover = id; paint(); showTip(id, e); } else if (id) moveTip(e);
    });
    cv.addEventListener('pointerleave', () => { st.hover = null; paint(); hideTip(); });
    cv.addEventListener('pointerup', e => {
      if (!drag) return;
      const moved = Math.hypot(e.clientX - drag.sx, e.clientY - drag.sy) > 4; drag = null;
      if (!moved) { const id = pick(e.clientX, e.clientY); select(id, false); opt.onPick?.(id); vel.az = vel.el = 0; }
    });
    cv.addEventListener('dblclick', () => resetView());
    cv.addEventListener('wheel', e => { e.preventDefault(); goal.dist = clamp(goal.dist * (1 + Math.sign(e.deltaY) * .1), S0 * .5, S0 * 12); st.zoomed = true; }, { passive: false });
    cv.addEventListener('pointerdown', e => { if (e.button === 2) { const id = pick(e.clientX, e.clientY); openMenu(e, id); } });

    // ---- tooltip, menus, overlay (DOM)
    const ui = document.createElement('div'); ui.className = 'v3-ui'; host.appendChild(ui);
    const tip = document.createElement('div'); tip.className = 'v3-tip'; ui.appendChild(tip);
    const svg = document.createElementNS('http://www.w3.org/2000/svg', 'svg'); svg.setAttribute('class', 'v3-lead'); ui.appendChild(svg);
    const labels = document.createElement('div'); labels.className = 'v3-labels'; ui.appendChild(labels);
    const nameOf = id => (opt.names && opt.names[id]) || id;
    function showTip(id, e) { if (!id) return hideTip(); tip.textContent = nameOf(id); tip.style.display = 'block'; moveTip(e); }
    function moveTip(e) { const r = host.getBoundingClientRect(); tip.style.left = (e.clientX - r.left + 14) + 'px'; tip.style.top = (e.clientY - r.top + 12) + 'px'; }
    function hideTip() { tip.style.display = 'none'; }
    let menu = null;
    function closeMenu() { menu?.remove(); menu = null; }
    function openMenu(e, id) {
      closeMenu(); hideTip();
      const r = host.getBoundingClientRect();
      menu = document.createElement('div'); menu.className = 'v3-menu';
      menu.style.left = Math.min(e.clientX - r.left, r.width - 250) + 'px'; menu.style.top = Math.min(e.clientY - r.top, r.height - 300) + 'px';
      if (id) {
        const op = meshesOf(id)[0]?.material.opacity ?? 1;
        menu.innerHTML = `<div class="h">${nameOf(id)}</div>
          <label>투명도<input type="range" min="0.05" max="1" step="0.05" value="${op}" data-a="op"></label>
          <label>전체 투명도<input type="range" min="0.05" max="1" step="0.05" value="${meshes[0].material.opacity}" data-a="opAll"></label>
          <button data-a="explode">${st.explodeTarget > .5 ? '조립' : '분해도'}</button>
          <button data-a="section">${st.section ? '단면 끄기' : '단면도'}</button>
          <button data-a="hide">숨기기</button><button data-a="iso">이것만 보기</button><button data-a="all">모두 보이기</button>
          <button data-a="detail">상세 설계 보기</button>`;
      } else {
        menu.innerHTML = `<div class="h">보기 설정</div>
          <label>1단 썬 입력 rpm<input type="number" value="${D.anim.rpm}" step="any" data-a="rpm"></label>
          <label>재생 배속<select data-a="slow">${[[1, '1× 실속도'], [.2, '0.2×'], [.05, '0.05×'], [.02, '0.02×'], [.005, '0.005×']].map(([v, l]) => `<option value="${v}" ${+D.anim.slow === v ? 'selected' : ''}>${l}</option>`).join('')}</select></label>
          <label>배경색<input type="color" value="${prefs.bg}" data-a="bg"></label>
          <label>음영<select data-a="shade">${[['pbr', '금속 (PBR)'], ['matte', '매트'], ['wire', '와이어프레임']].map(([v, l]) => `<option value="${v}" ${prefs.shade === v ? 'selected' : ''}>${l}</option>`).join('')}</select></label>
          <label>광원 세기<input type="range" min="0.2" max="2" step="0.05" value="${prefs.light}" data-a="light"></label>
          <label>광원 방향<input type="range" min="0" max="360" step="5" value="${prefs.lightAz}" data-a="lightAz"></label>
          <button data-a="${st.playing ? 'pause' : 'play'}">${st.playing ? '회전 정지' : '회전 재생'}</button><button data-a="home">기본 시점</button>`;
      }
      ui.appendChild(menu);
      menu.addEventListener('pointerdown', ev => ev.stopPropagation());
      menu.querySelectorAll('[data-a]').forEach(el => {
        const a = el.dataset.a, ev = el.tagName === 'BUTTON' ? 'click' : 'input';
        el.addEventListener(ev, () => {
          const v = el.value;
          if (a === 'op') setOpacity(id, +v); else if (a === 'opAll') setOpacity(null, +v);
          else if (a === 'explode') { setExplode(st.explodeTarget > .5 ? 0 : 1); closeMenu(); }
          else if (a === 'section') { setSection(!st.section); opt.onState?.(); closeMenu(); }
          else if (a === 'hide') { setVisible(id, false); closeMenu(); } else if (a === 'iso') { isolate(id); focus(id); closeMenu(); }
          else if (a === 'all') { showAll(); closeMenu(); } else if (a === 'detail') { closeMenu(); opt.onDetail?.(id); }
          else if (a === 'rpm') { D.anim.rpm = Math.max(0, +v || 0); opt.save?.(); }
          else if (a === 'slow') { D.anim.slow = +v; opt.save?.(); }
          else if (a === 'bg') { prefs.bg = v; paintBg(); savePrefs(); }
          else if (a === 'shade') { prefs.shade = v; applyShade(); savePrefs(); }
          else if (a === 'light') { prefs.light = +v; placeLights(); savePrefs(); }
          else if (a === 'lightAz') { prefs.lightAz = +v; placeLights(); savePrefs(); }
          else if (a === 'pause' || a === 'play') { st.playing = a === 'play'; closeMenu(); }
          else if (a === 'home') { resetView(); closeMenu(); }
        });
      });
    }
    function savePrefs() { D.view = Object.assign({}, prefs); opt.save?.(); }
    function onDocDown(e) { if (menu && !menu.contains(e.target)) closeMenu(); }
    document.addEventListener('pointerdown', onDocDown);

    // ---- leader lines (issues → part anchors, labels stacked on the right edge)
    function setIssues(list) {
      st.issues = (list || []).filter(x => parts.has(x.partId));
      labels.innerHTML = st.issues.map((x, k) => `<button class="v3-lab ${x.sev || 'bad'}" data-k="${k}"><b>${k + 1}</b><span>${x.title}</span></button>`).join('');
      labels.querySelectorAll('.v3-lab').forEach(b => b.onclick = () => { const it = st.issues[+b.dataset.k]; select(it.partId, true); opt.onIssue?.(it); });
    }
    const v3 = new TH.Vector3();
    function layoutLeaders() {
      const W = host.clientWidth, H = host.clientHeight; svg.setAttribute('viewBox', `0 0 ${W} ${H}`);
      if (!st.issues.length) { svg.innerHTML = ''; return; }
      const pts = st.issues.map((x, k) => { const p = parts.get(x.partId); v3.copy(p.center).applyMatrix4(root.matrixWorld).project(camera); return { k, x: (v3.x + 1) / 2 * W, y: (1 - v3.y) / 2 * H, z: v3.z }; });
      const col = W - 236, gap = 46, ord = pts.slice().sort((a, b) => a.y - b.y);
      let y = 24; ord.forEach(p => { p.ly = Math.max(y, Math.min(p.y, H - 24 - gap * (ord.length - 1 - ord.indexOf(p)))); y = p.ly + gap; });
      svg.innerHTML = ord.map(p => `<polyline points="${p.x},${p.y} ${col - 24},${p.ly} ${col},${p.ly}" /><circle cx="${p.x}" cy="${p.y}" r="3.5" />`).join('');
      const els = labels.children; ord.forEach(p => { const el = els[p.k]; if (el) { el.style.left = col + 'px'; el.style.top = (p.ly - 15) + 'px'; } });
    }

    function select(id, doFocus) { st.selected = id; paint(); if (id && doFocus) focus(id); }
    // keep the exploded assembly in frame unless the user has zoomed by hand
    function setExplode(v) { st.explodeTarget = clamp(v, 0, 1); if (!st.zoomed) goal.dist = fitDist() * (1 + explodeZoom * st.explodeTarget); }

    // ---- animation loop
    const reduce = window.matchMedia && window.matchMedia('(prefers-reduced-motion: reduce)').matches;
    let last = performance.now(), raf;
    const nLayers = 5, stagger = .12;
    let onScreen = true, slowFrames = 0, shadowTick = 0, curDPR = DPR, wasActive = true;
    const io = new IntersectionObserver(es => { onScreen = es[0].isIntersecting; }, { threshold: 0 }); io.observe(host);
    let frameDue = 0, sampleStart = performance.now(), sampleCount = 0;
    function frame(now) {
      raf = requestAnimationFrame(frame);
      if (!onScreen || document.hidden) { last = sampleStart = now; sampleCount = 0; frameDue = 0; return; }
      const limit = prefs.frameCap;
      if (limit > 0) {
        if (now + .5 < frameDue) return;
        frameDue = Math.max(frameDue + 1000 / limit, now + 500 / limit);
      }
      if (opt.onFrameRate) {
        sampleCount++;
        if (now - sampleStart >= 1000) {
          opt.onFrameRate(Math.round(1000 * sampleCount / (now - sampleStart)));
          sampleStart = now; sampleCount = 0;
        }
      }
      const rawDt = (now - last) / 1000, dt = Math.min(.05, rawDt); last = now;
      // adaptive resolution: drop pixel ratio when frames stay slow, restore when fast again
      if (rawDt > .034) slowFrames++; else if (rawDt < .02) slowFrames = Math.max(0, slowFrames - 1);
      if (slowFrames > 20 && curDPR > 1) { curDPR = Math.max(1, curDPR - .25); renderer.setPixelRatio(curDPR); slowFrames = 0; }
      const moving = Math.abs(st.explodeTarget - st.explode) > 1e-4;
      if (st.playing && !reduce) st.theta += dt * (+D.anim.rpm || 0) * (+D.anim.slow || 0) * 2 * Math.PI / 60;
      // explode easing (0.8 s for the full travel, staggered by layer)
      const sp = dt / .8; st.explode += clamp(st.explodeTarget - st.explode, -sp, sp);
      const span = 1 + stagger * (nLayers - 1);
      const eLayer = l => ease(clamp(st.explode * span - stagger * l, 0, 1));
      movers.forEach(m => { m.obj.position.z = m.base.z + m.dz * eLayer(m.layer); });
      // camera damping + inertia
      if (!drag) { goal.az += vel.az; goal.el = clamp(goal.el + vel.el, -1.45, 1.45); vel.az *= .9; vel.el *= .9; if (Math.abs(vel.az) < 1e-5) vel.az = 0; if (Math.abs(vel.el) < 1e-5) vel.el = 0; }
      const k = 1 - Math.pow(.0008, dt);
      view.az += (goal.az - view.az) * k; view.el += (goal.el - view.el) * k; view.dist += (goal.dist - view.dist) * k; view.target.lerp(goal.target, k);
      place();
      // kinematics: sun → carrier, ring fixed (verified phasing)
      const pullOut = separatePlanets ? ease(clamp(st.explode / .48, 0, 1)) : 0;
      const spread = separatePlanets ? ease(clamp((st.explode - .48) / .52, 0, 1)) : 0;
      if (separatePlanets) {
        ground.position.y = -maxR * 1.35 - 2 - spread * (expandedRadius - maxR);
        grid.position.y = ground.position.y + .02;
        blob.position.y = ground.position.y + .05;
      }
      let sunAng = st.theta;
      kin.forEach((p, i) => {
        const carA = sunAng / p.ratio;
        p.sun.rotation.z = sunAng + p.psi;
        p.car.rotation.z = carA + p.psi;
        const er = separatePlanets ? p.radialTravel * spread : eLayer(1) * (full ? p.a * .35 : 0);
        p.planets.forEach((pm, j) => {
          const phi = 2 * Math.PI * j / p.n + carA + p.psi;
          if (separatePlanets) pm.position.z = p.zb + p.car.position.z + p.axialTravel * pullOut;
          pm.position.x = (p.a + er) * Math.cos(phi); pm.position.y = (p.a + er) * Math.sin(phi);
          pm.rotation.z = p.phase[j] + phi - (sunAng - carA) * p.zs / p.zp;
        });
        sunAng = carA;
      });
      if (inRot) inRot.rotation.z = st.theta + (kin[0]?.psi || 0);
      if (outRot) outRot.rotation.z = sunAng; if (cage) cage.rotation.z = sunAng / 2;
      // section plane through the axis (world)
      if (st.section) { const a = st.secAngle * Math.PI / 180; plane.normal.set(-Math.cos(a), -Math.sin(a), 0); plane.constant = 0; plane.applyMatrix4(root.matrixWorld); }
      const active = moving || st.playing;   // gear spin barely moves the shadow: refresh every 6th frame, once more on stop
      if (renderer.shadowMap.enabled && (active ? ++shadowTick % 6 === 1 : wasActive)) renderer.shadowMap.needsUpdate = true;
      wasActive = active;
      renderer.render(scene, camera);
      layoutLeaders();
    }
    raf = requestAnimationFrame(frame);
    const ro = new ResizeObserver(() => { const w = host.clientWidth, h = host.clientHeight; if (!w || !h) return; renderer.setSize(w, h); camera.aspect = w / h; camera.updateProjectionMatrix(); });
    ro.observe(host);
    function dispose() {
      cancelAnimationFrame(raf); ro.disconnect(); io.disconnect(); document.removeEventListener('pointerdown', onDocDown);
      scene.traverse(o => { o.geometry?.dispose?.(); if (o.material) [].concat(o.material).forEach(m => m.dispose?.()); });
      scene.environment?.dispose?.(); renderer.dispose(); renderer.domElement.remove(); ui.remove();
    }
    return {
      axialView(side) { resetView(); vel.az = vel.el = 0; goal.el = 0; goal.az = side > 0 ? 0 : Math.PI; },
      setFrameCap(limit) { prefs.frameCap = limit; frameDue = 0; savePrefs(); },
      setBackground(color) { prefs.bg = color; paintBg(); savePrefs(); },
      setGrid(on) { grid.visible = prefs.grid = on; savePrefs(); },
      dispose, select, focus, resetView, setExplode, setSection, setIssues, setOpacity, showAll, setVisible, isolate,
      get state() { return st; }, getView: () => ({ az: goal.az, el: goal.el, dist: goal.dist, target: goal.target.toArray(), explode: st.explodeTarget, theta: st.theta, playing: st.playing, section: st.section, secAngle: st.secAngle, selected: st.selected }), setSectionAngle: a => { st.secAngle = a; }, togglePlay: () => (st.playing = !st.playing),
      partIds: () => [...parts.keys()]
    };
  }

  // per-part dimension schema: engine inputs (path into D) and geometry-only overrides (D.geom.part.key)
  function partSchema(R, D, id) {
    const L = layout(R, D), A = L.auto, out = [];
    const E = (path, label, val, o = {}) => out.push({ path, label, value: val, engine: true, ...o });
    const Gm = (part, key, label, o = {}) => { const v = D.geom?.[part]?.[key]; out.push({ path: `geom.${part}.${key}`, label, value: Number.isFinite(v) && v > 0 ? v : null, auto: A[part]?.[key], engine: false, ...o }); };
    const m = /^(sun|planet|ring|pin|needle|carrier)(\d?)$/.exec(id);
    if (m) {
      const i = m[2] ? +m[2] - 1 : 0, s = D.stages[i], g = R.ctx.stages[i]?.geo, lock = i === 1 && D.config.sharedRing;
      if (m[1] === 'sun') { E(`stages.${i}.s`, '잇수', s.s, { step: 1 }); E(`stages.${i}.m`, '모듈', s.m, { lock }); E(`stages.${i}.b`, '치폭', s.b); E(`stages.${i}.beta`, '비틀림각 °', s.beta, { lock }); E(`stages.${i}.xs`, '전위계수', s.xs); E(`stages.${i}.sunBore`, '보어 (0=중실)', s.sunBore, { auto: g?.rims?.[0]?.diameter }); Gm(`sun${i + 1}`, 'hubInOD', '입력측 축(허브) 외경'); Gm(`sun${i + 1}`, 'hubInL', '입력측 축(허브) 길이'); Gm(`sun${i + 1}`, 'hubOutOD', '출력측 축 외경'); Gm(`sun${i + 1}`, 'hubOutL', '출력측 축 길이'); }
      if (m[1] === 'planet') { E(`stages.${i}.p`, lock && D.stages[0].r == null ? '잇수 (링 공유 → 자동 = (R − S2)/2)' : '잇수', s.p, { step: 1, lock: lock && D.stages[0].r == null }); E(`stages.${i}.n`, '개수', s.n, { step: 1 }); E(`stages.${i}.xp`, '전위계수', s.xp); E(`stages.${i}.b`, '치폭 (썬과 공통)', s.b); E(`stages.${i}.planetBore`, '보어 (0=중실)', s.planetBore, { auto: g?.rims?.[1]?.diameter }); }
      if (m[1] === 'ring') {
        E(`stages.${i}.r`, '잇수 (빈칸 = 자동)', s.r, { step: 1, auto: g?.r });
        E(`stages.${i}.ringOD`, '외경', s.ringOD, { auto: g?.rims?.[2]?.diameter });
        E(`stages.${i}.xr`, '전위계수', s.autoRingShift ? g?.parts[2]?.x : s.xr, { lock: s.autoRingShift });
      }
      const shape = R.ctx.stages[i]?.carrier;
      if (m[1] === 'pin') {
        E(`stages.${i}.pin.d`, '핀 지름 (mm)', s.pin.d);
        E(`stages.${i}.pin.holeMode`, '홀 가공', s.pin.holeMode, { options: [['through', '관통'], ['blind', '깊이 지정']] });
        if (s.pin.holeMode === 'blind') E(`stages.${i}.pin.depth`, '원통 깊이 (mm)', s.pin.depth, { auto: shape?.hole?.autoDepth });
        E('', '핀 산출 길이 (mm)', shape?.pinLength?.toFixed(3), { readonly: true });
      }
      if (m[1] === 'needle') { E(`stages.${i}.bearing.C`, '동정격 C N', s.bearing.C); E(`stages.${i}.bearing.C0`, '정정격 C0 N', s.bearing.C0); E(`stages.${i}.bearing.count`, '유성당 개수', s.bearing.count, { step: 1 }); }
      if (m[1] === 'carrier') {
        E(`stages.${i}.pin.type`, '지지 형식', s.pin.type, { options: [['double', '양단 지지'], ['single', '편단 지지']] });
        E(`stages.${i}.carrier.width`, '캐리어 두께 (mm, 보스 제외)', s.carrier?.width, { auto: shape?.widthAuto ? shape.width : undefined });
        E('', '판 두께 (자동, mm)', shape ? shape.wallShort ? '부족 (두께 ≤ 안착 폭) · 0.3으로 표시' : (shape.double ? `${shape.wall.toFixed(2)} × 2` : shape.wall.toFixed(2)) : '—', { readonly: true });
        E(`stages.${i}.carrier.outerD`, '캐리어 최대외경 (mm)', s.carrier?.outerD, { auto: shape?.outerD });
        E(`stages.${i}.carrier.innerD`, '캐리어 내경 (mm)', s.carrier?.innerD, { auto: shape?.innerD });
        E(`stages.${i}.carrier.bossD`, '보스 외경 (mm)', s.carrier?.bossD, { auto: shape?.bossD });
        E(`stages.${i}.carrier.bossT`, '보스 두께 (mm, 0=없음)', s.carrier?.bossT);
        E(`stages.${i}.pin.d`, '유성 핀 지름 (mm)', s.pin.d);
        E(`stages.${i}.pin.holeMode`, '핀 홀 가공', s.pin.holeMode || 'through', { options: [['through', '관통'], ['blind', '깊이 지정 (막힌 홀)']] });
        if (s.pin.holeMode === 'blind') E(`stages.${i}.pin.depth`, '핀 홀 원통 깊이 (mm, 118° 끝 제외)', s.pin.depth, { auto: shape?.hole?.autoDepth });
        E('', '핀 길이 (자동, mm)', shape?.pinLength != null ? shape.pinLength.toFixed(2) : '—', { readonly: true });
        if (shape?.double) {
          E(`stages.${i}.carrier.groove`, '양단 중앙 홈의 축방향 폭 (mm)', s.carrier?.groove, { auto: shape.groove });
          E(`stages.${i}.carrier.pocket`, '유성 포켓 (mm)', s.carrier?.pocket, { auto: shape.pocket });
          E(`stages.${i}.carrier.pocketR`, '포켓 모서리 R (엔드밀 반경, mm)', s.carrier?.pocketR, { auto: shape.pocketR ?? undefined, step: .5 });
        } else E(`stages.${i}.carrier.axialClearance`, '지지판과 기어 사이 간격 (mm)', s.carrier?.axialClearance);
      }
    } else if (id === 'xroller') {
      E('outBearing.dp', '롤러 피치원 dp', D.outBearing.dp); E('outBearing.Z', '롤러 수', D.outBearing.Z, { step: 1 }); E('outBearing.Dw', '롤러 지름', D.outBearing.Dw); E('outBearing.Lw', '롤러 길이', D.outBearing.Lw);
      E('outBearing.preload', '예압 µm', D.outBearing.preload); E('outBearing.C', 'C N', D.outBearing.C); E('outBearing.C0', 'C0 N', D.outBearing.C0); Gm('xroller', 'width', '궤도륜 폭');
    }
    if (partLocked(D, id)) out.forEach(r => r.lock = true);
    return out;
  }


  // ------------------------------------------------------------------ part studio: one machined part (S · P · R · C) with live dimensions
  // opt: { R, D, id, dims: [{ a:[x,y,z], b:[x,y,z], text, path?, value?, lock?, kind? }], onEdit(path, value), view, section }
  // dims are in part coordinates (gear face 0 … b along +Z, axis = Z), the same frame the geometry is built in.
  function mountPart(host, opt) {
    const TH = T(); if (!TH || !host) return null;
    const { R, D, id } = opt, L = layout(R, D);
    const m = /^(sun|planet|ring|carrier)(\d?)$/.exec(id); if (!m) return null;
    const i = m[2] ? +m[2] - 1 : 0, x = L.S[i], g = x.g, st = x.st, b = st.b;
    const W0 = host.clientWidth || 640, H0 = host.clientHeight || 420;
    const renderer = new TH.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setPixelRatio(Math.min(1.5, window.devicePixelRatio || 1)); renderer.setSize(W0, H0);
    renderer.outputEncoding = TH.sRGBEncoding; renderer.toneMapping = TH.ACESFilmicToneMapping; renderer.setClearColor(0x000000, 0);
    renderer.localClippingEnabled = true;
    host.prepend(renderer.domElement);
    const scene = new TH.Scene(); scene.environment = makeEnv(renderer);
    scene.add(new TH.HemisphereLight(0xfffaf2, 0x5d5a54, .55));
    const key = new TH.DirectionalLight(0xfff3e4, 2.1); key.position.set(70, 90, 140); scene.add(key);
    const rim = new TH.DirectionalLight(0xd6a05a, .9); rim.position.set(-90, -40, -80); scene.add(rim);
    const camera = new TH.PerspectiveCamera(28, W0 / H0, .5, 5000);
    const clip = new TH.Plane(new TH.Vector3(0, -1, 0), 0);
    let section = !!opt.section;
    const mats = [];
    // 종합판정과 같은 선삭 질감 (거칠기 맵 + 거칠기 1.35배)
    const turned = turnedTexture(renderer);
    const mat = (c, metal = .95, rough = .26) => { const mm = new TH.MeshStandardMaterial({ color: c, metalness: metal, roughness: Math.min(1, rough * 1.35), roughnessMap: turned, envMapIntensity: 1.15, side: TH.DoubleSide, clippingPlanes: section ? [clip] : [] }); mats.push(mm); return mm; };
    const COL = { sun: 0xc89b4a, planet: 0x4f7fc4, ring: 0xd2d4d8, carrier: 0x3a3c42, pin: 0xf2f3f5 };
    const root = new TH.Group(); scene.add(root);
    const lathe_ = (ri, ro, z0, z1, c) => new TH.Mesh(lathe([[ri, z0], [ro, z0], [ro, z1], [ri, z1], [ri, z0]].map(([r, z]) => [Math.max(r, .01), z])), mat(c));
    const G = D.geom || {};
    if (m[1] === 'sun') {
      root.add(new TH.Mesh(gearGeometry(g.parts[0], g.at, b, st.beta, false, x.sunBore), mat(COL.sun)));
      const h = G['sun' + (i + 1)] || {}, rb = (x.sunBore || 0) / 2;
      if (h.hubInOD > 0 && h.hubInL > 0) root.add(lathe_(rb, h.hubInOD / 2, -h.hubInL, 0, COL.sun));
      if (h.hubOutOD > 0 && h.hubOutL > 0) root.add(lathe_(rb, h.hubOutOD / 2, b, b + h.hubOutL, COL.sun));
    } else if (m[1] === 'planet') {
      root.add(new TH.Mesh(gearGeometry(g.parts[1], g.at, b, -st.beta, false, x.planetBore), mat(COL.planet)));
    } else if (m[1] === 'ring') {
      const depth = L.shared ? L.gearEnd - (L.S[0].z0 || 0) : b;
      root.add(new TH.Mesh(gearGeometry(g.parts[2], g.at, depth, -st.beta, true, x.ringOD), mat(COL.ring, .9, .2)));
    } else {
      const shape = x.carrier;
      root.add(new TH.Mesh(carrierBody(shape), mat(COL.carrier, .85, .3)));
      if (opt.pins !== false) {
        const pg = turnedUV(new TH.CylinderGeometry(shape.diameter / 2, shape.diameter / 2, shape.pinLength, 48).rotateX(Math.PI / 2)), pm = mat(COL.pin, 1, .08);
        shape.pins.forEach(pp => { const q = new TH.Mesh(pg, pm); q.position.set(pp.x, pp.y, (shape.stock[0] + shape.stock[1]) / 2); root.add(q); });
      }
    }
    const box = new TH.Box3().setFromObject(root), ctr = box.getCenter(new TH.Vector3()), size = box.getSize(new TH.Vector3()).length();
    root.position.sub(ctr);
    const fitD = () => { const r = size / 2, v = Math.tan(14 * Math.PI / 180), asp = Math.max(.4, (host.clientWidth || W0) / (host.clientHeight || H0)); return r / Math.min(v, v * asp) * 1.12; };
    const view = Object.assign({ az: .75, el: .42, dist: fitD() }, opt.view || {});
    const place = () => { const ce = Math.cos(view.el); camera.position.set(view.dist * ce * Math.sin(view.az), view.dist * Math.sin(view.el), view.dist * ce * Math.cos(view.az)); camera.lookAt(0, 0, 0); };
    // the part axis (Z) lies horizontally on screen: rotate the part so +Z (output side) points to the right
    root.rotation.y = Math.PI / 2; root.position.applyAxisAngle(new TH.Vector3(0, 1, 0), Math.PI / 2);
    // ---- overlay: dimension lines + editable labels
    const ui = document.createElement('div'); ui.className = 'pv-ui'; host.appendChild(ui);
    const svg = document.createElementNS('http://www.w3.org/2000/svg', 'svg'); svg.setAttribute('class', 'pv-dim'); ui.appendChild(svg);
    const labs = document.createElement('div'); labs.className = 'pv-labs'; ui.appendChild(labs);
    let dims = opt.dims || [], showDims = opt.showDims !== false, editing = null;
    const drawLabels = () => {
      labs.innerHTML = dims.map((d, k) => `<button class="pv-lab ${d.path && !d.lock ? 'ed' : 'ro'} ${d.kind || ''}" data-k="${k}" title="${d.path && !d.lock ? '클릭해서 수정' : '계산값'}">${d.text}</button>`).join('');
      labs.querySelectorAll('.pv-lab').forEach(btn => btn.onclick = e => {
        e.stopPropagation(); const d = dims[+btn.dataset.k]; opt.onFocus?.(d);
        if (!d.path || d.lock) return;
        editing = +btn.dataset.k;
        const inp = document.createElement('input'); inp.type = 'number'; inp.step = 'any'; inp.className = 'pv-in'; inp.value = d.value ?? ''; inp.placeholder = d.auto != null ? '자동 ' + (+d.auto).toFixed(2) : '';
        inp.style.left = btn.style.left; inp.style.top = btn.style.top; labs.appendChild(inp); inp.focus(); inp.select();
        const done = ok => { if (editing === null) return; editing = null; const v = inp.value === '' ? null : Number(inp.value); inp.remove(); if (ok && (v === null || Number.isFinite(v))) opt.onEdit?.(d.path, v); };
        inp.onkeydown = ev => { if (ev.key === 'Enter') done(true); else if (ev.key === 'Escape') done(false); };
        inp.onblur = () => done(true);
      });
    };
    drawLabels();
    const v3 = new TH.Vector3(), toScreen = p => { v3.set(p[0], p[1], p[2]).applyMatrix4(root.matrixWorld).project(camera); return { x: (v3.x + 1) / 2 * host.clientWidth, y: (1 - v3.y) / 2 * host.clientHeight, ok: v3.z < 1 }; };
    function layoutDims() {
      if (!showDims) { svg.innerHTML = ''; labs.style.display = 'none'; return; }
      labs.style.display = '';
      let out = '';
      const els = labs.querySelectorAll('.pv-lab');
      dims.forEach((d, k) => {
        const A = toScreen(d.a), B = toScreen(d.b), el = els[k]; if (!el) return;
        if (!A.ok || !B.ok) { el.style.display = 'none'; return; }
        el.style.display = '';
        const dx = B.x - A.x, dy = B.y - A.y, len = Math.hypot(dx, dy) || 1, nx = -dy / len * 5, ny = dx / len * 5;
        const tick = P => `M${P.x - nx} ${P.y - ny}L${P.x + nx} ${P.y + ny}`;
        out += `<path class="${d.kind || ''}" d="M${A.x} ${A.y}L${B.x} ${B.y}${tick(A)}${tick(B)}"/>`;
        const ax = d.at === 'b' ? B.x : (A.x + B.x) / 2, ay = d.at === 'b' ? B.y : (A.y + B.y) / 2;
        const mx = ax + (d.off?.[0] || 0), my = ay + (d.off?.[1] || 0);
        el.style.left = mx + 'px'; el.style.top = my + 'px';
      });
      svg.innerHTML = out;
    }
    // ---- pointer orbit
    const cv = renderer.domElement; let drag = null;
    cv.style.touchAction = 'none';
    const touches = new Map(); let pinch0 = 0, pinchD = 0;
    const pinchDist = () => { const [a, b] = [...touches.values()]; return Math.hypot(a.x - b.x, a.y - b.y); };
    cv.addEventListener('pointerdown', e => { if (e.pointerType === 'touch') { touches.set(e.pointerId, { x: e.clientX, y: e.clientY }); if (touches.size === 2) { pinch0 = pinchDist(); pinchD = view.dist; drag = null; return; } } drag = { x: e.clientX, y: e.clientY, az: view.az, el: view.el }; cv.setPointerCapture(e.pointerId); });
    const untouch = e => { touches.delete(e.pointerId); if (touches.size < 2) pinch0 = 0; };
    cv.addEventListener('pointercancel', untouch);
    cv.addEventListener('pointermove', e => { if (touches.has(e.pointerId)) { touches.set(e.pointerId, { x: e.clientX, y: e.clientY }); if (touches.size === 2 && pinch0 > 0) { view.dist = Math.max(size * .6, Math.min(size * 8, pinchD * pinch0 / pinchDist())); return; } } if (!drag) return; view.az = drag.az - (e.clientX - drag.x) * .008; view.el = Math.max(-1.45, Math.min(1.45, drag.el + (e.clientY - drag.y) * .008)); });
    cv.addEventListener('pointerup', e => { untouch(e); drag = null; });
    cv.addEventListener('wheel', e => { e.preventDefault(); view.dist = Math.max(size * .6, Math.min(size * 8, view.dist * (1 + Math.sign(e.deltaY) * .1))); }, { passive: false });
    cv.addEventListener('dblclick', () => setView('iso'));
    function setView(name) {
      const V = { iso: [.75, .42], front: [-Math.PI / 2, 0], side: [0, 0], top: [0, 1.45] }[name] || [.75, .42];   // front = looking along the axis from the input side
      view.az = V[0]; view.el = V[1]; view.dist = fitD();
    }
    function setSection(on) { section = !!on; mats.forEach(mm => { mm.clippingPlanes = section ? [clip] : []; mm.needsUpdate = true; }); }
    let raf = 0;
    const frame = () => { raf = requestAnimationFrame(frame); place(); root.updateMatrixWorld(); renderer.render(scene, camera); layoutDims(); };
    frame();
    const ro = new ResizeObserver(() => { const w = host.clientWidth, h = host.clientHeight; if (!w || !h) return; renderer.setSize(w, h); camera.aspect = w / h; camera.updateProjectionMatrix(); });
    ro.observe(host);
    return {
      dispose() { cancelAnimationFrame(raf); ro.disconnect(); scene.traverse(o => { o.geometry?.dispose?.(); }); mats.forEach(mm => mm.dispose()); turned.dispose(); scene.environment?.dispose?.(); renderer.dispose(); renderer.domElement.remove(); ui.remove(); },
      getView: () => ({ az: view.az, el: view.el, dist: view.dist }), setView, setSection,
      setDims(list) { dims = list || []; drawLabels(); }, showDims(on) { showDims = !!on; }
    };
  }

  globalThis.PGD3D = { mount, mountPart, partsList, partSchema, layout, outline, gearGeometry, carrierBody, OPTIONS, optOf, partOn, partLocked, compName };
})();
