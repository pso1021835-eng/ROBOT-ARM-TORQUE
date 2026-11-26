# ROBOT-ARM-TORQUE — H2-X 오른팔 토크–각변위 (웹)

Unitree H2-X 전신이 서 있는 상태(골반 고정)에서 오른팔 7축을 슬라이더로 움직이고,
팔 자체 질량·손 하중(무게중심 오프셋 포함)을 바꾸면서 관절별 **감속기 출력축 토크–각변위**를 실시간으로 본다.
index.html을 연 화면이 곧 시뮬레이터 (GitHub Pages: main / root).

- 주소: `https://pso1021835-eng.github.io/ROBOT-ARM-TORQUE/` 
- 모델: [unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros) @ `157ff950351e5f535e03c5d85a16b7d594b3dbcc`,
  `robots/h2x_description/h2x.urdf` (SHA-256 `1d6ae92a…`), 메시 36개. 라이선스 BSD-3-Clause → `LICENSE-unitree_ros.txt`

## 계산
- `dynamics.js` (외부 라이브러리 없음, 브라우저·Node 공용)
  - 링크 질량·무게중심·관성텐서 = URDF 원본. 오른팔 외 관절은 0 rad 고정, 골반 높이 1.0237 m(발바닥 z=0)
  - 역동역학 RNEA, 질량행렬 = 단위가속도 RNEA, 적분 1 ms 반암시적 오일러
  - 구동: 속도·가속도 제한 궤적 → 피드포워드(실제 자세 · 기준 궤적 속도/가속도의 역동역학) + 관절별 PD
    (관성 스케일 D는 폐루프 고유값 λ = eig(D½M⁻¹D½) ≤ 3이 되게 축소 → 한 관절이 포화돼도 다른 관절이 떨지 않음)
  - 토크 한계 = URDF `effort` (명령 클립), 가동범위 = 기계식 스토퍼(충격량)
  - 가정값(URDF에 없음): 로터 반사관성 0.03/0.025/0.025/0.025/0.01/0.01/0.01 kg·m², 관절 점성 마찰 1.0 N·m·s/rad — 화면에서 변경
  - 표시 토크 = 감속기 출력축 토크 = 구동토크 − 로터 반사관성×각가속도 (정지 시 구동토크와 같음)
- 팔 질량 변경: 7링크 질량·관성 ×k, 무게중심 유지. 손 하중: 손 링크(`right_wrist_yaw_link`)에 균질구(r 3 cm)로 강체 결합, 1초 램프

## 검증
`node dynamics_test.js` — URDF로 Python(numpy)에서 독립 계산한 정역학 토크 18건과 비교(오차 ~1e-9 N·m),
질량행렬 대칭·RNEA 선형성, 자유진동 에너지 보존, 궤적 제한, 팔 뻗은 자세 0/1 kg Δτ = 이론(하중 중력모멘트의 관절축 투영),
과부하(20 kg @ 0.2 m)에서 발산·떨림 없음, 팔 질량 ×1.5 → 토크 ×1.5.

## 모델 재생성
`python tools/make_model.py <unitree_ros>/robots/h2x_description/h2x.urdf`
(로컬 MuJoCo 버전 `h2x_arm_sim`의 `urdf_model.py` 필요: 환경변수 `H2X_SIM_DIR`)
