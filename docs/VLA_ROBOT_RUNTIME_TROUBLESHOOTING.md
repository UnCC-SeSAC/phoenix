# VLA Robot Runtime Troubleshooting

HW validation은 다음 순서로 진행한다.

```text
Happy-path 실행
→ 성공하면 종료
→ 실패하면 이 문서 검색
→ 같은 증상의 기존 해결법 적용
→ 계속 실패할 때만 추가 진단
→ 해결 후 새 정보만 업데이트
```

이미 PASS한 Camera, HEF, Depth, TF 단계를 매번 장시간 재검증하지 않는다.
일시적 복구와 root cause 해결을 구분하며, 원인이 확정되지 않았으면 `미확정`으로
기록한다.

## HEF는 실행되지만 `detections=[]`

### 증상

HEF inference는 실행되지만 person/fire detection이 0건이다.

### 기존 해결법

- `/home/lemma/Hailo/yolo26_split_test.py` 기준 output/postprocess를 사용한다.
- 최신 ROS overlay가 수정 source의 backend를 실제로 로드하는지 확인한다.

### 원인

실제 HEF는 단일 NMS output이 아니라 6개 raw neural head를 출력하므로
`best_sim_postprocess.onnx` 후처리가 필요하다. stale ROS installed backend가 수정
source 대신 로드된 경우도 있었다.

### 정상 기준

live person bbox/confidence 출력.

## LD19 `/scan_raw` 0 Hz와 TF timeout

### 증상

```text
/scan_raw 0 Hz
→ SLAM 갱신 중단
→ map→odom TF 중단
→ TF timeout/extrapolation
```

환경은 LD19, `/dev/ldlidar → /dev/ttyUSB0`, baudrate `230400`이다.

### 기존 해결법

serial 중복 점유 확인 후 LD19 driver를 완전히 종료하고 authoritative launch를 clean
restart한다. 이후 `/scan_raw`와 `map→odom` continuity를 확인한다.

### 원인

intermittent parser/scan-assembly stall로 확인했으나 세부 trigger는 미확정이다.

### 정상 기준

`/scan_raw` 약 9.91 Hz, `map→odom` continuity PASS.

## Pi Docker process/exec resource exhaustion

### 증상

새 process/exec 실행이 불안정하거나 실패해 HW downstream test를 시작하지 못한다.

### 기존 해결법

1. 다른 팀원의 Pi/Docker 사용 여부를 확인한다.
2. 전체 runtime 종료 허가를 확인한다.
3. 실행 중 ROS2/test process를 정리한다.
4. 필요할 때 container만 clean restart한다.
5. 최소 stack으로 happy-path를 다시 실행한다.

container, image, volume, Hailo 환경을 삭제하지 않는다. clean restart 후에도 재발할
때만 resource 원인을 상세 진단한다.

### 원인

미확정.

### 정상 기준

새 process/exec가 안정적으로 실행되고 최소 HW stack이 정상 기동한다.

## 새 항목 작성 형식

새 HW 오류가 실제로 발생하고 해결됐을 때만 아래 형식으로 추가한다.

```text
### 증상
무엇이 안 되는지

### 기존 해결법
가장 빠른 복구 방법

### 원인
확정된 원인 또는 미확정

### 정상 기준
재사용 가능한 최소 정상 지표
```

긴 실행 로그, 디버깅 일지, 실패한 명령어 목록은 남기지 않는다.
