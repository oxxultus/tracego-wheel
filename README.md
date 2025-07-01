# TraceGo

TraceGo 시스템의 바퀴 제어 모듈입니다. PID 기반 속도 및 방향 제어 등 하드웨어 바퀴 구동 로직을 담당합니다.

- 주요 기능
    - 적외선 센서 인식
    - PID 연산
    - 라인트레이서
    - 보드간 통신

## 하드웨어
- 보드: Arduino Uno (R3)
    - 모듈: AFMotor + moter driver
    - 모듈: TCRT5000 배열 5개


## 설치

해당 리포지토리를 사용하려면 PlatformIO를 사용해야 합니다.

```
git clone https://github.com/oxxultus/tracego-wheel.git
```

- 빌드를 진행한 뒤 업로드를 진행하면됩니다.

### 종속성
- [tracego-wheel](https://github.com/oxxultus/tracego-core.git): `운반 중앙 제어`
