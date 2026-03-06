# ESP32 등록 지연 vs 정상 동작 기기 로그 비교 분석

## 1. 타이밍 비교

### ESP32 (등록 지연)
```
17704ms: connected
20649ms: AUTH_CMPL success=1
21977ms: client_started: first READ (handle=42) - HID Info
22068ms: MTU=247
22158ms: READ_EVT HID ReportMap handle=44
22247ms: WRITE_EVT handle=49 (HID_CCCD=0x0001)
22337ms: WRITE_EVT handle=52 (HID BootKB CCCD=0x0001)
22345ms: service_change_indication (after MTU+200ms): ESP_OK

[Service Changed 후 재발견]
23080ms: READ_EVT HID Info handle=42 (재발견)
23095ms: READ_EVT HID ReportMap handle=44 (재발견)
23110ms: WRITE_EVT handle=49 (HID_CCCD=0x0001) (재활성화)
23125ms: WRITE_EVT handle=52 (HID BootKB CCCD=0x0001) (재활성화)

❌ HUD CCCD (handle=61) 활성화 없음
```

### 정상 동작 기기 (btsnoop 로그)
```
ev=01784: WRITE handle=0x001A val=01:00  (HUD CCCD 활성화)
ev=01791: WRITE handle=0x001C val=19:4d:0b:01:00:2f  (FFF2 명령 전송 시작)
```

## 2. 핵심 차이점

### 2.1 HUD CCCD 활성화 여부

**ESP32 (현재):**
- ✅ HID CCCD 활성화 (handle=49, 52)
- ❌ **HUD CCCD (handle=61) 활성화 없음**
- ❌ TIME_REQ 시작 안됨
- ❌ 시간 명령 수신 없음

**정상 동작 기기:**
- ✅ HID CCCD 활성화
- ✅ **HUD CCCD (handle=0x001A=26) 활성화 있음**
- ✅ FFF2 (handle=0x001C=28) 명령 전송 시작

### 2.2 Service Changed Indication 후 재발견

**ESP32:**
- Service Changed Indication 전송 후 (22345ms)
- 735ms 후 재발견 시도 (23080ms)
- 하지만 **HID 서비스만 재발견**
- **HUD 서비스 재발견 없음**

**정상 동작 기기:**
- Service Changed Indication이 있었는지 불명확
- 하지만 처음부터 HUD CCCD 활성화

### 2.3 Handle 번호 차이

**ESP32:**
- HUD READ (FFF1): handle=58
- HUD WRITE (FFF2): handle=60
- HUD CCCD: handle=61

**정상 동작 기기:**
- HUD READ (FFF1): handle=0x0019 (25)
- HUD WRITE (FFF2): handle=0x001C (28)
- HUD CCCD: handle=0x001A (26)

> Handle 번호는 서비스 등록 순서에 따라 달라질 수 있으므로, UUID로 비교해야 함.

## 3. 문제 분석

### 3.1 왜 HUD CCCD가 활성화되지 않는가?

1. **Service Changed Indication 타이밍 문제:**
   - MTU 협상 직후 200ms 딜레이로 전송
   - 앱이 HID 서비스 discovery를 완료하기 전에 Service Changed를 받았을 가능성
   - 앱이 Service Changed를 받고 HID만 재발견하고 HUD는 재발견하지 않음

2. **앱의 서비스 Discovery 순서:**
   - 정상 기기: HID → HUD 순서로 discovery
   - ESP32: HID만 discovery, HUD는 discovery 안함

3. **Service Changed Indication의 범위:**
   - Service Changed는 전체 서비스 데이터베이스 변경을 의미
   - 하지만 앱이 HID 서비스만 재발견하는 것은 비정상

### 3.2 정상 기기의 동작 패턴

```
1. HID 서비스 discovery
2. HID CCCD 활성화
3. HUD 서비스 discovery
4. HUD CCCD 활성화 (handle=0x001A)
5. FFF2 (handle=0x001C) 명령 전송 시작
```

## 4. 해결 방안

### 방안 1: Service Changed Indication 제거 또는 지연
- 현재 200ms → 500ms로 변경함
- 또는 Service Changed Indication을 AUTH_CMPL 후로 이동

### 방안 2: Service Changed Indication 제거
- Service Changed Indication을 전송하지 않음
- 앱이 초기 discovery에서 HUD 서비스를 찾도록 함

### 방안 3: HUD 서비스를 HID보다 먼저 등록
- GATT 서비스 등록 순서를 HUD → HID로 변경
- 앱이 HUD를 먼저 discovery하도록 유도

### 방안 4: Service Changed Indication을 더 늦게 전송
- HID 서비스 discovery 완료 후 전송
- 또는 HID CCCD 활성화 후 전송

## 5. 권장 사항

1. **우선 확인:**
   - 현재 500ms 딜레이로 변경했으므로, 테스트 후 HUD CCCD 활성화 여부 확인

2. **만약 500ms로도 안되면:**
   - Service Changed Indication을 AUTH_CMPL 후로 이동
   - 또는 Service Changed Indication 제거

3. **최후의 수단:**
   - HUD 서비스를 HID보다 먼저 등록

## 6. 핵심 문제 요약

**ESP32의 문제:**
- Service Changed Indication 전송 후 앱이 HUD 서비스를 재발견하지 않음
- HUD CCCD가 활성화되지 않아 TIME_REQ가 시작되지 않음
- 결과적으로 시간 동기화가 이루어지지 않음

**정상 기기의 동작:**
- HUD CCCD가 정상적으로 활성화됨
- FFF2 명령 전송이 시작됨

