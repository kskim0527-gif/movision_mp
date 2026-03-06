# 정상 연결 로그 분석 및 비교

## 파일 구성
- **ESP32 로그 (1-385줄)**: 등록 지연 문제가 있었던 버전
- **정상 동작 기기 (386-737줄)**: 정상 작동하는 기기

## 핵심 발견 사항

### 1. Service Discovery 순서

#### ESP32 (문제 버전)
```
0x0038..0xFFFF 0xFFEA  (HUD service)
0x0028..0x0037 0x1812  (HID service)
```
- **HUD 서비스가 먼저 발견됨** (handle 0x0038부터)

#### 정상 동작 기기
```
0x0028..0x0037 0x1812  (HID service)
0x0038..0xFFFF 0xFFEA  (HUD service)
```
- **HID 서비스가 먼저 발견됨** (handle 0x0028부터)
- **HUD 서비스가 나중에 발견됨** (handle 0x0038부터)

### 2. HUD Characteristic Handles

#### ESP32 (문제 버전)
```
decl=0x0039 props=0x12 value=0x003A uuid=0xFFF1  (HUD READ)
decl=0x003B props=0x0C value=0x003C uuid=0xFFF2  (HUD WRITE)
```

#### 정상 동작 기기
```
decl=0x0039 props=0x12 value=0x003A uuid=0xFFF1  (HUD READ)
decl=0x003B props=0x0C value=0x003C uuid=0xFFF2  (HUD WRITE)

또는

decl=0x0018 props=0x12 value=0x0019 uuid=0xFFF1  (HUD READ - 다른 연결?)
decl=0x001B props=0x0C value=0x001C uuid=0xFFF2  (HUD WRITE - 다른 연결?)
```

### 3. HUD CCCD 활성화

#### ESP32 (문제 버전)
- handle 0x003B val=0x0001: 5회 (WRITE characteristic의 CCCD?)
- handle 0x003E val=0x0001: 5회
- **handle 0x001A가 보이지 않음**

#### 정상 동작 기기
- **handle 0x001A val=0x0001: 1회** (ev=01784) ← **이것이 HUD CCCD!**
- 이후 handle 0x001C (FFF2)에 명령들이 전송됨 (ev=01791~)

### 4. HUD FFF2 Write 패턴

#### ESP32 (문제 버전)
- handle 0x001C (28): 23회 WRITE
- 많은 명령들이 전송되고 있음

#### 정상 동작 기기
- handle 0x001C (28): 17회 WRITE
- ev=01791부터 명령 전송 시작 (HUD CCCD 활성화 직후)

### 5. Discovery 시퀀스

#### ESP32 (문제 버전)
```
00001: ReadByTypeReq start=0x0038 end=0xFFFF type=0x2802 (Service Changed)
00003: ReadByTypeReq start=0x0038 end=0xFFFF type=0x2803 (Characteristic)
00004: ReadByTypeRsp (2 entries found)
```
- **HUD 서비스(0x0038)부터 discovery 시작**

#### 정상 동작 기기
```
00001: ReadByTypeReq start=0x0038 end=0xFFFF type=0x2802 (Service Changed)
00003: ReadByTypeReq start=0x0038 end=0xFFFF type=0x2803 (Characteristic)
00004: ReadByTypeRsp (2 entries found)
```
- 동일한 패턴이지만, **정상 기기는 HID를 먼저 discovery**

### 6. 핵심 차이점 요약

| 항목 | ESP32 (문제) | 정상 기기 |
|------|-------------|-----------|
| Service 순서 | HUD 먼저, HID 나중 | **HID 먼저, HUD 나중** |
| HUD CCCD Handle | 불명확 (0x003B?) | **0x001A (명확)** |
| HUD FFF2 Handle | 0x001C (28) | **0x001C (28)** - 동일 |
| HUD FFF1 Handle | 0x003A (58) | **0x0019 (25)** 또는 0x003A |
| Discovery 시작 | 0x0038 (HUD) | **0x0028 (HID)** |

## 결론

1. **서비스 생성 순서가 중요**: 정상 기기는 **HID 서비스가 먼저 생성**되고, 그 다음 HUD 서비스가 생성됨
2. **Handle 번호 차이**: ESP32와 정상 기기의 handle 번호가 다르지만, 이는 서비스 생성 순서에 따라 결정됨
3. **HUD CCCD 위치**: 정상 기기에서 HUD CCCD는 **handle 0x001A**에 위치 (ev=01784에서 활성화)
4. **Discovery 패턴**: 정상 기기는 HID 서비스를 먼저 discovery하고, 나중에 HUD 서비스를 discovery함

## 권장 사항

ESP32 코드에서 **서비스 생성 순서를 HID 먼저, HUD 나중으로 변경**해야 할 가능성이 높음.

