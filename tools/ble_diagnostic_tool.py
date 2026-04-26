import asyncio
import platform
import logging
from bleak import BleakScanner, BleakClient

# 진단 대상 UUID (FFEA 서비스와 그 하위 특성들)
HUD_SERVICE_UUID = "0000ffea-0000-1000-8000-00805f9b34fb"
HUD_CHAR_UUID = "0000fff1-0000-1000-8000-00805f9b34fb"

logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')
logger = logging.getLogger(__name__)

async def run_diagnostic():
    logger.info("=== MOVISION BLE 보안 진단 도구 시작 ===")
    
    # 1. 주변 기기 스캔
    logger.info("주변 MOVISION/Mando 기기를 찾는 중...")
    target_addr = None
    devices = await BleakScanner.discover(timeout=5.0)
    for d in devices:
        name = d.name if d.name else "Unknown"
        if "MOVISION" in name.upper() or "MANDO" in name.upper() or "HUD" in name.upper():
            logger.info(f"발견된 기기: {name} [{d.address}]")
            target_addr = d.address
            break
    
    if not target_addr:
        logger.error("대상 기기를 찾지 못했습니다. 기기가 켜져 있고 페어링 모드인지 확인하세요.")
        return

    # 2. 연결 시도
    logger.info(f"연결 시도 중: {target_addr}...")
    async with BleakClient(target_addr) as client:
        connected = client.is_connected
        logger.info(f"연결 상태: {'성공' if connected else '실패'}")
        
        if not connected:
            return

        # 3. 서비스 및 특성 정보 분석
        logger.info("서비스 및 특성 정보를 분석하는 중...")
        for service in client.services:
            logger.info(f"Service: {service.uuid} ({service.description})")
            for char in service.characteristics:
                props = ", ".join(char.properties)
                logger.info(f"  └─ Characteristic: {char.uuid} [{props}]")
                
                # 보안 설정 경고 (iOS 기준)
                if char.uuid.lower() == HUD_CHAR_UUID.lower():
                    if 'write' in char.properties and 'write-without-response' in char.properties:
                        if not any(p in ['authenticated-signed-writes', 'encrypt-write'] for p in char.properties):
                            logger.warn("⚠️  주의: 이 특성은 암호화(Encryption)를 강제하지 않습니다. 아이폰에서 페어링 팝업이 뜨지 않을 수 있습니다.")

        # 4. 페어링 상태 확인 (OS 수준)
        # Windows/macOS의 경우 OS 제어판에서 페어링을 먼저 해야 할 수도 있습니다.
        try:
            # 특성 읽기를 시도하여 보안 인터럽트가 발생하는지 확인
            logger.info(f"데이터 읽기 테스트 ({HUD_CHAR_UUID})...")
            val = await client.read_gatt_char(HUD_CHAR_UUID)
            logger.info(f"읽기 결과: {val.hex().upper() if val else 'Empty'}")
            logger.info("✅ 페어링 없이 데이터 읽기 성공 (현재 기기는 보안이 필요 없는 상태입니다.)")
        except Exception as e:
            logger.info(f"상태 메시지: 보안 이슈가 감지되었을 수 있습니다: {e}")

    logger.info("=== 진단 완료 ===")

if __name__ == "__main__":
    if platform.system() == "Windows" or platform.system() == "Darwin":
        asyncio.run(run_diagnostic())
    else:
        logger.error("이 도구는 현재 Windows 및 macOS 환경을 지원합니다.")
