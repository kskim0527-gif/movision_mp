import asyncio
from bleak import BleakScanner, BleakClient

async def run():
    print("스캔 중...")
    devices = await BleakScanner.discover(timeout=5.0)
    target = None
    for d in devices:
        if d.name and "Mando HUD T" in d.name:
            target = d
            break
            
    if not target:
        print("기기를 찾을 수 없습니다.")
        return
        
    print(f"기기 발견: {target.name} [{target.address}]")
    print("연결하여 제공 중인 서비스 목록을 확인합니다...")
    
    # 일부러 장시간 접속하여 GATT를 확인
    async with BleakClient(target.address, timeout=10.0) as client:
        if not client.is_connected:
            print("연결 실패!")
            return
            
        print("연결 성공! 서비스 목록:")
        for service in client.services:
            print(f"- 서비스: {service.uuid} (설명: {service.description})")
            for char in service.characteristics:
                print(f"   => 통신채널(Char): {char.uuid} (속성: {','.join(char.properties)})")

if __name__ == "__main__":
    asyncio.run(run())
