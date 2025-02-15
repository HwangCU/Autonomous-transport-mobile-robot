import asyncio
import aiohttp
from LoadScale.scale import LoadScale
from HorizonBalance.horizebalancer import HorizonBalancer

scale = LoadScale(dt_pin=5, sck_pin=6, reference_unit=8600)
balancer = HorizonBalancer()

pkilogram = -1

URL = 'http://c103.duckdns.org:8502/api/weight/'

async def read_scale(err_cnt=0):
    if err_cnt > 10:
        print("Too many errors in scale reading. Stopping...")
        return None
    
    try:
        value = scale.scale_read()
        if value is None:
            print(f"Scale returned None ({err_cnt + 1}/11), retrying...")
            await asyncio.sleep(0.1)  # 재시도 전 잠시 대기 추가
            return await read_scale(err_cnt + 1)
        return value
    except Exception as e:
        print(f"Scale reading error ({err_cnt + 1}/11): {e}")
        await asyncio.sleep(0.1)  # 재시도 전 잠시 대기 추가
        return await read_scale(err_cnt + 1)

async def send_kilogram():
    global pkilogram
    
    async with aiohttp.ClientSession() as session:
        while True:
            try:
                kilogram = await read_scale()
                print(f"Kilogram: {kilogram}")
                if kilogram is None:
                    print("Failed to read scale after multiple attempts")
                    await asyncio.sleep(1)
                    continue
                    
                if abs(kilogram - pkilogram) >= 1:
                    pkilogram = kilogram
                    print("send kilogram")
                    # HTTP POST 요청으로 데이터 전송
                    async with session.post(url=URL, json={'weight': kilogram}) as response:
                        if response.status == 200:
                            print(f"Kilogram sent: {kilogram}")
                        else:
                            print(f"Failed to send data: {response.status}")
                await asyncio.sleep(0.1)
            except Exception as e:
                print(f"Error in send_kilogram: {e}")
                await asyncio.sleep(1)

async def balance_control(err_cnt=0):
    if err_cnt > 10:
        print("Too many errors in balance control. Stopping...")
        return
    
    try:
        while True:
            balancer.balance_step()
            await asyncio.sleep(0.18) # 무부하 상태 0.08 권장, 부하 상태 0.2 이상 권장
    except OSError as e:
        print(f"Balance control error ({err_cnt + 1}/11): {e}")
        await balance_control(err_cnt + 1)
    except Exception as e:
        print(f"Unexpected error in balance control: {e}")
        balancer.cleanup()  # 예상치 못한 에러 발생 시 정리

async def main():
    try:
        # 두 작업을 동시에 실행
        await asyncio.gather(
            send_kilogram(),
            balance_control()
        )
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        scale.cleanAndExit()
        balancer.cleanup()

if __name__ == "__main__":
    asyncio.run(main())