import asyncio
import aiohttp
import json
from LoadScale.scale import LoadScale
from HorizonBalance.horizebalancer import HorizonBalancer
from typing import Optional

class LoadHorize:
    def __init__(self):
        self.scale = LoadScale(dt_pin=5, sck_pin=6, reference_unit=8600)
        self.balancer = HorizonBalancer()
        self.pkilogram = -1
        self.server_url = 'http://c103.duckdns.org:8502/api/weight/'
        self.logger_prefix = "[LoadHorize]"

    async def read_scale(self, err_cnt: int = 0) -> Optional[float]:
        if err_cnt > 10:
            print(f"{self.logger_prefix} Too many errors in scale reading. Stopping...")
            return None
        
        try:
            value = self.scale.scale_read()
            if value is None:
                print(f"{self.logger_prefix} Scale returned None ({err_cnt + 1}/11), retrying...")
                await asyncio.sleep(0.1)
                return await self.read_scale(err_cnt + 1)
            return value
        except Exception as e:
            print(f"{self.logger_prefix} Scale reading error ({err_cnt + 1}/11): {e}")
            await asyncio.sleep(0.1)
            return await self.read_scale(err_cnt + 1)

    async def send_kilogram(self) -> None:
        async with aiohttp.ClientSession() as session:
            while True:
                try:
                    kilogram = await self.read_scale()
                    print(f"Kilogram: {kilogram}")
                    if kilogram is None:
                        print("Failed to read scale after multiple attempts")
                        await asyncio.sleep(1)
                        continue
                        
                    if abs(kilogram - self.pkilogram) >= 1:
                        self.pkilogram = kilogram
                        print("send kilogram")

                        payload = {
                            "weight": kilogram,
                            "robot_id": "user"
                        }
                        async with session.post(url=self.server_url, data=json.dumps(payload)) as response:
                            if response.status == 200:
                                print(f"Kilogram sent: {kilogram}")
                            else:
                                print(f"Failed to send data: {response.status}")
                    await asyncio.sleep(0.7)
                except Exception as e:
                    print(f"Error in send_kilogram: {e}")
                    await asyncio.sleep(1)

    async def balance_control(self, err_cnt: int = 0) -> None:
        if err_cnt > 10:
            print("Too many errors in balance control. Stopping...")
            return
        
        try:
            while True:
                self.balancer.balance_step()
                await asyncio.sleep(0.18) # 무부하 상태 0.08 권장, 부하 상태 0.2 이상 권장
        except OSError as e:
            print(f"Balance control error ({err_cnt + 1}/11): {e}")
            await self.balance_control(err_cnt + 1)
        except Exception as e:
            print(f"Unexpected error in balance control: {e}")
            self.balancer.cleanup()  # 예상치 못한 에러 발생 시 정리

    async def main(self) -> None:
        try:
            await asyncio.gather(
                self.send_kilogram(),
                self.balance_control()
            )
        except KeyboardInterrupt:
            print("\nShutting down...")
        except Exception as e:
            print(f"Error in main: {e}")
        finally:
            self.cleanup()

    def cleanup(self) -> None:
        self.scale.cleanAndExit()
        self.balancer.cleanup()

class LoadHorizeTest:
    def __init__(self):
        self.server_url = 'http://c103.duckdns.org:8502/api/weight/'
        self.send_kilogram_err_cnt = 0

    async def send_kilogram(self) -> None:
        while True:
            try:
                async with aiohttp.ClientSession() as session:
                    async with session.post(url=self.server_url, data=json.dumps({'weight': 65, 'robot_id': 'user'})) as response:
                        if response.status == 200:
                            print(f"Kilogram sent: 65")
                        else:
                            print(f"Failed to send data: {response.status}")
                await asyncio.sleep(5)
            except Exception as e:
                if self.send_kilogram_err_cnt > 10:
                    print("Too many errors in send_kilogram. Stopping...")
                    return
                print(f"Error in send_kilogram: {e}")
                self.send_kilogram_err_cnt += 1
                await asyncio.sleep(1)
    
    async def balance_control(self) -> None:
        while True:
            print("balance control: Motor 90 DEG / Sensor 94 DEG")
            await asyncio.sleep(0.18)

    async def main(self) -> None:
        try:
            await asyncio.gather(
                self.send_kilogram(),
                self.balance_control()
            )
        except KeyboardInterrupt:
            print("\nShutting down...")
        except Exception as e:
            print(f"Error in main: {e}")
    
    def cleanup(self) -> None:
        print("cleanup")
        return

if __name__ == "__main__":
    asyncio.run(LoadHorize().main())