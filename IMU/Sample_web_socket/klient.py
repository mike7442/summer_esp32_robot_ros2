import asyncio
import websockets
print("скрипт запущен")

last_digit = 9
uri = f"ws://192.168.0.10{last_digit}/ws"
print(uri)

async def listen():
    async with websockets.connect(uri) as websocket:
        print("Подключено к серверу")
        try:
            while True:
                message = await websocket.recv()
                print("Получено сообщение:", message)
        except websockets.exceptions.ConnectionClosed:
            print("Соединение закрыто")

if __name__ == "__main__":
    asyncio.run(listen())
