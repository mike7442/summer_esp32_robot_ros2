import asyncio
import websockets
print("скрипт запущен")

last_digit = 4
uri = f"ws://192.168.0.10{last_digit}/ws"

import struct

def unpack6BytesToFloats(bytes_data):
    floats = []
    for i in range(6):
        chunk = bytes_data[i*4:(i+1)*4]
        value = struct.unpack('<f', chunk)[0]
        floats.append(value)
    return floats

async def listen():
    async with websockets.connect(uri) as websocket:
        print("Подключено к серверу")
        try:
            while True:
                message = await websocket.recv()
                result = unpack6BytesToFloats(message)
                print("Получено сообщение:", result)
        except websockets.exceptions.ConnectionClosed:
            print("Соединение закрыто")

if __name__ == "__main__":
    asyncio.run(listen())
