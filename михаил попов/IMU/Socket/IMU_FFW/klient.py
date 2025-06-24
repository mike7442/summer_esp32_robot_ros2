import socket
import struct

def unpack6BytesToFloats(bytes_data):
    floats = []
    for i in range(6):
        chunk = bytes_data[i*4:(i+1)*4]
        value = struct.unpack('<f', chunk)[0]
        floats.append(value)
    return floats

def main():
    last_digit = 100  # меняйте только это число
    host = f"192.168.0.{last_digit}"
    port = 12345           # Порт сервера (пример)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, port))
    print("Подключено к серверу")

 ##   count = 0
    try:
        while True:
            data = sock.recv(24)  # ожидаем 6 float (6*4=24 байта)
            if len(data) == 24:
                result = unpack6BytesToFloats(data)
                print(result)
            elif len(data) < 24:
                print("Получено недостаточно данных:", len(data))
                if not data:
                    print("Соединение закрыто сервером")
                    break
                continue
            result = unpack6BytesToFloats(data)
##            count += 1
##            print(f"№{count}: {result}")
    except Exception as e:
        print("Ошибка:", e)
    finally:
        sock.close()
        print("Соединение закрыто")

if __name__ == "__main__":
    main()
