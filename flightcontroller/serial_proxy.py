import serial
import socket
import select
import time

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

print('connecting...')

time.sleep(1)

print('ready.')

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(('0.0.0.0', 20000))
server.listen(1)

while True:
    (client, address) = server.accept()
    f = client.makefile()

    print('new client')

    while True:
        (readable, _, __) = select.select([ser, f], [], [])

        if readable[0] == f:
            msg = f.readline()
            if msg == '' or ord(msg[0]) == 0: break
            ser.write(msg)
            print('< ' + msg[:-1])
        else:
            msg = ser.readline()
            client.send(msg)
            print('> ' + msg[:-1])

    print('client disconnected')

    client.close()