import serial
import socket
import select
import time
import pygame
import thread
import os.path
import os

override_active = True

# Connect to controller
print('connecting to manual override...')

pygame.init()

try:
    js = pygame.joystick.Joystick(0)
    js.init()
except pygame.error:
    override_active = False

# Connect to Arduino
print('connecting to IO controller...')

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(1)

print('ready.')

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(('0.0.0.0', 20000))
server.listen(1)

def manual_override():
    import os
    import os.path

    try:
        while True:
            events = pygame.event.get()

            for event in events:
                if event.type == pygame.JOYBUTTONUP and event.button == 12: # Triangle
                    print('manual override invoked!')
                    fail()

            if not os.path.exists('/dev/input/js0'):
                print('lost connection to manual override!')
                fail()

            time.sleep(0.1)
    except:
        print('sending shutdown command...')

        # Sent twice to deal with any previous messages
        ser.write('shutdown\nshutdown\n')

        os._exit(1)

if override_active:
    thread.start_new_thread(manual_override, ())
else:
    print('\033[91m\033[1mWARNING:\033[0m \033[91mNO MANUAL OVERRIDE AVAILABLE\033[0m')

try:
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
except KeyboardInterrupt:
    print('shutting down...')

    js.quit()
    server.close()