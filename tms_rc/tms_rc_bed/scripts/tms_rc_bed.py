import wiringpi as pi
from websocket_server import WebsocketServer
from enum import IntEnum
import time
import threading

TIME = 20

class Pin(IntEnum):
    LINK_UP = 6
    LINK_DOWN = 13
    HEAD_UP = 19
    HEAD_DOWN = 26
    FOOT_UP = 12
    FOOT_DOWN = 16
    HEIGHT_UP = 20
    HEIGHT_DOWN = 21


command_to_pin = {
    1 : Pin.LINK_UP,
    2 : Pin.LINK_DOWN,
    3 : Pin.HEAD_UP,
    4 : Pin.HEAD_DOWN,
    5 : Pin.FOOT_UP,
    6 : Pin.FOOT_DOWN,
    7 : Pin.HEIGHT_UP,
    8 : Pin.HEIGHT_DOWN,
}

def pinMode(pin, is_output):
    pi.pinMode(int(pin), is_output)

def digitalWrite(pin, is_high):
    pi.digitalWrite(int(pin), is_high)

def stop_all():
    digitalWrite(Pin.LINK_UP,1)
    digitalWrite(Pin.LINK_DOWN,1)
    digitalWrite(Pin.HEAD_UP,1)
    digitalWrite(Pin.HEAD_DOWN,1)
    digitalWrite(Pin.FOOT_UP,1)
    digitalWrite(Pin.FOOT_DOWN,1)
    digitalWrite(Pin.HEIGHT_UP,1)
    digitalWrite(Pin.HEIGHT_DOWN,1)
    pinMode(Pin.LINK_UP,0)
    pinMode(Pin.LINK_DOWN,0)
    pinMode(Pin.HEAD_UP,0)
    pinMode(Pin.HEAD_DOWN,0)
    pinMode(Pin.FOOT_UP,0)
    pinMode(Pin.FOOT_DOWN,0)
    pinMode(Pin.HEIGHT_UP,0)
pinMode(Pin.HEIGHT_DOWN,0)

def move(client, server, message):
    thread = threading.Thread(target=_move, args=([client, server, message]))
    thread.start()

return_msg = "Success"
event = threading.Event()
def _move(client, server, message):
    global callback_count, stop_flag, event, return_msg
    with lock:
        callback_count += 1
    msg = message.split(',')
    command = msg[0]
    try:
        pin = command_to_pin[int(command)]
    except KeyError as instance:
        with lock:
            print(f"{command} is not included commands\n[invoke] stop_all")
            stop_all()
            return_msg = "Canceled"
            callback_count -= 1
            if callback_count >= 1:
                event.set()
            else:
                event.clear()
                server.send_message(client, "Canceled")
        return
    
    if callback_count >= 2:
        with lock:
            print("[Request Abort] Because another thread works!")
            server.send_message(client, "Abort")
            callback_count -= 1
        return
    tim = TIME
    try:
        tim = float(msg[1])
    except IndexError:
        tim = TIME
    with action_lock:
        print(f"[start]  {command} : {pin.name} for {tim} seconds")
        # ボタンを一度押せばリモコンが起動する
        pinMode(pin, 1)
        digitalWrite(pin, 0)
        time.sleep(0.1)
        digitalWrite(pin, 1)
        pinMode(pin, 0)
        time.sleep(0.1)
        # ２回目にボタンを押せばそのボタンの動作をする
        pinMode(pin, 1)
        digitalWrite(pin, 0)
        #time.sleep(tim)
        event.wait(timeout=tim)
        digitalWrite(pin, 1)
        pinMode(pin, 0)
        print(f"[end] {command} : {pin.name}")
    with lock:
        callback_count -= 1
        if callback_count <= 0:
            event.clear()
            server.send_message(client, return_msg)
            return_msg = "Success"

callback_count = 0
stop_flag = False

lock = threading.Lock()
action_lock = threading.Lock()
server = WebsocketServer(9989, host='192.168.4.131')
if __name__ == '__main__':
    # global server
    pi.wiringPiSetupGpio()
    stop_all()

    #server = WebsocketServer(9989, host='192.168.4.131')
    print("tms_rc_bed ready...")

    server.set_fn_message_received(move)
    server.run_forever()
