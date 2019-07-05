# import wiringpi2 as pi
import asyncio
import websockets
from enum import IntEnum

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

# def pinMode(pin, is_output):
#     pi.pinMode(int(pin), is_output)

# def digitalWrite(pin, is_high):
#     pi.digitalWrite(int(pin), is_high)

# def stop_all():
#     digitalWrite(Pin.LINK_UP,1)
#     digitalWrite(Pin.LINK_DOWN,1)
#     digitalWrite(Pin.HEAD_UP,1)
#     digitalWrite(Pin.HEAD_DOWN,1)
#     digitalWrite(Pin.FOOT_UP,1)
#     digitalWrite(Pin.FOOT_DOWN,1)
#     digitalWrite(Pin.HEIGHT_UP,1)
#     digitalWrite(Pin.HEIGHT_DOWN,1)
#     pinMode(Pin.LINK_UP,0)
#     pinMode(Pin.LINK_DOWN,0)
#     pinMode(Pin.HEAD_UP,0)
#     pinMode(Pin.HEAD_DOWN,0)
#     pinMode(Pin.FOOT_UP,0)
#     pinMode(Pin.FOOT_DOWN,0)
#     pinMode(Pin.HEIGHT_UP,0)
#     pinMode(Pin.HEIGHT_DOWN,0)


async def move(websocket, path):
    command = await websocket.recv()
    try:
        pin = command_to_pin[int(command)]
    except KeyError as instance:
        print(f"{command} is not included commands\n[invoke] stop_all")
        #stop_all()
        return

        

    print(f"[start]  {command} : {pin.name}")
    # pinMode(pin, 1)
    # digitalWrite(pin, 0)
    await asyncio.sleep(TIME)
    # digitalWrite(pin, 1)
    # pinMode(pin, 0)
    print(f"[end] {command} : {pin.name}")
    


if __name__ == '__main__':
    # pi.wiringPiSetupGpio()
    # stop_all()
    start_server = websockets.serve(move, 'localhost', 9989)
    print("tms_rc_bed ready...")

    loop = asyncio.get_event_loop()
    loop.run_until_complete(start_server)
    loop.run_forever()