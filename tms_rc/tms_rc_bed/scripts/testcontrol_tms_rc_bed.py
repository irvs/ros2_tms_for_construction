import asyncio
import websockets

async def control():
    while True:
        async with websockets.connect('ws://localhost:9989') as websocket:
            num = input("[tms_rc_bed control number] >")
            await websocket.send(num)
            print(f"> send {num}")

asyncio.get_event_loop().run_until_complete(control())