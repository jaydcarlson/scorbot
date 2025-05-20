import websocket

from scorbot import JointCmdOpcode, ScorbotCmd, ScorbotStatus


def on_message(ws, message):
    status = ScorbotStatus.unpack(message)
    print(status)
    # exit()

def on_error(ws, error):
    print("Error:", error)

def on_close(ws, close_status_code, close_msg):
    print("Closed:", close_status_code, close_msg)

def on_open(ws):
    print("Connection opened")
    # cmd = ScorbotCmd()
    # cmd.shoulder_lift = (JointCmdOpcode.SET_ANGLE, 0.0)
    # print(cmd)
    # # send ws message as binary
    # msg = cmd.pack()
    # print(msg)
    # ws.send(msg, websocket.ABNF.OPCODE_BINARY)
    # ws.close()  # Optional: close after sending

if __name__ == "__main__":
    websocket.enableTrace(True)  # Optional: print debug trace
    ws = websocket.WebSocketApp(
        "ws://192.168.0.161:8080",  # Change to your server's URL
        on_open=on_open,
        on_message=on_message,
        on_error=on_error,
        on_close=on_close
    )
    ws.run_forever(ping_interval=0)

"""Client using the asyncio API."""

# import asyncio
# import logging
# from websockets.asyncio.client import connect

# from scorbot import JointCmdOpcode, ScorbotCmd, ScorbotStatus

# # Configure logging
# logging.basicConfig(
#     level=logging.DEBUG,
#     format='%(asctime)s - %(name)s - %(levelname)s - %(funcName)s - %(lineno)d - %(message)s'
# )

# async def hello():
#     async with connect("ws://192.168.0.161:8080") as websocket:
#         cmd = ScorbotCmd()
#         cmd.shoulder_pan = (JointCmdOpcode.SET_ANGLE, 45.0)
#         print(cmd)
#         await websocket.send(cmd.pack())
#         message = await websocket.recv()
#         status = ScorbotStatus.unpack(message)
#         print(status)


# if __name__ == "__main__":
#     asyncio.run(hello())