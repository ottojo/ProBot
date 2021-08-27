import asyncio

import matplotlib.colors
import matplotlib.pyplot as plt
import numpy as np
import websockets as websockets


class Robot:

    @classmethod
    async def create(cls, address):
        self = Robot()
        self.controlSocket = await websockets.connect("ws://" + address + "/control")
        self.cameraSocket = await websockets.connect("ws://" + address + "/camera")
        self.cameraTask = asyncio.create_task(self.listen_for_message())
        return self

    async def listen_for_message(self):
        fg = plt.figure()
        ax = fg.gca()
        self.imageDisplay = ax.imshow(np.zeros((8, 8), dtype=np.float32), norm=matplotlib.colors.Normalize(10, 40))
        # plt.show()
        while True:
            try:
                print('Listening for a message...')
                message = await self.cameraSocket.recv()
                print("< {}".format(message))
                image = np.frombuffer(message, np.float32)
                image = image.reshape((8, 8))
                self.imageDisplay.set_data(np.rot90(image, 2))
                plt.draw()
                plt.pause(1e-3)
            except websockets.ConnectionClosed as cc:
                print('Connection closed')
            except Exception as e:
                print('Something happened')

    def getCameraImage(self):
        pass

    async def motorOn(self):
        await self.controlSocket.send("start")

    async def motorOff(self):
        await self.controlSocket.send("stop")

    async def left(self):
        await self.controlSocket.send("left")

    async def right(self):
        await self.controlSocket.send("right")

    async def forwards(self):
        await self.controlSocket.send("forwards")

    async def backwards(self):
        await self.controlSocket.send("backwards")
