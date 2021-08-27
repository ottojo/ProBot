import asyncio

import pygame

from robot import Robot

pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()
print(joystick.get_name())


async def main():
    robot = await Robot.create("192.168.0.7:81")

    while True:
        pygame.event.pump()
        if (joystick.get_axis(0) > 0.5 and -0.5 < joystick.get_axis(1) < 0.5) or joystick.get_hat(0)[0] == 1:
            await robot.right()
            await robot.motorOn()
        elif joystick.get_axis(0) < -0.5 and -0.5 < joystick.get_axis(1) < 0.5 or joystick.get_hat(0)[0] == -1:
            await robot.left()
            await robot.motorOn()
        elif joystick.get_axis(1) > 0.5 and -0.5 < joystick.get_axis(0) < 0.5 or joystick.get_hat(0)[1] == -1:
            await robot.backwards()
            await robot.motorOn()
        elif joystick.get_axis(1) < -0.5 and -0.5 < joystick.get_axis(0) < 0.5 or joystick.get_hat(0)[1] == 1:
            await robot.forwards()
            await robot.motorOn()
        else:
            await robot.motorOff()

        cameraImage = robot.getCameraImage()

        await asyncio.sleep(1 / 20)


asyncio.run(main())
