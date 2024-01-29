# MazeSolver
## Project of first year of robotics : implementing a maze solving algorithm on a simulation using CoppeliaSim.

Initially, we considered implementing a code to map the maze as a matrix, aiming for precise control over the robot's movements. However, we soon realized that this approach wasn't feasible for the type of robot we had at our disposal. Instead of opting for a completely random code, or what I like to call a "vacuum cleaner robot," where the robot simply runs into each wall before changing direction in the hope of eventually reaching the black square, we chose a code that allows us to track each wall detected by a sensor. When the sensor on one side detects a wall, we follow it until the sensor on the opposite side detects another wall, at which point we switch to following that one.

Before proceeding with that, we wanted to ensure that the black square was not located along the edges of the maze walls. Therefore, at first, the robot would circumnavigate the exterior walls of the maze to check if the black square was not positioned on the side. After this initial check, the robot would then follow the first wall it detects and subsequently switch to another as soon as it detects it. This approach ensures that the robot systematically explores the entire maze while maintaining a stable and predictable trajectory.

Subsequently, we implemented real-time trajectory correction using a proportional corrector. This correction relies on diagonal sensors at the rear of the robot.The error is simply the difference between the data from these sensors and the wanted distance from the wall, and it is then added to or subtracted from both motors. This correction is further adjusted using a correction coefficient, chosen based on the principles of control systems in automation, as well as through experimentation and iterative refinement.

We encountered several issues with our code. We noticed that depending on the PC, the variables we chose might not be adequate anymore. Additionally, we did our best to correct the trajectory properly, sacrificing the speed of the robot for adequate precision and stability. We strived to adhere to the instructions as closely as possible, even attempting to ensure that the robot returns to the red square as soon as possible. However, unfortunately, we couldn't find a way to differentiate between an interior wall and an exterior wall. Thus, even though the robot may take some time to return to the red square, it will likely explore the majority, if not all, of the maze, with a correct trajectory. Moreover, increasing the precision of the robot might lead it to get stuck in an infinite loop around certain walls. Hence, the coefficient of the corrector is not very high.

Regarding the instruction to identify multiple black squares before returning to the red square, we implemented it in the code. However, it's possible that the robot detects the same black square multiple times, and we don't have a sure way to exclude this possibility.





