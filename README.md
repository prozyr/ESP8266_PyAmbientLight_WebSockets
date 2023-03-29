# Python-ambientLight

Program Description
This program monitors the main monitor screen by sampling its image,
dividing it into zones that are distributed across multiple tasks to take advantage of the multi-core architecture of the processor.
The result of such a task is the average of the colors in the zone. If all tasks return a result, those results are averaged.
This result is sent using WebSockets to the SmartDesk project, which, among other things, manages the desk's LED lighting.

The program is designed to optimize the use of the computer's resources while providing a seamless integration with the SmartDesk project.
By sampling the screen in zones and distributing the workload, the program takes advantage of the multi-core architecture of the processor,
allowing for a faster and more efficient processing of the image. The resulting average color is then sent to the SmartDesk project using WebSockets,
allowing for a real-time update of the desk's LED lighting.

Overall, this program provides an innovative solution for integrating the computer screen with the SmartDesk project,
improving the user experience and making the desk more functional and customizable.
