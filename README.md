# Robo-games-2020

## Task 
![Screenshot 2024-02-17 012338](https://github.com/vinwann/Robo-games-2020/assets/72453709/8ef8f512-7bf9-4bfa-9eda-04b80f52b0f9)

The goal of the robot is to navigate to the green box and get back to the start of the maze in the shortest time possible. Along the path are humans infected with COVID-19 the robot should avoid them. All humans wear a red colored t-shirt which was used to detect the humans. The blue blob represents sanitizers and the robot should collect them to reduce the infection level.

The project is done in Webots 2020B and python

OpenCV was NOT used and the image processing algorithm was made from scratch 

## Image processing

https://github.com/vinwann/Robo-games-2020/assets/72453709/72e1470e-4e94-44f7-8990-96085fe4552c

Detecting the human

![image](https://github.com/vinwann/Robo-games-2020/assets/72453709/aad25e52-f8e4-4d2d-b685-aea35763e6b8)

On the left is the video feed from the camera. On the right is a processed image where all the red pixels on the human is detected.

For the simplicity of computation at first, only a small part of the image is processed, and when an object of interest is detected a larger area of the image is processed.

In cases where a human is in front of a wall in a junction. The wall is detected as well. The robot would avoid getting close to that wall to turn

![image](https://github.com/vinwann/Robo-games-2020/assets/72453709/413ac9d8-d057-4e99-b725-207e97ba8559)
