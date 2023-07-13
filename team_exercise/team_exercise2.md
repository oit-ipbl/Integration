# Team exercise2

[README](../README.md)

---

## Objectives

Integrate image processing and robotics techniques and develop one  mini-game which includes both of the technologies. The mini-game can be used as a subset of the final competition.
Additionaly, all teams have to submit a slide to explain the mini-game.

## Prerequisite

- All members of your team have to finish all exercises.
  - [robots](https://github.com/oit-ipbl/robots)
  - [image processing](https://github.com/oit-ipbl/image_processing)
  - integration
    - [Message exchange between a python program running on Windows and a ROS node running on ubuntu](../win_single/win_single.md)
    - [Message exchange between a ROS node and multi Windows programs](../win_multi/win_multi.md)

## Deadline

- Submission URL for the poster introducing the game under development will open after Aug. 17.
- The URL and submission method will be published at SLACK #general channel.
- You have to submit the poster until Aug. 26.

## Team development

- Implement one mini-game, which includes windows side image processing technology and ROS side robotics technology.
  - The mini-game must contains at least two image processing games, and at least one robot movement on ROS.

- **You must complete this with team members!**
  - Break down the mini-game development into some tasks, and assign them to all members.
- We recommend developing together in the form of pair programming in the beginning.
  - Pair programming is a programming style in which two people look at the same screePair programming is a Pair programming is a 
  - Please make use of Pop and Miro.
    - See [Communication tools](https://github.com/oit-ipbl/portal/blob/main/setup/commtools.md)
- Here are some ideas to help you develop your team mini-game. Please refer to them.
  - They are just ideas, and we can not ensure anything about them.  
Use the list as a hint for the exercise and let's discuss with your team members.

### Mini-game ideas

If you and your team members have no ideas, the following list may help you.

- There are red and blue pillars around the robot on the stage simulator. The robot sends the color name (e.x. red and blue) of the objects around it to the Windows side. In windows side, python program presents the received color name from ROS side to the user, so the user presents a colored paper or object. When the color name specified by the user from the Windows side to the ROS side, the robot moves toward the pillar of the sent color.

- Calculation problem
  - Calculate the displayed formula (for example, the addition formula of two-digit numbers) and answer the calculation result with your finger.
  - Create a calculation problem using ```random``` (see [Challenge [Hands2]](https://github.com/oit-ipbl/image_processing/blob/main/advanced/holistic.md#challengehands2)).
- Staring Contest
  - If you don't laugh for 5 seconds after seeing the image presented by the robot, the human will win.
  - For example, you can use facial landmarks to determine if you laughed.
- Follow Me.
  - Move the object on the stage simulator, and control the robot to follow it.
  - User can send start signal of follow me with hand gesture that can be recognized by Windows side program using media-pipe.
  - cf. [This movie](https://www.youtube.com/watch?v=8-CcklPzvyo) shows an example follow me robot. It's not a simulation but real robots.

**Make interesting games!**

- Make a poster which explains your games.

  - Team name and member list.
  - Overview of the mini game created by tour team.
  - Screen shots.
  - Game Rules and how to play
  - Team members' roles.
    - Leader.
    - Algorithm desiginer.
    - Programmer (image, ROS etc.)
    - Slide designer.
    - Article writer.
    - Supporter.
    - ...