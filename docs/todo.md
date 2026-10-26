# To Do List
## General
- [X] Refactor entire codebase
- [X] Create files for camera, A* Search planner

## Moveement
- [X] Figure out how to do manual movement
    - [X] Add movement functions
        - [X] Face up/down/left/right
        - [X] Rotate clockwise/counter-clockwise
        - [X] Move forward (by 1 robot unit)
    - [X] Add manual controls to manually control robot in debug screen

## Map
- [X] Figure out how to get 2D occupancy map
    - [X] Maybe break map into grid and use A* Search or Dijkstra?
    - [X] Find a way to add a way to do dijkstra/A* search
- [X] Find how to locate robot on the map
- [X] Find how to locate goals on the map
- [X] Find how to add trail for robot on the map
- [X] Create topdown map to do A* Search on
    - Map broken up into pixels
    - Black for open
    - Grey for safety margin
    - White for wall
    - Green for robot center
    - Yellow for goal center

## Planner
- [X] Create A* Search
- [X] Find a way to utilise the planning map

## Camera Recording
- [X] Figure out how to get stream of robot and save it somehow

## Visualise
- [X] Create UI using Qt
- [X] Show current time running
- [X] Show robot pose (position, rotation, etc)
- [X] Show goal and end path
- [X] Show current top down world map
- [X] Show live camera feed from robot
