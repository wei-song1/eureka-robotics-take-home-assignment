# To Do List
## General
- [X] Refactor entire codebase
- [ ] Create files for camera, A* Search planner

## Moveement
- [X] Figure out how to do manual movement
    - [X] Add movement functions
        - [X] Face up/down/left/right
        - [X] Rotate clockwise/counter-clockwise
        - [X] Move forward (by 1 robot unit)
    - [X] Add manual controls to manually control robot in debug screen

## Map
- [X] Figure out how to get 2D occupancy map
    - [ ] Maybe break map into grid and use A* Search or Dijkstra?
    - [ ] Find a way to add a way to do dijkstra/A* search
- [X] Find how to locate robot on the map
- [X] Find how to locate goals on the map
- [X] Find how to add trail for robot on the map
- [ ] Add current time to top-down map since it's only updated when moving
- [ ] Create topdown map to do A* Search on
    - Map broken up into pixels of columns/rows
    - 0 for cannot go
    - 1 for open
    - 2/3 for close to wall to penalise going close to wall if possible
    - Expand wall outwards by robot radius + padding, then make the edges 2/3

## Planner
- [ ] Create A* Search
- [ ] Find a way to utilise the planning map

## Camera Recording
- [ ] Figure out how to get stream of robot and save it somehow

## Visualise
- [ ] Create UI using Qt
- [ ] Show current time running
- [ ] Show robot pose (position, rotation, etc)
- [ ] Show goal and end path
- [ ] Show current top down world map
- [ ] Show live camera feed from robot
