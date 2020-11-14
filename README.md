=============================================
DoN AI Navigation Plugin for Unreal Engine 4
=============================================

This plugin provides a 3D dynamic pathfinding system for use with Unreal Engine 4. This system was first developed for my game [DoN The Nature Game](http://www.drunkonnectar.com/) and I decided to make a plugin out of the navigation module as a gift to the Unreal community.

It's primarly designed for Flying AI creatures based in dynamic or procedural worlds that need to solve complex pathfinding tasks and other behavior that cannot be readily solved  by Unreal's native AI navigation system and also for maps that are too complex to be solved by simple ray-tracing/sweeping heuristics or by waypoint systems. 

Where possible, I recommend that you use Unreal's native AI navigation or simpler navigation strategies; use this system when the more conventional strategies cannot solve your pathfinding usecases.

The plugin provides the following:
* Navigation Manager actor for configuring the system
* "Fly To" behavior tree node that can be readily dropped into your behavior trees
* Nodes for managing dynamic collision and pathfinding in your scene.
* A pathfinding API that advanced users can use for custom navigation queries from either Blueprints or C++

=============================================
Download Sample Project
=============================================
I've created a sample project for you to quickly test the system and understand the different usecases it covers. 

Please visit the following page to grab it:

http://www.drunkonnectar.com/3d-pathfinding-ue4/

=============================================
Overview and Tutorial
=============================================
[![Youtube video - Overview and Tutorial](http://www.drunkonnectar.com/wp-content/uploads/2016/03/ThumbnailWithYoutubeIcon.jpg)](https://www.youtube.com/watch?v=6Tr_K551zvI)

=============================================
Technical Overview
=============================================
For a technical overview of the project, please visit this link:<br>
http://www.drunkonnectar.com/3d-pathfinding-ue4/#TechnicalOverview

=============================================
Update
=============================================
Update by yhyu13@github from 4.19 to 4.25