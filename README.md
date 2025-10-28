# Ship Hull Coverage - Parker Gutierrez & Alexandria Nilles
This is the ship hull coverage project.
The idea is that a swarm of robots is deployed onto a ship's hull with the goal of covering the entire hull, with some task in mind.
This task could be scraping barnacles off which reduce fuel efficiency greatly, or inspect the hull for various structural integrity flaws.

## The problem
This is a coverage problem meant for decentralized swarms. Salt water absorbs RF signals underwater, so robots may only communicate at close range.
Without some kind of unique engineering solution or specialized communication method, close-range communication is a good option.

Robots drive around randomly attached to the hull of the ship. While cleaning/inspecting they keep track of where they've been using dead reckoning plus an internal model of the ship's hull.
When robots come near one another, they exchange information. They share with each other their entire model of what's already been covered.
Robots know where the uncovered parts of the ship's hull is and will navigate there. Eventually, robots will have shared all their models with one another.
Once each robot is satisfied, it will drive back up to the surface.

The novel part of this problem is the navigation of both convex and concave surfaces (I believe), as well as providing a coverage algorithm for them (not novel probably)

## About the simulator
This simulator is done using Pymunk, a physics simulator.
