# 2-D-Physics-Engine-for-Spring-Mass-Systems

This is a 2-D physics engine that I'm building for a larger game development project.

So far, I'm working on implementing the physics for simple spring-mass systems.

The primary focus (at the moment) is building force generators for gravity and static forces
since those are necessary for basically any physics engine.

Primary Components:
  1. System State: a structure that tracks various object properties, e.g. position, velocity,
                   and orientation.
  2. Force Generator(s): a structure that applies forces to the objects in the scene.
                         There are seperately implemented force generators for gravity,
                         springs, and static forces.
  3. Rigid Body Object: a structure that simply simulates the properties of a rigid-body
                        object.
  
Next, I'm going to work on implementing a constraints solver (which is what the Matrix and
Constraint files are for).

Thanks for checking out my project!
