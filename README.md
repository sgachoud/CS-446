# Project proposals
 
# Plasticity and Explosions
Simon Wicky, Ludo Hoffstetter, Sébastien Gachoud

## Summary
The general idea behind project is to showcase an explosion, which would happen close to a mesh and permanently damage it.
To produce this effect, we would need three components namely:

* Plasticity
* Friction
* Box
* Punctual explosion

## Plastic deformations
If an object is too strongly stretched or bent, its rest shape will change. We want our simulation to behave in this sense.

### Technical approach
To implement plasticity, we will need to:
 
* Compute per edges tensions (length difference)  
  We need a way to tell whether an edge has been stretched too much
* Compute per vertices bending (curvature difference)  
  We need a way to tell whether a part of the mesh (a vertex) has been bent too much
* Adapt the rest length and curvature if the difference exceeds a given threshold

It is not sure that this approach is enough for the plasticity to look realistic.

## Box and Friction
To keep the simulation interesting, the object will be bounded to a box. The walls of the box will be maybe transparent so we can actually see the object. Friction will also be added so the object will eventually stop itself on the ground.

### Technical Approach
The walls are very straightforward to implement, by modifying the floor constraint.
The friction can be implemented using a constraint that force vertices near the floor and walls to stay in place.

## Punctual Explosion

As stated in the summary, we would like to add point explosion. We would be able to trigger an explosion at a given coordinates with a given force. It would then create a shockwave traversing the mesh, which will react accordingly. If we fix the mesh somewhere, it could for example wiggle around and deform itself if the explosion was too strong.


