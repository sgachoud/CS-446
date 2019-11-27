# Project proposals
 
# Collisions and Plasticity
Simon Wicky, Ludo Hoffstetter, Sébastien Gachoud

## Summary
The general idea behind project is to showcase an explosion, which would happen close to a mesh and permanently damage it.
To produce this effect, we would need three components namely:

* Plasticity
* Collision
* Punctual explosion

As it is more interesting to implement the first two components, the actual explosion will be moved to the extended goals.


## Collisions
This part of the project is about implementing collision within the same mesh, e.g. the eight-figure mesh provided will not be able to self-intersect anymore, when dragged around. 
### Technical approach
To implement collision, we will need to:

* Detect collisions:  
  For each point, determine whether it is inside or outside the mesh
* Determine what to do with a point inside the mesh  
  We cannot just push it to the nearest edge, it could go through the model
* Resolve the collision by moving the point where it should be with the help of a constraint

We can already anticipate difficulties to determine where the problematic vertex should go. 
The collision constraint should have a strong weight to look realistic.

The collision with the floor will be kept as is.

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

## Extended goals

As stated in the summary, we would like to add point explosion. We would be able to trigger an explosion at a given coordinates with a given force. It would then create a shockwave traversing the mesh, which will react accordingly. If we fix the mesh somewhere, it could for example wiggle around and deform itself if the explosion was too strong.



