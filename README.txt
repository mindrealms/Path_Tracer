PATH TRACING README

How to run:
- standard format of input file, then output file, then path of light probe file, number of samples (an integer) **
[ eg. /mithril.xml /invincible.png /probe/cathedral.hdr 20 ]
if you’re not using a probe file, just type “none” or whatever other invalid thing you want

Structure of code:
- traceScene: loops through image pixels and traces rays through them, then tonemaps. Also prints out time time elapsed for render to finish.

- tracePixel: loops through all x,y pixels and shoots N_SAMPLES rays (of random direction) through each of them, and averages output value. The value of N_SAMPLES can be changed in the pathtracer.h file. 

- traceRay: traces each ray shot through a pixel. If intersection with an object is found, it checks the type of the material of that surface, computes a random next direction based on it, and first, if the surface is not ideal reflective, it computes light contribution (directLighting). Then it weighs the probability of continuing (1) initially by 80% for the first 5 non-mirror bounces, as they are more important, then (2) by the magnitude of the bsdf. Continuing (pdf_rr) is then determined by comparison with a random number ( such that, asymptotically on average, it gives the same result) and if the ray's life has been temporarily extended (unfortunately rays aren't eternal beings in the CG world, sorry rays), then (1) the function is either recursively called without depth increase for all mirror/refractive surfaces, or (2) recursively called with a depth increase for all other surface types. Finally, if the first ray through the pixel intersects with the light, emission is counted in order for us to be able to see it.

- computeBSDF: based on the type of material, an incoming ray, the surface normal, and a next direction vector, it computes the pdf of sampling that next direction (used to divide radiance at the end in traceRay).

- directLighting: computes contribution of light on a given point in the scene; if occluded, or if mirror reflective or refractive, direct lighting is not counted. Lights are provided by scene's getPathLights() function, which returns a vector of PathLight structs (all light objects in the scene), then samples a random point inside a random triangle chosen from a random light in that vector.

- sampleNextDir: computes next direction based on object material and incoming ray. For diffuse and glossy surfaces, outgoing vector is chosen randomly over the hemisphere (then weighed accordingly for glossy surfaces in computeBSDF). For mirror surfaces, mirror direction is always picked with pdf = 1. For refractive, the next direction is chosen in getRefractVec (either a reflected, internal reflected, or refracted, with pdf = 1). Returns a vec4 that contains the outgoing direction in .xyz, and the pdf of sampling that direction in the .w component.

- getMirrorVec: computes perfect mirror reflection vector based on incoming direction and surface normal.

- getRefractVec: weighs probability of sampling a refracted or reflected ray by computing the schlick approximation, or internal reflection vector.

- checkType: checks the type of material of a surface in the scene. Handles illum 2,5, and 7. 
	2: weighs diffuse and specular energy from file, whichever is greater dominates object material (does not handle mixed diffuse/glossy).
	5: ideal reflective (mirror)
	7: refractive (fresnel)
returns integer specifying materials (enums in pathtracer.h), or invalid if material (illum value) is not handled. 

- toneMap: tone maps final radiance values using the reinhard operator. 

- quasiMonteCarlo: uses van der Corput/Halton sequence to sample pixels through the stratified sampling grid, instead of uniform-randomly. Standard is with BASE_X = 2 and BASE_Y = 3, but you can change these values from pathtracer.h if needed. You can also change the dimension of the grid that each pixel is subdivided into by changing the value of GRID_DIM (default is 10 x 10).  
**** Images of comparison:

----- In Scene.cpp ------
created a struct PathLight, that stores a light id, number of triangles on light, emission of light (assumption: uniform emission among all triangles of a single light source), average position of light (avg of centroid of all triangles), area (sum of all triangle areas), intensity (magnitude of emission), and a vector of all faces (each face consisting of 3 vertices), to be used for random-point-on-triangle computation.
------------------------- 
