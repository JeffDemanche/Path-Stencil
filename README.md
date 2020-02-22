# How to Run

My program takes the following arguments: [scenefile] [output] [samples] [hdrfile] [aperture] [focal]

 - scenefile - Path to the XML scenefile.
 - output - Path to the output PNG image.
 - samples - A positive number of path samples per pixel.
 - hdrfile - Path to a .hdr lightprobe image (the circular one). If this file can't be found, the environment will be black, so just put some string here to ignore IBL.
 - aperture - Aperture size, 0 is no DoF, larger values make out of focus regions more out of focus.
 - focal - Focal distance. Distance at which objects are in focus.

# Bugs

The only bug is I'm hard multiplying all emission values by 5 because otherwise the render is very dark and I can't figure out why.

# Features

 - BRDFs: Diffuse and Phong BRDFs live in separate functions in `PathTracer.cpp`, but reflections and refractions are more integrated into `traceRay`. I did write separate functions to calculate reflection and refraction vectors, and also for fresnel.

 - Soft shadows: `directLightEstimator` calculates direct light intensity for a point. It gets called for every emissive triangle in the scene, and randomly samples incoming light from points that lie on that light. This sampling gives soft shadows.

 - Indirect lighting: Implemented in `traceRay`.

 - Russian roulette: Paths are terminated with a constant probability `pdf_rr`, and then the lighting vector is normalized by that probability.

 - Event splitting: Implemented in `traceRay`.

 - Tonemapping: Using Reinhard operator.

### Extra features

 - Image-based lighting: I use [this](https://www.flipcode.com/archives/HDR_Image_Reader.shtml) library to load .hdr files into memory. HDR files store the three channels for each pixel in a single 32-bit float to save space, so this library parses that. To implement IBL, every ray which doesn't intersect with geometry in the scene is mapped to a pixel on the HDR image according to the equations on [this](https://www.pauldebevec.com/Probes/) site to determine its intensity. I don't have a user-defined control on environment intensity coefficient, but it would be trivial to implement (just multiplying the output of the `sampleHDR` function).

 - Depth of Field: I implemented a simple depth of field with help from [this](https://www.keithlantz.net/2013/03/path-tracer-depth-of-field/) site and [this](https://medium.com/@elope139/depth-of-field-in-path-tracing-e61180417027) article. The variables it introduces are aperture size and focal length.

# Images

cornell_vanilla.png - Cornell box with mirror reflection, refraction, diffuse BRDF, direct and indirect lighting, russian roulette, soft shadows, tonemapping.

conell_dof.png - Cornell box with DoF enabled. You can see the rear sphere is in focus and the near sphere is blurry. Aperture size = 0.2, focal distance = 4.4.

hdr_refraction.png - A dialectric sphere with an HDR image-based lighting map.

hdr_phong.png - A phong-shaded sphere with HDR lighting. This is noisy because the HDR lighting is only computed through indirect rays. This was 250 samples.

indirect.png - Indirect bounce light sampling only. 150 samples.

direct.png - Single bounce direct lighting only.
