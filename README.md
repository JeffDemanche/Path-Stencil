QT:
The stencil code is written in QT and will therefore require QT Creator to be installed. QT Creator 4.5.0 Community edition has been tested and works.

How to build:
The included project files should be all you need to build, so just open the project in QT Creator and build the project.

How to run:
The compiled application takes two command line arguments.
The first argument is the path to the scene file that is being loaded to be rendered.
The second argument is the path of the file that the rendered image will be written out to.

Additional command line arguments can be added in main.cpp. The stencil code uses QCommandLineParser to parse the command line arguments, so take a look at the documentation if you want to add more arguments.

About the code:
The main files you should need to edit/look at are pathtracer.cpp and the files in the scene/ directory, however feel free to modify any other code you like.

pathtracer.cpp is the class that gets passed in a scene and an array of pixels to fill in the color for, so this is where you should implement most of the actual path tracing.

In the scene directory, there is the Mesh class, which represents the objects that you are casting rays to collide with. Each mesh has a list of vertices, normals, colors and uvs, all of which are the same length, one for each vertex. They also contain a list of Vector3i's which correspond to the faces of the mesh. Each Vector3i describes a triangle in the mesh and contains 3 indices into the vertex, normal, etc arrays. There is also a list of integers, one for each face, that contains the indices of the material for that face. These indices index into the final array, the material array. This array is loaded from the .mat file that likely comes with the .obj file that you are loading, and will contain whatever material properties were specified for the mesh.

Scene files:
The stencil code uses a modified version of the CS123 scene files, which are xml files. The full spec for the scene files can be found in scenefile.pdf, however the stencil code only supports primitives of type "mesh". It does not support cubes, cylinders, etc., however meshes can be used to get any of those shapes, and if you want, you can add support for primitives. The materials of the scenefile can be used, or the materials that are parsed from each meshes associated .mat file can be used, whichever you find better.

Two example scenes can be found in the example-scenes directory. One of them should render a motorcycle, and the other should render a medieval boat.

Because pathtracers don't use point lights, but instead use area light, you can either create area lights in the scene file or create meshes in your scene that have emissive materials. To create area lights in the scene, create a light whose type is area, and specify a position, color, direction, width, and height. An example of an area light defined in a scene file can be found in the example scene scene1.xml.