# Computational Design of Telescoping Structures

Christopher Yu, Keenan Crane, Stelian Coros\
Carnegie Mellon University

This is the design system presented in "Computational Design of Telescoping Structures," published at SIGGRAPH 2017.
The website, paper, and other media can be found [here](http://www.cs.cmu.edu/~christoy/Projects/Telescopes.html).

## Requirements

* [Unity](https://unity3d.com/)
* [Gurobi](http://www.gurobi.com/) - Free licenses are available for academic users.
* [OpenSCAD](http://www.openscad.org/) - For compiling the output .scad file to a printable STL.

Unfortunately, our system may not run out-of-the-box on non-Windows operating systems, due to the inclusion of DLLs to get around Unity's inability to link to external libraries.

## Overview and controls

We recommend running the design system in Unity's play mode, rather than building a standalone executable.

Our design system can be loosely organized into phases, determined by what type of object is being worked with at the moment. The system begins in the _spline phase_. Optionally, one can begin with a mesh, and use our skeletonization routines to convert it to a 1D curve network. Alternatively, one can simply sketch a curve network using our spline tools.

Once a satisfactory spline has been obtained, Shift-P will finalize the spline and advance to the _discrete curve phase_. Here, one can run curvature and torsion smoothing flows to modify the curve. One can then run the solver to approximate the current curve by a piecewise helical curve with torsional impulses. If the result is acceptable, Enter will then finalize the curve and advance to the _torsion impulse curve phase_. In this phase, there is nothing to do but press Shift-I to create the telescope and advance to the _telescope phase_.

Once the telescope is created, Q and E retract and extend the telescope. ] (right bracket) performs one iteration of collision projection. Shift-Enter will export the telescope to an OpenSCAD file, which can then be compiled to a printable STL.

The operation of the system is controlled in part by the parameters in `Constants.cs`. Each parameter is commented with a description of what it affects.

### General camera controls:

* WASD moves the camera.
* Right-click-drag rotates the camera.

### Mesh skeletonization:

* Shift-G: Perform one iteration of volume-minimizing flow.
* Shift-H: Perform quadric edge collapse to convert the collapsed mesh to a 1D curve network.

### Spline phase:

* Shift-N adds a new spline.
* Shift-B adds a new bulb.
* Right-clicking on a spline control point inserts a new control point before it.
* Scroll wheel when hovering over a juncture resizes it.
* Left click drags around all elements.
* Drag a spline endpoint into a bulb to attach the spline to the bulb at that endpoint.
* Shift-P finalizes the spline and converts it to a discrete curve.

### Discrete curve phase:

* Shift-L toggles curvature flow on/off
* Shift-K toggles torsion flow on/off
* Ctrl-Shift-L replaces all curvature with the average value
* Ctrl-Shift-K replaces all torsion with the average value
* Shift-O attempts to approximate the current curve by a piecewise constant impulse curve, using a QP with linear constraints.
* Ctrl-Shift-O attempts to do the same thing by solving a linear system instead (not described in paper).
* After making an approximation, Enter confirms and finalizes the approximate curve.

### Impulse curve phase:

* Shift-I creates a telescope.

### Telescope phase:

* Q retracts the telescope, E extends it.
* ] (right bracket) performs one iteration of constraint projection on telescope attachment points.
* Shift-Enter exports the current structure to an OpenSCAD file. (This can be quite slow.)
