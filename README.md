# master repo adapted from proj4

To do simulation, compile the repo in the same way as previous projects, and then run `./clothsim -f ../scene/env.json`.

Features currently implemented:
1. Added 4 planes to block movement outside the window. Planes are defined in `scene/test.json`. Can add up to 6 planes, named plane1, plane2, ..., plane6. TODO: Fix plane shape.
2. Added initial speed which is to right until hit the right plane and bounce back. Then same action when bouncing on left plane. The speed is defined as a parameter of mass point objects.
3. remove springs, and put mass points in random positions in a range of specified space at the start of simulation instead of in a grid. 
