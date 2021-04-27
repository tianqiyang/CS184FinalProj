# master repo adapted from proj4

## build
To do simulation, 
1. first replace the ext folder with the one in proj4 repo. 
2. Then compile the repo in the same way as previous projects:  (e.g. for mac)
- `mkdir build`
- `cd build`
- `cmake ..`
- `make`
3. Finally run `./clothsim -f ../scene/env.json`.

## current feature
Features currently implemented:
1. Point mass model for birds. 
2. Internal forces cohesion, separation and alignment, and external force wind
3. interactive features, including:
- Set designated direction (x, y, z).
- Weights for Cohesion, Separation and Cohesionn.
- Set number of birds for simulation.
- Set simulation speed of birds by number of frames/s and number of steps/frame.
- Set a way to render birds (wireframe or ball).

## current bug
1. When separation weight increased, can never go down again. 
