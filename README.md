Simulation folder contents and placeholders

Created files:
- control_rw.m  : simple LQR controller placeholder
- dynamics_rw.m : simple dynamics placeholder
- SIMULINK_SIM.m : script to open and run `SIMULATION_ONE` Simulink model
- SIMULATION.mlx : placeholder text for live script

Notes about Simulink model files (.slx / .slxc):
- Binary Simulink files cannot be recreated as text here. If you have the original `SIMULATION_ONE.slx` and `SIMULATION_ONE.slxc`, copy them into this folder so you can open them in MATLAB/Simulink.
- I created two placeholder files named `SIMULATION_ONE.slx.placeholder` and `SIMULATION_ONE.slxc.placeholder` to remind you to add real model files.

How to run:
1. In MATLAB, set the current folder to this `simulation` directory:

   cd '/Users/bbtp/Documents/GitHub/CUBESAT_THESI/CUBESAT_THESI/simulation'

2. Run the build or simulation scripts, for example:

   run('build_reaction_wheel_model.m')
   SIMULINK_SIM

If you want, I can also copy in a real `.slx` model if you provide it, or I can try to export a minimal programmatic Simulink model from MATLAB if you allow me to run MATLAB on your machine.
