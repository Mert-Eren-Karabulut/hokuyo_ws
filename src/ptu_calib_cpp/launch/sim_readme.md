# Terminal 1: Start simulation
roslaunch ptu_calib_cpp sim_gazebo.launch

# Terminal 2: Acquire tilt data
roslaunch ptu_calib_cpp sim_acquire.launch mode:=tilt

# Terminal 3: Acquire pan data  
roslaunch ptu_calib_cpp sim_acquire.launch mode:=pan

# Terminal 4: Run the solver
roslaunch ptu_calib_cpp solve.launch