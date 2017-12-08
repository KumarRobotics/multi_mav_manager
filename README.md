# multi_mav_manager

This is a multi MAV manager which leverages mav_manager and quadrotor_control
for each agent.  It leverages CAPT for multi-agent planning to ensure there are
no collisions and provides additional formation functionalities.

**Note:** There are a packages that require submodules. Follow the instructions below to initalize the submodules.

### Submodules
###### Package (submodule):
capt (include/capt)

Please initialize the submodules by running the following in the repository directory:

    $ git submodule init
    $ git submodule update

**Note:** This is currently dependent on the feat/line_tracker_min_jerk_duration
branch of quadrotor_control
