- You are helping me build and clean up ros1 packages for a duckiebot challenge. 
- All of the ros packages and their nodes run in a docker container defined by Dockerfile and docker-compose.yaml (entrypoint.sh)
    - workflow: see start.sh and attach_tmux.sh

- The packages for challenge 3 (ch3_obstacle_avoidance) and challenge 4 (ch4_mapping_pathplanning) are already implemented.
- challenges 3 and 4 build upon challenge 1 (lane following and red line stopping) and challenge 2 (intersection driving)
- Since challenges 3 and 4 built upon the results of challenges 1 and 2, the original challenge 1 and 2 code (that lives somewhere in this repo, but I don't know exactly on which branch anymore) is no out of date because challenge 3 and 4 optimized their logic aswell (ch3 and ch4 were started from copies of ch1 and ch2's code)

- In the next steps, we are going to rebuild challenges 1 and 2 from challenge 4's code.
- challenges 1 and 2 will get their own respective package with copies / modified versions of challenge 4' nodes (only the ones necessary)
- challenges 1 and 2 will each get their own launch file
