- You are working exclusively on the mapping_pathfinding ROS1 package in `src/packages/`

- Parts of the mapping have already been implemented.

- You can find all relevant information about what this package needs to achieve in `src/packages/docs`:
    - `01-information.md` contains all the original information about the challenge the package needs fulfill
    - `02-additional-information.md` contains additional information given to us including some of my initial thoughts, as well as strict requirements that need to be fulfilled. 
    - these files are in german, but the rest of the packages primary language should be english.

- the package runs in a docker container. You can find all of the docker and workflow related files in the repo root:
    - `Dockerfile`: Docker Image definition
    - `docker-compose.yml`: Container building
    - `start.sh`: Container starting, networking, etc.
    - `attach_tmux.sh`: Creating a new tmux session with 4 panes attached to the running container
