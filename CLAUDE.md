- You are working exclusively on the ch4_mapping_pathfinding ROS1 package in `src/packages/`

- Parts of the mapping have already been implemented.

- You can find all relevant information about what this package needs to achieve in `src/packages/ch4_mapping_pathfinding/docs`:
    - `01-information.md` contains all the original information about the challenge the package needs fulfill
    - `02-additional-information.md` contains additional information given to us including some of my initial thoughts, as well as strict requirements that need to be fulfilled. 
    - these files are in german, but the rest of the packages primary language should be english.

- the package runs in a docker container. You can find all of the docker and workflow related files in the repo root:
    - `Dockerfile`: Docker Image definition
    - `docker-compose.yml`: Container building
    - `start.sh`: Container starting, networking, etc.
    - `attach_tmux.sh`: Creating a new tmux session with 4 panes attached to the running container

- Claude Fable 5 (which is more capable than you Claude Opus 4.8) already analyzed the current state and came up with a plan to complete the challenge. It generated two files in `src/packages/ch4_mapping_pathfinding/docs/`:
    - `current_implementation-fable-analysis.md`: An analysis of what the architecture / package can already do now
    - `plan-fable.md`: A detailed plan of the steps necessary to complete all of the requirements for this package. This is what needs to be implemented!

- General instructions:
    - Write and run offline tests whenever applicable (no robot driving required) but only on bigger changes
    - Continuously document the state of the package in `src/packages/ch4_mapping_pathfinding/docs/current-state.md` in a structured way. Keep track of where we are in the development phase, TODOs, next steps in a well structured way.
    - Write any new code and comments in english. You can leave the existing german code as is, only convert it to english if you actually change it.
