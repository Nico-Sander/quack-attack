# Documentation of the issues we faced

## Hardware not working as expected

### Symptoms

- LCD screen not turning on
- Top button not having any effect
- Battery button not having any effect
- Unable to turn off duckiebot without unplugging cables
- roscore not discoverable
- http://trick.local not accessible

### Steps taken
- Check the containers running on the duckiebot:

    - Connect to the duckiebot via SSH:
        ```bash
        ssh duckie@trick.local
        ```
        (password: quackquack)

    - Check the status of the docker containers:
        ```bash
        docker ps -a
        ```
    - Output:
        ```bash
        CONTAINER ID   IMAGE                                              COMMAND                  CREATED       STATUS                      PORTS     NAMES
        bdb7ec9a67ef   duckietown/dt-code-api:v4.1.0-arm64v8              "/entrypoint.sh bash…"   8 weeks ago   Exited (128) 43 hours ago             code-api
        4d4f187c6eba   duckietown/dt-device-proxy:v4.2.0-arm64v8          "/entrypoint.sh bash…"   8 weeks ago   Dead                                  device-proxy
        060335d2194c   duckietown/dt-car-interface:v4.1.0-arm64v8         "/entrypoint.sh bash…"   8 weeks ago   Dead                                  car-interface
        b96a1958ca7e   duckietown/dt-device-health:v4.2.1-arm64v8         "/entrypoint.sh bash…"   8 weeks ago   Exited (128) 4 days ago               device-health
        952bd41ccd50   duckietown/dt-rosbridge-websocket:v4.1.0-arm64v8   "/entrypoint.sh bash…"   8 weeks ago   Exited (128) 43 hours ago             rosbridge-websocket
        087e307b5631   duckietown/dt-duckiebot-interface:v4.3.4-arm64v8   "/entrypoint.sh bash…"   8 weeks ago   Exited (128) 43 hours ago             duckiebot-interface
        d7c8a5b8c055   duckietown/dt-ros-commons:v4.3.0-arm64v8           "/entrypoint.sh bash…"   8 weeks ago   Exited (127) 4 days ago               ros
        afab58a54378   duckietown/dt-files-api:v4.1.0-arm64v8             "/entrypoint.sh bash…"   8 weeks ago   Exited (128) 43 hours ago             files-api
        2ad1719093e6   duckietown/dt-device-online:v4.3.0-arm64v8         "/entrypoint.sh bash…"   8 weeks ago   Exited (127) 4 days ago               device-online
        2440c7fb60bc   duckietown/dt-device-dashboard:v4.1.0-arm64v8      "/entrypoint.sh bash…"   8 weeks ago   Up 26 minutes (unhealthy)             dashboard
        691d2841a85e   duckietown/portainer:daffy-arm64v8                 "/portainer --host=u…"   8 weeks ago   Up 26 minutes                         portainer
        ```
    - Results:
        - `ros` containre exited -> explains why the roscore is not reachable
        - `device-proxy` is dead -> explains why http://trick.local is not reachable
        - `duckiebot-interface` and `car-interface` exited -> explains why LCD and buttons are not working

- Remove all the containers:
    ```bash
    docker rm -f $(docker ps -aq)
    ```

    - Result:
        - Removal of some containers fails:

            ```bash
            bdb7ec9a67ef
            952bd41ccd50
            087e307b5631
            afab58a54378
            2440c7fb60bc
            691d2841a85e
            Error response from daemon: container 4d4f187c6eba42776e88ff5d51775bb5c3c930ae60cf803133c1d119a7a7447a: driver "overlay2" failed to remove root filesystem: unlinkat /var/lib/docker/overlay2/ba94e12a3efd7bef18900596c8970901e28607fb1ec3756313fdaf0b755666a2/diff/usr/lib/aarch64-linux-gnu/libv4l2.so.0.0.999999: structure needs cleaning
            Error response from daemon: container 060335d2194c4092e70d8df4eb7a30061ecce9f627c828ee2081eb8b1a8aa63f: driver "overlay2" failed to remove root filesystem: unlinkat /var/lib/docker/overlay2/f854453512ba41313f3493a763644e2c7f7c8ce6e79559cd22074335576a0793/diff/usr/lib/aarch64-linux-gnu/libv4lconvert.so.0.0.999999: structure needs cleaning
            Error response from daemon: container b96a1958ca7e0fb379fcca2ce99aaf4836ae7b89c50a8ca67cb5c13d209f70c6: driver "overlay2" failed to remove root filesystem: unlinkat /var/lib/docker/overlay2/5aefb1930495ccdd694406c6c05ca0df11ee62fbaf415eb4b17d02d4141353d2/diff/usr/lib/aarch64-linux-gnu/tegra/libcuda.so: structure needs cleaning
            Error response from daemon: container d7c8a5b8c055a4879c6a7ccf0ca4bfaccc7ba3b4beadc5ff742392f613e484a3: driver "overlay2" failed to remove root filesystem: unlinkat /var/lib/docker/overlay2/792ccc9a68384264a029ae95b5d25b7c5e3c3d0e76965eb0933d4f9a62e0476a/diff/usr/lib/aarch64-linux-gnu/libv4lconvert.so.0.0.999999: structure needs cleaning
            Error response from daemon: container 2ad1719093e600cdfb37e7b6fed05fed1910ccf61c405a45606385ff74e363b8: driver "overlay2" failed to remove root filesystem: unlinkat /var/lib/docker/overlay2/50a09cd67acd0b47b39cca199513de0a1516242ad37b2f1aea897b3341595746/diff/usr/lib/aarch64-linux-gnu/libdrm_nvdc.so: structure needs cleaning
            ```

        - Running `docker ps -a` again shows that some containers are left over:

            ```bash
            CONTAINER ID   IMAGE                                        COMMAND                  CREATED       STATUS                PORTS     NAMES
            4d4f187c6eba   duckietown/dt-device-proxy:v4.2.0-arm64v8    "/entrypoint.sh bash…"   8 weeks ago   Removal In Progress             device-proxy
            060335d2194c   duckietown/dt-car-interface:v4.1.0-arm64v8   "/entrypoint.sh bash…"   8 weeks ago   Removal In Progress             car-interface
            b96a1958ca7e   duckietown/dt-device-health:v4.2.1-arm64v8   "/entrypoint.sh bash…"   8 weeks ago   Removal In Progress             device-health
            d7c8a5b8c055   duckietown/dt-ros-commons:v4.3.0-arm64v8     "/entrypoint.sh bash…"   8 weeks ago   Removal In Progress             ros
            2ad1719093e6   duckietown/dt-device-online:v4.3.0-arm64v8   "/entrypoint.sh bash…"   8 weeks ago   Removal In Progress             device-online
            ```

        - Error message `structure needs cleaning` reveals that the SD card likely suffored **filesystem corruption**

    - Force a Filesystem Repair
        
        - 1. Request a filesystem check on the next boot:

            ```bash
            sudo touch /forcefsck
            ```

        - 2. Reboot the duckiebot

            ```bash
            sudo reboot
            ```

        - Results:
            
            - After rebooting and ssh-ing back into the duckiebot, here are the container states:

                ```bash
                CONTAINER ID   IMAGE                                        COMMAND                  CREATED       STATUS    PORTS     NAMES
                4d4f187c6eba   duckietown/dt-device-proxy:v4.2.0-arm64v8    "/entrypoint.sh bash…"   8 weeks ago   Dead                device-proxy
                060335d2194c   duckietown/dt-car-interface:v4.1.0-arm64v8   "/entrypoint.sh bash…"   8 weeks ago   Dead                car-interface
                b96a1958ca7e   duckietown/dt-device-health:v4.2.1-arm64v8   "/entrypoint.sh bash…"   8 weeks ago   Dead                device-health
                d7c8a5b8c055   duckietown/dt-ros-commons:v4.3.0-arm64v8     "/entrypoint.sh bash…"   8 weeks ago   Dead                ros
                2ad1719093e6   duckietown/dt-device-online:v4.3.0-arm64v8   "/entrypoint.sh bash…"   8 weeks ago   Dead                device-online
                ```
            -> This indicates that the filesystem corruption could not be fixed.

    - Factory reset the docker engine.
        
        1. Stop the docker service to release any locks on the files:

            ```bash
            sudo systemctl stop docker
           ```

        2. Aggressively remove the entire Docker data directory (this wipes all images, containers, and corrupted overlay files)

            ```bash
            sudo rm -rf /var/lib/docker
            ```

            Output:

                ```bash
                rm: cannot remove '/var/lib/docker/overlay2/5aefb1930495ccdd694406c6c05ca0df11ee62fbaf415eb4b17d02d4141353d2/diff/usr/lib/aarch64-linux-gnu/tegra/libcuda.so': Structure needs cleaning
                rm: cannot remove '/var/lib/docker/overlay2/5aefb1930495ccdd694406c6c05ca0df11ee62fbaf415eb4b17d02d4141353d2/diff/usr/lib/aarch64-linux-gnu/tegra/libnvomx.so': Structure needs cleaning
                rm: cannot remove '/var/lib/docker/overlay2/f854453512ba41313f3493a763644e2c7f7c8ce6e79559cd22074335576a0793/diff/usr/lib/aarch64-linux-gnu/libv4lconvert.so.0.0.999999': Structure needs cleaning
                rm: cannot remove '/var/lib/docker/overlay2/ba94e12a3efd7bef18900596c8970901e28607fb1ec3756313fdaf0b755666a2/diff/usr/lib/aarch64-linux-gnu/libv4l2.so.0.0.999999': Structure needs cleaning
                rm: cannot remove '/var/lib/docker/overlay2/792ccc9a68384264a029ae95b5d25b7c5e3c3d0e76965eb0933d4f9a62e0476a/diff/usr/lib/aarch64-linux-gnu/libv4lconvert.so.0.0.999999': Structure needs cleaning
                rm: cannot remove '/var/lib/docker/overlay2/50a09cd67acd0b47b39cca199513de0a1516242ad37b2f1aea897b3341595746/diff/usr/lib/aarch64-linux-gnu/libdrm_nvdc.so': Structure needs cleaning
                ```

            -> Even the aggressive `rm -rf` command was not able to remove the broken containers.

        3. Trying to trick docker into thinking the corruped files are missing by renaming the directory

            ```bash
            sudo mv /var/lib/docker /var/lib/docker_corrupted
            ```

            ```bash
            sudo reboot
            ```

            -> After that `docker ps -a` shows that there are no more containers

    - Reinstalling the duckiebot software stack using the DuckieTownShell `dts`

        - Trick `dts` into accepting the modern `docker compose` command instead of the old `docker-compose` it relies on:

            ```bash
            sudo bash -c 'echo -e "#!/bin/bash\ndocker compose \"\$@\"" > /usr/local/bin/docker-compose'
            sudo chmod +x /usr/local/bin/docker-compose
            docker-compose version
            ```

        - Update the duckiebot

            ```bash
            dts duckiebot update trick
            ```
        -> Unfortunately this gets stuck on pulling the `dt-files-api` docker images. No solution has been found for this.

    - Reflashing the Duckiebots SD-card according to the guide at: https://docs.duckietown.com/daffy/opmanual-duckiebot/setup/setup_sd_card/cli.html

        - Flash succeeded
        - Running `dts duckiebot update trick` now also succeeds and pulls the necessary images.

---

Total time invested: 6












