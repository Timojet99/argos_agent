# FreiCAR Base

## Overview
This repository contains:

- The `freicar_agent`, a simple example agent node that demonstrates how to
  send steering commands to the real-world cars and how to interact with the
  chaperone. Also contains an URDF description of the FreiCARs.
- The `freicar_chaperone`, a node which must run if multiple agents are active
  at the same time. It tries to detect and prevent impending collisions by
  keeping track of all agents state and sending out stop commands. It also
  prevents cars from driving off-track.
- `freicar_msgs`, some essential message definitions for communication between
  the agent, chaperone and the car hardware.
- `freicar_launch`, launch files for the ZED and Realsense cameras

## Development

Please note that this code follows the Google Style Guide accessible at: https://google.github.io/styleguide/cppguide.html

We have added several formatting and static code checks.
These checks are run automatically whenever you commit to your repository by using githooks.
In particular, we use the tool `pre-commit` (https://pre-commit.com/).
You can install them by following these steps:
1. Install python dependencies: `pip install -r requirements.txt`
2. Install apt dependencies: `apt-get install cppcheck`
3. Install [pre-commit](https://pre-commit.com/) githook scripts: `pre-commit install`
4. Update [pre-commit]: `pre-commit autoupdate`

You can manually run the checks on all files using `pre-commit run --all-files`.

To skip the checks, use the `--no-verify` flag when committing.

For various settings, see the respective configuration files of the tools: `.clang-format`, `CPPLINT.cfg`,
and `pyproject.toml`.
