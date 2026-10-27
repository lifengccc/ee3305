# EE3305 - ROS 2 Jazzy workspace helpers
#
# Works two ways:
#   * from the Arch host         -> wraps `distrobox enter ros2-jazzy`
#   * from inside the container  -> runs directly
#
# Usage:
#   make build
#   make run
#   make run CPP=True
#   make run HEADLESS=True LIBGL=True
#   make shell
#
# The host PATH is replaced with Ubuntu's default PATH so the Arch
# pyenv shims (~/.pyenv/shims) do not leak into the container.
# ROS 2 Jazzy must use /usr/bin/python3 (3.12).

SHELL := /bin/bash
.DEFAULT_GOAL := help

CONTAINER ?= ros2-jazzy
HEADLESS  ?= False
LIBGL     ?= False
# Force Gazebo discovery onto loopback (fixes multi-interface hosts). Disable with: make run GZ_IP=
GZ_IP     ?= 127.0.0.1

# CPP is a GNU make built-in (defaults to "cc -E"); only override its default.
ifeq ($(origin CPP),default)
  CPP := False
endif

WS := $(CURDIR)

ENV := export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin; \
       unset PYENV_ROOT PYENV_SHELL PYENV_VERSION; \
       export QT_QPA_PLATFORM=xcb; \
       if [ -n "$(GZ_IP)" ]; then export GZ_IP=$(GZ_IP); fi; \
       source /opt/ros/jazzy/setup.bash; \
       cd "$(WS)"

ifeq ($(wildcard /opt/ros/jazzy/setup.bash),)
  EXEC  := distrobox enter $(CONTAINER) -- /bin/bash -c
  WHERE := via distrobox ($(CONTAINER))
else
  EXEC  := /bin/bash -c
  WHERE := inside the container
endif

.PHONY: help check build clean rebuild run run-cpp run-headless sim-slam sim-nav2 shell deps teleop kill

help:
	@echo "EE3305 ROS 2 Jazzy helpers ($(WHERE))"
	@echo
	@echo "  make build          colcon build --symlink-install"
	@echo "  make rebuild        clean, then build"
	@echo "  make run            run.launch.py (Python nodes)"
	@echo "  make run-cpp        run.launch.py (C++ nodes)"
	@echo "  make run-headless   run.launch.py without the Gazebo GUI"
	@echo "  make sim-slam       sim_slam.launch.py"
	@echo "  make sim-nav2       sim_nav2.launch.py"
	@echo "  make shell          interactive bash with ROS + workspace sourced"
	@echo "  make check          show ROS / python3 / build state"
	@echo "  make deps           apt install the project's ROS packages"
	@echo "  make teleop         TurtleBot3 keyboard teleop"
	@echo "  make kill           kill lingering Gazebo processes"
	@echo
	@echo "  Variables: CPP=$(CPP) HEADLESS=$(HEADLESS) LIBGL=$(LIBGL) CONTAINER=$(CONTAINER)"

check:
	@$(EXEC) '$(ENV); \
	  echo "ROS_DISTRO=$${ROS_DISTRO:-<not set>}"; \
	  echo "python3=$$(command -v python3)"; \
	  python3 --version; \
	  echo "workspace=$(WS)"; \
	  if [ -f install/setup.bash ]; then echo "workspace built"; else echo "workspace not built"; fi'

build:
	@$(EXEC) '$(ENV); colcon build --symlink-install'

clean:
	@$(EXEC) '$(ENV); rm -rf build install log'

rebuild: clean build

run:
	@$(EXEC) '$(ENV); \
	  if [ ! -f install/setup.bash ]; then echo "Workspace not built. Run: make build"; exit 1; fi; \
	  source install/setup.bash; \
	  ros2 launch ee3305_bringup run.launch.py cpp:=$(CPP) headless:=$(HEADLESS) libgl:=$(LIBGL)'

run-cpp:
	@$(MAKE) --no-print-directory run CPP=True

run-headless:
	@$(MAKE) --no-print-directory run HEADLESS=True

sim-slam:
	@$(EXEC) '$(ENV); source install/setup.bash; ros2 launch ee3305_bringup sim_slam.launch.py headless:=$(HEADLESS) libgl:=$(LIBGL)'

sim-nav2:
	@$(EXEC) '$(ENV); source install/setup.bash; ros2 launch ee3305_bringup sim_nav2.launch.py headless:=$(HEADLESS) libgl:=$(LIBGL)'

shell:
	@$(EXEC) '$(ENV); [ -f install/setup.bash ] && source install/setup.bash; exec bash'

deps:
	@$(EXEC) 'sudo apt update && sudo apt install -y ros-jazzy-turtlebot3-gazebo ros-jazzy-turtlebot3-teleop ros-jazzy-turtlebot3-cartographer ros-jazzy-nav2-map-server ros-jazzy-turtlebot3-navigation2 ros-jazzy-nav2-route'

teleop:
	@$(EXEC) '$(ENV); export TURTLEBOT3_MODEL=burger; ros2 run turtlebot3_teleop teleop_keyboard'

kill:
	-@$(EXEC) 'pkill -9 -f "gz sim" 2>/dev/null; pkill -9 -f "gz-sim" 2>/dev/null; true'
