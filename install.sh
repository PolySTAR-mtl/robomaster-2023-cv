#!/bin/bash

# Installs the necessary scripts/config files. Binary will remain in the repo folder

# Control script
mkdir -p ~/.local/bin/
ln -s $(pwd)/polystar.py ~/.local/bin/polystar

# Systemd services
mkdir -p ~/.config/systemd/user/
ln -s $(pwd)/polystar.service ~/.config/systemd/user/polystar.service
ln -s $(pwd)/polystar_recorder.service ~/.config/systemd/user/polystar_recorder.service

