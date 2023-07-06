#!/bin/bash

# Installs the necessary scripts/config files. Binary will remain in the repo folder

ln -s $(pwd)/polystar.py ~/.local/bin/polystar
cp polystar.service /etc/systemd/user/
