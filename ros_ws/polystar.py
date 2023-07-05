#!/bin/env python

"""
@file polystar.py
@brief General control script

@details The polystar.py script supports the following commands : 
- polystar enable / disable : control systemd daemon (launch on startup)
- polystar start / kill / restart : manually kill daemon
- polystar run / stop : control pipeline
- polystar set-enemy (blue|red) : sets enemy color
- polystar shoot (on|off) : allows sending shooting orders
"""

import argparse

def is_daemon_running():
    pass

def enable():
    pass

def disable():
    pass

def start():
    pass

def kill():
    pass

def restart():
    kill()
    start()

def run():
    pass

def stop():
    """
    Stops the pipeline (running, but idle)
    """
    pass

def main():
    pass

if __name__ == "__main__":
    main()
