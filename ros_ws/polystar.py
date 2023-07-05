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
import sys

# Parser

parser = argparse.ArgumentParser()
subparser = parser.add_subparsers(dest='command', help='Command to run')
subparser.add_parser('enable', help='Enable daemon')
subparser.add_parser('disable', help='Disable daemon')
subparser.add_parser('start', help='Manually start daemon')
subparser.add_parser('kill', help='Manually kill daemon')
subparser.add_parser('restart', help='Manually restart daemon')
subparser.add_parser('run', help='Run pipeline')
subparser.add_parser('stop', help='Stop pipeline')

set_enemy_parser = subparser.add_parser('set-enemy', help='Set enemy color')
set_enemy_parser.add_argument('color', choices=['blue', 'red'], help='Enemy color')

shoot_parser = subparser.add_parser('shoot')
shoot_parser.add_argument('val', choices=['on', 'off'])

# Commands

def is_daemon_running():
    pass

def enable():
    print('Enabling daemon')
    print('Success')

def disable():
    print('Disabling daemon')
    print('Success')

def start():
    print('Starting daemon')
    print('Success')

def kill():
    print('Killing daemon')
    print('Success')

def restart():
    kill()
    start()

def run():
    print('Running the pipeline')

    print('Success')

def stop():
    """
    Stops the pipeline (running, but idle)
    """
    print('Stopping the pipeline')

    print('Success')

def set_enemy(color):
    pass

def shoot(val):
    pass

def main():
    args = parser.parse_args()
    if args.command == 'enable':
        enable()
    elif args.command == 'disable':
        disable()
    elif args.command == 'start':
        start()
    elif args.command == 'kill':
        kill()
    elif args.command == 'restart':
        restart()
    elif args.command == 'run':
        run()
    elif args.command == 'stop':
        stop()
    elif args.command == 'set-enemy':
        set_enemy(args.color)
    elif args.command == 'shoot':
        shoot(args.val)
    else:
        print(f'Unsupported command "{args.command}"', file=sys.stderr)
        sys.exit(-1)

if __name__ == "__main__":
    main()
