#!/usr/bin/env python

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
import subprocess

# Parser

parser = argparse.ArgumentParser()
subparser = parser.add_subparsers(dest='command', help='Command to run : ', required=True)
subparser.add_parser('enable', help='Enable daemon')
subparser.add_parser('disable', help='Disable daemon')
subparser.add_parser('start', help='Manually start daemon')
subparser.add_parser('kill', help='Manually kill daemon')
subparser.add_parser('restart', help='Manually restart daemon')
subparser.add_parser('run', help='Run pipeline')
subparser.add_parser('stop', help='Stop pipeline')
subparser.add_parser('status', help='Get pipeline status')

set_enemy_parser = subparser.add_parser('set-enemy', help='Set enemy color')
set_enemy_parser.add_argument('color', choices=['blue', 'red'], help='Enemy color')

shoot_parser = subparser.add_parser('shoot')
shoot_parser.add_argument('val', choices=['on', 'off'])

# Commands

sysctl = '/bin/systemctl --user'

def is_daemon_running():
    proc = subprocess.run(f'{sysctl} status polystar', shell=True, capture_output=True, text=True)
    return proc.stdout.contains('active (running)')

def call_systemd(command: str):
    proc = subprocess.run(f'{sysctl} {command} polystar --no-pager', shell=True, capture_output=True)
    if proc.returncode == 0:
        print('Success')
    else:
        print(f'Error :\n{proc.stderr.decode()}')
    return proc.stdout

def enable():
    print('Enabling daemon')
    return call_systemd('enable')

def disable():
    print('Disabling daemon')
    return call_systemd('disable')

def start():
    print('Starting daemon')
    return call_systemd('start')

def kill():
    print('Killing daemon')
    return call_systemd('stop')

def restart():
    kill()
    start()

def status():
    print(call_systemd('status').decode())

def run():
    print('Running the pipeline')

    print('Success')

def stop():
    """
    Stops the pipeline (running, but idle)
    """
    print('Stopping the pipeline')

    print('Unimplemented')

def set_enemy(color):
    print(f'Setting color to {color}')

    color_int = 0 if color == 'red' else 1
    try:
        proc = subprocess.run(f'rosrun dynamic_reconfigure dynparam set /decision enemy_color {color_int}', shell=True, capture_output=True, timeout=10)
    except TimeoutError as te:
        print(f'Error : Unresponsive after {te.timeout} seconds')
    
    if proc.returncode == 0:
        print('Success')
    else:
        print(f'Error :\n{proc.stderr.decode()}')

def shoot(val):
    print(f'Shooting to {val}')

    print('Unimplemented')

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
    elif args.command == 'status':
        status()
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
