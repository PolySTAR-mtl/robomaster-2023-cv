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
import yaml

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
subparser.add_parser('checklist', help='Pre-match checkup')
subparser.add_parser('record', help='Start recording')

set_enemy_parser = subparser.add_parser('set-enemy', help='Set enemy color')
set_enemy_parser.add_argument('color', choices=['blue', 'red'], help='Enemy color')

shoot_parser = subparser.add_parser('shoot')
shoot_parser.add_argument('val', choices=['on', 'off'])

# Commands

sysctl = '/bin/systemctl --user'

def is_daemon_running():
    proc = subprocess.run(f'{sysctl} status polystar', shell=True, capture_output=True, text=True)
    return 'active (running)' in proc.stdout

def get_params(node: str):
    try:
        proc = subprocess.run(f'rosrun dynamic_reconfigure dynparam get {node}', shell=True, capture_output=True, timeout=10)
    except TimeoutError as te:
        print(f'Error : Unresponsive after {te.timeout} seconds')

    return yaml.load(proc.stdout.decode())


def call_systemd(command: str, service='polystar'):
    proc = subprocess.run(f'{sysctl} {command} {service} --no-pager', shell=True, capture_output=True)
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
    print('Unimplemented')

def stop():
    """
    Stops the pipeline (running, but idle)
    """
    print('Stopping the pipeline')

    print('Unimplemented')

def dynreconf(node: str, param: str, value: any):
    try:
        proc = subprocess.run(f'rosrun dynamic_reconfigure dynparam set {node} {param} {value}', shell=True, capture_output=True, timeout=10)
    except TimeoutError as te:
        print(f'Error : Unresponsive after {te.timeout} seconds')

    if proc.returncode == 0:
        print('Success')
    else:
        print(f'Error :\n{proc.stderr.decode()}')

    return proc

def set_enemy(color):
    print(f'Setting color to {color}')

    color_int = 0 if color == 'red' else 1
    dynreconf('decision', 'enemy_color', color_int)

def shoot(val):
    print(f'Shooting to {val}')

    print('Unimplemented')

def checklist():
    running = is_daemon_running()
    enemy_color = 'red' if get_params('decision')['enemy_color'] == 0 else 'blue'

    print(f'Is daemon running : {running}')
    print(f'Enemy color       : {enemy_color}')

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
    elif args.command == 'checklist':
        checklist()
    else:
        print(f'Unsupported command "{args.command}"', file=sys.stderr)
        sys.exit(-1)

if __name__ == "__main__":
    main()
