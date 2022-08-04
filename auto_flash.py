import os
import argparse
import cflib.crtp

parser = argparse.ArgumentParser(description='Crazyflie Flashing Utility')
parser.add_argument('-p', '--path', help='firmware path', default='./cf2.bin', action='store')
parser.add_argument('-u', '--uri', help='drone uri', default=None, required=False)
parser.add_argument('-a', '--all', help='flash all nearby drones', action='store_true', default=False)
args = parser.parse_args()

if __name__ == '__main__':
    cflib.crtp.init_drivers()
    print('...........Scanning...........')
    uris = set()
    for i in range(3):
        for interface in cflib.crtp.scan_interfaces():
            uris.add(interface[0])
    uris = list(uris)
    if len(uris) <= 0:
        print('No drones nearby, exit.')
    else:
        print('Nearby drones (index, uri): ', end="")
        for index, uri in enumerate(uris):
            print(f'({index}, {uri}) ', end="")
        print()
        if args.all:
            print('flashing all drones')
            for uri in uris:
                print(f'flashing drone {uri}')
                os.system(f'cfloader flash {args.path} stm32-fw -w {uri}')
        elif args.uri is not None:
            print(f'trying to flash drone {args.uri}')
            os.system(f'cfloader flash {args.path} stm32-fw -w {args.uri}')
        else:
            index = 0
            is_continue = 'y'
            while 'y' in is_continue:
                try:
                    index = int(input('input drone index: '))
                    if index in range(len(uris)):
                        print(f'trying to flash drone {uris[index]}')
                        os.system(f'cfloader flash {args.path} stm32-fw -w {uris[index]}')
                        is_continue = input('continue? (y or n): ')
                    else:
                        print('wrong index, please re input!')
                except (TypeError, ValueError):
                    print('wrong index, please re input!')
