import os
import argparse
import cflib.crtp

parser = argparse.ArgumentParser(description='Test for argparse')
parser.add_argument('--path', '-p', help='firmware path', default='./cf2.bin', action='store_true')
parser.add_argument('--uri', '-u', help='drone uri', default=None, required=False)
parser.add_argument('--all', '-a', help='flash all nearby drones', action='store_true', default=False)
args = parser.parse_args()

if __name__ == '__main__':
    cflib.crtp.init_drivers()
    print('...........Scanning...........')
    uris = set()
    for i in range(3):
        for interface in cflib.crtp.scan_interfaces():
            uris.add(interface[0])
    uris = list(uris)
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
        while True:
            index = int(input('input drone index: '))
            if index in range(len(uris)):
                break
            else:
                print('wrong index, please re input!')
        print(f'trying to flash drone {uris[index]}')
        os.system(f'cfloader flash {args.path} stm32-fw -w {uris[index]}')
