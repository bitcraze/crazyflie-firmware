import logging
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)
uri = 'usb://0'


def get_persistent_state(cf, complete_param_name):
    wait_for_callback_event = Event()

    def state_callback(complete_name, state):
        print(f'{complete_name}: {state}')
        wait_for_callback_event.set()

    cf.param.persistent_get_state(complete_param_name, state_callback)
    wait_for_callback_event.wait()


def persist_parameter(cf, complete_param_name):
    wait_for_callback_event = Event()

    def is_stored_callback(complete_name, success):
        if success:
            print(f'Persisted {complete_name}!')
        else:
            print(f'Failed to persist {complete_name}!')
        wait_for_callback_event.set()

    cf.param.persistent_store(complete_param_name, callback=is_stored_callback)
    wait_for_callback_event.wait()


def clear_persistent_parameter(cf, complete_param_name):
    wait_for_callback_event = Event()

    def is_stored_cleared(complete_name, success):
        if success:
            print(f'Cleared {complete_name}!')
        else:
            print(f'Failed to clear {complete_name}!')
        wait_for_callback_event.set()

    cf.param.persistent_clear(complete_param_name, callback=is_stored_cleared)
    wait_for_callback_event.wait()


if __name__ == '__main__':
    cflib.crtp.init_drivers()
    print("...........Scanning...........")
    uris = set()
    for i in range(2):
        for interface in cflib.crtp.scan_interfaces():
            uris.add(interface[0])
    print("Nearby devices: ", list(uris))

    uri = input("Input URI: ")
    cf = Crazyflie()
    with SyncCrazyflie(uri, cf=cf) as scf:
        param_name = 'ADHOC.MY_UWB_ADDRESS'
        param_value = input("Input MY_UWB_ADDRESS: ")

        print(">>>>>>>>>>>>>>>>>>>>>>")
        print(f'Set {param_name} to {param_value}')
        scf.cf.param.set_value(param_name, param_value)

        print('Store the new value in persistent memory')
        persist_parameter(scf.cf, param_name)

        print("<<<<<<<<<<<<<<<<<<<<<<")
        get_persistent_state(scf.cf, param_name)

        # print('Clear the persisted parameter')
        # clear_persistent_parameter(scf.cf, param_name)
        #
        # print('The new state is:')
        # get_persistent_state(scf.cf, param_name)
