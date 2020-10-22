#!/usr/bin/env python3
import tkinter
from tkinter import *
import tkinter.ttk as ttk
import zmq
import threading
import time


class Crazyflie(ttk.Frame):
    def __init__(self, parent, ident):
        ttk.Frame.__init__(self, parent)

        self['padding'] = 10

        self._name = Label(self, text="Crazyflie #{}".format(ident+1))
        self._name.grid(row=0, column=0)

        self._status = Label(self, text="IDLE", fg='grey', font=("ubuntu", 33), width=10)
        self._status.grid(row=1, column=0)

        self._battery_label = Label(self, text="Battery:")
        self._battery_label.grid(row=2, column=0)

        self._battery_frame = ttk.Frame(self)
        self._battery_frame.grid(row=3, column=0, sticky="ew")
        self._battery_frame.columnconfigure(1, weight=2)

        self._battery_voltage = ttk.Label(self._battery_frame, text="3.0V", padding=(0,0,10,0))
        self._battery_voltage.grid(row=0, column=0)
        
        self._battery_bar = ttk.Progressbar(self._battery_frame, orient=HORIZONTAL)
        self._battery_bar['value'] = 50
        self._battery_bar.grid(row=0, column=1, sticky="ew")

        self._time_frame = ttk.Frame(self)
        self._time_frame.grid(row=4, column=0)

        up_time = Label(self._time_frame, text="Up time: ", fg="grey", font=("ubuntu", 20))
        up_time.grid(row=0, column=0)

        self._up_time_label = Label(self._time_frame, text="0:00:00", font=("ubuntu", 20))
        self._up_time_label.grid(row=0, column=1)

        flight_time = Label(self._time_frame, text="Flight time: ", fg="grey", font=("ubuntu", 20))
        flight_time.grid(row=1, column=0)

        self._flight_time_label = Label(self._time_frame, text="0:00:00", font=("ubuntu", 20))
        self._flight_time_label.grid(row=1, column=1)


    
    def set_state(self, state):
        if state == "idle":
            self._status.config(text="IDLE", fg="grey")
        elif state == "disconnected":
            self._status.config(text="Offline", fg="grey")
        elif state == "crashed":
            self._status.config(text="Crashed", fg="purple")
        elif state == "charging":
            self._status.config(text="Charging", fg="red")
        elif state == "ready":
            self._status.config(text="Ready", fg="orange")
        elif state == "flying":
            self._status.config(text="Flying", fg="green")
        elif state == "hovering":
            self._status.config(text="Take off", fg="green")
        elif state == "landing":
            self._status.config(text="Landing", fg="green")
        else:
            self._status.config(text="ERROR", fg="purple")
            print("Error, state", state, "not handled")

    def set_battery(self, voltage):
        self._battery_voltage['text'] = "{:.2f}V".format(voltage)

        percent = (voltage - 3.0)*100.0/1.1

        self._battery_bar['value'] = percent
    
    def set_uptime(self, ms):
        seconds = int(ms/1000) % 60
        minutes = int((ms/1000)/60) % 60
        hours = int((ms/1000)/3600)
        self._up_time_label['text'] = "{}:{:02}:{:02}".format(hours, minutes, seconds)
        if ms == 0:
            self._up_time_label['fg'] = "grey"
        else:
            self._up_time_label['fg'] = "black"
    
    def set_flighttime(self, ms):
        seconds = int(ms/1000) % 60
        minutes = int((ms/1000)/60) % 60
        hours = int((ms/1000)/3600)
        self._flight_time_label['text'] = "{}:{:02}:{:02}".format(hours, minutes, seconds)
        if ms == 0:
            self._flight_time_label['fg'] = "grey"
        else:
            self._flight_time_label['fg'] = "black"


root = tkinter.Tk()
root.title("ICRA2019 Bitcraze Control Tower")

content = ttk.Frame(root)
content.grid(column=0, row=0)
root.columnconfigure(0, weight=1)
root.rowconfigure(0, weight=1)

cfs = []

for i in range(9):
    r = int(i/3)
    c = int(i%3)
    cf = Crazyflie(content, i)
    cf.grid(column=c, row=r)
    cfs.append(cf)

    cf.set_state("error")
    cf.set_battery(3.0+(i/10.0))

context = zmq.Context()

socket = context.socket(zmq.PULL)
# socket.connect("tcp://bitcrazeDemo:5555")
socket.connect("tcp://127.0.0.1:5555")
socket.setsockopt(zmq.RCVTIMEO, 1000)


def receive_thread():
    last_updated = [0]*len(cfs)
    while True:
        try:
            report = socket.recv_json()
            print(report)

            cfs[report['id']].set_battery(report['battery'])
            cfs[report['id']].set_state(report['state'])
            cfs[report['id']].set_uptime(report['uptime'])
            cfs[report['id']].set_flighttime(report['flighttime'])
            last_updated[report['id']] = time.time()
        except zmq.error.Again:
            pass

        for i in range(len(cfs)):
            if last_updated[i] < (time.time()-1):
                cfs[i].set_state("idle")
                cfs[i].set_battery(0)
                cfs[i].set_uptime(0)
                cfs[i].set_flighttime(0)


cfs[0].set_uptime(300000)
cfs[0].set_flighttime(140000)

threading.Thread(target=receive_thread, daemon=True).start()

root.mainloop()
