"""
This module facilitates communication with the AR.Drone.
"""

import select
import socket
import threading
import multiprocessing

import drone_lib


class DroneNetworkProcess(multiprocessing.Process):
    """Drone Network Process.

    This process collects data from the video and navigation data port, converts the
    data, and sends it to the IPCThread.
    """

    def __init__(self, nav_pipe, command_pipe):
        multiprocessing.Process.__init__(self)
        self.nav_pipe = nav_pipe
        self.command_pipe = command_pipe

    def run(self):
        print('Initializing drone navigation data and command channels...')
        nav_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        nav_socket.setblocking(0)
        nav_socket.bind(('', drone_lib.DRONE_NAVDATA_PORT))
        nav_socket.sendto("\x01\x00\x00\x00".encode(), ('192.168.1.1', drone_lib.DRONE_NAVDATA_PORT))

        stopping = False
        while not stopping:
            inputready, _, _ = select.select([nav_socket, self.command_pipe], [], [])
            for i in inputready:
                if i == nav_socket:
                    while True:
                        try:
                            data = nav_socket.recv(65535)
                        except IOError:
                            # We consumed every packet from the socket and
                            # continue with the last one
                            break
                    navdata = drone_lib.decode_navdata(data)
                    self.nav_pipe.send(navdata)
                elif i == self.command_pipe:
                    _ = self.command_pipe.recv()
                    stopping = True
                    break
        nav_socket.close()


class IPCThread(threading.Thread):
    """Inter Process Communication Thread.

    This thread collects the data from the DroneNetworkProcess and forwards
    it to the Drone.
    """

    def __init__(self, drone):
        threading.Thread.__init__(self)
        self.drone = drone
        self.stopping = False

    def run(self):
        while not self.stopping:
            inputready, _, _ = select.select([self.drone.nav_pipe], [], [], 1)
            for i in inputready:
                if i == self.drone.nav_pipe:
                    while self.drone.nav_pipe.poll():
                        navdata = self.drone.nav_pipe.recv()
                    self.drone.navdata = navdata

    def stop(self):
        """Stop the IPCThread activity."""
        self.stopping = True
