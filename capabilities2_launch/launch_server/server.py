from launch_server.launch_process import LaunchManager
from multiprocessing.connection import Listener

class LaunchServer:
    def __init__(self):
        self.address = ('localhost', 6000)
        self.listener = Listener(self.address, authkey=b'capabilities2')
        self.manager  = LaunchManager()

    def initialize(self):
        self.connection = self.listener.accept()
        print("[Launch Server] connection Accepted from {self.listener.last_accepted}")

    def run(self):
        while True:
            # message would be a dictionary with the format of {"command": , "package": , "launch_file": }
            message_incoming = self.connection.recv()

            command = message_incoming["command"]

            if ((command=="start") or (command=="stop") or (command=="status")):
                package_name = message_incoming["package"]
                launch_file_name = message_incoming["launch_file"]
            else:
                package_name = ""
                launch_file_name = ""

            if (command=="start"):
                print("[Launch Server] start request for {package_name}/{launch_file_name}")
                status = self.manager.start(package_name=package_name, launch_file_name=launch_file_name)
                result = status["status"]

                print("[Launch Server] start response of {package_name}/{launch_file_name}: {result}")

            elif (command=="stop"):
                print("[Launch Server] stop request for {package_name}/{launch_file_name}")
                status = self.manager.stop(package_name=package_name, launch_file_name=launch_file_name)
                result = status["status"]

                print("[Launch Server] stop response of {package_name}/{launch_file_name}: {result}")

            elif (command=="status"):
                print("[Launch Server] status request for {package_name}/{launch_file_name}")
                status = self.manager.status(package_name=package_name, launch_file_name=launch_file_name)
                result = status["status"]

                print("[Launch Server] status response of {package_name}/{launch_file_name}: {result}")

            elif (command=="exit"):
                break

            else:
                print("[Launch Server] does not support command of type : {command}")
                break

            self.connection.send(status)

        self.listener.close


if __name__ == "__main__":

    server = LaunchServer()
    server.initialize()
    server.run()