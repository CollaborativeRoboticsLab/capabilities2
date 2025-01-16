from multiprocessing import Process
import launch
from ros2launch.api.api import parse_launch_arguments
from ros2launch.api import get_share_file_path_from_package
from ament_index_python.packages import PackageNotFoundError
from ros2launch.api import MultipleLaunchFilesError

class LaunchProcess(Process):

    def __init__(
            self, 
            package_name, 
            launch_file_name, 
            launch_file_arguments=[],
            option_extensions={},
            noninteractive=True, 
            debug=False,
            args=None
            ):
        """
        LaunchProcess objects represent ros2 launch files that need to be started is run in a separate process

        :param: package_name name of the package where launch file resides
        :param: launch_file_name name of the launch file
        :param: launch_file_arguments  launch file arguments as a list
        :param: option_extensions  option extensions as a dictionary
        :param: noninteractive flag for non-interactiveness
        :param: debug flag for enabling debug info
        :param: args additional arguments
        """
        super().__init__()
        self.package_name = package_name
        self.launch_file_name = launch_file_name
        self.launch_file_arguments = launch_file_arguments
        self.noninteractive = noninteractive
        self.debug = debug
        self.option_extensions=option_extensions
        self.args = args
        self.path = None

        try:
            self.path = get_share_file_path_from_package(
                package_name=self.package_name, 
                file_name=self.launch_file_name)
            
        except PackageNotFoundError as exc:
            raise RuntimeError(f"Package '{self.package_name}' not found: {exc}")
        except (FileNotFoundError, MultipleLaunchFilesError) as exc:
            raise RuntimeError(str(exc))

    def run(self):
        print(f"Launch file {self.launch_file_name} from package {self.package_name} started with pid [{self.pid}")

        launch_service = launch.LaunchService(
            argv=self.launch_file_arguments,
            noninteractive=self.noninteractive,
            debug=self.debug)
        
        parsed_launch_arguments = parse_launch_arguments(self.launch_file_arguments)

        launch_description = launch.LaunchDescription([
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.AnyLaunchDescriptionSource(
                    self.path
                ),
                launch_arguments=parsed_launch_arguments,
            ),
        ])

        for name in sorted(self.option_extensions.keys()):
            result = self.option_extensions[name].prelaunch(launch_description, self.args)
            launch_description = result[0]

        launch_service.include_launch_description(launch_description)

        launch_service.run()

    def stop(self):
        print(f"Stopping Launch file {self.launch_file_name} from package {self.package_name}")
        self.kill()

class LaunchManager:
    def __init__(self):
        self.processes = {}

    def start(self, package_name, launch_file_name):
        name = package_name + "/" + launch_file_name
        
        if name not in self.processes:
            try:
                self.processes[name] = LaunchProcess(package_name=package_name, launch_file_name=launch_file_name)
                self.processes[name].start()
            except:
                return {"status": "error occured"}
            else:
                return {"status": "successfuly started"}
        else:
            return {"status": "already started. ignoring"}
        
    def status(self, package_name, launch_file_name):
        name = package_name + "/" + launch_file_name

        if name in self.processes:
            if self.processes[name].is_alive():
                return {"status": "running"}
            else:
                return {"status": "failed"}
        else:
            return {"status": "stopped"}
        

    def stop(self, package_name, launch_file_name):
        name = package_name + "/" + launch_file_name

        if name in self.processes:
            try:
                self.processes[name].stop()
                self.processes[name].join()
            except:
                return {"status": "error occured"}
            else:
                return {"status": "successfuly stopped"}
        else:
            return {"status": "already stopped. ignoring"}


if __name__ == "__main__":
    launch_process = LaunchProcess(
        package_name="nav_stack",
        launch_file_name="system.launch.py"
    )
    
    launch_process.start()  # Start the process
    
    import time
    time.sleep(10)  # Let the process run for 10 seconds
    
    launch_process.stop()  # Signal the process to stop
    launch_process.join()  # Wait for the process to terminate
    print("Launch process has exited.")