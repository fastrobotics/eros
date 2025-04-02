# Headers
import sys,getopt,os
from optparse import OptionParser
import subprocess
import pdb
import shutil
from util.Helpers import *
import os.path
from os import path
import socket
from os.path import expanduser

folder_list = [ '~/var/log/output/',
                '~/storage/stage/',
                '~/storage/DATALOGS/',
                '~/storage/SNAPSHOT/DEVICESNAPSHOT/',
                '~/storage/SNAPSHOT/SYSTEMSNAPSHOT/',
                '~/tmp/',
                '~/test/',
                '~/test/storage/stage/',
                '~/test/storage/DATALOGS/',
                '~/test/storage/SNAPSHOT/DEVICESNAPSHOT/',
                '~/test/storage/SNAPSHOT/SYSTEMSNAPSHOT/']
full_apt_dependency_list = [ 'libncurses5-dev',
                    'libncursesw5-dev',
                    'libtinyxml-dev',
                    'zip',
                    'caffeine',
                    'ntp',
		    'net-tools',
            'openssh-server',
            'python3-pip',
            'googletest',
            'graphviz',
            'clang-format',
            'doxygen',
            'ros-noetic-joy']
target_apt_dependency_list = [ 'libncurses5-dev',
                    'libncursesw5-dev',
                    'libtinyxml-dev',
                    'zip',
                    'caffeine',
                    'ntp',
		    'net-tools',
            'openssh-server',
            'python3-pip']
full_pip_dependency_list = ['pre-commit','post-merge', 'gcovr']
target_pip_dependency_list = []
config_file_list = [    'DeviceList.json',
                        'SnapshotConfig.xml',
                        'SyncConfig.xml']

def run_setup(mode):
    apt_dep_list = []
    pip_dep_list = []
    if mode == "full":
        apt_dep_list = full_apt_dependency_list
        pip_dep_list = full_pip_dependency_list
    elif mode == "target":
        target_dep_list = full_apt_dependency_list
        target_dep_list = full_pip_dependency_list
    cwd = os.getcwd()
    if(os.path.basename(cwd) != "eros"):
        print(CRED + "Must be run from eros directory. Exiting." + CEND)
        return
    print(CGREEN + "Running Simple Setup..." + CEND)
    print(CGREEN + "Creating Directories..." + CEND)
    for i in range(0,len(folder_list)):
        os.system("mkdir -p " + folder_list[i])
    print(CGREEN + "Installing Dependencies via apt..." + CEND)
    for i in range(0,len(apt_dep_list)):
        os.system("sudo apt --yes --force-yes install " + apt_dep_list[i])
    print(CGREEN + "Installing Dependencies via pip..." + CEND)
    for i in range(0,len(pip_dep_list)):
        os.system("pip3 install " + pip_dep_list[i])
    if(mode == "full"):
        os.system("python3 -m pip install --user cookiecutter")
        os.system("sudo snap install cookiecutter --edge")
        #os.system('sudo bash -c "curl -L https://raw.githubusercontent.com/riboseinc/plantuml-install/master/ubuntu.sh | bash"')
    print(CGREEN + "Installing other dependencies from source..." + CEND)
    home = expanduser("~")
    hostname = socket.gethostname()
    # Install json
    if(path.exists("/usr/local/include/nlohmann/json.hpp") == False):
        os.system("mkdir -p ~/other_packages/")
        os.chdir(home + "/other_packages/")
        os.system("wget https://github.com/plantuml/plantuml/releases/download/v1.2025.1/plantuml-1.2025.1.jar")
        if(path.exists(home+"/other_packages/json") == True):
            shutil.rmtree(home + "/other_packages/json/")
        os.system("git clone https://github.com/nlohmann/json.git")
        os.system("cd json")
        os.system("mkdir ~/other_packages/json/build/")
        os.chdir(home + "/other_packages/json/build")
        os.system("cmake ..")
        os.system("make")
        os.system("sudo make install")
        
        os.chdir(cwd)
    print(CGREEN + "Simple Setup Complete!!!" + CEND)


# Main
def main():
    parser = OptionParser("simple_setup.py [options]")
    parser.add_option("-m","--mode",dest="mode",default="full",help="development,target [default: %default]")
    (opts,args) = parser.parse_args()
    if(opts.mode == "full"):
        print(CGREEN + "Installing full dependencies." + CEND)
        run_setup("full")
    elif(opts.mode == "target"):
        print(CGREEN + "Installing dependencies for only a target." + CEND)
        run_setup("target")
    

if __name__ == "__main__":
    main()
