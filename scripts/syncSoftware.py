# Headers
import sys,getopt,os
from optparse import OptionParser
import subprocess
import pdb
from util.Helpers import *

# Defines
CRED = '\33[31m'
CYELLOW = '\33[33m'
CGREEN = '\33[32m'
CBLUE = '\33[34m'
CEND = '\033[0m'
def sync_remote(syncconfig_file,device_name):
    print(CGREEN + "Sync Started to: " + device_name + CEND)
    folders = ReadSyncConfig(syncconfig_file)
    for folder in folders:
        if((folder.Type == 'Source') or (folder.Type == 'Config')):            
            subprocess.call("rsync -iart " + folder.Directory + "/* robot@" + device_name + ":" + folder.Directory + " --exclude=*.html --exclude=*.js --exclude=*.png --exclude=*.map --exclude=*.md5 | grep '^<' | awk '{ print $2 }'",shell=True)
    print(CGREEN + "Sync Completed to: " + device_name  + CEND)
        

# Main
def main():
    parser = OptionParser("syncSoftware.py [options]")
    parser.add_option("-s","--syncmode",dest="syncmode",default="all",help="remote, [default: %default]")
    parser.add_option("-d","--devices",dest="devices",default="",help="DeviceName1,DeviceName2,... [default: %default]")
    parser.add_option("-c","--config_dir",dest="config_dir",default= os.environ['HOME'] + "/catkin_ws/src/config/",help="Location where Config dir should be found. default=" + os.environ['HOME'] + "/catkin_ws/src/config")
    (opts,args) = parser.parse_args()
    if (opts.syncmode =="remote"):
        devices = opts.devices.split(",")
        remotes = devices
        for device in remotes:
            print(CGREEN + "Syncing Remote: " + device + CEND)
            sync_remote(opts.config_dir + "SyncConfig.xml",device)   
    else:
        print(CRED + 'Sync Mode: ' + opts.syncmode + ' Not Supported.' + CEND)

if __name__ == "__main__":
    main()
