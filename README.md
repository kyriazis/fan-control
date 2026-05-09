# fan-control
Simple cpu/disk fan control using IPMI and probing /sys interface.  A simple PID controller is used for controlling the fans.

The main idea behind this is to not have dependencies on external binaries, so as to minimize overhead spent in forking
other processes.  There are 2 external file system dependencies:

- /dev/ipmi0 IPMI interface to set fan pwm
- /sys interface to read disk temperatures without having to call smartctl.

/dev/ipmi0 and /sys are accessed directly using ioctl() for IPMI and read() for /sys files.

Tested and developed on a Supermicro X11-series motherboard, using proxmox as a host environment.  The script is running
on an LXC container with hosts' /dev and /sys punched through the container in /root/dev and /root/sys respectively.

The controller loop is running every 10 seconds.  At that time interval forking additional processes for reading drive temps
and setting fans is too much overhead.  In addition, the ipmitool command does not have a direct command-line option to specify
a direct path to the IPMI device, just an IPMI "number" (ie. device 0, device 1, etc.).  All handles to /dev and /sys
files are kept open so there is no additional overhead for opening and closing files at a 10 second interval.

As an example, in one systemd run, I observed the following output during a restart of the service, which is not bad:
Consumed 1.966s CPU time over 11h 17min 32.779s wall clock time, 4.1M memory peak

The IPMI raw commands are specific to X11-series motherboards.  Modify as needed for your own use.
The case I am using have separate compartments for drives, and numbered (CPU) fans are used in the motherboard compartment
while the lettered fans are used for the drive compartment.

Cheers..
