#!/usr/bin/perl

use strict;
use warnings;
use Fcntl;
use Time::HiRes qw(time);

#
# ---------
# PID Controller class
# ---------
#

package PIDController;

sub new {
my ($class, %args) = @_;
    my $self = {
        kp       => $args{kp}       || 1.0,
        ki       => $args{ki}       || 0.0,
        kd       => $args{kd}       || 0.0,
        setpoint => $args{setpoint} || 40.0, # Target temperature
        out_min  => $args{out_min}  || 0,    # Minimum fan speed (e.g., 0 or 20% PWM)
        out_max  => $args{out_max}  || 255,  # Maximum fan speed (e.g., 100 or 255 PWM)
        
        # Internal state memory
        integral    => 0,
        last_input  => undef,
        last_time   => undef,
        last_output => 0,
    };
    return bless $self, $class;
}

sub update {
    my ($self, $current_input) = @_;
    my $now = Time::HiRes::time();

    # Initialize timing and state on the first run
    if (!defined $self->{last_time}) {
        $self->{last_time} = $now;
        $self->{last_input} = $current_input;
        return $self->{out_min}; 
    }

    my $dt = $now - $self->{last_time};
    
    # Prevent division by zero if called too rapidly
    return $self->{last_output} if $dt <= 0; 

    # Reverse acting error for cooling: 
    # Current Temp > Target Temp = Positive Error = Faster Fan
    my $error = $current_input - $self->{setpoint};

    # 1. Proportional Term
    my $p_term = $self->{kp} * $error;

    # 2. Integral Term (with Anti-Windup clamping)
    $self->{integral} += $self->{ki} * $error * $dt;
    if ($self->{integral} > $self->{out_max}) {
        $self->{integral} = $self->{out_max};
    } elsif ($self->{integral} < $self->{out_min}) {
        $self->{integral} = $self->{out_min};
    }

    # 3. Derivative Term (on measurement, not error, to prevent derivative kick)
    my $d_input = $current_input - $self->{last_input};
    my $d_term = $self->{kd} * ($d_input / $dt);

    # 4. Compute Total Output
    my $output = $p_term + $self->{integral} + $d_term;

    # Clamp final output to hardware limits
    if ($output > $self->{out_max}) {
        $output = $self->{out_max};
    } elsif ($output < $self->{out_min}) {
        $output = $self->{out_min};
    }

    # Save state for the next loop iteration
    $self->{last_input}  = $current_input;
    $self->{last_time}   = $now;
    $self->{last_output} = $output;

    return $output;
}

# Optional helper to change setpoint on the fly
sub set_target {
    my ($self, $new_target) = @_;
    $self->{setpoint} = $new_target;
}

1;

#
# --------
# ipmi related subroutines
# --------
#

# send bytes down an IPMI channel
sub ipmi_send {
	my (@bytes) = @_;

	# 1. Open the IPMI device
	my $devnode = "/root/dev/ipmi0";
	sysopen(my $fh, $devnode, Fcntl::O_RDWR) or die "Cannot open $devnode: $!\n";

	# 2. Define standard IPMI Constants
	my $IPMI_SYSTEM_INTERFACE_ADDR_TYPE = 0x0c;
	my $IPMI_BMC_CHANNEL                = 0x0f;

	# IOCTL Magic numbers calculated via _IOR() macro on 64-bit Linux:
	# _IOR(0x69, 13, 40) -> (2 << 30) | (40 << 16) | (0x69 << 8) | 13
	my $IPMICTL_SEND_COMMAND = 0x8028690D; 

	# 3. Setup the payload (Get Device ID: NetFn = 0x06, Cmd = 0x01)
	my $netfn = shift @bytes;
	my $cmd = shift @bytes;

	# Pack the raw data array into a binary string
	my $data_buffer = pack("C*", @bytes); 

	# 4. Construct C-Structures
	# struct ipmi_system_interface_addr { int addr_type; short channel; char lun; }
	# Size: 4 + 2 + 1 (+ 1 byte padding) = 8 bytes
	my $addr_struct = pack("i< s< c x", $IPMI_SYSTEM_INTERFACE_ADDR_TYPE, $IPMI_BMC_CHANNEL, 0);

	# struct ipmi_msg { unsigned char netfn; unsigned char cmd; unsigned short data_len; unsigned char *data; }
	# Size: 1 + 1 + 2 + 4 (padding) + 8 (pointer) = 16 bytes
	my $msg_struct = pack("C C S< x4 P", $netfn, $cmd, length($data_buffer), $data_buffer);

	# struct ipmi_req { unsigned char *addr; unsigned int addr_len; long msgid; struct ipmi_msg msg; }
	# Size: 8 (pointer) + 4 (len) + 4 (padding) + 8 (long) + 16 (msg_struct) = 40 bytes
	my $msgid = 1; # Sequence ID
	my $req_struct = pack("P I< x4 q< a16", $addr_struct, length($addr_struct), $msgid, $msg_struct);

	# 5. Send the command via IOCTL
	#print "Sending IPMI Command (NetFn: $netfn, Cmd: $cmd)...\n";
	my $ret = ioctl($fh, $IPMICTL_SEND_COMMAND, $req_struct);
	if (!defined $ret) {
	    die "IPMICTL_SEND_COMMAND failed: $!\n";
	}

	close($fh);
}

# initialize IPMI fans
sub ipmi_init {
	# set full fans (also manual mode)
	ipmi_send(0x30, 0x45, 0x01, 0x01);
	sleep(1);
	# set zone 0 to 50%%
	ipmi_send(0x30, 0x70, 0x66, 0x01, 0x00, 0x32);
	sleep(1);
	# set zone 1 to 50%
	ipmi_send(0x30, 0x70, 0x66, 0x01, 0x01, 0x32);
}

# update fans to a particular pwm value
sub update_drive_fans {
	my ($pwm) = @_;

	ipmi_send(0x30, 0x70, 0x66, 0x01, 0x01, $pwm);
}

sub update_cpu_fans {
	my ($pwm) = @_;

	ipmi_send(0x30, 0x70, 0x66, 0x01, 0x00, $pwm);
}


#
# drive temp subroutines
#

my %driveFiles;
my @drives;
# initialize / open all drive files and keep them open so we can easily read drive temps
sub drive_temps_init {
	my $drive;

	@drives = glob("/root/sys/class/block/sd?");

	for $drive (@drives) {
	        my @f = glob("$drive/device/hwmon/hwmon*/temp1_input");
		open(my $F, "< $f[0]") || die ("oops");
		$driveFiles{$drive} = $F;
	}
}

# gets temps from drives and return the hottest one
sub get_drive_temp {
	my $drive;
	my $F;
	my @line;
	my $maxTemp = 0;

	for $drive (@drives) {
		$F = $driveFiles{$drive};
		seek($F, 0, 0);
		@line = <$F>;
		chop @line;
		my $temp = $line[0];
		$temp = $temp / 1000;
		if ($temp > $maxTemp) { $maxTemp = $temp; }
	}

	return $maxTemp;
}

#
# cpu temp subroutines
#

my %cpuFiles;
# initialize / open all cpu thermal zones and keep them open so we can easily read zone temps
sub cpu_temps_init {
	my $zone;
	my @zones = glob("/root/sys/class/thermal/thermal_zone*/temp");
	my $C;

	for $zone (@zones) {
		open(my $C, "< $zone") || die("oops");
		$cpuFiles{$zone} = $C;
	}
}

# get cpu zone temps and return the hottest one
sub get_cpu_temp {
	my $zone;
	my $C;
	my @line;
	my $maxTemp = 0;

	for $zone (keys %cpuFiles) {
		$C = $cpuFiles{$zone};
		seek($C, 0, 0);
		@line = <$C>;
		chop @line;
		my $temp = $line[0];
		$temp = $temp / 1000;
		if ($temp > $maxTemp) { $maxTemp = $temp; }
	}

	return $maxTemp;
}

#
# -------
#   main
# -------
#

sub main {

	# create PID controllers
	my $cpuPid = PIDController->new (
		kp => 5.0,
		ki => 0.0,
		kd => 0.0,
		setpoint => 60.0,
		out_min => 30,
		out_max => 100
	);
	my $drivePid = PIDController->new (
		kp => 20.0,
		ki => 0.0,
		kd => 0.0,
		setpoint => 50.0,
		out_min => 30,
		out_max => 100
	);

	# initialize
	drive_temps_init();
	cpu_temps_init();
	ipmi_init();

	my $oldCpuTemp;
	my $oldCpuPwm;
	my $oldDriveTemp;
	my $oldDrivePwm;
	$oldCpuPwm = 0; $oldCpuTemp = 0;
	$oldDrivePwm = 0; $oldDriveTemp = 0;
	while(1) {
		# run PID for cpu temps
		my $cpuTemp = get_cpu_temp();
		my $cpuFanPwm = $cpuPid->update($cpuTemp);

		# run PID for drive temps
		my $driveTemp = get_drive_temp();
		my $driveFanPwm = $drivePid->update($driveTemp);

		# only update PWM values if they differ (to minimize work)
		$cpuFanPwm = int($cpuFanPwm);
		if ($cpuFanPwm != $oldCpuPwm) {
			update_cpu_fans($cpuFanPwm);
		}
		$driveFanPwm = int($driveFanPwm);
		if ($driveFanPwm != $oldDrivePwm) {
			update_drive_fans($driveFanPwm);
		}

		# only print out is something changes
		if(($oldCpuTemp != $cpuTemp) || ($oldCpuPwm != $cpuFanPwm) || 
			($oldDriveTemp != $driveTemp) || ($oldDrivePwm != $driveFanPwm)) {
			print localtime() . 
				"  Cpu Temp: " . $cpuTemp . "  pwm: " . $cpuFanPwm . 
				"  Drive Temp: " . $driveTemp . "  pwm: " . $driveFanPwm .
				"\n";
		}

		# update old values
		$oldCpuPwm = $cpuFanPwm;
		$oldCpuTemp = $cpuTemp;
		$oldDrivePwm = $driveFanPwm;
		$oldDriveTemp = $driveTemp;

		# wait for next iteration
		sleep(10);
	}
}


main();
