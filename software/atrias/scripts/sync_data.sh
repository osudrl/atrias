#!/bin/bash
# Author: Mikhail S Jones
# Date: 2013-05-09
# Description: Organizes bagfiles into directories based on date modified. 
# Syncs data to hurst attic network drive and moves original copies into an archive. 


# Exit if any command fails
set -e

# Declare some colors
HEADER="\033[1m\033[95m"
OKBLUE="\033[1m\033[94m"
OKGREEN="\033[1m\033[92m"
WARNING="\033[1m\033[93m"
FAIL="\033[1m\033[91m"
ENDC="\033[0m"



# Startup message
printf "\n${OKBLUE}[Starting!]${ENDC} sync_data.sh\n\n"

# Define computers
ROBOT_HOSTNAME="i1000a-3"
HURST="//attic.engr.oregonstate.edu/hurst"
MOUNT_DIR="/mnt/attic"
SOURCE_DIR="$(rospack find atrias)/bagfiles"
SORT_DIR="$(rospack find atrias)/bagfiles/sorted"
SYNC_DIR="/mnt/attic/experimental-data/atrias-2-1"

# Make sure we have cifs utilities to mount drive
printf "${OKGREEN}[Checking for sshfs...]${ENDC}\n"
if [ ! $(hash sshfs 2>/dev/null) ]; then
	  printf "${OKGREEN}[sshfs found]${ENDC}\n"
else
	sudo apt-get install sshfs -y
fi



# Check that script is being executed from correct computer
if [ "$HOSTNAME" != "$ROBOT_HOSTNAME" ]; then
	printf "${FAIL}[Error!]${ENDC} Must execute sync_data.sh from the robot computer!\n"
	exit 0
fi

# Check to see if there are files to sync
if [ -z "$(ls $SOURCE_DIR)"]; then
	printf "${FAIL}[Nothing to sync]${ENDC}\n"
	exit 0
fi


# Prompt user for username and password and mount attic drive
printf "${OKGREEN}[Mounting ${HURST} to ${MOUNT_DIR}...]${ENDC}\n"

if [ -n "$(mount | grep "on ${MOUNT_DIR} type")" ]; then
	# Already mounted
	printf "${FAIL}[Already mounted! Unmount and re-run]${ENDC}\n"
	exit 0
	
else
	# If the mount target folder does not already exist, create it
	if [ ! -d "$MOUNT_DIR" ]; then
		sudo mkdir -p "$MOUNT_DIR"
	fi
	sudo chown drl:drl "$MOUNT_DIR"
	sudo chmod a+rw "$MOUNT_DIR"
	
	# Prompt with user info to mount
	echo -n "Enter your Engineering username: "
	read -e ENGR_USERNAME

	# Mount network drive
	sshfs "${ENGR_USERNAME}@access.engr.orst.edu:/nfs/attic/hurst" "${MOUNT_DIR}"

	# Check mount worked and we can access the target sync folder
	if [ ! -d "$SYNC_DIR" ]; then
		printf "${FAIL}[Error!]${ENDC} Cannot find the mounted drive sync directory!\n"
		exit 0
	fi
	
fi
printf "${OKGREEN}[Done!]${ENDC}\n\n"



# Sort files by date
printf "${OKGREEN}[Sorting files by date...]${ENDC}\n"

# Reset sorted file counter
sorted_count=0

# Set the internal field spearator
IFS=$'\n'

# Find total number of files to transfer and create file list
file_list=$(find "$SOURCE_DIR" -maxdepth 1 -name "*.bag" -o -name "*.mat" -type f)
total_count=$(find "$SOURCE_DIR" -maxdepth 1 -name "*.bag" -o -name "*.mat" -type f | wc -l)

# Loop through each item in file list
for file in $file_list; do

	# Increment counter
	$((sorted_count++))

	# Parse out file name and path
	file_name=$(basename "$file")
	file_path=$(dirname "$file")
 
	# Get the files modified date
	file_date=$(date -d @$(stat --format %Y "$file") +%F)
 
	# If the folder does not already exist, create it
	if [ ! -d "${SORT_DIR}/${file_date}" ]; then
		mkdir -p "${SORT_DIR}/${file_date}"
	fi

	# Now transfer the file to the target directory
	mv -i "$file" "${SORT_DIR}/${file_date}/${file_name}"
	printf "${OKBLUE}[${sorted_count}/${total_count}]${ENDC} ${file_name} ${HEADER}moved to${ENDC} ${SORT_DIR}/${file_date}\n"
 
done

# Set the internal field spearator
unset IFS
printf "${OKGREEN}[Done!]${ENDC}\n\n"



# Sync files to network drive
printf "${OKGREEN}[Rsync files to network drive...]${ENDC}\n"
rsync -rlptDcvz --progress "${SORT_DIR}/" "${SYNC_DIR}/"
printf "${OKGREEN}[Done!]${ENDC}\n\n"



# Delete robot computer copies of files
printf "${OKGREEN}[Deleting files from robot computer...]${ENDC}\n"
sudo rm -r "$SORT_DIR"
printf "${OKGREEN}[Done!]${ENDC}\n\n"



# Unmount drive
printf "${OKGREEN}[Unmounting ${HURST} from ${MOUNT_DIR}...]${ENDC}\n"
sudo umount "$MOUNT_DIR"
printf "${OKGREEN}[Done!]${ENDC}\n\n"



# Exit message
printf "${OKBLUE}[Completed]${ENDC} sync_data.sh\n"
exit 0


# vim:noexpandtab
