#!/bin/bash
# Since PathPlanner loads ALL autos when using AutoBuilder, even the ones that have been previously deployed 
# and broken autos/paths breaks Robot Code, this script will automatically SSH into the robot and
# clear out the deploy folder. Keep in mind there are OTHER things in the deploy folder, so use this with CAUTION!
set +x
echo "Careful! This will delete the ENTIRE deploy folder AND the robot code. Are you SURE you want to continue?"
read -p "Press Enter to continue."
set -x
ssh admin@10.32.56.2 <<'ENDSSH'
set -x
cd ../
cd lvuser
rm FRC_Programming_2024.jar
rm -rf deploy
/usr/local/frc/bin/frcKillRobot.sh -r -t 
set +x
ENDSSH
set +x
echo "Cleared."
read -p "Reboot the rio? (y/n) " RESP
if [ "$RESP" = "y" ]; then
  ssh admin@10.32.56.2 reboot
else
  echo "Not rebooting the roboRIO. Please do it later!"
fi

