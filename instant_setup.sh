_IS_VERBOSE=false
_IS_HELP=false
_LANGUAGE=cpp   # default implementation language is cpp, set -p flag for python

# handle inputs
while [ $# -gt 0 ]; do
  case $1 in
    # handle
    -h | --help)
      _IS_HELP=true
      ;;
    -v | --verbose)
      _IS_VERBOSE=true
      ;;
    -p |--python)
      _LANGUAGE=python
      ;;
  esac
  shift
done

# custom run function that prints verbose if verbose mode is turned on, else does nothing
function my_run () {
  if [ $_IS_VERBOSE = true ]; then
    echo "\$ $@"
  fi	
  "$@"	
}
	
# help flag
if [ $_IS_HELP = true ]; then
  echo "my custom little setup script:
  -h | --help) 	help: this current message
  -v | --verbose) enable verbose mode
  -p | --python) 	build for python instead of cpp"
else
  _ORIGINAL_DIR=$(pwd)
  _SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
  cd $_SCRIPT_DIR
  
  my_run cd ${_LANGUAGE}_impl
  my_run source /opt/ros/humble/setup.bash    # if using ROS Humble
  # my_run source /opt/ros/jazzy/setup.bash   # if using ROS Jazzy
  my_run colcon build
  my_run cd ..
  my_run source ${_LANGUAGE}_impl/install/setup.bash
  my_run cd $_ORIGINAL_DIR
  
  if [ $_LANGUAGE = cpp ]; then
    echo "
You have the following commands available:
	ros2 launch edubot sim.launch.py	# 1 Launch sim and rviz
	ros2 launch edubot rviz.launch.py	# 2 Launch rviz and joints
	ros2 run edubot robot_hw		      # 3 Start hardware driver
	ros2 run controllers example_traj	# 4 Start example trajectory"
	
    read -p "Choice [1...4] or none to skip: " _CHOSEN_OPTION
    case $_CHOSEN_OPTION in
      1)
        my_run ros2 launch edubot sim.launch.py
        ;;
      2)
        my_run ros2 launch edubot rviz.launch.py
        ;;
      3)
        my_run ros2 run edubot robot_hw
        ;;
      4)
        my_run ros2 run controllers example_traj
        ;;
      *)
        ;;
    esac
  else
    echo "
You have the following commands available:
  ros2 run controllers example_traj # 1 Start example trajectory
  ros2 run controllers draw_traj    # 2 Start drawing trajectory
  ros2 run controllers jacobian_traj# 3 Start jacobian velocity trajectory
  ros2 run controllers grab_ab_traj # 4 Start pick and place trajectory
  ros2 run controllers grab_ba_traj # 5 Start reverse pick and place trajectory
  ros2 run controllers berry_traj   # 6 Start pick and place berry trajectory
  ros2 run controllers wipe_traj    # 7 Start wipe trajectory
  ros2 run controllers demo_traj    # 8 Start demo trajectory
  ros2 run controllers empty_traj   # 9 Start empty trajectory"

	
    while : ; do
      read -p "Choice [1...9] or NONE to skip: " _CHOSEN_OPTION
      case $_CHOSEN_OPTION in
        1)
          my_run ros2 run controllers example_traj  # || break
          ;;
        2)
          my_run ros2 run controllers draw_traj     #	|| break # 2 Start drawing trajectory
          ;;
        3)
          my_run ros2 run controllers jacobian_traj # || break # 2 Start drawing trajectory
          ;;
        4)
          my_run ros2 run controllers grab_ab_traj  # || break 	# 2 Start pick and place trajectory
          ;;
        5)
          my_run ros2 run controllers grab_ba_traj  # || break 	# 2 Start reverse pick and place trajectory
          ;;
        6)
          my_run ros2 run controllers berry_traj    # || break 	# 2 Start pick and place berry trajectory
          ;;
        7)
          my_run ros2 run controllers wipe_traj     #	|| break # 2 Start wipe trajectory"
          ;;
        8)
          my_run ros2 run controllers demo_traj     #	|| break # 2 Start wipe trajectory"
          ;;
        9)
          my_run ros2 run controllers empty_traj    #	|| break # 2 Start wipe trajectory"
          ;;
        *)
          break
          ;;
      esac
    done
  fi
fi
