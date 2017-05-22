python serial_controller.py -p 9002 -s '/dev/serial/by-id/*Prolific*' &
python serial_controller.py -p 9001 -s '/dev/serial/by-id/*Arduino*' -b 9600 -m -e '1,1,1,0!#' &
