#!/bin/bash
export OMP_NUM_THREADS=1

host=${1:-localhost}
port=${2:-3100}

python3 ./Run_Player.py -i $host -p $port -u 1  -t FCP-debug -P 1 -D 1 &
python3 ./Run_Player.py -i $host -p $port -u 11 -t FCP-debug -P 1 -D 1 &