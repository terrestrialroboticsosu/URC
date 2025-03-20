#!/bin/bash

if [ "$1" == "stop" ]; then
    echo "Stopping socat..."
    
    pid=$(ps aux | grep 'socat' | grep -v 'grep' | awk '{print $2}')
    if [ -n "$pid" ]; then
        kill $pid
        echo "socat process stopped."
    else
        echo "No socat process found."
    fi
    exit 0
fi


sudo apt install -y socat

nohup socat -d -d pty,raw,echo=0 pty,raw,echo=0 &

soc_pid=$!

sudo rm -f /dev/ttyACM0
sudo ln -s /dev/pts/4 /dev/ttyACM0

echo "socat is running in the background with PID $soc_pid and symbolic link created."
echo "To stop socat, run the script with 'stop' argument: ./test.sh stop"
