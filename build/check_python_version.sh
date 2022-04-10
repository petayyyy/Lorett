#!/bin/bash

echo "Check Python version"
echo "You have: "
pyv=$(python -V 2>&1 | grep -Po '(?<=Python )(.+)')
if [[ -z "$pyv" ]]
then
    pyv="$(python3 -V)" 
fi
echo $pyv

if [[ *"2."* != "$pyv" ]] ;then
pyv=''
else
pyv='3'
fi

#sudo apt-get install python$pyv-numpy -y
