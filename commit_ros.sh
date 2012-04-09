#!/bin/bash

cp -r /opt/ros/diamondback/stacks/nxt/nxt_terminator ~/ee149_nxt_ros 
cp -r /opt/ros/diamondback/stacks/nxt/ghost_bot ~/ee149_nxt_ros 

git add .
git commit -am $1
git push
