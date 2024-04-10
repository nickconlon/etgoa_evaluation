#!/usr/bin/env bash

sshpass -p nick1234 ssh -o StrictHostKeyChecking=no ubuntu@192.168.20.213 "rosnode kill -a; sudo shutdown -h now"
