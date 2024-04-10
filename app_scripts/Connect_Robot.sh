#!/usr/bin/env bash

sshpass -p unicorn ssh -o StrictHostKeyChecking=no cohrint@192.168.20.126 "source ./network.sh 155 126; ./startup.sh aspen"

