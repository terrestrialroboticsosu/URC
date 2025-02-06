#!/bin/bash

set -e

sshpass -p lunabotics2024 scp -r src/ include/ meson.build fros@192.168.0.10:~/code/