#!/bin/sh

#! down first then up
[[ $(cat /sys/class/net/w*/operstate) = down ]] && echo "false" || echo "true"
