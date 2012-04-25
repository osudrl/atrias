#!/bin/bash
ps aux | tail -n +2 | awk '{ system("sudo taskset -p 1 "$2) }'
