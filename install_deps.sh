#!/bin/bash
set -e

echo "=========================================="
echo " Updating package lists...                "
echo "=========================================="
apt-get update

echo "=========================================="
echo " Checking for missing dependencies...     "
echo "=========================================="
rosdep update
rosdep install --from-paths src --ignore-src -r -y

echo "=========================================="
echo " Dependencies up to date!                 "
echo "=========================================="