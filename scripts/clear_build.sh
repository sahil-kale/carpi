#!/bin/bash
rm -rf build/ install/ log/
find . -name "*.egg-info" -type d -exec rm -rf {} +
find . -name "__pycache__" -type d -exec rm -rf {} +
