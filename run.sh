#!/bin/bash

# Simple run script for conda environment
echo "Starting Quest VR XLeRobot Controller..."
echo ""

# Check if conda is installed
if ! command -v conda &> /dev/null; then
    echo "❌ Error: conda not found"
    exit 1
fi

# Check if xlerobot environment exists
if ! conda env list | grep -q "^xlerobot "; then
    echo "❌ Conda environment 'xlerobot' not found!"
    echo "Please run ./install.sh first"
    exit 1
fi

# Activate conda environment
echo "Activating conda environment 'xlerobot'..."
source $(conda info --base)/etc/profile.d/conda.sh
conda activate xlerobot

# Check if lerobot is installed
if ! python -c "import lerobot" 2>/dev/null; then
    echo "❌ lerobot not installed!"
    echo "Please run ./install.sh first"
    exit 1
fi

echo "✅ Environment ready"
echo ""

# Run the packaged teleop controller
python -m sst_xlerbot.teleop.quest_vr_xlerobot_controller_no_base "$@"
