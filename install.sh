#!/bin/bash

echo "======================================"
echo "SST XLeRobot Installation Script"
echo "Using Miniconda Environment: xlerobot"
echo "======================================"
echo ""

# Check if conda is installed
if ! command -v conda &> /dev/null; then
    echo "❌ Error: conda not found. Please install Miniconda first."
    exit 1
fi
echo "✅ Conda found"
echo ""

# Create conda environment
echo "[1/4] Creating conda environment 'xlerobot'..."
if conda env list | grep -q "^xlerobot "; then
    echo "⚠️  Environment 'xlerobot' already exists"
    read -p "Do you want to remove and recreate it? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        conda env remove -n xlerobot -y
        conda create -n xlerobot python=3.10 -y
        echo "✅ Environment recreated"
    else
        echo "✅ Using existing environment"
    fi
else
    conda create -n xlerobot python=3.10 -y
    echo "✅ Environment created"
fi
echo ""

# Activate environment
echo "[2/4] Activating conda environment..."
source $(conda info --base)/etc/profile.d/conda.sh
conda activate xlerobot
echo "✅ Environment activated"
echo ""

# Install lerobot
echo "[3/4] Installing lerobot with feetech support..."
echo "  This provides: torch, datasets, feetech-servo-sdk, etc."
cd lerobot
pip install -e ".[feetech]"
if [ $? -eq 0 ]; then
    echo "✅ lerobot[feetech] installed successfully"
else
    echo "❌ Error installing lerobot"
    exit 1
fi
cd ..
echo ""

# Install sst_xlerbot package
echo "[4/4] Installing sst_xlerbot package..."
echo "  This adds: rerun-sdk and sst_xlerbot modules"
pip install -e .
if [ $? -eq 0 ]; then
    echo "✅ sst_xlerbot installed successfully"
else
    echo "❌ Error installing sst_xlerbot"
    exit 1
fi
echo ""

echo "======================================"
echo "✅ Installation Complete!"
echo "======================================"
echo ""
echo "Installed packages:"
echo "  • lerobot[feetech] - Robot control library"
echo "  • sst_xlerbot - Quest VR teleoperation tools"
echo ""
echo "To run the program:"
echo "  1. Activate conda environment: conda activate xlerobot"
echo "  2. Run dataset recording: sst-record --config recording/config_recording.yaml"
echo ""
echo "Or use the run script:"
echo "  ./run.sh"
echo ""
echo "To setup USB permissions (Linux only):"
echo "  sudo usermod -a -G dialout \$USER"
echo "  (then logout and login again)"
echo ""
