import subprocess
import sys
import os
from pathlib import Path

def test_setup():    
    commands = [
        "pip install .",
        "python -c \"from main import main; main()\""
    ]
    
    for cmd in commands:
        subprocess.run(cmd, shell=True)

def build_windows_exe():
    subprocess.run([sys.executable, "create_win_inst.py"])

def create_installer():
    subprocess.run([
        "makensis",
        "make_installer.nsi"
    ])

if __name__ == "__main__":
    test_setup()
    
    if sys.platform == "win32":
        build_windows_exe()
        create_installer()