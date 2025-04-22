import subprocess
import sys
import shutil

def run_cmd(cmd: str):
    print(cmd)
    subprocess.run(cmd, shell=True)

def check_cmd(cmd: str, install_hint: str) -> bool:
    if not shutil.which(cmd):
        print(f" {cmd} not found\n {install_hint}")
        return False
    return True

def main():
    commands = [
        "pip install .",
        "python -c \"from main import main; main()\""
    ]
    for cmd in commands:
        run_cmd(cmd)

    if not check_cmd("pyinstaller", "Installing pyinstaller..."):
        run_cmd([ sys.executable, "-m", "pip", "install", "pyinstaller" ])
        
    if sys.platform == "win32":
        if not check_cmd("makensis", "Install NSIS"):
            return
        
        run_cmd([ sys.executable, "build_installer.py" ])

        run_cmd(["makensis", "make_installer.nsi"])

if __name__ == "__main__":
    main()