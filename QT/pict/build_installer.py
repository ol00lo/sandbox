import subprocess
import sys
import shutil

def run_cmd(cmd: list):
    print(' '.join(cmd))
    result = subprocess.run(cmd, shell=True, check=True)
    return result

def check_cmd(cmd: str, install_hint: str):
    if not shutil.which(cmd):
        print(f" {cmd} not found\n {install_hint}")
        return False
    return True

def win_main():
    if not check_cmd("makensis", "Install NSIS."):
        return

    run_cmd([ sys.executable, "build_executable.py" ])

    run_cmd(["makensis", "make_installer.nsi"])

if __name__ == "__main__":
    if sys.platform == "win32":
        win_main()