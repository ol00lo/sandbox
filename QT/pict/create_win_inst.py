import subprocess
import sys
import os
from pathlib import Path

def create():
    project_dir = Path(__file__).parent
    icon_path = project_dir / 'resources' / 'p.ico'
    main_script = project_dir / 'main.py'

    cmd = [
        sys.executable, "-O", "-m", "PyInstaller",
        "-y", "--clean", "-w", "--onedir", "-i", str(icon_path),
        "--add-data", f"{project_dir / 'resources'}{os.pathsep}resources",
        "--name", "PictApp",
        str(main_script)
    ]
    result = subprocess.run(cmd, check=True)

    if result.returncode == 0:
        output_dir = project_dir / 'dist' / 'PictApp'
        print(f"\nBuild successful! Application created in: {output_dir}")
    else:
        print("Build failed!", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    create()