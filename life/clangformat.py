import time
import os

directory = os.path.dirname(os.path.abspath(__file__))

for root, dirs, files in os.walk(directory):
    if 'build' in dirs:
        dirs.remove('build')
    for file in files:
        if file.endswith('.cpp') or file.endswith('.hpp') or file.endswith('.h'):
            command = f'clang-format {file}'
            modfile = os.popen(command).read()
            with open(file) as f:
                curfile = f.read()
            if curfile!= modfile:
                os.system(f"clang-format -i {file}")
                print(f"File  {file}  has changed") 

print("DONE")
time.sleep(2)