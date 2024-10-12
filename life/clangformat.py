import time
import os
import sys

source_dirs = sys.argv[1:]

for directory in source_dirs:
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith('.cpp') or file.endswith('.hpp') or file.endswith('.h'):
                filepath = os.path.join(root, file)
                command = f'clang-format {filepath}'    
                modfile = os.popen(command).read()    
                with open(filepath) as f:
                    curfile = f.read()
                if curfile != modfile:
                    with open(filepath, 'w') as f:
                        f.write(modfile)
                    print(f"File {filepath} has been changed")

print("DONE")
time.sleep(2)