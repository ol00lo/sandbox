import time
import os

directory = os.path.dirname(os.path.abspath(__file__))

neededfiles = []
if os.name == "nt":
    exefile = os.path.join(directory, "build", "Debug", "simlife.exe")
elif os.name == "posix":
    exefile = os.path.join(directory, "build", "Makefile")
    
exetime = os.path.getmtime(exefile) 

for root, dirs, files in os.walk(directory):
    if 'build' in dirs:
        dirs.remove('build')
    for file in files:
        if file.endswith('.cpp') or file.endswith('.hpp'):
            filetime = os.path.getmtime(file)
            if filetime > exetime:
                neededfiles.append(file)


for file in neededfiles:
    print(file)
    os.system(f"clang-format -i {file}") 

print("DONE")
time.sleep(2)