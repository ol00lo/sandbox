set "BASE=%~dp0..\build\pybind"

if exist "%BASE%\Release\py_graph_pr*.pyd" (
    set "PYTHONPATH=%BASE%\Release"
) else if exist "%BASE%\Debug\py_graph_pr*.pyd" (
    set "PYTHONPATH=%BASE%\Debug"
) else (
    echo py_graph_pr*.pyd not found
    exit /b 1
)

python -m unittest discover -s "%~dp0tests" -p "*.py"
