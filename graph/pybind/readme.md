# Запуск Python-тестов

## 1. Запуск всех тестов

`run_py_tests.bat` для запуска всех тестов на Windows

Запуск всех тестов на Linux:
```bash
PYTHONPATH="$(dirname "$0")/../build/python/Debug" \
python3 -m unittest discover -s "$(dirname "$0")/tests" -p "*.py"
```
---

## 2. Запуск конкретных тестов:
Windows:
```power shell
$env:PYTHONPATH = "$PSScriptRoot\..\build\python\Debug"
python -m unittest tests.tests.TestArithmeticNodes.test_plus_node
```

Linux:
```bash
PYTHONPATH="$(dirname "$0")/../build/python/Debug" \
python3 -m unittest tests.tests.TestArithmeticNodes.test_plus_node
```