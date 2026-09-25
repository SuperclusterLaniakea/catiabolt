@echo off
setlocal
cd /d "%~dp0"

if exist "venv\Scripts\python.exe" (
    set "PY=venv\Scripts\python.exe"
) else (
    set "PY=python"
)

echo ============================================================
echo   CATIA Bolt Designer  -  parameterized bolt modeling tool
echo   All types / All precision classes / Metric & Imperial
echo   Starting ... please open CATIA before drawing
echo ============================================================
echo.

"%PY%" "src\run.py"
set "RC=%errorlevel%"

if not "%RC%"=="0" (
    echo.
    echo [ERROR] Exited with code %RC%
    echo   Possible causes:
    echo     1. Missing dependencies - run:
    echo        pip install -r requirements.txt
    echo     2. Missing Visual C++ Redistributable - PyQt6 needs it
    echo     3. If only Draw fails, the UI still opens fine;
    echo        open CATIA first, then click Draw to CATIA
    echo.
    pause
) else (
    echo Done.
    pause
)

endlocal
