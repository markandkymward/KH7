@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
set "VIEWER_SCRIPT=%SCRIPT_DIR%tools\imu_board_viewer.py"
set "PYTHON_EXE=C:\Users\marka\AppData\Local\Python\pythoncore-3.14-64\python.exe"

if exist "%PYTHON_EXE%" (
  "%PYTHON_EXE%" "%VIEWER_SCRIPT%"
  goto :end
)

where py >nul 2>nul
if %ERRORLEVEL%==0 (
  py "%VIEWER_SCRIPT%"
  goto :end
)

where python >nul 2>nul
if %ERRORLEVEL%==0 (
  python "%VIEWER_SCRIPT%"
  goto :end
)

echo Could not find a Python interpreter.
echo Install Python and rerun this launcher.
pause

:end
endlocal
