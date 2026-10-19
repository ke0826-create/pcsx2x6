@echo off
cd /d "%~dp0"
echo Pushing branch rollback to github.com/ke0826-create/pcsx2x6
echo If a browser opens, sign in to GitHub as ke0826-create and allow access.
echo.
git push -u origin rollback
echo.
echo Done. Success if you see: rollback -^> rollback
pause
