@ECHO OFF
setlocal
set "lock=%temp%\wait%random%.lock"

start "robot1" 9>"%lock%1" java -jar run.jar 1 0.3 D:\Desktop\CapoRobotFearBasedControl_10_2015\CommonResources\MazeRoboLabEmptyMapWithGate.roson 1.25 1 0.25 1 1.25 4 -1.57
start "robot2" 9>"%lock%2" java -jar run.jar 2 0.3 D:\Desktop\CapoRobotFearBasedControl_10_2015\CommonResources\MazeRoboLabEmptyMapWithGate.roson 1.25 4 0.25 1 1.25 1 1.57
call :Wait

start "robot1" 9>"%lock%1" java -jar run.jar 1 0.3 D:\Desktop\CapoRobotFearBasedControl_10_2015\CommonResources\MazeRoboLabEmptyMapWithGate.roson 1.25 1 0.25 1 1.25 4 -1.57
start "robot2" 9>"%lock%2" java -jar run.jar 2 0.3 D:\Desktop\CapoRobotFearBasedControl_10_2015\CommonResources\MazeRoboLabEmptyMapWithGate.roson 1 4 0.25 1 1 1 1.57
start "robot3" 9>"%lock%3" java -jar run.jar 3 0.3 D:\Desktop\CapoRobotFearBasedControl_10_2015\CommonResources\MazeRoboLabEmptyMapWithGate.roson 1.5 4 0.25 1 1.5 1 1.57
call :Wait

start "robot1" 9>"%lock%1" java -jar run.jar 1 0.3 D:\Desktop\CapoRobotFearBasedControl_10_2015\CommonResources\MazeRoboLabEmptyMapWithGate.roson 1 1 0.25 1 1 4 -1.57
start "robot2" 9>"%lock%2" java -jar run.jar 2 0.3 D:\Desktop\CapoRobotFearBasedControl_10_2015\CommonResources\MazeRoboLabEmptyMapWithGate.roson 1.5 1 0.25 1 1.5 4 -1.57
start "robot3" 9>"%lock%3" java -jar run.jar 3 0.3 D:\Desktop\CapoRobotFearBasedControl_10_2015\CommonResources\MazeRoboLabEmptyMapWithGate.roson 1 4 0.25 1 1 1 1.57
start "robot4" 9>"%lock%4" java -jar run.jar 4 0.3 D:\Desktop\CapoRobotFearBasedControl_10_2015\CommonResources\MazeRoboLabEmptyMapWithGate.roson 1.5 4 0.25 1 1.5 1 1.57
call :Wait

echo Koniec eksperymentu
goto:eof

rem start "robot1" 9>"%lock%1" java -jar runGate.jar 1 0.3 D:\Desktop\CapoRobotFearBasedControl_10_2015\CommonResources\MazeRoboLabEmptyMapWithGate.roson 1 1 0.25 1 1 4 -1.57
rem start "robot2" 9>"%lock%2" java -jar runGate.jar 2 0.3 D:\Desktop\CapoRobotFearBasedControl_10_2015\CommonResources\MazeRoboLabEmptyMapWithGate.roson 1 4 0.25 1 1 1 1.57
rem start "robot2" 9>"%lock%3" java -jar runGate.jar 3 0.3 D:\Desktop\CapoRobotFearBasedControl_10_2015\CommonResources\MazeRoboLabEmptyMapWithGate.roson 1.5 4 0.25 1 1.5 1 1.57
rem start "robot1" 9>"%lock%4" java -jar runGate.jar 4 0.3 D:\Desktop\CapoRobotFearBasedControl_10_2015\CommonResources\MazeRoboLabEmptyMapWithGate.roson 1.5 1 0.25 1 1.5 4 -1.57



:Wait
1>nul 2>nul ping /n 2 ::1
for %%N in (1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20) do (
  (call ) 9>"%lock%%%N" || goto :Wait
) 2>nul
del "%lock%*"
GOTO:eof

