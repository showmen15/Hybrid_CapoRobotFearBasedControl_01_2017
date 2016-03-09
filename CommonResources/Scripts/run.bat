@ECHO OFF
call :sub_message1
call :sub_message

Echo prawdziwy koniec tej zabawy
goto:eof

:sub_message
   Echo this is a subroutine
goto:eof
   
:sub_message1
   Echo koniec jest bliski
goto:eof
