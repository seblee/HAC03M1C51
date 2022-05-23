copy ..\OBJ\T5L51.hex ..\Download
cd ..\Download
srec_cat.exe T5L51.hex -Intel -o T5L51.bin -Binary 
copy T5L51.bin  ..\..\T5L_PRJ\DWIN_SET
echo.