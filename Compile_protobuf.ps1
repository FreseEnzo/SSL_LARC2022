cd nanopb-0.4.5-windows-x86\generator-bin
 .\protoc.exe -I="..\..\SSL_Firmware" --nanopb_out="..\..\SSL_Firmware\Lib" ..\..\SSL_Firmware\grSim_Commands.proto
 .\protoc.exe -I="..\..\SSL_Firmware" --nanopb_out="..\..\SSL_Firmware\Lib" ..\..\SSL_Firmware\Feedback.proto
copy ..\pb.h ..\..\SSL_Firmware\Lib
copy ..\pb_common.c ..\..\SSL_Firmware\Lib
copy ..\pb_common.h ..\..\SSL_Firmware\Lib
copy ..\pb_decode.c ..\..\SSL_Firmware\Lib
copy ..\pb_decode.h ..\..\SSL_Firmware\Lib
copy ..\pb_encode.c ..\..\SSL_Firmware\Lib
copy ..\pb_encode.h ..\..\SSL_Firmware\Lib
cd ..\..