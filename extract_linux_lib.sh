set -x
set -o


mkdir linux_lib
cd linux_lib

mv ../AirLib/lib/libAirLib.a $PWD
mv ../AirLib/deps/MavLinkCom/lib/libMavLinkCom.a $PWD
mv ../AirLib/deps//rpclib/lib/librpc.a $PWD

