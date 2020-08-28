World ini digunakan untuk dilaunch dengan sitl px4\
Cara menggunakannya :
1. Copy file ke directory ../Firmware/Tools/sitl_gazebo/worlds/
2. Buka file ../Firmware/platforms/posix/cmake/sitl_target.cmake
3. Pergi ke line 85 yang berisi\
   set(worlds none empty baylands ksql_airport mcmillan_airfield sonoma_raceway warehouse windy ArenaKRTI)
4. Masukan nama world file ke dalam tanda kurung, seperti contoh ArenaKRTI
