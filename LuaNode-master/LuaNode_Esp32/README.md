# HOW TO BUILD LuaNode for ESP32
---------------------------------

* Download ESP-IDF souces from the link: [esp-idf](https://github.com/espressif/esp-idf), and then setup the environtment according to idf documents.

* To build firmware for ESP32 (WROOM32) Change directory to `LuaNode32`, and then compile the project by executing the command: `idf.py build`.

* If you want to utilize File System (SPIFFS) for LuaNode, you have to change the partition file for SPIFFS first. To setup SPIFFS, input command: `idf.py menuconfig`, then go to `Partition Table` menu to select the file named, `partition_luanode.csv`, on the `LuaNode32` directory as the used partition file.

* To build for ESP32C6, change dir to `LuaNode32C6`, then set target by executing command `idf.py set-target esp32c6`, following `idf.py build`.

# FIRMWARE
---------------------------------

The firmware we built in the firmware directory is for test. You can flash it 
to your device to test LuaNode. The flash address for the firmware we provided
is `0x0`.

# USAGE
---------------------------------

## For esp32c6

When you run LuaNode on ESP32C6, you will see the LuaNode startup log as the following figure shows.

![github](https://github.com/Nicholas3388/LuaNode/raw/master/images/startup.png "startup")

LuaNode use 1M space on chip flash as file system, when the firmware first run, the file system is formated, and then a init Lua file named `init.lua` is created. When the LuaNode startup each time, the `init.lua` file is run firstly.

### Generate firmware for esp32c6

* Firstly, you should setup [esp-idf](https://github.com/espressif/esp-idf), and then download LuaNode repository. 

* Secondly, enter the directory named LuaNode32C6, execute command `idf.py menuconfig` on your console, and then go to the partition config page, and setup the partition file as we provide, make sure your selection is the same as the following figure. The partition file we used named `partitions_luanode.csv`. In the custom partition file, we use the first 1M on chip flash to store App, and use the following 1M space as file system.

![github](https://github.com/Nicholas3388/LuaNode/raw/master/images/partitions.png "partition")

* Setup the uart for input/output log or command. Open the file named `uart_handler.c`, change the Macro named `TXD_PIN`, `RXD_PIN`, and `USE_UART` to any pin you like to use.

* Finally, use `idf.py build` to build the firmware for Esp32c6.

* You can also use the prebuild firmware place on directory: `firmware`, the firmware for esp32c6 named `LuaNode32C6.bin`. Flash the binary file to address 0x0.

### Run with ESPlorer

LuaNodeC6 is run compatible with [ESPlorer](https://github.com/4refr0nt/ESPlorer "Esplorer"). When LuaNode run on ESP32C6, the ESPlorer will log the powerup info as following figure, you can see that ESP32C6 is detected as a NodeMCU by ESPlorer. 

![github](https://github.com/Nicholas3388/LuaNode/raw/master/images/esplorer_run.png "Esplorer")

You can use ESPlorer as a Lua code editor. Firstly, you should setup the COM you used to connect ESP32C6. Usages of ESPlorer is show as following figure, enter your code on left textfield, and then click `Send to ESP` button to run your code. Finally, you'll get your results on the right window.

![github](https://github.com/Nicholas3388/LuaNode/raw/master/images/run_lua_code.png "Esplorer")

For more details about usage of ESPlorer, please check the ESPlorer [repository](https://github.com/4refr0nt/ESPlorer "Esplorer")

### More feature for LuaNode-C6 is coming soon

We are working with Lua Module for Zigbee now. With this Zigbee module, you can write Lua code to enable Zigbee.
