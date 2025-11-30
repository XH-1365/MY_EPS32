idf_component_register(SRCS "node.c" "file.c" "gpio.c" "utils.c"
					REQUIRES lua utils spiffs driver
					INCLUDE_DIRS "include")
