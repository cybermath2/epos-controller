CC		= clang
FLAGS		=  -Wall -ggdb -I./deps -L/usr/local/lib -lEposCmd -lftd2xx -lpthread
SOURCE_FILES	= $(shell find . -type f -name '*.c')
HEADER_FILES	= $(shell find . -type f -name '*.h') # include these as well to force rebuild

TARGET		= epos_controller

.PHONY: clean

all: $(TARGET)

$(TARGET): $(SOURCE_FILES) $(HEADER_FILES)
	$(CC) $(FLAGS) $(SOURCE_FILES) -o $@

clean:
	rm -f $(TARGET)
