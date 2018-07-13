.PHONY: clean lib

SRC = api
LIB = libvl53l1_api.a
LDFLAGS :=
CFLAGS := -Iapi/core -Iapi/platform -std=c99

SRC_FILES := $(wildcard $(SRC)/*/*.c)
SRC_FILES := $(wildcard $(SRC)/*/*.c)
OBJ_FILES := $(patsubst $(SRC)/*/*.c,$(SRC)/*/%.o,$(SRC_FILES))

$(LIB): $(OBJ_FILES)
	$(CC) $(LDFLAGS) $(CFLAGS) -fPIC -shared -o $@ $^

$(SRC)/%.o: $(SRC)/%.c
	$(CC) $(LDFLAGS) $(CFLAGS) -c -o $@ $<

test: test.c
	$(CC) $(CFLAGS) $(LDFLAGS) -Lvl53l1_api -o test test.c
