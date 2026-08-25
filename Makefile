# Builds libvl53l1x.so: the ST VL53L1X API, the Linux platform layer and the
# simplified wrapper in python_lib/, for use from C.
#
# The Python module does not need this. `pip install .` runs setup.py, which
# compiles the same sources into a separate vl53l1x_python*.so that the ctypes
# code in python/VL53L1X.py loads.

LIB := libvl53l1x.so
EXAMPLES := examples/distance

CFLAGS := -Iapi/core -Iapi/platform -Ipython_lib -std=c99 -O2 -Wall -fPIC
LDLIBS := -lpthread

SRC_FILES := $(wildcard api/core/*.c api/platform/*.c python_lib/*.c)
OBJ_FILES := $(SRC_FILES:.c=.o)

.PHONY: all examples clean

all: $(LIB)

$(LIB): $(OBJ_FILES)
	$(CC) -shared -o $@ $^ $(LDLIBS)

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

examples: $(EXAMPLES)

# The rpath saves callers having to set LD_LIBRARY_PATH to run from the
# source tree. Drop it if you install the library somewhere on the loader path.
$(EXAMPLES): %: %.c $(LIB)
	$(CC) $(CFLAGS) -o $@ $< -L. -lvl53l1x -Wl,-rpath,$(CURDIR)

clean:
	rm -f $(OBJ_FILES) $(LIB) $(EXAMPLES)
