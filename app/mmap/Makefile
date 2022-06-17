objects := $(patsubst %.c, %.o, $(wildcard *.c))

LDFLAGS += -lpthread

all: mmap

mmap:$(objects)
	$(CC) -o mmap $(objects) $(LDFLAGS)

.PHONY: clean
clean:
	rm -rf $(objects) mmap
