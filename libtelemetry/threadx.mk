libparrot-objs += parrot/libtelemetry/src/producer.o
libparrot-objs += parrot/libtelemetry/src/registrator.o
libparrot-objs += parrot/libtelemetry/src/wrapper.o
libparrot-objs += parrot/libtelemetry/src/variable.o
libparrot-objs += parrot/libtelemetry/src/shdctx.o
libparrot-objs += parrot/libtelemetry/src/consumer.o

subdir-ccflags-y += -std=c++11
