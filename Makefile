EE_BIN = ber_the_house.elf
EE_OBJS = main.o
EE_LIBS = -lgskit -ldmakit -lpad -lm
EE_INCS += -I$(PS2DEV)/gsKit/include -I$(PS2SDK)/ports/include
EE_LDFLAGS += -L$(PS2DEV)/gsKit/lib -L$(PS2SDK)/ports/lib

EE_CFLAGS += -DgsKit_prim_triangle_gouraud=gsKit_prim_triangle_gouraud_3d

all: $(EE_BIN)

include $(PS2SDK)/samples/Makefile.pref
include $(PS2SDK)/samples/Makefile.eeglobal
