FILE=skeleton
FILE2=GenerateCity

########
#   Directories
S_DIR=Source
B_DIR=Build
GLMDIR =../glm

########
#   Output
EXEC=$(B_DIR)/$(FILE)

# default build settings
CC_OPTS=-c -pipe -Wall -O3 -Wno-switch -ggdb -g3
LN_OPTS=
CC=g++ -fopenmp

########
#       SDL options
SDL_CFLAGS := $(shell sdl2-config --cflags)
GLM_CFLAGS := -I$(GLMDIR)
SDL_LDFLAGS := $(shell sdl2-config --libs)

########
#   This is the default action
all:Build


########
#   Object list
#
OBJ = $(B_DIR)/$(FILE).o $(B_DIR)/$(FILE2).o


########
#   Objects

$(B_DIR)/$(FILE).o : $(S_DIR)/$(FILE).cpp $(S_DIR)/SDLauxiliary.h $(S_DIR)/GenerateCity.h
	$(CC) $(CC_OPTS) -o $(B_DIR)/$(FILE).o $(S_DIR)/$(FILE).cpp $(SDL_CFLAGS) $(GLM_CFLAGS)

$(B_DIR)/$(FILE2).o : $(S_DIR)/$(FILE2).cpp $(S_DIR)/SDLauxiliary.h $(S_DIR)/GenerateCity.h
	$(CC) $(CC_OPTS) -o $(B_DIR)/$(FILE2).o $(S_DIR)/$(FILE2).cpp $(SDL_CFLAGS) $(GLM_CFLAGS)

########
#   Main build rule
Build : $(OBJ) Makefile
	$(CC) $(LN_OPTS) -o $(EXEC) $(OBJ) $(SDL_LDFLAGS)


clean:
	rm -f $(B_DIR)/*
