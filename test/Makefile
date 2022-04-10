HOMEDIR = /home/misys/
INCDIR = $(HOMEDIR)ichthusCAN/

OBJ = test

CPP = g++
CPPFLAGS += -pthread
CPPFLAGS += -std=c++17
CPPFLAGS += -Wall -Wextra -pedantic
CPPFLAGS += -g

INC = -I$(INCDIR)
INC += -I$(INCDIR)include/

LIBDBC = dbcppp
LIBCAN = can

LIBDIR = $(HOMEDIR)ichthusCAN/



all: $(OBJ).cpp
	$(CPP) -o $(OBJ) $(OBJ).cpp $(INC) -L$(LIBDIR) -l$(LIBDBC) -l$(LIBCAN) $(CPPFLAGS)
