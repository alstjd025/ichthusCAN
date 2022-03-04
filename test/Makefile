HOMEDIR = /home/misys/
INCDIR = $(HOMEDIR)libcan/

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

LIBDIR = $(HOMEDIR)libcan/



all: $(OBJ).cpp
	$(CPP) -o $(OBJ) $(OBJ).cpp $(INC) -L$(LIBDIR) -l$(LIBDBC) -l$(LIBCAN) $(CPPFLAGS)