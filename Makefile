CXXFLAGS = -g -std=c++11 -O2

DEPS = main.cpp Makefile *.h

IFLAG = -I./common/
IFLAG += -I./common/recordmgr/
IFLAG += -I./

#DFLAG = -DUSE_TRACE

main: $(DEPS)
	$(CXX) main.cpp -o $@ $(CXXFLAGS) $(IFLAG) $(DFLAG)