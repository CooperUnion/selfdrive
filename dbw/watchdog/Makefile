CXX = g++

src = $(wildcard ./src/*.cpp)
obj = $(src:.cpp=.o)
all: clean watchdog

watchdog: $(obj)
	$(CXX) -pthread -o $@ $^ -lsocketcan -latomic
	$(RM) $(obj)

clean:
	$(RM) $(obj) watchdog
