CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2
TARGET = test_monitor
SOURCE = test.cpp

$(TARGET): $(SOURCE)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SOURCE) -lrt -pthread

clean:
	rm -f $(TARGET)

.PHONY: clean 