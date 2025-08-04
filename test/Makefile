CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2
TARGET = test_monitor
SOURCE = test.cpp

$(TARGET): $(SOURCE)
	$(CXX) $(CXXFLAGS) -lrt -pthread -o $(TARGET) $(SOURCE)

clean:
	rm -f $(TARGET)

.PHONY: clean 

#g++ -v test.cpp -o test_monitor -lrt -pthread