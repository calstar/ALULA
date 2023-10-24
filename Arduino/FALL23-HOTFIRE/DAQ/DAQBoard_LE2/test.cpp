#include <iostream>

struct HX711 {
    int count = 0;
    float read() {
        return count++;
    }
};

typedef struct struct_hx711 {
private:
  float reading;
  const static int ROLLING_AVG_BUFFER_SIZE = 10;
  float rollingAvgBuffer[ROLLING_AVG_BUFFER_SIZE] = {};
  float sum = 0;
  int numReadings = 0;
  int index = 0;

public:
  HX711 scale;
  int clk;
  int gpio;
  float offset;
  float slope;

  float read() {
    float newReading = scale.read();

    if (numReadings < ROLLING_AVG_BUFFER_SIZE) {
      numReadings++;
    }
    else {
      sum -= rollingAvgBuffer[index];
    }
    rollingAvgBuffer[index] = newReading;
    index = (index + 1) % ROLLING_AVG_BUFFER_SIZE;
    sum += newReading;

    return sum / numReadings;
  }
} struct_hx711;

int main() {
    HX711 board = HX711();
    struct_hx711 daq;
    daq.scale = board;

    int count = 0;

    while (count < 20) {
        std::cout << daq.read() << std::endl;
        count++;
       // std::cout << "count: " << count << std::endl;
    }

    return 0;
}