#include <iostream>

struct HX711 {
    int count = 0;
    float read() {
        return count++;
    }
};

struct Adafruit_MAX31855 {
    int count = 0;
    float read() {
        return count--;
    }
};


struct Filter {
private:
  const static int ROLLING_AVG_BUFFER_SIZE = 10;
  float rollingAvgBuffer[ROLLING_AVG_BUFFER_SIZE] = {};
  float sum = 0;
  int numReadings = 0;
  int index = 0;

public:
  void addReading(float newReading) {
    if (numReadings < ROLLING_AVG_BUFFER_SIZE) {
      numReadings++;
    }
    else {
      sum -= rollingAvgBuffer[index];
    }
    rollingAvgBuffer[index] = newReading;
    index = (index + 1) % ROLLING_AVG_BUFFER_SIZE;
    sum += newReading;
  }

  float getReading() {
    if (numReadings == 0) {
      return -1;
    }
    return sum / numReadings;
  }

  void resetReadings() {
    sum = 0;
    numReadings = 0;
    index = 0;
  }
};

template <class Board>
struct struct_data_board {
private:
  Filter filter; 

  float getDataFromBoard(bool isFilteredData) {
    float newReading = scale.read();
    filter.addReading(newReading);

    if (isFilteredData) {
      return filter.getReading();
    }
    else {
      return newReading;
    }
  }

public:
  Board scale;
  float offset;
  float slope;
  float reading;

  struct_data_board(Board scale, float offset, float slope) {
    this->scale = scale;
    this->offset = offset;
    this->slope = slope;
  }

  float readFilteredData() {
    return getDataFromBoard(true);
  }

  float readRawData() {
    return getDataFromBoard(false);
  }

  void resetReading() {
    filter.resetReadings();
  }
};

struct struct_hx711 : struct_data_board<HX711> {
public:
  int clk;
  int gpio;

  struct_hx711(HX711 scale, int clk, int gpio, float offset, float slope) : struct_data_board(scale, offset, slope) {
    this->clk = clk;
    this->gpio = gpio;
  }
};

class struct_max31855 : struct_data_board<Adafruit_MAX31855> {  
public:
  int cs;

  struct_max31855(Adafruit_MAX31855 scale, float cs, float offset, float slope) : struct_data_board(scale, offset, slope) {
    this->cs = cs;
  }
};


int main() {
    struct_hx711 hx711 = struct_hx711(HX711(), 2, 3, 3, 5);
    struct_max31855 max31855 = struct_max31855(Adafruit_MAX31855(), 2, 3, 3);

    int count = 0;

    while (count < 14) {
        std::cout << hx711.readFilteredData() << std::endl;
        count++;
    }
    hx711.resetReading();
    while (count < 20) {
        std::cout << hx711.readFilteredData() << std::endl;
        count++;
    }

    return 0;
}