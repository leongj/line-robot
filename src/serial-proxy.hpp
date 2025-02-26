#ifndef SERIAL_PROXY_HPP
#define SERIAL_PROXY_HPP

#include <string>
#include <vector>
#include <Arduino.h>
#include <Wire.h>

class SerialProxy {
    private:
      std::vector<std::string> history;
      size_t max_lines;
      size_t current_index;
    
    public:
        SerialProxy(size_t lines = 20) : max_lines(lines), current_index(0) {
            history.resize(lines);
        }
    
        void println(const String& str) {
            if (Serial) {
                Serial.println(str);
            }

            history[current_index] = str.c_str();
            current_index = (current_index + 1) % max_lines;
        }

        void println(const std::string& str) {
            if (Serial) {
                Serial.println(str.c_str());
            }

            history[current_index] = str;
            current_index = (current_index + 1) % max_lines;
        }

        void println(const char* str) {
            if (Serial) {
                Serial.println(str);
            }

            history[current_index] = str;
            current_index = (current_index + 1) % max_lines;
        }

        void println(int num, int base = 10) {
            if (Serial) {
                Serial.println(num, base);
            }

            history[current_index] = String(num, base).c_str();
            current_index = (current_index + 1) % max_lines;
        }
    
        std::vector<std::string> tail(size_t lines) const {
            std::vector<std::string> last_n_lines;
            if (lines > max_lines) {
                lines = max_lines;
            }
            for (size_t i = 0; i < lines; ++i) {
                size_t index = (current_index - lines + i + max_lines) % max_lines;
                if (!history[index].empty()) {
                    last_n_lines.push_back(history[index]);
                }
            }
            return last_n_lines;
        }

        std::vector<std::string> all() const {
            std::vector<std::string> ordered_history;
            for (size_t i = 0; i < max_lines; ++i) {
                size_t index = (current_index + i) % max_lines;
                if (!history[index].empty()) {
                ordered_history.push_back(history[index]);
                }
            }
            return ordered_history;
        }
        
        std::string last() const {
            size_t index = (current_index - 1 + max_lines) % max_lines;
            return history[index];
        }
};

#endif // SERIAL_PROXY_HPP