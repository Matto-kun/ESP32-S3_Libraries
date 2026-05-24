#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gpio_api.hpp"

constexpr uint8_t kOutPin = 7U; // Adjust to your board's Output pin
constexpr uint8_t kInPin  = 5U; // Adjust to your board's Input pin (e.g., BOOT button)

extern "C" void app_main(void)
{
    gpio_cfg_t in_cfg = {};
    in_cfg.pin_num = kInPin;
    in_cfg.direction = eGpioDirInput;
    in_cfg.pull = eGpioPullUp; // Assuming button is active-low with internal pull-up
    GPIO out_pin(kOutPin, eGpioDirOutput);
    GPIO in_pin(in_cfg);

    if(out_pin.init() != ESP_OK) {
        printf("Failed to init output pin\n");
        return;
    }
    
    if(in_pin.init() != ESP_OK) {
        printf("Failed to init input pin\n");
        return;
    }
    
    bool output_state = false;
    bool prev_input_state = true;
    
    while (true) {
        bool current_input_state = false;
        in_pin.read(current_input_state);
        
        // Detect falling edge (button press)
        if (prev_input_state && !current_input_state) {
            output_state = !output_state;
            out_pin.write(output_state);
            printf("Output toggled to: %d\n", output_state);
        }
        
        prev_input_state = current_input_state;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}