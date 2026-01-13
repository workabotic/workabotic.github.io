---
title: "Smart Dimmer Controlled by Mobile App"
date: 2024-08-01 18:00:00
description: A smart dimmer system built using an ESP32 microcontroller, controlled via a mobile app to adjust the brightness of an incandescent light bulb.
author: lucasmazzetto
keywords: android, microcontroller, embedded, IoT, espressif, ESP32, esp32, esp-idf, RTOS, FreeRTOS, dimmer, TRIAC, mobile app, automation, smart home, dimmer circuit, home automation, Wi-Fi dimmer, electronics, embedded systems, electrical engineering, DIY.
---

This project develops a smart dimmer system for controlling the brightness of an incandescent light bulb via a mobile app. Built on an ESP32, it uses phase control with a zero-crossing detection circuit and a TRIAC-based power stage for AC waveform synchronization. The work includes hardware design, embedded firmware with ESP-IDF, and an Android app communicating through a web server.

## Introduction 

Dimmers are electronic devices typically connected to lights to control their brightness. Similar circuits can also be used in various applications, such as controlling the speed of an electric motor or setting the temperature of an electric oven. The basic idea is to limit the power supplied from a source to the load by altering the waveform of the applied voltage.

The key component used to achieve this goal is the TRIAC. This electronic component has three terminals: one called the "gate," which is responsible for allowing or blocking the passage of current through the other two terminals. By default, the TRIAC operates as an open circuit, preventing current flow. However, when the gate is triggered, the component behaves like a closed switch, allowing current to flow and providing power to the load until the next cycle of the supply waveform.

For example, when considering a light bulb as the load, it is possible to allow only a portion of the current to flow to it, thus regulating the light brightness. To control the brightness as desired, it is necessary to trigger the TRIAC gate at the appropriate time. If the gate is triggered too early, full power will be supplied to the light bulb, causing it to glow very brightly. On the other hand, if the gate is triggered too late, the light will be very dim or may even go out. It is important to note that this dimmer design is specifically intended for use with incandescent light bulbs and may not perform well with LED lights due to inherent differences in their construction.

For this project, an oscilloscope was used to analyze signal waveforms during development, allowing verification of timing, voltage levels, and overall behavior, which supported debugging and software refinement. The complete project source code is available on [GitHub](https://github.com/workabotic/smart_dimmer_controlled_by_mobile_app) and can be cloned, reviewed, modified, or reused, with contributions and suggestions for improvement welcome.

## Materials

The components used in this project are listed below:

- 1x ESP32 microcontroller
- 1x TRIAC
- 1x 4N25 optocoupler
- 1x MOC3021 optocoupler
- 1x transformer
- 4x diodes
- 2x 100 Ω resistors
- 1x 330 Ω resistor
- 1x 10 kΩ resistor


## Methods

The main objective of this project was to trigger a TRIAC using an ESP32 microcontroller, a low-cost device capable of Wi-Fi and Bluetooth connectivity without external components. The ESP32, which features a 32-bit dual-core processor and low power consumption, was selected as a solution suitable for IoT applications and simple embedded systems. The prototype was developed using a DOIT ESP32 Devkit V1 board. The embedded firmware was implemented using the Espressif IoT Development Framework (ESP-IDF), while the Android mobile application was developed with Android Studio and used to interface with the ESP32 by sending brightness control commands.

A transformer was used to step down the mains voltage to approximately 12 V peak-to-peak AC. This reduced AC voltage was then rectified using four diodes arranged in a bridge rectifier configuration, producing an output of approximately 6 V DC. In addition to rectification, this stage provided a stable positive voltage required for the zero-crossing detection circuit, which was used to detect the zero crossings of the AC waveform. Accurate zero-crossing detection was essential for proper TRIAC timing and precise dimming control.

A 10 kΩ resistor was used as a pull-up on a microcontroller input to ensure stable logic levels. The remaining resistors specified in the materials list were employed for current limiting and protection in different parts of the circuit, ensuring that all components operated within their safe current ratings. A 4N25 optocoupler was used for zero-crossing detection and to provide electrical isolation between the microcontroller and the low-voltage DC circuitry. The MOC3021 optocoupler was used to interface with the TRIAC, providing isolation and drive capability so that the ESP32’s low-voltage control signals could safely trigger the high-voltage TRIAC and switch the AC load.

In summary, the ESP32 received zero-crossing signals from the 4N25 optocoupler, which produced a high-level output as the mains waveform approached zero. This zero-crossing instant was used as a reference to calculate the appropriate trigger delay for activating the TRIAC through the MOC3021, enabling control of the dimming operation.

The complete circuit schematic used in this project is shown below.

![Complete circuit schematic]({{ site.url }}{{ site.baseurl }}/public/images/smart-dimmer-controlled-by-mobile-app/complete_circuit_schematic.webp)
*Figure 1 — Complete circuit schematic.*

In the following sections, the circuit, along with the embedded firmware and app code, will be explained in more detail.

### Zero-crossing Detector

A zero-crossing detector circuit is essential for identifying when the supply voltage crosses zero. This information is then used to calculate the trigger time for the TRIAC in each cycle of the supply voltage waveform. The image below illustrates the zero-crossing with a sinusoidal wave and the trigger time.

![Zero-crossing and TRIAC trigger time illustration]({{ site.url }}{{ site.baseurl }}/public/images/smart-dimmer-controlled-by-mobile-app/zero-crossing_and_triac_trigger_time_illustration.webp)
*Figure 2 — Zero-crossing and TRIAC trigger time illustration.*

As the ESP32 only reads positive voltage signals below 5 V, it was necessary to reduce and rectify the input voltage read by the microcontroller. A transformer was used to step down the supply voltage, resulting in a signal that was a reduced copy of the source signal (127 V / 220 V AC) at approximately 12 V AC. Following this, a full bridge rectifier converted the negative part of the input voltage to positive, producing a signal with a maximum peak of less than 6 V DC. Additionally, a 10 kΩ resistor acted as a pull-up resistor on the microcontroller input, ensuring that it output a high signal when the power grid waveform approached zero. The circuit schematic below illustrates this setup.

![Zero-crossing detector circuit schematic]({{ site.url }}{{ site.baseurl }}/public/images/smart-dimmer-controlled-by-mobile-app/zero-crossing_detector_circuit_schematic.webp)
*Figure 3 — Zero-crossing detector circuit schematic.*

After setting up the circuit, the optocoupler output was observed in relation to the rectifier output, and its behavior matched the waveform illustrated below. The rectifier output was measured as the blue waveform, while the optocoupler output, shown in orange, was used as the input signal to the microcontroller.

![Rectifier output and 4N25 optocoupler output]({{ site.url }}{{ site.baseurl }}/public/images/smart-dimmer-controlled-by-mobile-app/rectifier_output_and_4n25_optocoupler_output.webp)
*Figure 4 — Rectifier output (blue) and 4N25 optocoupler output (orange).*

When the rectifier output approached zero, the optocoupler generated a short pulse slightly before the zero-crossing point, at a voltage below 1 V, and stopped shortly after when the voltage reached the same level. This behavior occurred because the LED inside the optocoupler ceased conduction at low voltage levels. It was observed that this pulse had a noticeable duration and needed to be taken into account when determining the zero-crossing instant. As a result, the zero-crossing time was approximated as the midpoint of the pulse produced by the optocoupler.

By connecting the optocoupler output to the ESP32, the signal logic levels were identified by the microcontroller. An algorithm was then implemented to calculate the average time between transitions from low to high logic levels and from high to low, allowing the zero-crossing time to be estimated.

The following code implemented an Interrupt Service Routine (ISR) for zero-crossing detection, capturing timestamps on both the rising and falling edges of the optocoupler signal. Because of ISR execution time constraints, the calculation of the average time between these timestamps was handled in a separate function running within a task. In addition, a timer was used to schedule a function that generated a short pulse on the output pin after the rising edge, allowing the zero-crossing time estimation to be verified.

```c
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/err.h"

/* Interrupt and GPIO */
#define ESP_INTR_FLAG_DEFAULT 0
#define INPUT_PIN GPIO_NUM_27
#define OUTPUT_PIN GPIO_NUM_33

/* Task Handle */
static TaskHandle_t task_handle = NULL;

/* Timestamps used in logic flow and trigger calculation */
static uint64_t rising_time = 0;
static uint64_t falling_time = 0;
static uint64_t zero_crossing_time = 0;

/* Powergrid sine period in us */
static uint16_t period = 0;

/* Flags used in the logic flow */
static bool is_crossing_zero = false;

/* Timer used to trigger the TRIAC */
static esp_timer_handle_t trigger_timer;

/**
 * @brief Interrupt Service Routine (ISR) for zero-crossing detection.
 *
 * This ISR is triggered on both rising and falling edges of the input signal. 
 * It manages the timing and triggering of actions based on zero-crossing 
 * events, adjusting the timer and updating the state variables accordingly.
 *
 * @param arg Not used in this implementation.
 */
void IRAM_ATTR crossing_zero_isr_handler(void *arg)
{
    esp_err_t ret;

    const uint64_t current_time = esp_timer_get_time();
    const bool current_state = gpio_get_level(INPUT_PIN);

    /* Rising edge detected */
    if (current_state && !is_crossing_zero) {
        ret = esp_timer_start_once(trigger_timer, zero_crossing_time);
        ESP_ERROR_CHECK(ret);

        /* Store the period in microseconds to calculate the trigger time */
        period = current_time - rising_time;
        rising_time = current_time;

    /* Falling edge detected */
    } else if (!current_state && is_crossing_zero) {
        /* Store the falling time to estimate the zero-crossing time */
        falling_time = current_time;
    }

    /* Update the zero-crossing state */
    is_crossing_zero = current_state;

    /* Resume a task from ISR */
    xTaskResumeFromISR(task_handle);
}

/**
 * @brief Callback function for the trigger timer.
 *
 * This function is called when the timer expires. It toggles the output 
 * pin to generate a trigger signal. After the sequence, the 
 * `is_triggering` flag is set to false.
 *
 * @param arg Not used in this implementation.
 */
static void trigger_timer_callback(void *arg)
{
    esp_err_t ret;

    /* Apply a small pulse as trigger */
    ret = gpio_set_level(OUTPUT_PIN, 1);
    ESP_ERROR_CHECK(ret);
    
    const TickType_t delay = 1 / portTICK_PERIOD_MS;
    vTaskDelay(delay);

    ret = gpio_set_level(OUTPUT_PIN, 0);
    ESP_ERROR_CHECK(ret);
}

/**
 * @brief Controls the operation of a smart dimmer system.
 *
 * This function initializes and manages the operation of a smart dimmer 
 * system. It configures timers, GPIO pins, interrupt service routines, 
 * and tasks to control the brightness of a lighting system based on 
 * zero-crossing detection. The system adjusts the brightness of the lights 
 * according to the detected brightness level and zero-crossing timing.
 *
 * @param arg Not used in this implementation.
 */
void smart_dimmer_control(void *arg)
{
    esp_err_t ret;

    /* Timer configuration structure and creation */
    const esp_timer_create_args_t trigger_timer_args = {
        .callback = &trigger_timer_callback, 
        .name = "trigger"
    };

    ret = esp_timer_create(&trigger_timer_args, &trigger_timer);
    ESP_ERROR_CHECK(ret);

    /* Configure GPIO input */
    esp_rom_gpio_pad_select_gpio(INPUT_PIN);
    ret = gpio_set_direction(INPUT_PIN, GPIO_MODE_INPUT);
    ESP_ERROR_CHECK(ret);

    ret = gpio_set_intr_type(INPUT_PIN, GPIO_INTR_ANYEDGE);
    ESP_ERROR_CHECK(ret);

    ret = gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    ESP_ERROR_CHECK(ret);

    /* Add the ISR handler for the specified GPIO pin */
    ret = gpio_isr_handler_add(INPUT_PIN, crossing_zero_isr_handler, NULL);
    ESP_ERROR_CHECK(ret);

    /* Configure GPIO output */
    ret = gpio_set_direction(OUTPUT_PIN, GPIO_MODE_OUTPUT);
    ESP_ERROR_CHECK(ret);

    /* Infinite loop */
    for (;;) {
        /* Suspend the task until it is resumed externally */
        vTaskSuspend(NULL);
        
        if (!is_crossing_zero && rising_time != 0 && falling_time != 0) {
            /* Calculate zero-crossing time */
            zero_crossing_time = (uint64_t)((falling_time - rising_time) / 2);
        }
    }
}

void app_main(void)
{
    /* Run the trigger configuration and calculations in a dedicated core */
    xTaskCreate(smart_dimmer_control, "smart_dimmer_control",
                configMINIMAL_STACK_SIZE, NULL,
                configMAX_PRIORITIES - 1, &task_handle);
}
```

Once the implementation was completed and tested, the zero-crossing estimate trigger was observed to align with the midpoint of the pulse generated by the optocoupler, as desired, and to closely match the actual zero of the rectifier output, as shown in the following figures.

![Optocoupler output and ESP32 Trigger output]({{ site.url }}{{ site.baseurl }}/public/images/smart-dimmer-controlled-by-mobile-app/optocoupler_output_and_esp32_trigger_at_the_zero_crossing.webp)
*Figure 5 — Optocoupler output (blue) and ESP32 Trigger at the zero-crossing (orange).*


![Rectifier output and ESP32 Trigger at the zero-crossing]({{ site.url }}{{ site.baseurl }}/public/images/smart-dimmer-controlled-by-mobile-app/rectifier_output_and_esp32_trigger_at_the_zero_crossing.webp)
*Figure 6 — Rectifier output (blue) and ESP32 Trigger at the zero-crossing (orange).*

With the zero-crossing detection verified, the next focus shifted to triggering the TRIAC at precise points after each zero-crossing to control the light’s brightness.

### Triggering the TRIAC

The next step was to trigger the TRIAC at a desired time after the input voltage crossed zero. This step was essential for the dimmer circuit, which relied on phase control to adjust the amount of power delivered to the load. In this approach, the TRIAC was triggered at a specific point within each AC cycle, effectively removing a portion of the waveform. By varying the trigger point, the portion of the AC waveform applied to the load was controlled, thereby regulating the delivered power.

The trigger time was expressed in terms of brightness, where lower brightness values corresponded to later TRIAC trigger times and, consequently, to a reduced current supplied to the load. Since the frequency of the electrical network corresponds to the period of the sinusoidal supply signal, it was also taken into account that the zero-crossing detector operated twice per cycle, effectively doubling the detection frequency. Therefore, the brightness value, defined in the range from 0% to 100%, was converted into the appropriate trigger time to achieve the desired dimming effect. As a result, the trigger time $$t$$ was expressed as a function of the brightness $$b$$ and the frequency $$f$$ using the following relation:

$$
t(b,f) = \left(1 - \frac{b}{100}\right)\left(\frac{10^6}{2f}\right)
$$

In this expression, the trigger time $$t$$ was given in microseconds, the brightness value $$b$$ ranged from 0 to 100, and $$f$$ represented the frequency of the electrical network in hertz. Since frequency is the inverse of the period, the equation was also expressed in terms of the signal period instead of frequency.

To implement this expression in the code, the trigger time calculation was performed after each zero-crossing detection, taking the selected brightness value into account. First, the period of the electrical network was estimated by subtracting the previously recorded rising edge timestamp from the current one within the ISR. The trigger time was then calculated in an auxiliary function, since floating-point operations were constrained inside the ISR. Finally, still within the ISR, the timer scheduled after the rising edge was configured using the computed trigger time.

The code employing these adjustments was structured as follows, assuming a brightness setting of 50%:

```c
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/err.h"

/* Interrupt and GPIO */
#define ESP_INTR_FLAG_DEFAULT 0
#define INPUT_PIN GPIO_NUM_27
#define OUTPUT_PIN GPIO_NUM_33

/* Task Handle */
static TaskHandle_t task_handle = NULL;

/* Timestamps used in logic flow and trigger calculation */
static uint64_t rising_time = 0;
static uint64_t falling_time = 0;
static uint64_t zero_crossing_time = 0;
static uint64_t trigger_time = 0;

/* Powergrid sine period in us */
static uint16_t period = 0;

/* Brightness intensity in percentage */
static uint8_t brightness = 50;

/* Flags used in the logic flow */
static bool is_crossing_zero = false;

/* Timer used to trigger the TRIAC */
static esp_timer_handle_t trigger_timer;

/**
 * @brief Interrupt Service Routine (ISR) for zero-crossing detection.
 *
 * This ISR is triggered on both rising and falling edges of the input signal. 
 * It manages the timing and triggering of actions based on zero-crossing 
 * events, adjusting the timer and updating the state variables accordingly.
 *
 * @param arg Not used in this implementation.
 */
void IRAM_ATTR crossing_zero_isr_handler(void *arg)
{
    esp_err_t ret;

    const uint64_t current_time = esp_timer_get_time();
    const bool current_state = gpio_get_level(INPUT_PIN);

    /* Rising edge detected */
    if (current_state && !is_crossing_zero) {
        /* If the trigger isn't in the dead zone */
        if (trigger_time < period) {
            ret = esp_timer_start_once(trigger_timer, trigger_time);
            ESP_ERROR_CHECK(ret);
        }

        /* Store the period in microseconds to calculate the trigger time */
        period = current_time - rising_time;
        rising_time = current_time;

    /* Falling edge detected */
    } else if (!current_state && is_crossing_zero) {
        /* Store the falling time to estimate the zero-crossing time */
        falling_time = current_time;
    }

    /* Update the zero-crossing state */
    is_crossing_zero = current_state;

    /* Resume a task from ISR */
    xTaskResumeFromISR(task_handle);
}

/**
 * @brief Callback function for the trigger timer.
 *
 * This function is called when the timer expires. It toggles the output 
 * pin to generate a trigger signal. After the sequence, the 
 * `is_triggering` flag is set to false.
 *
 * @param arg Not used in this implementation.
 */
static void trigger_timer_callback(void *arg)
{
    esp_err_t ret;

    /* Apply a small pulse as trigger */
    ret = gpio_set_level(OUTPUT_PIN, 1);
    ESP_ERROR_CHECK(ret);
    
    const TickType_t delay = 1 / portTICK_PERIOD_MS;
    vTaskDelay(delay);

    ret = gpio_set_level(OUTPUT_PIN, 0);
    ESP_ERROR_CHECK(ret);
}

/**
 * @brief Controls the operation of a smart dimmer system.
 *
 * This function initializes and manages the operation of a smart dimmer 
 * system. It configures timers, GPIO pins, interrupt service routines, 
 * and tasks to control the brightness of a lighting system based on 
 * zero-crossing detection. The system adjusts the brightness of the lights 
 * according to the detected brightness level and zero-crossing timing.
 *
 * @param arg Not used in this implementation.
 */
void smart_dimmer_control(void *arg)
{
    esp_err_t ret;

    /* Timer configuration structure and creation */
    const esp_timer_create_args_t trigger_timer_args = {
        .callback = &trigger_timer_callback, 
        .name = "trigger"
    };

    ret = esp_timer_create(&trigger_timer_args, &trigger_timer);
    ESP_ERROR_CHECK(ret);

    /* Configure GPIO input */
    esp_rom_gpio_pad_select_gpio(INPUT_PIN);
    ret = gpio_set_direction(INPUT_PIN, GPIO_MODE_INPUT);
    ESP_ERROR_CHECK(ret);

    ret = gpio_set_intr_type(INPUT_PIN, GPIO_INTR_ANYEDGE);
    ESP_ERROR_CHECK(ret);

    ret = gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    ESP_ERROR_CHECK(ret);

    /* Add the ISR handler for the specified GPIO pin */
    ret = gpio_isr_handler_add(INPUT_PIN, crossing_zero_isr_handler, NULL);
    ESP_ERROR_CHECK(ret);

    /* Configure GPIO output */
    ret = gpio_set_direction(OUTPUT_PIN, GPIO_MODE_OUTPUT);
    ESP_ERROR_CHECK(ret);

    /* Infinite loop */
    for (;;) {
        /* Suspend the task until it is resumed externally */
        vTaskSuspend(NULL);

        if (is_crossing_zero) {
            /* Calculate trigger time based on zero-crossing detection */
            trigger_time = (uint64_t)((1 - brightness / 100.0f) * period +
                                      zero_crossing_time);

        } else if (!is_crossing_zero && rising_time != 0 && falling_time != 0) {
            /* Calculate zero-crossing time */
            zero_crossing_time = (uint64_t)((falling_time - rising_time) / 2);
        }
    }
}

void app_main(void)
{
    /* Run the trigger configuration and calculations in a dedicated core */
    xTaskCreate(smart_dimmer_control, "smart_dimmer_control",
                configMINIMAL_STACK_SIZE, NULL,
                configMAX_PRIORITIES - 1, &task_handle);
}
```

Once the code was successfully compiled and deployed on the microcontroller, the pulse generated by the ESP32 was observed to align with the peak of the rectifier waveform, thereby remaining synchronized with the power supply waveform, as shown in the image below.

![Rectifier output and ESP32 trigger at the half of the brightness]({{ site.url }}{{ site.baseurl }}/public/images/smart-dimmer-controlled-by-mobile-app/rectifier_output_and_esp32_trigger_at_half_brightness.webp)
*Figure 7 — Rectifier output (blue) and ESP32 trigger at the half of the brightness (orange).*

However, a small adjustment was made to the previous code to ensure proper operation by extending the TRIAC triggering beyond the calculated trigger time until the next zero-crossing detection. This modification prevented issues related to high brightness levels and potential timing errors in the zero-crossing estimation.

For example, when the brightness was set close to 100% and the zero-crossing was estimated to occur earlier than it actually did, the TRIAC could be triggered prematurely. This situation could lead to an insufficient current being supplied to the load, since the TRIAC would stop conducting immediately after the actual zero-crossing. By keeping the TRIAC in the triggered state until the next rising edge of the zero-crossing detector, it was ensured that it remained conducting long enough to supply the required current for the selected brightness, effectively compensating for timing inaccuracies. The code implementing these adjustments is shown below:

```c
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/err.h"

/* Interrupt and GPIO */
#define ESP_INTR_FLAG_DEFAULT 0
#define INPUT_PIN GPIO_NUM_27
#define OUTPUT_PIN GPIO_NUM_33

/* Task Handle */
static TaskHandle_t task_handle = NULL;

/* Timestamps used in logic flow and trigger calculation */
static uint64_t rising_time = 0;
static uint64_t falling_time = 0;
static uint64_t zero_crossing_time = 0;
static uint64_t trigger_time = 0;

/* Powergrid sine period in us */
static uint16_t period = 0;

/* Brightness intensity in percentage */
static uint8_t brightness = 50;

/* Flags used in the logic flow */
static bool is_crossing_zero = false;
static bool is_triggering = false;

/* Timer used to trigger the TRIAC */
static esp_timer_handle_t trigger_timer;

/**
 * @brief Interrupt Service Routine (ISR) for zero-crossing detection.
 *
 * This ISR is triggered on both rising and falling edges of the input signal. 
 * It manages the timing and triggering of actions based on zero-crossing 
 * events, adjusting the timer and updating the state variables accordingly.
 *
 * @param arg Not used in this implementation.
 */
void IRAM_ATTR crossing_zero_isr_handler(void *arg)
{
    esp_err_t ret;

    const uint64_t current_time = esp_timer_get_time();
    const bool current_state = gpio_get_level(INPUT_PIN);

    /* Rising edge detected */
    if (current_state && !is_crossing_zero) {
        /* Turn off the active trigger */
        ret = gpio_set_level(OUTPUT_PIN, 0);
        ESP_ERROR_CHECK(ret);

        /* If there is no trigger active and trigger isn't in the dead zone */
        if (!is_triggering && trigger_time < period) {
            ret = esp_timer_start_once(trigger_timer, trigger_time);
            ESP_ERROR_CHECK(ret);

            is_triggering = true;
        }

        /* Store the period in microseconds to calculate the trigger time */
        period = current_time - rising_time;
        rising_time = current_time;

    /* Falling edge detected */
    } else if (!current_state && is_crossing_zero) {
        /* Store the falling time to estimate the zero-crossing time */
        falling_time = current_time;
    }

    /* Update the zero-crossing state */
    is_crossing_zero = current_state;

    /* Resume a task from ISR */
    xTaskResumeFromISR(task_handle);
}

/**
 * @brief Callback function for the trigger timer.
 *
 * This function is called when the timer expires. It toggles the output 
 * pin to generate a trigger signal. After the sequence, the 
 * `is_triggering` flag is set to false.
 *
 * @param arg Not used in this implementation.
 */
static void trigger_timer_callback(void *arg)
{
    esp_err_t ret;

    /* Activate the trigger */
    ret = gpio_set_level(OUTPUT_PIN, 1);
    ESP_ERROR_CHECK(ret);
    is_triggering = false;
}

/**
 * @brief Controls the operation of a smart dimmer system.
 *
 * This function initializes and manages the operation of a smart dimmer 
 * system. It configures timers, GPIO pins, interrupt service routines, 
 * and tasks to control the brightness of a lighting system based on 
 * zero-crossing detection. The system adjusts the brightness of the lights 
 * according to the detected brightness level and zero-crossing timing.
 *
 * @param arg Not used in this implementation.
 */
void smart_dimmer_control(void *arg)
{
    esp_err_t ret;

    /* Timer configuration structure and creation */
    const esp_timer_create_args_t trigger_timer_args = {
        .callback = &trigger_timer_callback, 
        .name = "trigger"
    };

    ret = esp_timer_create(&trigger_timer_args, &trigger_timer);
    ESP_ERROR_CHECK(ret);

    /* Configure GPIO input */
    esp_rom_gpio_pad_select_gpio(INPUT_PIN);
    ret = gpio_set_direction(INPUT_PIN, GPIO_MODE_INPUT);
    ESP_ERROR_CHECK(ret);

    ret = gpio_set_intr_type(INPUT_PIN, GPIO_INTR_ANYEDGE);
    ESP_ERROR_CHECK(ret);

    ret = gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    ESP_ERROR_CHECK(ret);

    /* Add the ISR handler for the specified GPIO pin */
    ret = gpio_isr_handler_add(INPUT_PIN, crossing_zero_isr_handler, NULL);
    ESP_ERROR_CHECK(ret);

    /* Configure GPIO output */
    ret = gpio_set_direction(OUTPUT_PIN, GPIO_MODE_OUTPUT);
    ESP_ERROR_CHECK(ret);

    /* Infinite loop */
    for (;;) {
        /* Suspend the task until it is resumed externally */
        vTaskSuspend(NULL);

        if (is_crossing_zero) {
            /* Calculate trigger time based on zero-crossing detection */
            trigger_time = (uint64_t)((1 - brightness / 100.0f) * period +
                                      zero_crossing_time);

        } else if (!is_crossing_zero && rising_time != 0 && falling_time != 0) {
            /* Calculate zero-crossing time */
            zero_crossing_time = (uint64_t)((falling_time - rising_time) / 2);
        }
    }
}

void app_main(void)
{
    /* Run the trigger configuration and calculations in a dedicated core */
    xTaskCreate(smart_dimmer_control, "smart_dimmer_control",
                configMINIMAL_STACK_SIZE, NULL,
                configMAX_PRIORITIES - 1, &task_handle);
}
```

The following waveform illustrated the result of these adjustments in the triggering code. It was observed that the trigger started at 50% of the rectifier waveform but only ended at the rising edge of the optocoupler zero-crossing detection. This behavior ensured that the TRIAC remained conducting and supplied adequate current to the load for the entire duration corresponding to the intended brightness level.

![Optocoupler output and ESP32 Trigger output adjusted]({{ site.url }}{{ site.baseurl }}/public/images/smart-dimmer-controlled-by-mobile-app/optocoupler_output_and_esp32_trigger_adjusted.webp)
*Figure 8 — Optocoupler output (blue) and ESP32 Trigger adjusted (orange).*

In the following image, with the brightness value set to 100%, the trigger was observed to initiate precisely at the zero-crossing and to continue until the rising edge of the optocoupler zero-crossing detection.

![Optocoupler output and ESP32 Trigger output adjusted at 100% brightness]({{ site.url }}{{ site.baseurl }}/public/images/smart-dimmer-controlled-by-mobile-app/optocoupler_output_and_esp32_trigger_adjusted_100.webp)
*Figure 9 — Optocoupler output (blue) and ESP32 Trigger adjusted at 100% brightness (orange).*

At this stage, it was possible to accurately control the TRIAC triggering instant to adjust the light brightness as required. To achieve this, the microcontroller was electrically isolated using an optocoupler, which was used to drive and trigger the TRIAC, as illustrated in the following circuit:

![TRIAC trigger circuit schematic]({{ site.url }}{{ site.baseurl }}/public/images/smart-dimmer-controlled-by-mobile-app/triac_trigger_circuit_schematic.webp)
*Figure 10 — TRIAC trigger circuit schematic.*

Therefore, when the brightness value was set to 50% and the complete circuit was connected with the light bulb as the load, the resulting waveform observed at the TRIAC was as illustrated below:

![Power supply and TRIAC output with brightness value at 50%]({{ site.url }}{{ site.baseurl }}/public/images/smart-dimmer-controlled-by-mobile-app/power_supply_and_triac_output_with_brightness_at_50.webp)
*Figure 11 — Power supply (blue) and TRIAC output (orange) with brightness value at 50%.*

It was observed that the light brightness was reduced compared to its conventional operation. When the brightness parameter was set to 25%, the light intensity decreased further, and the TRIAC output exhibited the corresponding waveform shown next:

![Power supply and TRIAC output with brightness value at 25%]({{ site.url }}{{ site.baseurl }}/public/images/smart-dimmer-controlled-by-mobile-app/power_supply_and_triac_output_with_brightness_at_25.webp)
*Figure 12 — Power supply (blue) and TRIAC output (orange) with brightness value at 25%.*

When the brightness was set to 75%, the light intensity increased, and the resulting waveform is shown below:

![Power supply and TRIAC output with brightness value at 75%]({{ site.url }}{{ site.baseurl }}/public/images/smart-dimmer-controlled-by-mobile-app/power_supply_and_triac_output_with_brightness_at_75.webp)
*Figure 13 — Power supply (blue) and TRIAC output (orange) with brightness value at 75%.*

The ESP32 firmware successfully controlled the light brightness. However, in this implementation, changing the brightness level required recompiling and reuploading the code to the microcontroller. As a result, the next phase of the project focused on modifying the implementation to allow real-time brightness adjustments through network connectivity, removing the need for recompilation.

## Wifi Access Point and Web Server

To enable dynamic brightness adjustment, the ESP32 was configured to control the TRIAC over a network connection. This required setting up the ESP32 to connect via Wi-Fi and host a web server capable of receiving the desired brightness value. To accomplish this, a Wi-Fi access point was established, allowing users to connect directly to the ESP32 and communicate with its web server. In addition, a fixed IP address was assigned to the ESP32 to simplify access to the server. 

The following code shows how the access point and fixed IP configuration were implemented on the ESP32:

```c
#include <netdb.h>
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "lwip/err.h"
#include "nvs_flash.h"

/* Wifi Config */
#define WIFI_SSID "DIMMER"
#define WIFI_PASS "password"
#define WIFI_CHANNEL 1
#define MAX_STA_CONN 1

/* IP config for the WiFi AP */
#define STATIC_IP_ADDR "192.168.1.1"
#define GATEWAY_ADDR "192.168.1.1"
#define NETMASK_ADDR "255.255.255.0"

/**
 * @brief Initializes the Wi-Fi access point (AP) mode with static IP 
 * configuration.
 * 
 * @return void
 */
static void wifi_ap_init(void)
{
    esp_err_t ret;

    /* Initialize the network interface */
    ret = esp_netif_init();
    ESP_ERROR_CHECK(ret);

    /* Create the default event loop */
    ret = esp_event_loop_create_default();
    ESP_ERROR_CHECK(ret);

    /* Create the default Wi-Fi AP network interface */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    ESP_ERROR_CHECK(ret);

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .channel = WIFI_CHANNEL,
            .password = WIFI_PASS,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };

    /* Initializes the Wi-Fi driver with the default configuration */
    ret = esp_wifi_set_mode(WIFI_MODE_AP);
    ESP_ERROR_CHECK(ret);

    ret = esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    ESP_ERROR_CHECK(ret);

    /* Set static IP information */
    esp_netif_ip_info_t ip_info;

    memset(&ip_info, 0, sizeof(esp_netif_ip_info_t));

    ip_info.ip.addr = ipaddr_addr(STATIC_IP_ADDR);
    ip_info.gw.addr = ipaddr_addr(GATEWAY_ADDR);
    ip_info.netmask.addr = ipaddr_addr(NETMASK_ADDR);

    /* Keep the netif instance for setting IP info */
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();

    /* Stop DHCP server before setting static IP info */
    ret = esp_netif_dhcps_stop(ap_netif);
    ESP_ERROR_CHECK(ret);

    ret = esp_netif_set_ip_info(ap_netif, &ip_info);
    ESP_ERROR_CHECK(ret);

    /* Start DHCP server to assign IPs to other connected devices */
    ret = esp_netif_dhcps_start(ap_netif);
    ESP_ERROR_CHECK(ret);

    ret = esp_wifi_start();
    ESP_ERROR_CHECK(ret);
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {

        ret = nvs_flash_erase();
        ESP_ERROR_CHECK(ret);
        
        ret = nvs_flash_init();
        ESP_ERROR_CHECK(ret);
    }

    /* Initialize WiFi in AP mode */
    wifi_ap_init();
}
```

Once the code was successfully executed on the ESP32, a new access point appeared among the available Wi-Fi networks, with the microcontroller assigned a fixed IP address. After confirming that this step worked as expected, the microcontroller was available for communication, allowing the web server to be set up to receive data.

To achieve this, one function was used to initialize the web server and another to handle incoming requests, as shown in the following code snippet. The **http_server_init** function started the server using default settings and registered a handler for HTTP GET requests at the root endpoint (**'/'**). The **http_request_handler** function processed incoming requests by extracting the brightness parameter from the URL query string and converting it into an integer within the range of 0 to 100. The current brightness value was then formatted into a response buffer and sent back to the client as the HTTP response. This is shown in the next code section:

```c
#include <netdb.h>
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "lwip/err.h"
#include "nvs_flash.h"

/* Wifi Config */
#define WIFI_SSID "DIMMER"
#define WIFI_PASS "password"
#define WIFI_CHANNEL 1
#define MAX_STA_CONN 1

/* IP config for the WiFi AP */
#define STATIC_IP_ADDR "192.168.1.1"
#define GATEWAY_ADDR "192.168.1.1"
#define NETMASK_ADDR "255.255.255.0"

/* Brightness intensity in percentage */
static uint8_t brightness = 0;

/**
 * @brief Handles HTTP GET requests, extracts a "brightness" query parameter, 
 * and responds with the brightness value.
 *
 * This function processes incoming HTTP GET requests to extract the 
 * "brightness" parameter from the URL query string.
 *
 * @param req Pointer to the HTTP request.
 * 
 * @return ESP_OK on success.
 */
static esp_err_t http_request_handler(httpd_req_t *req)
{
    esp_err_t ret;

    char buffer[16];
    size_t buffer_length;

    buffer_length = httpd_req_get_url_query_len(req) + 1;

    /* Extracts the query string into a buffer */
    if ((buffer_length > 1) && (buffer_length <= sizeof(buffer))) {

        ret = httpd_req_get_url_query_str(req, buffer, buffer_length);
        
        if (ret == ESP_OK) {
            char param[5] = { 0 };

            ret = httpd_query_key_value(buffer, "brightness", 
                                        param, sizeof(param));

            /* Converts the "brightness" parameter value to an integer */
            if (ret == ESP_OK) {
                int value = atoi(param);

                /* Ensure value is within range 0-100 */
                if (value > 100) {
                    brightness = 100;
                } else if (value < 0) {
                    brightness = 0;
                } else {
                    brightness = (uint8_t)value;
                }
            }
        }
    }

    /* Buffer to hold brightness value (up to 3 digits + null terminator) */
    char response_buffer[5];

    snprintf(response_buffer, sizeof(response_buffer), "%d", brightness);

    /* Send current brightness value as response */
    ret = httpd_resp_send(req, response_buffer, strlen(response_buffer));
    ESP_ERROR_CHECK(ret);

    return ESP_OK;
}

/**
 * @brief Initializes and starts the HTTP server.
 *
 * @return void
 */
static void http_server_init(void)
{
    esp_err_t ret;

    /* Creates an HTTP server handle and default server configuration */
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root_uri = { 
            .uri = "/",
            .method = HTTP_GET,
            .handler = http_request_handler,
            .user_ctx = NULL 
        };

        /* Registers a URI handler for the root ("/") endpoint */
        ret = httpd_register_uri_handler(server, &root_uri);
        ESP_ERROR_CHECK(ret);

    } else {
        ESP_LOGE("HTTP_SERVER", "Failed to start server");
    }
}

/**
 * @brief Initializes the Wi-Fi access point (AP) mode with static IP 
 * configuration.
 * 
 * @return void
 */
static void wifi_ap_init(void)
{
    esp_err_t ret;

    /* Initialize the network interface */
    ret = esp_netif_init();
    ESP_ERROR_CHECK(ret);

    /* Create the default event loop */
    ret = esp_event_loop_create_default();
    ESP_ERROR_CHECK(ret);

    /* Create the default Wi-Fi AP network interface */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    ESP_ERROR_CHECK(ret);

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .channel = WIFI_CHANNEL,
            .password = WIFI_PASS,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };

    /* Initializes the Wi-Fi driver with the default configuration */
    ret = esp_wifi_set_mode(WIFI_MODE_AP);
    ESP_ERROR_CHECK(ret);

    ret = esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    ESP_ERROR_CHECK(ret);

    /* Set static IP information */
    esp_netif_ip_info_t ip_info;

    memset(&ip_info, 0, sizeof(esp_netif_ip_info_t));

    ip_info.ip.addr = ipaddr_addr(STATIC_IP_ADDR);
    ip_info.gw.addr = ipaddr_addr(GATEWAY_ADDR);
    ip_info.netmask.addr = ipaddr_addr(NETMASK_ADDR);

    /* Keep the netif instance for setting IP info */
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();

    /* Stop DHCP server before setting static IP info */
    ret = esp_netif_dhcps_stop(ap_netif);
    ESP_ERROR_CHECK(ret);

    ret = esp_netif_set_ip_info(ap_netif, &ip_info);
    ESP_ERROR_CHECK(ret);

    /* Start DHCP server to assign IPs to other connected devices */
    ret = esp_netif_dhcps_start(ap_netif);
    ESP_ERROR_CHECK(ret);

    ret = esp_wifi_start();
    ESP_ERROR_CHECK(ret);
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {

        ret = nvs_flash_erase();
        ESP_ERROR_CHECK(ret);
        
        ret = nvs_flash_init();
        ESP_ERROR_CHECK(ret);
    }

    /* Initialize WiFi in AP mode */
    wifi_ap_init();

    /* Initialize the HTTP server */
    http_server_init();
}
```

By merging the TRIAC triggering logic with the Wi-Fi access point and web server implementation, the light brightness was effectively adjusted over the network by sending brightness parameters to the web server. These parameters were decoded by the server and used to update the brightness accordingly. To ensure reliable operation, the ESP32’s dual-core architecture was utilized by running the TRIAC control task on a different core from the web server. This separation prevented one function from blocking the other, which was essential for maintaining precise TRIAC triggering. Without this approach, delays introduced by the web server could have interfered with the TRIAC control timing and affected the resulting output.

To achieve this, the RTOS function **xTaskCreatePinnedToCore** was used to assign the TRIAC dimmer control task to a secondary core, while the web server handler continued to run on the main core, as demonstrated in the following complete code:

```c
#include <netdb.h>
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/err.h"
#include "nvs_flash.h"

/* Wifi Config */
#define WIFI_SSID "DIMMER"
#define WIFI_PASS "password"
#define WIFI_CHANNEL 1
#define MAX_STA_CONN 1

/* IP config for the WiFi AP */
#define STATIC_IP_ADDR "192.168.1.1"
#define GATEWAY_ADDR "192.168.1.1"
#define NETMASK_ADDR "255.255.255.0"

/* Interrupt and GPIO */
#define ESP_INTR_FLAG_DEFAULT 0
#define INPUT_PIN GPIO_NUM_27
#define OUTPUT_PIN GPIO_NUM_33

/* Task Handle */
static TaskHandle_t task_handle = NULL;

/* Timestamps used in logic flow and trigger calculation */
static uint64_t rising_time = 0;
static uint64_t falling_time = 0;
static uint64_t zero_crossing_time = 0;
static uint64_t trigger_time = 0;

/* Powergrid sine period in us */
static uint16_t period = 0;

/* Brightness intensity in percentage */
static uint8_t brightness = 0;

/* Flags used in the logic flow */
static bool is_crossing_zero = false;
static bool is_triggering = false;

/* Timer used to trigger the TRIAC */
static esp_timer_handle_t trigger_timer;

/**
 * @brief Interrupt Service Routine (ISR) for zero-crossing detection.
 *
 * This ISR is triggered on both rising and falling edges of the input signal. 
 * It manages the timing and triggering of actions based on zero-crossing 
 * events, adjusting the timer and updating the state variables accordingly.
 *
 * @param arg Not used in this implementation.
 */
void IRAM_ATTR crossing_zero_isr_handler(void *arg)
{
    esp_err_t ret;

    const uint64_t current_time = esp_timer_get_time();
    const bool current_state = gpio_get_level(INPUT_PIN);

    /* Rising edge detected */
    if (current_state && !is_crossing_zero) {
        /* Turn off the active trigger */
        ret = gpio_set_level(OUTPUT_PIN, 0);
        ESP_ERROR_CHECK(ret);

        /* If there is no trigger active and trigger isn't in the dead zone */
        if (!is_triggering && trigger_time < period) {
            ret = esp_timer_start_once(trigger_timer, trigger_time);
            ESP_ERROR_CHECK(ret);

            is_triggering = true;
        }

        /* Store the period in microseconds to calculate the trigger time */
        period = current_time - rising_time;
        rising_time = current_time;

    /* Falling edge detected */
    } else if (!current_state && is_crossing_zero) {
        /* Store the falling time to estimate the zero-crossing time */
        falling_time = current_time;
    }

    /* Update the zero-crossing state */
    is_crossing_zero = current_state;

    /* Resume a task from ISR */
    xTaskResumeFromISR(task_handle);
}

/**
 * @brief Callback function for the trigger timer.
 *
 * This function is called when the timer expires. It toggles the output 
 * pin to generate a trigger signal. After the sequence, the 
 * `is_triggering` flag is set to false.
 *
 * @param arg Not used in this implementation.
 */
static void trigger_timer_callback(void *arg)
{
    esp_err_t ret;

    /* Activate the trigger */
    ret = gpio_set_level(OUTPUT_PIN, 1);
    ESP_ERROR_CHECK(ret);
    is_triggering = false;
}

/**
 * @brief Controls the operation of a smart dimmer system.
 *
 * This function initializes and manages the operation of a smart dimmer 
 * system. It configures timers, GPIO pins, interrupt service routines, 
 * and tasks to control the brightness of a lighting system based on 
 * zero-crossing detection. The system adjusts the brightness of the lights 
 * according to the detected brightness level and zero-crossing timing.
 *
 * @param arg Not used in this implementation.
 */
void smart_dimmer_control(void *arg)
{
    esp_err_t ret;

    /* Timer configuration structure and creation */
    const esp_timer_create_args_t trigger_timer_args = {
        .callback = &trigger_timer_callback, 
        .name = "trigger"
    };

    ret = esp_timer_create(&trigger_timer_args, &trigger_timer);
    ESP_ERROR_CHECK(ret);

    /* Configure GPIO input */
    esp_rom_gpio_pad_select_gpio(INPUT_PIN);
    ret = gpio_set_direction(INPUT_PIN, GPIO_MODE_INPUT);
    ESP_ERROR_CHECK(ret);

    ret = gpio_set_intr_type(INPUT_PIN, GPIO_INTR_ANYEDGE);
    ESP_ERROR_CHECK(ret);

    ret = gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    ESP_ERROR_CHECK(ret);

    /* Add the ISR handler for the specified GPIO pin */
    ret = gpio_isr_handler_add(INPUT_PIN, crossing_zero_isr_handler, NULL);
    ESP_ERROR_CHECK(ret);

    /* Configure GPIO output */
    ret = gpio_set_direction(OUTPUT_PIN, GPIO_MODE_OUTPUT);
    ESP_ERROR_CHECK(ret);

    /* Infinite loop */
    for (;;) {
        /* Suspend the task until it is resumed externally */
        vTaskSuspend(NULL);

        if (is_crossing_zero) {
            /* Calculate trigger time based on zero-crossing detection */
            trigger_time = (uint64_t)((1 - brightness / 100.0f) * period +
                                      zero_crossing_time);

        } else if (!is_crossing_zero && rising_time != 0 && falling_time != 0) {
            /* Calculate zero-crossing time */
            zero_crossing_time = (uint64_t)((falling_time - rising_time) / 2);
        }
    }
}

/**
 * @brief Handles HTTP GET requests, extracts a "brightness" query parameter, 
 * and responds with the brightness value.
 *
 * This function processes incoming HTTP GET requests to extract the 
 * "brightness" parameter from the URL query string.
 *
 * @param req Pointer to the HTTP request.
 * 
 * @return ESP_OK on success.
 */
static esp_err_t http_request_handler(httpd_req_t *req)
{
    esp_err_t ret;

    char buffer[16];
    size_t buffer_length;

    buffer_length = httpd_req_get_url_query_len(req) + 1;

    /* Extracts the query string into a buffer */
    if ((buffer_length > 1) && (buffer_length <= sizeof(buffer))) {

        ret = httpd_req_get_url_query_str(req, buffer, buffer_length);
        
        if (ret == ESP_OK) {
            char param[5] = { 0 };

            ret = httpd_query_key_value(buffer, "brightness", 
                                        param, sizeof(param));

            /* Converts the "brightness" parameter value to an integer */
            if (ret == ESP_OK) {
                int value = atoi(param);

                /* Ensure value is within range 0-100 */
                if (value > 100) {
                    brightness = 100;
                } else if (value < 0) {
                    brightness = 0;
                } else {
                    brightness = (uint8_t)value;
                }
            }
        }
    }

    /* Buffer to hold brightness value (up to 3 digits + null terminator) */
    char response_buffer[5];

    snprintf(response_buffer, sizeof(response_buffer), "%d", brightness);

    /* Send current brightness value as response */
    ret = httpd_resp_send(req, response_buffer, strlen(response_buffer));
    ESP_ERROR_CHECK(ret);

    return ESP_OK;
}

/**
 * @brief Initializes and starts the HTTP server.
 *
 * @return void
 */
static void http_server_init(void)
{
    esp_err_t ret;

    /* Creates an HTTP server handle and default server configuration */
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root_uri = { 
            .uri = "/",
            .method = HTTP_GET,
            .handler = http_request_handler,
            .user_ctx = NULL 
        };

        /* Registers a URI handler for the root ("/") endpoint */
        ret = httpd_register_uri_handler(server, &root_uri);
        ESP_ERROR_CHECK(ret);

    } else {
        ESP_LOGE("HTTP_SERVER", "Failed to start server");
    }
}

/**
 * @brief Initializes the Wi-Fi access point (AP) mode with static IP 
 * configuration.
 * 
 * @return void
 */
static void wifi_ap_init(void)
{
    esp_err_t ret;

    /* Initialize the network interface */
    ret = esp_netif_init();
    ESP_ERROR_CHECK(ret);

    /* Create the default event loop */
    ret = esp_event_loop_create_default();
    ESP_ERROR_CHECK(ret);

    /* Create the default Wi-Fi AP network interface */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    ESP_ERROR_CHECK(ret);

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .channel = WIFI_CHANNEL,
            .password = WIFI_PASS,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };

    /* Initializes the Wi-Fi driver with the default configuration */
    ret = esp_wifi_set_mode(WIFI_MODE_AP);
    ESP_ERROR_CHECK(ret);

    ret = esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    ESP_ERROR_CHECK(ret);

    /* Set static IP information */
    esp_netif_ip_info_t ip_info;

    memset(&ip_info, 0, sizeof(esp_netif_ip_info_t));

    ip_info.ip.addr = ipaddr_addr(STATIC_IP_ADDR);
    ip_info.gw.addr = ipaddr_addr(GATEWAY_ADDR);
    ip_info.netmask.addr = ipaddr_addr(NETMASK_ADDR);

    /* Keep the netif instance for setting IP info */
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();

    /* Stop DHCP server before setting static IP info */
    ret = esp_netif_dhcps_stop(ap_netif);
    ESP_ERROR_CHECK(ret);

    ret = esp_netif_set_ip_info(ap_netif, &ip_info);
    ESP_ERROR_CHECK(ret);

    /* Start DHCP server to assign IPs to other connected devices */
    ret = esp_netif_dhcps_start(ap_netif);
    ESP_ERROR_CHECK(ret);

    ret = esp_wifi_start();
    ESP_ERROR_CHECK(ret);
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {

        ret = nvs_flash_erase();
        ESP_ERROR_CHECK(ret);
        
        ret = nvs_flash_init();
        ESP_ERROR_CHECK(ret);
    }

    /* Initialize WiFi in AP mode */
    wifi_ap_init();

    /* Initialize the HTTP server */
    http_server_init();

    /* Run the trigger configuration and calculations in a dedicated core */
    xTaskCreatePinnedToCore(smart_dimmer_control, "smart_dimmer_control",
                            configMINIMAL_STACK_SIZE, NULL,
                            configMAX_PRIORITIES - 1, &task_handle, 1);
}
```

After deploying the code on the ESP32, the system was tested by connecting to the access point and entering the ESP32’s IP address in a web browser. The web server initially displayed the current brightness value, which was set to zero. To change the brightness, the desired value was included as a query string parameter in the URL. The web server responded with the updated brightness value, and the light adjusted accordingly. As a result, the next step of the project focused on developing a mobile application capable of sending brightness control requests to the ESP32 web server, improving the overall usability of the smart dimmer.

### Mobile App

At this stage, both the dimmer circuit and the ESP32 firmware were operating correctly. However, the user interface remained fairly basic, as brightness adjustments required manually entering parameters in the URL. To improve usability, the development of a mobile application was identified as a more user-friendly solution for controlling the smart dimmer. A simple Android application featuring a seek bar interface was implemented to allow intuitive brightness adjustment. 

The following code defines the user interface for the Android app. It consists of a **SeekBar** for adjusting the brightness and a **TextView** that displays the current brightness value:

```xml
<?xml version="1.0" encoding="utf-8"?>
<androidx.constraintlayout.widget.ConstraintLayout xmlns:android="http://schemas.android.com/apk/res/android"
    xmlns:app="http://schemas.android.com/apk/res-auto"
    xmlns:tools="http://schemas.android.com/tools"
    android:layout_width="match_parent"
    android:layout_height="match_parent"
    tools:context=".MainActivity">

    <SeekBar
        android:id="@+id/seekBarID"
        android:layout_width="180dp"
        android:layout_height="wrap_content"
        android:layout_marginBottom="368dp"
        android:rotation="0"
        android:scaleX="2"
        android:scaleY="2"
        app:layout_constraintBottom_toBottomOf="parent"
        app:layout_constraintEnd_toEndOf="parent"
        app:layout_constraintHorizontal_bias="0.497"
        app:layout_constraintStart_toStartOf="parent" />

    <TextView
        android:id="@+id/brightnessTextID"
        android:layout_width="wrap_content"
        android:layout_height="wrap_content"
        android:layout_marginBottom="16dp"
        android:text="0"
        android:textSize="32sp"
        app:layout_constraintBottom_toTopOf="@+id/seekBarID"
        app:layout_constraintEnd_toEndOf="parent"
        app:layout_constraintStart_toStartOf="parent" />

</androidx.constraintlayout.widget.ConstraintLayout>
```

In the app code, the user interface was designed and the functionality for sending requests to the ESP32 web server was implemented. To simplify server communication, the Volley library was used, as shown in the following code. 

Adjusting and releasing the seek bar triggered the **onStopTrackingTouch** event, which sent an HTTP request to the ESP32’s IP address with the updated brightness value, following the same process used in the web browser. To enable this functionality, internet access permissions were added to the app settings, and the Volley library was integrated to handle the network communication.

```java
package com.example.dimmer;

import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;
import android.widget.SeekBar;
import android.widget.TextView;

import com.android.volley.Request;
import com.android.volley.RequestQueue;
import com.android.volley.Response;
import com.android.volley.VolleyError;
import com.android.volley.toolbox.StringRequest;
import com.android.volley.toolbox.Volley;

public class MainActivity extends AppCompatActivity {
    SeekBar seekBar;
    TextView brightness;
    RequestQueue requestQueue;
    String url;
    Integer progress;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Creates the components interfaces.
        seekBar = (SeekBar) findViewById(R.id.seekBarID);
        brightness = (TextView) findViewById(R.id.brightnessTextID);

        // Creates a new request Queue.
        requestQueue = Volley.newRequestQueue(this);

        seekBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int i, boolean b) {
                progress = i;
                brightness.setText(String.valueOf(progress));
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
                // Adds URL parameter
                url = "http://192.168.1.1/?brightness=" + String.valueOf(progress);

                // Requests a string response from the provided URL
                StringRequest stringRequest = new StringRequest(
                        Request.Method.GET, 
                        url,
                        new Response.Listener<String>() {
                            @Override
                            public void onResponse(String response) {
                                // Displays the response string
                                brightness.setText(response);
                            }
                        }, 
                        new Response.ErrorListener() {
                            @Override
                            public void onErrorResponse(VolleyError error) {
                                brightness.setText("Connection error!");
                            }
                        }
                );

                requestQueue.add(stringRequest);
            }
        });
    }
}
```

To enable internet access, the following code was added to the **AndroidManifest.xml** file:

```xml
<uses-permission android:name="android.permission.INTERNET" />
```

To include the Volley library, the following line was added to the dependencies section of the **build.gradle** file:

```
implementation 'com.android.volley:volley:1.2.1'
```

Once all configurations were completed, the final step involved compiling and installing the app on an Android smartphone. After installation, the device was connected to the ESP32 access point, allowing the light brightness to be adjusted directly using the seek bar.

## Results

The project was successfully completed, and the system operated as intended. The mobile app was able to set the light brightness by sending HTTP requests containing the desired brightness value to the ESP32 microcontroller. Upon receiving these requests, the ESP32 adjusted the TRIAC triggering accordingly, allowing real-time control of the light intensity through the app.

![Smart dimmer controlled by mobile app]({{ site.url }}{{ site.baseurl }}/public/images/smart-dimmer-controlled-by-mobile-app/smart_dimmer_controlled_by_mobile_app.webp)
*Figure 14 — Smart dimmer controlled by mobile app.*

Beyond its current implementation, it provides a foundation for developing a smart dimmer product. Future work could focus on designing a dedicated PCB and a case to improve durability, safety, and aesthetics, while further evaluating the system to optimize user experience.

![Smart dimmer controlled by mobile app]({{ site.url }}{{ site.baseurl }}/public/images/smart-dimmer-controlled-by-mobile-app/complete prototype_working_on_the_breadboard.gif)
*Figure 15 — Complete prototype working on the breadboard.*

The current implementation uses the seek bar’s **onStopTrackingTouch** event to send the brightness value to the microcontroller. However, this event is not ideal, as it causes abrupt changes in brightness. A different event could be used to provide smoother transitions and a better user experience when adjusting the light intensity.

## Conclusion

The proof of concept for this smart dimmer system was successfully completed, and the system operates reliably, allowing real-time brightness control via the mobile app and ESP32 microcontroller. 

Some final considerations for this project relate to compatibility, access, and communication methods. The system was specifically designed for conventional incandescent lights and may not perform optimally with LEDs or other non-resistive loads, due to differences in their electrical behavior.

Regarding network access, the web server is available only to devices connected directly to the ESP32 access point. Devices connecting to this access point will be temporarily disconnected from other networks while sending commands to the dimmer, which may affect convenience in certain use cases.

Finally, the web server currently uses HTTP GET requests to receive brightness parameters for simplicity. For integration into larger systems, such as smart home setups, using HTTP POST requests is recommended. Additionally, if the server is exposed to the internet, implementing authentication and encryption is essential to maintain security and prevent unauthorized access.