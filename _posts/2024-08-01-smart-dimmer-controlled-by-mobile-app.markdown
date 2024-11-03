---
title:  "Smart Dimmer Controlled by Mobile App"
date: 2024-08-01 18:00:00
description: Project documentation and development of a smart dimmer using an ESP32 microcontroller, with a mobile app to control the brightness of an incandescent light bulb.
author: lucasmazz
keywords: IoT, dimmer, electronic, electronics, TRIAC, triac, ESP32, esp32, microcontroller, control, mobile, app
---

Dimmers are electronic devices typically connected to lights to control their brightness. Similar circuits can also be used in various applications, such as controlling the speed of an electric motor or setting the temperature of an electric oven. The basic idea is to limit the power supplied from a source to the load by altering the waveform of the applied voltage.

The key component used to achieve this goal is the TRIAC. This electronic component has three terminals: one called the "gate," which is responsible for allowing or blocking the passage of current through the other two terminals. By default, the TRIAC operates as an open circuit, preventing current flow. However, when the gate is triggered, the component behaves like a closed switch, allowing current to flow and providing power to the load until the next cycle of the supply waveform.

For example, when considering a lamp as the load, it is possible to allow only a portion of the current to flow to it, thus regulating the light brightness. To control the brightness as desired, it is necessary to trigger the TRIAC gate at the appropriate time. If the gate is triggered too early, full power will be supplied to the lamp, causing it to glow very brightly. On the other hand, if the gate is triggered too late, the light will be very dim or may even go out. It is important to note that this dimmer design is specifically intended for use with incandescent light bulbs and may not perform well with LED lights due to inherent differences in their construction.

Before start, be careful if you want to replicate this project. This circuit uses voltages that can cause serious injury. If you don’t have experience with electricity, look for someone who can help you with the circuit. It would also be nice to have an oscilloscope, to check the waveforms and verify that everything is working correctly.

![Smart dimmer controlled by mobile app]({{ site.url }}{{ site.baseurl }}/public/images/smart-dimmer-controlled-by-mobile-app/smart_dimmer_controlled_by_mobile_app.webp)
*Figure 1 — Smart dimmer controlled by mobile app.*

## Materials

The list of components used in this project are listed below:

- 1x ESP32 microcontroller
- 1x TRIAC
- 1x 4N25 optocoupler
- 1x MOC3021 optocoupler
- 1x transformer
- 4x diodes
- 2x 100 Ω resistors
- 1x 330 Ω resistor
- 1x 10K Ω resistor


## Overview

The main idea is to trigger the TRIAC using an ESP32 microcontroller, which is a relatively cheap device capable of connecting to Wi-Fi and Bluetooth without any external needs. Moreover, It features a 32-bit dual-core processor and offers low power consumption, making it a robust solution for diverse IoT applications and an excellent option for simple embedded systems. The development board used in the prototype is a DOIT ESP32 Devkit V1. This project utilizes the Espressif IoT Development Framework (ESP-IDF) to code and program the device. Additionally, Android Studio is used to develop the Android mobile app, which interfaces with the ESP32 and sends commands with the desired brightness.

A transformer is used to reduce the power voltage to around 12 V peak-to-peak AC. Four diodes in a bridge rectifier configuration to convert the stepped-down AC voltage from the transformer into about 6 V DC. More importantly, they provide a positive voltage for the zero-crossing detector circuit to function, which detects the AC power wave's zero-crossing, which is crucial for timing the TRIAC and controlling dimming.

A 10k resistor acts as a pull-up resistor in the microcontroller input, ensuring stable logic levels. The other resistors listed in the materials are used for current limiting and protection in various parts of the circuit, ensuring that components function within their safe current ratings. The 4N25 optocoupler is used for zero-crossing detection and electrical isolation between the microcontroller and the low voltage DC side. The MOC3021 optocoupler interfaces with the TRIAC, isolating and driving it so the ESP32's low voltage signals can safely trigger the high voltage TRIAC to switch the AC load on and off.

In summary, the ESP32 receives zero-crossing detection signals from the 4N25 optocoupler, which outputs a high signal when the power grid wave approaches zero. This zero-crossing time serves as a reference to calculate the trigger timing for activating the TRIAC via the MOC3021, ensuring control over dimming operations.

The complete circuit schematic is shown below.

![Complete circuit schematic]({{ site.url }}{{ site.baseurl }}/public/images/smart-dimmer-controlled-by-mobile-app/complete_circuit_schematic.webp)
*Figure 2 — Complete circuit schematic.*

## Zero-crossing Detector

A zero-crossing detector circuit is essential for identifying when the supply voltage crosses zero. This information is then used to calculate the trigger time for the TRIAC in each cycle of the supply voltage waveform. The image below illustrates the zero-crossing with a sinusoidal wave and the trigger time.

![Zero-crossing and TRIAC trigger time illustration]({{ site.url }}{{ site.baseurl }}/public/images/smart-dimmer-controlled-by-mobile-app/zero-crossing_and_triac_trigger_time_illustration.webp)
*Figure 3 — Zero-crossing and TRIAC trigger time illustration.*

As the ESP32 only reads positive voltage signals below 5 V, it is necessary to reduce and rectify the input voltage that will be read by the microcontroller. A transformer is used to step down the supply voltage, resulting in a signal that is a reduced copy of the source signal (127 V / 220 V AC) at about 12 V AC. Following this, a full bridge rectifier converts the negative part of the input voltage to positive, producing a signal with a maximum peak of less than 6 V DC. Additionally, a 10k resistor acts as a pull-up resistor in the microcontroller input, ensuring it outputs a high signal when the power grid wave approaches zero. The circuit schematic below illustrates this setup.

![Zero-crossing detector circuit schematic]({{ site.url }}{{ site.baseurl }}/public/images/smart-dimmer-controlled-by-mobile-app/zero-crossing_detector_circuit_schematic.webp)
*Figure 4 — Zero-crossing detector circuit schematic.*

By setting up this circuit, the expected output of the optocoupler, in relation to the rectifier, will look like the waveform illustrated below. The blue wave represents the rectifier output, while the orange wave represents the optocoupler output, which serves as the input to the microcontroller.

![Rectifier output and 4N25 optocoupler output]({{ site.url }}{{ site.baseurl }}/public/images/smart-dimmer-controlled-by-mobile-app/rectifier_output_and_4n25_optocoupler_output.webp)
*Figure 5 — Rectifier output (blue) and 4N25 optocoupler output (orange).*

When the output of the rectifier approaches zero, the optocoupler triggers a small pulse just before the zero-crossing point, at a voltage below 1 V, and stops shortly after when the voltage reaches the same level. This happens because the LED inside the optocoupler stops conducting at this low voltage. It is important to note that this pulse has a significant duration and must be considered when calculating the zero-crossing moment. Therefore, the zero-crossing time can be approximated as occurring at the middle of the pulse triggered by the optocoupler.

By connecting the optocoupler output to the ESP32, the signal's logic level can be identified. An algorithm can then calculate the average time between transitions from low to high logic levels and vice versa to estimate the zero-crossing time.

The following code implements an Interrupt Service Routine (ISR) for zero-crossing detection, capturing timestamps on both rising and falling edges of the optocoupler signal. Due to ISR speed constraints, average time calculations between these timestamps are performed in a separate function allocated to a task. Additionally, a timer in the code schedules a function to generate a small pulse on the output pin after the rising edge, to verify the zero-crossing time estimation.

```c
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/err.h"

/* Interruption and GPIO */
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
 * and tasks to controlthe brightness of a lighting system based on 
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

    /* Infinity loop */
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

If everything has gone well, the zero-crossing estimate trigger should match the midpoint of the optocoupler-triggered pulse and closely match the actual zero of the rectifier output, as illustrated in the following figures.

![Optocoupler output and and ESP32 Trigger output]({{ site.url }}{{ site.baseurl }}/public/images/smart-dimmer-controlled-by-mobile-app/optocoupler_output_and_esp32_trigger_at_the_zero_crossing.webp)
*Figure 6 — Optocoupler output (blue) and ESP32 Trigger at the zero-crossing (orange).*


![Rectifier output and ESP32 Trigger at the zero-crossing]({{ site.url }}{{ site.baseurl }}/public/images/smart-dimmer-controlled-by-mobile-app/rectifier_output_and_esp32_trigger_at_the_zero_crossing.webp)
*Figure 7 — Rectifier output (blue) and ESP32 Trigger at the zero-crossing (orange).*

## Triggering the TRIAC

The next step is to trigger the TRIAC at a desired time after the input voltage crosses zero. This is essential for dimmer circuits, which use phase control to adjust the amount of power delivered to the load. In phase control, the TRIAC is triggered at a specific point in each AC cycle, effectively cutting part of the waveform. By changing the point at which the TRIAC is triggered, the amount of the AC waveform applied to the load is controlled, thereby regulating the power.

The trigger time can be represented in terms of brightness. A lower brightness value corresponds to a later TRIAC trigger time, resulting in less current being supplied to the load. Since the frequency of the electrical network corresponds to each complete cycle of a sinusoidal signal, it is important to note that the zero-crossing detector operates twice per period of the supply waveform, effectively doubling the detection frequency. Therefore, the brightness value, which ranges from 0% to 100%, must be accurately converted to the appropriate trigger time to achieve the desired dimming effect. Consequently, the trigger time $$t$$ can be represented considering the brightness $$b$$ and the frequency $$f$$ by the following expression.

$$
t(b,f) = \left(1 - \frac{b}{100}\right)\left(\frac{10^6}{2f}\right)
$$

In this expression, the trigger time $$t$$ is given in microseconds, the brightness value $$b$$ ranges from 0 to 100, and $$f$$ is the frequency of the electrical network in Hz. Given that frequency is the inverse of the period, the equation can also be expressed using the period instead of frequency.

To implement this expression in the code, the trigger time calculation must be performed after each zero-crossing detection, taking into account the brightness value. First, it is necessary to calculate the period of the electrical network. This can be done in the ISR by subtracting the last recorded rising edge time from the current time, providing an estimated period. The trigger time is then computed in an auxiliary function, as floating point operations are constrained within the ISR. Subsequently, in the ISR, the timer executed after the rising edge is set with the calculated trigger time.

For example, the code incorporating these adjustments can be structured as follows, assuming a brightness setting of 50%.

```c
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/err.h"

/* Interruption and GPIO */
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
 * and tasks to controlthe brightness of a lighting system based on 
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

    /* Infinity loop */
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

If everything is executed successfully, upon compiling and deploying the code on the microcontroller, the pulse triggered by the ESP32 should align with the peak of the rectifier waveform, thereby synchronizing with the waveform of the power supply, as shown in the image below.

![Rectifier output and ESP32 trigger at the half of the brightness]({{ site.url }}{{ site.baseurl }}/public/images/smart-dimmer-controlled-by-mobile-app/rectifier_output_and_esp32_trigger_at_half_brightness.webp)
*Figure 8 — Rectifier output (blue) and ESP32 trigger at the half of the brightness (orange).*

However, a small adjustment to the previous code is needed to ensure proper operation: extending the TRIAC triggering beyond the calculated time until the next zero-crossing detection. This prevents issues related to high brightness levels and timing errors in zero-crossing estimation.

For example, if the brightness is set to near 100% and the zero-crossing is miscalculated to occur earlier than it actually does, the TRIAC might be triggered too soon. This could result in inadequate current supply to the light, as the TRIAC would stop conducting immediately after the real zero-crossing. By maintaining the TRIAC in the triggered state until the next rising edge of the zero-crossing detector, we ensure it remains closed to supply the necessary current for the desired brightness, overcoming any timing inconsistencies. Below is the code incorporating these adjustments.

```c
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/err.h"

/* Interruption and GPIO */
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
        /* Turnoff the active trigger */
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
 * and tasks to controlthe brightness of a lighting system based on 
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

    /* Infinity loop */
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

The next waveform illustrates the result of these adjustments in the triggering code. Notice that the trigger starts at 50% of the rectifier waveform, but only ends at the rising edge of the optocoupler zero-crossing detection. This ensures that the TRIAC remains conducting, thereby supplying adequate current to the load for the entire duration of the intended brightness period.

![Optocoupler output and and ESP32 Trigger output adjusted]({{ site.url }}{{ site.baseurl }}/public/images/smart-dimmer-controlled-by-mobile-app/optocoupler_output_and_esp32_trigger_adjusted.webp)
*Figure 9 — Optocoupler output (blue) and ESP32 Trigger adjusted (orange).*

In the following image, with the brightness value set to 100%, the trigger initiates right at the zero-crossing and continues until the rising edge of the optocoupler zero-crossing detection. 

![Optocoupler output and and ESP32 Trigger output adjusted at 100% brightness]({{ site.url }}{{ site.baseurl }}/public/images/smart-dimmer-controlled-by-mobile-app/optocoupler_output_and_esp32_trigger_adjusted_100.webp)
*Figure 10 — Optocoupler output (blue) and ESP32 Trigger adjusted at 100% brightness (orange).*

At this point, web can accurately control when to trigger the TRIAC to adjust the light's brightness as desired. To achieve this, it is essential to isolate the microcontroller using an optocoupler, which will effectively trigger the TRIAC, as illustrated in the following circuit.

![TRIAC trigger circuit schematic]({{ site.url }}{{ site.baseurl }}/public/images/smart-dimmer-controlled-by-mobile-app/triac_trigger_circuit_schematic.webp)
*Figure 11 — TRIAC trigger circuit schematic.*

Therefore, assuming the brightness value is 50%, connecting the complete circuit with the lamp as a load should result in the following waveform on the TRIAC.

![Power supply and TRIAC output with brightness value at 50%]({{ site.url }}{{ site.baseurl }}/public/images/smart-dimmer-controlled-by-mobile-app/power_supply_and_triac_output_with_brightness_at_50.webp)
*Figure 12 — Power supply (blue) and TRIAC output (orange) with brightness value at 50%.*

It is expected that the brightness of the light will be reduced, compared to its conventional operation. If the brightness parameter is set to 25%, the light brightness will decrease even more and the TRIAC output will have the following waveform.

![Power supply and TRIAC output with brightness value at 25%]({{ site.url }}{{ site.baseurl }}/public/images/smart-dimmer-controlled-by-mobile-app/power_supply_and_triac_output_with_brightness_at_25.webp)
*Figure 13 — Power supply (blue) and TRIAC output (orange) with brightness value at 25%.*

If the brightness is set to 75%, the light brightness will increase and the waveform will be similar to the next one.

![Power supply and TRIAC output with brightness value at 75%]({{ site.url }}{{ site.baseurl }}/public/images/smart-dimmer-controlled-by-mobile-app/power_supply_and_triac_output_with_brightness_at_75.webp)
*Figure 14 — Power supply (blue) and TRIAC output (orange) with brightness value at 75%.*

The ESP32 code currently sets the brightness level of the light effectively. However, due to its current design, adjusting the brightness requires recompiling and uploading the code to the microcontroller each time. Therefore, the next phase of this project involves modifying the code to enable real-time brightness adjustments via network connectivity, eliminating the need for recompilation.

## Wifi Access Point and Web Server

The ESP32 must control the TRIAC via the network to enable dynamic adjustment. Therefore, configuring the ESP32 to connect to Wi-Fi and set up a web server to receive the desired brightness value is necessary. To achieve this, a Wi-Fi access point can be established for users to connect and communicate with the ESP32 web server. Additionally, an IP address can be fixed for the ESP32, making it easier to communicate with the web server. The following code demonstrates how to set up an access point with a fixed IP address on the ESP32.

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

If the code has executed successfully on the ESP32, a new access point should appear among nearby Wi-Fi networks. The microcontroller’s IP address will remain fixed. If everything has worked correctly up to this point, the microcontroller is now available for communication, allowing us to set up a web server to receive data.

To achieve this, one function initializes the web server and another handles requests, as shown in the following code snippet. The *http_server_init* function starts the server with default settings and registers a handler for HTTP GET requests at the root endpoint (‘/’). The *http_request_handler* function processes incoming requests by extracting the "brightness" parameter from the URL query string, converting it to an integer within the range of 0 to 100. It then formats the current brightness value into a response buffer and sends it back to the client as the HTTP response.

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

By merging the TRIAC trigger code with the Wi-Fi access point and web server code, we can effectively change the brightness of the light over the network by sending brightness parameters to the web server. The server decodes these parameters and adjusts the brightness accordingly. To ensure smooth operation, we must utilize the ESP32's dual cores by running the TRIAC control function on a different core than the web server. This separation prevents one function from blocking the other, which is crucial for the precise execution of the TRIAC trigger code. Without this separation, any delay in the web server could interfere with the TRIAC triggering process and affect the desired output.

To achieve this, the RTOS function *xTaskCreatePinnedToCore* can be used. This method allocates the TRIAC dimmer control function to a parallel core, while the web server handler function runs on the main core, as demonstrated in the following complete code.

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

/* Interruption and GPIO */
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
        /* Turnoff the active trigger */
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
 * and tasks to controlthe brightness of a lighting system based on 
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

    /* Infinity loop */
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

After deploying the code on the ESP32, we can test it by connecting to the access point and entering the ESP32's IP address in a web browser. The web server will display the current brightness value (initially zero). To change the brightness, include the desired value as a query string parameter in the URL. The web server will then respond with the new brightness value, and the light will adjust accordingly. Consequently, the next step of this project is to develop a mobile app capable of sending requests with the brightness parameters to the ESP32 web server, enhancing the usability of the smart dimmer.

## Mobile App

Currently, the dimmer circuit and the ESP32 program are functioning well. However, the user interface is still quite basic, requiring manual input of brightness parameters in the URL to make changes. To enhance user experience, developing a mobile app would provide a much more user-friendly way to operate the smart dimmer. A simple Android app with a seek bar interface would allow users to easily adjust the brightness. The code for the user interface, including this seek bar, can be seen below.

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

In the app code, you'll need to design the user interface and implement the functionality for sending requests to the ESP32 web server. To simplify the interactions with the server, we can utilize the Volley library, as demonstrated in the following code. When the seek bar is adjusted and released, it triggers the *onStopTrackingTouch* event. This event sends an HTTP request to the ESP32’s IP address with the updated brightness parameters, similar to the process used in the web browser. To ensure this functionality works, we must add internet access permissions to the app settings and integrate the Volley library for communication.

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

To add the internet connection permission, just insert the following line of code in the *AndroidManifest.xml* file.

```xml
<uses-permission android:name="android.permission.INTERNET" />
```

To add the library, it is necessary to insert the following line in the *build.gradle* file dependencies.

```
implementation 'com.android.volley:volley:1.2.1'
```

In conclusion, once everything is properly configured, the final step is to compile and install the app on an Android smartphone. After installation, we can connect to the ESP32 access point and adjust the light’s brightness using the seek bar. This seamless integration of hardware and software not only enhances the user experience but also demonstrates the potential of IoT technology in creating smart, user-friendly solutions.

![Smart dimmer controlled by mobile app]({{ site.url }}{{ site.baseurl }}/public/images/smart-dimmer-controlled-by-mobile-app/complete prototype_working_on_the_breadboard.gif)
*Figure 15 — Complete prototype working on the breadboard.*

## Final Considerations

Here are some final considerations for the development of this project:

1. **Compatibility**: The project is designed for conventional incandescent lights and will not work well with LED lights due to differences in their operation.
2. **Access**: The web server is accessible only to devices connected to the ESP32 access point. Be aware that devices will disconnect from other networks when connecting to the ESP32's access point to send commands to the dimmer.
3. **HTTP Method**: The web server uses HTTP GET requests for simplicity in receiving brightness parameters. For integration into larger systems like a smart home, consider using HTTP POST requests. Additionally, if the web server is exposed to the internet, implement authentication and encryption to ensure security.

The complete source code used in this project is available on [GitHub](https://github.com/workabotic/smart_dimmer_controlled_by_mobile_app).
