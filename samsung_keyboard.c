#include "samsung_keyboard.h"

#include "nrf_gpio.h"

#include "hwconfig.h"

void samsung_keyboard_init(void)
{
    nrf_gpio_cfg_output(KSO0);
    nrf_gpio_cfg_output(KSO1);
    nrf_gpio_cfg_output(KSO2);
    nrf_gpio_cfg_output(KSO3);
    nrf_gpio_cfg_output(KSO4);
    nrf_gpio_cfg_output(KSO5);
    nrf_gpio_cfg_output(KSO6);
    nrf_gpio_cfg_output(KSO7);
    nrf_gpio_cfg_output(KSO8);
    nrf_gpio_cfg_output(KSO9);
    nrf_gpio_cfg_output(KSO10);
    nrf_gpio_cfg_output(KSO11);
    nrf_gpio_cfg_output(KSO12);
    nrf_gpio_cfg_output(KSO13);
    nrf_gpio_cfg_output(KSO14);
    nrf_gpio_cfg_output(KSO15);

    nrf_gpio_cfg_input(KSI0, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(KSI1, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(KSI2, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(KSI3, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(KSI4, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(KSI5, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(KSI6, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(KSI7, NRF_GPIO_PIN_NOPULL);
}
