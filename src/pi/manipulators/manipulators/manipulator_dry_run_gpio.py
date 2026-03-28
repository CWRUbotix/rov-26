import time

import lgpio

pin1 = 24
pin2 = 26

def main() -> None:
    gpio_chip = lgpio.gpiochip_open(0)
    lgpio.gpio_claim_input(gpio_chip, pin1)
    lgpio.gpio_write(gpio_chip, pin1, 0)
    lgpio.gpio_claim_input(gpio_chip, pin2)
    lgpio.gpio_write(gpio_chip, pin2, 0)

    while True:
        print('on')
        lgpio.gpio_write(gpio_chip, pin1, 1)
        lgpio.gpio_write(gpio_chip, pin2, 1)

        time.sleep(2)

        print('off')
        lgpio.gpio_write(gpio_chip, pin1, 0)
        lgpio.gpio_write(gpio_chip, pin2, 0)

        time.sleep(2)
if __name__ == '__main__':
    main()

