import time

import lgpio

pin = 0

def main() -> None:
    gpio_chip = lgpio.gpiochip_open(0)
    lgpio.gpio_claim_input(gpio_chip, pin)
    data: int = lgpio.gpio_read(gpio_chip, pin)
    lgpio.gpio_write(gpio_chip, pin, 0)
    count = 0
    while True:
        if data:
                print(f'data before manip switch: {data}')
        if (count % 2 == 0):
            data = lgpio.gpio_write(gpio_chip, pin, 1)
        else:
            data = lgpio.gpio_write(gpio_chip, pin, 0)

            print(f'data after manip switch:{data}')
        time.sleep(2)
        count+=1
if __name__ == '__main__':
    main()


#     with SMBus(I2C_BUS) as bus:
#         while True:
#             print('on')
#             write_all = i2c_msg.write(ADDRESS, [0x06, 0b00000000])

#             bus.i2c_rdwr(write_all)

#             time.sleep(2)
#             print('off')

#             a = i2c_msg.write(ADDRESS, [0x06, 0b11111111])

#             bus.i2c_rdwr(a)
#             time.sleep(2)


# if __name__ == '__main__':
#     main()
