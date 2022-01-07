#!/usr/bin/env python3
# *** stop_motors.py ***
# -> emergency stop motors from moving

import lgpio as lg

fwd = [19, 16, 22, 25]
bwd = [4, 18, 12, 5]

h = lg.gpiochip_open(0)
for i,j in zip(fwd, bwd):
  lg.gpio_write(h, i, 0)
  lg.gpio_write(h, j, 0)
lg.gpiochip_close(h)
