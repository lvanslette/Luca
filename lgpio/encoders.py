#!/usr/bin/env python3
# *** encoders.py ***
# test encoder's functionality

import lgpio as lg
import time
import smbus


DEVICE = 0x20   # used to reference the MCP23017 chip
IODIRA = 0x00   # sets the direction (input/output) of each of the A pins
GPIOA = 0x12    # read this register when you want to read pins from A
OLATA = 0x14    # read this register when you want to write to pins
IODIRB = 0x01    # same for B side
GPIOB = 0x13    # same for B side
OLATB = 0x15    # same for B side

def gpio_setup():
  bus = smbus.SMBus(1)
  # set pins for A side as GPA6 as output, rest input (input is 1, output is 0)
  # 01011111 = 0x5F
  bus.write_byte_data(DEVICE, IODIRB, 0x0F)
  # set output of all 7 output bits to 0
  bus.write_byte_data(DEVICE, IODIRA, 0xB7)
  # set GPB0 - GPB3 as read (encoders) 
  # 00001111 = 15 = 0x0F
  #bus.write_byte_data(DEVICE, IODIRB, 0xFF)
  # set output to 0 initially (write 0000 0000)
  return bus

def convert_to_binary(num):
  binary = str(bin(num).replace("0b", ""))
  # if the binary number is less than 4 digits, add more zeros
  while len(binary) < 4:
    #binary = "0".join(binary)
    binary = "0" + binary

  return binary

def get_encoder_counts(bus):
  # read status of each ir sensor connected to the bus in the form of a list of
  # ints
  num = bus.read_byte_data(DEVICE, GPIOB)
  binary = convert_to_binary(num)
  status = [int(x) for x in binary]
  return status

if __name__ == '__main__':
  # setup lgpio pins
  bus = gpio_setup()
  counters = [0,0,0,0]
  # get initial state of encoder counters
  last_status = get_encoder_counts(bus)
  while True:
    # get the status of each encoder (whether each sensor is reading a 1 or
    # 0)
    status = get_encoder_counts(bus)
    # for each encoder, compare the current status to previous status
    changes = [a^b for a,b in zip(status, last_status)]
    #print(changes)
    # if status has changed, increment the number of counts
    counters = [a+b for a,b in zip(counters, changes)]
    print(counters, end="\r")

    last_status = status


