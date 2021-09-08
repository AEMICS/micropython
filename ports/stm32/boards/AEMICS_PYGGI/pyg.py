"""
@author: grobben
https://github.com/peterhinch/micropython-epaper/blob/master/epd.py
"""

# LM75 Temperature sensor
from machine import I2C
import pyb

I2C_BUS = const(3)

# LM75 7bit I2C address, A-pins =0
LM75_ADDR = const(0x48)

# BQ24160_addr 7bit I2C address
BQ24160_ADDR = const(0x6B)


class LM75Exception(Exception):
    pass


class LM75:
    # registers
    TEMP_REGISTER = const(0)
    CONF_REGISTER = const(1)
    CONF_w_mask = const(0x1F)
    THYST_REGISTER = const(2)
    TOS_REGISTER = const(3)

    def __init__(self):  # Check existence and wake it
        self._i2c = I2C(I2C_BUS)
        devices = self._i2c.scan()
        if not LM75_ADDR in devices:
            raise LM75Exception("No LM75 device detected")
        self.wake()

    # Internal single byte I2C communication
    def _read(self, reg, num=1):
        data = self._i2c.readfrom_mem(LM75_ADDR, reg, num)
        data = int.from_bytes(data, "big")
        return data

    def _write(self, reg, data, num=1):
        self._i2c.writeto_mem(LM75_ADDR, reg, data.to_bytes(num, "big"))

    def wake(self):
        self._write(self.CONF_REGISTER, 0)

    def sleep(self):
        # put sensor in shutdown mode
        self._write(self.CONF_REGISTER, 1)

    def write_conf(self, data):
        self._write(self.CONF_REGISTER, data)

    def temperature(self):
        # return temperature as integer in Celsius
        temp = self._read(self.TEMP_REGISTER, 2)
        temperature = temp / 256

        # sign bit: subtract once to clear, 2nd time to add its value
        return temperature if temperature < 128 else temperature - 256

class BQ24160Exception(Exception):
    pass

class BQ24160:
    # registers

    STAT_CONTR = const(0)
    STAT_CONTR_w_mask = const(0x88)

    BATT_SUP_STAT = const(1)
    BATT_SUP_STAT_w_mask = const(0x09)

    CONTROL = const(2)
    CONT_BATT_VOLT = const(3)
    REVISION = const(4)
    BATT_TERM = const(5)

    VIN_DPPM = const(6)
    VIN_DPPM_w_mask = const(0x3F)

    SAFE_TIM_NTC_MON = const(7)
    SAFE_TIM_NTC_MON_w_mask = const(0xF9)

    def __init__(self):
        self._i2c = I2C(I2C_BUS)
        devices = self._i2c.scan()
        if not BQ24160_ADDR in devices:
            raise BQ24160Exception("No BQ24160 device detected")
        self.default()
        self.timer = pyb.Timer(16, freq=0.1)
        # Can't put a 'self' in a timer callback so direct command
        self.timer.callback(lambda t: I2C(3).writeto_mem(0x6B, 0, b"\x80"))
        self.pin_chg_dis = pyb.Pin.board.CHARGE_DISABLE
        self.pin_chg_dis.init(mode=pyb.Pin.OUT)

    def default(self):
        self.write_control(0b10000000)  # reset

        self.write_control(0b00000010)  # .. disable charge
        self.write_batt_stat(0b00000001)  # .. No Batt

    # Internal single byte I2C communication
    def _read_one(self, reg):
        data = self._i2c.readfrom_mem(BQ24160_ADDR, reg, 1)
        data = int.from_bytes(data, "little")
        return data

    def _write_one(self, reg, data):
        self._i2c.writeto_mem(BQ24160_ADDR, reg, data.to_bytes(1, "little"))

    # Direct pin control
    def set_charge_off(self):
        # if no battery connected, this will shut down the module and restart
        self.pin_chg_dis.on()

    def set_charge_on(self):
        self.pin_chg_dis.off()

    def set_charge_voltage(
        self, voltage
    ):  # Set battery regulator voltage (3.5V-4.44V w/ steps of 0.02V)
        set_voltage = (voltage - 3.50) * 100
        set_voltage_int = round(set_voltage)
        reg = set_voltage_int << 1
        reg &= ~0x02  # Bitmask
        self.write_batt_control(reg)

    # Reading
    def read_stat_contr(self):
        return self._read_one(self.STAT_CONTR)

    def read_batt_stat(self):
        return self._read_one(self.BATT_SUP_STAT)

    def read_control(self):
        return self._read_one(self.CONTROL)

    def read_batt_control(self):
        return self._read_one(self.CONT_BATT_VOLT)

    def read_revision(self):
        return self._read_one(self.REVISION)

    def read_batt_term(self):
        return self._read_one(self.BATT_TERM)

    def read_vin_dppm(self):
        return self._read_one(self.VIN_DPPM)

    def read_safe_tim_ntc_mon(self):
        return self._read_one(self.SAFE_TIM_NTC_MON)

    # Writing
    @staticmethod
    def _mask_check(data, mask):
        if data.__class__.__name__ != "int":
            raise BQ24160Exception("Data must be of type int")

        if data & (~mask & 0xFF):
            raise BQ24160Exception("Data out of range for register")

    def write_stat_contr(self, data):
        self._mask_check(data, self.STAT_CONTR_w_mask)
        self._write_one(self.STAT_CONTR, data & self.STAT_CONTR_w_mask)

    def write_batt_stat(self, data):
        self._mask_check(data, self.BATT_SUP_STAT_w_mask)
        self._write_one(self.BATT_SUP_STAT, data & self.BATT_SUP_STAT_w_mask)

    def write_control(self, data):
        self._mask_check(data, 0xFF)
        self._write_one(self.CONTROL, data)

    def write_batt_control(self, data):
        self._mask_check(data, 0xFF)
        self._write_one(self.CONT_BATT_VOLT, data)

    def write_batt_term(self, data):
        self._mask_check(data, 0xFF)
        self._write_one(self.BATT_TERM, data)

    def write_vin_dppm(self, data):
        self._mask_check(data, 0xFF)
        self._write_one(self.VIN_DPPM, data & self.VIN_DPPM_w_mask)

    def write_safe_tim_ntc_mon(self, data):
        self._mask_check(data, self.SAFE_TIM_NTC_MON_w_mask)
        self._write_one(self.SAFE_TIM_NTC_MON, data & self.SAFE_TIM_NTC_MON_w_mask)
