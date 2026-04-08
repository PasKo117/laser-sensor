#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SICK OD Mini Pro - Полная программа управления
Плата: Rock Pi S
Все команды из J4 и J5 документации

Команды J4: Чтение измерений, статусы, управление
Команды J5: Настройка параметров сенсора
"""

import serial
import time
import struct
import threading
import sys
import termios
import tty
from datetime import datetime

# ============================================================================
# КОНФИГУРАЦИЯ
# ============================================================================
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600
TIMEOUT = 1.0
READ_INTERVAL = 0.5

# ============================================================================
# ПРОТОКОЛ (J1, J2)
# ============================================================================
STX = 0x02
ETX = 0x03
ACK = 0x06
NAK = 0x15

# Команды (J2)
CMD_READ_MEASUREMENT = 0x43  # 'C'
CMD_WRITE_SETTING = 0x57     # 'W'
CMD_READ_SETTING = 0x52      # 'R'

# ============================================================================
# АДРЕСА ПАРАМЕТРОВ (J5)
# ============================================================================
class Addr:
    MODEL_TYPE = (0x01, 0x00)           # Тип модели
    MEAS_MODE = (0x40, 0x04)            # Режим измерения
    NEAR_THRESH = (0x41, 0x00)          # Порог ближней зоны
    FAR_THRESH = (0x41, 0x02)           # Порог дальней зоны
    OBSB_THRESH = (0x41, 0x04)          # Порог ObSB
    OBSB_HYST = (0x41, 0x06)            # Гистерезис ObSB
    OUT_POLARITY = (0x40, 0x08)         # Полярность выхода
    SAMPLING = (0x40, 0x06)             # Период дискретизации
    AVERAGING = (0x40, 0x0A)            # Усреднение
    ALARM = (0x40, 0x0C)                # Настройка тревоги
    ALARM_HOLD = (0x41, 0x08)           # Alarm Hold/Clamp значение
    DISPLAY = (0x40, 0x0E)              # Настройка дисплея
    HYSTERESIS = (0x41, 0x10)           # Гистерезис
    THRESHOLD = (0x40, 0x12)            # Уровень порога
    ZERO_SHIFT = (0x41, 0x12)           # Сдвиг нуля
    SENSITIVITY = (0x40, 0x14)          # Чувствительность

# ============================================================================
# ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
# ============================================================================
class SensorState:
    def __init__(self):
        self.running = True
        self.menu_active = False
        self.model_type = 'B035'
        self.last_value = 0
        self.last_status = "OK"
        self.counter = 0
        self.lock = threading.Lock()

state = SensorState()
ser = None

# ============================================================================
# ФУНКЦИИ ПРОТОКОЛА
# ============================================================================

def calculate_bcc(data_bytes):
    """Вычисляет BCC как XOR (J1)"""
    bcc = 0
    for byte in data_bytes:
        bcc ^= byte
    return bcc

def build_command(command, data1, data2):
    """Формирует команду: STX | CMD | DATA1 | DATA2 | ETX | BCC"""
    bcc = calculate_bcc([command, data1, data2])
    frame = [STX, command, data1, data2, ETX, bcc]
    return bytes(frame)

def send_command(command, data1, data2, wait_time=0.1):
    """Отправляет команду и получает ответ"""
    global ser
    if ser is None or not ser.is_open:
        return None, "Порт закрыт"
    
    cmd = build_command(command, data1, data2)
    ser.flushInput()
    ser.flushOutput()
    ser.write(cmd)
    time.sleep(wait_time)
    response = ser.read(10)
    return response, None

def parse_response(response):
    """Парсит ответ от датчика"""
    if response is None or len(response) < 6:
        return None, None, f"Короткий ответ ({len(response) if response else 0} байт)"
    
    if response[0] != STX:
        return None, None, f"Нет STX (0x{response[0]:02X})"
    
    if response[1] == NAK:
        error_code = response[2]
        error_msgs = {
            0x02: "Адрес недействителен",
            0x04: "BCC недействителен",
            0x05: "Недействительная команда",
            0x06: "Значение вне спецификации",
            0x07: "Значение вне диапазона"
        }
        return None, None, f"Ошибка 0x{error_code:02X}: {error_msgs.get(error_code, 'Неизвестно')}"
    
    if response[1] != ACK:
        return None, None, f"Нет ACK (0x{response[1]:02X})"
    
    # Проверка BCC ответа
    expected_bcc = calculate_bcc([response[1], response[2], response[3]])
    if response[5] != expected_bcc:
        return None, None, f"Ошибка BCC (ожидался 0x{expected_bcc:02X})"
    
    if response[4] != ETX:
        return None, None, f"Нет ETX (0x{response[4]:02X})"
    
    return response[2], response[3], "OK"

def read_setting(addr_high, addr_low):
    """Чтение настройки (J5)"""
    response, err = send_command(CMD_READ_SETTING, addr_high, addr_low)
    if err:
        return None, None, err
    upper, lower, status = parse_response(response)
    return upper, lower, status

def write_setting(addr_high, addr_low, value_high, value_low):
    """Запись настройки (J5, K1)"""
    # Сначала читаем текущее значение
    _, _, status = read_setting(addr_high, addr_low)
    if status != "OK":
        return status
    
    # Затем записываем новое
    response, err = send_command(CMD_WRITE_SETTING, value_high, value_low)
    if err:
        return err
    _, _, status = parse_response(response)
    return status

def write_to_eeprom():
    """Запись настроек в EEPROM (J4: A0h 00h)"""
    response, err = send_command(CMD_READ_MEASUREMENT, 0xA0, 0x00)
    if err:
        return err
    _, _, status = parse_response(response)
    return status

# ============================================================================
# ФУНКЦИИ J4 - ЧТЕНИЕ ИЗМЕРЕНИЙ И СТАТУСОВ
# ============================================================================

def read_measurement():
    """Чтение измеренного значения (J4: B0h 01h)"""
    response, err = send_command(CMD_READ_MEASUREMENT, 0xB0, 0x01)
    if err:
        return None, err
    upper, lower, status = parse_response(response)
    if status != "OK":
        return None, status
    value = struct.unpack('>h', bytes([upper, lower]))[0]
    return value, "OK"

def read_output_status():
    """Чтение статуса выхода (J4: B0h 02h)"""
    response, err = send_command(CMD_READ_MEASUREMENT, 0xB0, 0x02)
    if err:
        return None, err
    upper, lower, status = parse_response(response)
    if status != "OK":
        return None, status
    output_on = (lower & 0x01) == 1
    return output_on, "OK"

def convert_to_mm(value_raw, model_type='B035'):
    """Конвертация в миллиметры (Таблица G)"""
    if model_type == 'B015':
        unit = 0.001  # 1 μm
    else:
        unit = 0.01   # 10 μm (B035, B100)
    return value_raw * unit

def read_model_type():
    """Чтение типа модели (J5: 01h 00h)"""
    upper, lower, status = read_setting(0x01, 0x00)
    if status != "OK":
        return 'B035', status
    model_map = {0x0F: 'B015', 0x23: 'B035', 0x64: 'B100'}
    model = model_map.get(upper, 'B035')
    return model, "OK"

# ============================================================================
# ФУНКЦИИ J4 - УПРАВЛЕНИЕ
# ============================================================================

def dismiss_setting():
    """Отмена настроек (J4: A0h 01h)"""
    response, err = send_command(CMD_READ_MEASUREMENT, 0xA0, 0x01)
    if err:
        return err
    _, _, status = parse_response(response)
    return status

def laser_control(on_off):
    """Управление лазером (J4: A0h 03h / A0h 02h)"""
    data2 = 0x03 if on_off else 0x02
    response, err = send_command(CMD_READ_MEASUREMENT, 0xA0, data2)
    if err:
        return err
    _, _, status = parse_response(response)
    return status

def zero_reset(execute):
    """Сброс/восстановление нуля (J4: A1h 00h / A1h 01h)"""
    data2 = 0x00 if execute else 0x01
    response, err = send_command(CMD_READ_MEASUREMENT, 0xA1, data2)
    if err:
        return err
    _, _, status = parse_response(response)
    return status

def key_lock(execute):
    """Блокировка/разблокировка кнопок (J4: A1h 04h / A1h 05h)"""
    data2 = 0x04 if execute else 0x05
    response, err = send_command(CMD_READ_MEASUREMENT, 0xA1, data2)
    if err:
        return err
    _, _, status = parse_response(response)
    return status

def initialize_sensor():
    """Инициализация всех параметров (J4: 40h 00h)"""
    response, err = send_command(CMD_READ_MEASUREMENT, 0x40, 0x00)
    if err:
        return err
    _, _, status = parse_response(response)
    return status

# ============================================================================
# ФУНКЦИИ J5 - НАСТРОЙКА ПАРАМЕТРОВ
# ============================================================================

def set_measurement_mode(mode):
    """Режим измерения (J5: 40h 04h)"""
    modes = {'2pt': (0x00, 0x00), '1pt': (0x00, 0x01), 'obsb': (0x00, 0x02)}
    if mode not in modes:
        return "Неверный режим"
    status = write_setting(0x40, 0x04, modes[mode][0], modes[mode][1])
    if status == "OK":
        status = write_to_eeprom()
    return status

def set_near_threshold(value_mm):
    """Порог ближней зоны (J5: 41h 00h)"""
    if state.model_type == 'B015':
        raw = int(value_mm / 0.001)
    else:
        raw = int(value_mm / 0.01)
    upper = (raw >> 8) & 0xFF
    lower = raw & 0xFF
    status = write_setting(0x41, 0x00, upper, lower)
    if status == "OK":
        status = write_to_eeprom()
    return status

def set_far_threshold(value_mm):
    """Порог дальней зоны (J5: 41h 02h)"""
    if state.model_type == 'B015':
        raw = int(value_mm / 0.001)
    else:
        raw = int(value_mm / 0.01)
    upper = (raw >> 8) & 0xFF
    lower = raw & 0xFF
    status = write_setting(0x41, 0x02, upper, lower)
    if status == "OK":
        status = write_to_eeprom()
    return status

def set_obsb_threshold(value_mm):
    """Порог ObSB (J5: 41h 04h)"""
    if state.model_type == 'B015':
        raw = int(value_mm / 0.001)
    else:
        raw = int(value_mm / 0.01)
    upper = (raw >> 8) & 0xFF
    lower = raw & 0xFF
    status = write_setting(0x41, 0x04, upper, lower)
    if status == "OK":
        status = write_to_eeprom()
    return status

def set_obsb_hysteresis(value_mm):
    """Гистерезис ObSB (J5: 41h 06h)"""
    if state.model_type == 'B015':
        raw = int(value_mm / 0.001)
    else:
        raw = int(value_mm / 0.01)
    upper = (raw >> 8) & 0xFF
    lower = raw & 0xFF
    status = write_setting(0x41, 0x06, upper, lower)
    if status == "OK":
        status = write_to_eeprom()
    return status

def set_output_polarity(polarity):
    """Полярность выхода (J5: 40h 08h)"""
    polarities = {'light_on': (0x00, 0x00), 'dark_on': (0x00, 0x01)}
    if polarity not in polarities:
        return "Неверная полярность"
    status = write_setting(0x40, 0x08, polarities[polarity][0], polarities[polarity][1])
    if status == "OK":
        status = write_to_eeprom()
    return status

def set_sampling_period(period):
    """Период дискретизации (J5: 40h 06h)"""
    periods = {
        '500us': (0x00, 0x00),
        '1000us': (0x00, 0x01),
        '2000us': (0x00, 0x02),
        '4000us': (0x00, 0x03),
        'auto': (0x00, 0x04)
    }
    if period not in periods:
        return "Неверный период"
    status = write_setting(0x40, 0x06, periods[period][0], periods[period][1])
    if status == "OK":
        status = write_to_eeprom()
    return status

def set_averaging(count):
    """Усреднение (J5: 40h 0Ah)"""
    counts = {'1': (0x00, 0x00), '8': (0x00, 0x01), '64': (0x00, 0x02), '512': (0x00, 0x03)}
    if count not in counts:
        return "Неверное значение"
    status = write_setting(0x40, 0x0A, counts[count][0], counts[count][1])
    if status == "OK":
        status = write_to_eeprom()
    return status

def set_alarm(alarm_type):
    """Настройка тревоги (J5: 40h 0Ch)"""
    alarms = {'clamp': (0x00, 0x00), 'hold': (0x00, 0x01)}
    if alarm_type not in alarms:
        return "Неверный тип"
    status = write_setting(0x40, 0x0C, alarms[alarm_type][0], alarms[alarm_type][1])
    if status == "OK":
        status = write_to_eeprom()
    return status

def set_alarm_hold(value):
    """Alarm Hold/Clamp значение (J5: 41h 08h)"""
    upper = (value >> 8) & 0xFF
    lower = value & 0xFF
    status = write_setting(0x41, 0x08, upper, lower)
    if status == "OK":
        status = write_to_eeprom()
    return status

def set_display(on_off):
    """Настройка дисплея (J5: 40h 0Eh)"""
    displays = {'on': (0x00, 0x00), 'off': (0x00, 0x01)}
    if on_off not in displays:
        return "Неверное значение"
    status = write_setting(0x40, 0x0E, displays[on_off][0], displays[on_off][1])
    if status == "OK":
        status = write_to_eeprom()
    return status

def set_hysteresis(value_mm):
    """Гистерезис (J5: 41h 10h)"""
    if state.model_type == 'B015':
        raw = int(value_mm / 0.001)
    else:
        raw = int(value_mm / 0.01)
    upper = (raw >> 8) & 0xFF
    lower = raw & 0xFF
    status = write_setting(0x41, 0x10, upper, lower)
    if status == "OK":
        status = write_to_eeprom()
    return status

def set_threshold(level):
    """Уровень порога (J5: 40h 12h)"""
    levels = {'base': (0x00, 0x00), 'p400': (0x00, 0x01), 'p200': (0x00, 0x02), 'p100': (0x00, 0x03)}
    if level not in levels:
        return "Неверный уровень"
    status = write_setting(0x40, 0x12, levels[level][0], levels[level][1])
    if status == "OK":
        status = write_to_eeprom()
    return status

def set_zero_shift(value_mm):
    """Сдвиг нуля (J5: 41h 12h)"""
    if state.model_type == 'B015':
        raw = int(value_mm / 0.001)
    else:
        raw = int(value_mm / 0.01)
    upper = (raw >> 8) & 0xFF
    lower = raw & 0xFF
    status = write_setting(0x41, 0x12, upper, lower)
    if status == "OK":
        status = write_to_eeprom()
    return status

def set_sensitivity(sensitivity):
    """Чувствительность (J5: 40h 14h)"""
    sens = {
        'auto': (0x00, 0x00),
        '1': (0x00, 0x01),
        '2': (0x00, 0x02),
        '3': (0x00, 0x03),
        '4': (0x00, 0x04),
        '5': (0x00, 0x05),
        '6': (0x00, 0x06)
    }
    if sensitivity not in sens:
        return "Неверное значение"
    status = write_setting(0x40, 0x14, sens[sensitivity][0], sens[sensitivity][1])
    if status == "OK":
        status = write_to_eeprom()
    return status

def read_all_settings():
    """Чтение всех настроек"""
    settings = {}
    
    # Режим измерения
    upper, lower, _ = read_setting(0x40, 0x04)
    if upper is not None:
        modes = {(0x00, 0x00): '2-Pt', (0x00, 0x01): '1-Pt', (0x00, 0x02): 'ObSB'}
        settings['Mode'] = modes.get((upper, lower), 'Unknown')
    
    # Период дискретизации
    upper, lower, _ = read_setting(0x40, 0x06)
    if upper is not None:
        periods = {(0x00, 0x00): '500μs', (0x00, 0x01): '1000μs', 
                   (0x00, 0x02): '2000μs', (0x00, 0x03): '4000μs', (0x00, 0x04): 'AUTO'}
        settings['Sampling'] = periods.get((upper, lower), 'Unknown')
    
    # Усреднение
    upper, lower, _ = read_setting(0x40, 0x0A)
    if upper is not None:
        avgs = {(0x00, 0x00): '1', (0x00, 0x01): '8', (0x00, 0x02): '64', (0x00, 0x03): '512'}
        settings['Averaging'] = avgs.get((upper, lower), 'Unknown')
    
    # Полярность
    upper, lower, _ = read_setting(0x40, 0x08)
    if upper is not None:
        pols = {(0x00, 0x00): 'Light ON', (0x00, 0x01): 'Dark ON'}
        settings['Polarity'] = pols.get((upper, lower), 'Unknown')
    
    # Тревога
    upper, lower, _ = read_setting(0x40, 0x0C)
    if upper is not None:
        alarms = {(0x00, 0x00): 'Clamp', (0x00, 0x01): 'Hold'}
        settings['Alarm'] = alarms.get((upper, lower), 'Unknown')
    
    # Дисплей
    upper, lower, _ = read_setting(0x40, 0x0E)
    if upper is not None:
        settings['Display'] = 'ON' if (upper, lower) == (0x00, 0x00) else 'OFF'
    
    # Чувствительность
    upper, lower, _ = read_setting(0x40, 0x14)
    if upper is not None:
        if upper == 0x00 and lower == 0x00:
            settings['Sensitivity'] = 'AUTO'
        else:
            settings['Sensitivity'] = str(lower)
    
    return settings

# ============================================================================
# МЕНЮ
# ============================================================================

def print_menu():
    """Вывод меню"""
    print("\n" + "=" * 80)
    print(" " * 25 + "МЕНЮ УПРАВЛЕНИЯ SICK OD Mini Pro")
    print("=" * 80)
    print("\n  ═══════════════════════════════════════════════════════════════════════════")
    print("  【J4】КОМАНДЫ ЧТЕНИЯ И УПРАВЛЕНИЯ")
    print("  ═══════════════════════════════════════════════════════════════════════════")
    print("    [1]  Чтение измеренного значения (B0 01)")
    print("    [2]  Чтение статуса выхода (B0 02)")
    print("    [3]  Чтение типа модели (01 00)")
    print("    [4]  Чтение всех настроек")
    print("  ───────────────────────────────────────────────────────────────────────────")
    print("    [5]  Запись настроек в EEPROM (A0 00)")
    print("    [6]  Отмена настроек (A0 01)")
    print("  ───────────────────────────────────────────────────────────────────────────")
    print("    [7]  Лазер ВКЛ (A0 03)")
    print("    [8]  Лазер ВЫКЛ (A0 02)")
    print("  ───────────────────────────────────────────────────────────────────────────")
    print("    [9]  Сброс нуля (A1 00)")
    print("    [10] Восстановление нуля (A1 01)")
    print("  ───────────────────────────────────────────────────────────────────────────")
    print("    [11] Блокировка кнопок (A1 04)")
    print("    [12] Разблокировка кнопок (A1 05)")
    print("  ───────────────────────────────────────────────────────────────────────────")
    print("    [13] Инициализация сенсора (40 00)")
    print()
    print("  ═══════════════════════════════════════════════════════════════════════════")
    print("  【J5】НАСТРОЙКА ПАРАМЕТРОВ")
    print("  ═══════════════════════════════════════════════════════════════════════════")
    print("    [14] Режим измерения (40 04)")
    print("    [15] Период дискретизации (40 06)")
    print("    [16] Усреднение (40 0A)")
    print("    [17] Полярность выхода (40 08)")
    print("    [18] Настройка тревоги (40 0C)")
    print("    [19] Настройка дисплея (40 0E)")
    print("    [20] Чувствительность (40 14)")
    print("    [21] Уровень порога (40 12)")
    print("  ───────────────────────────────────────────────────────────────────────────")
    print("    [22] Порог ближней зоны (41 00)")
    print("    [23] Порог дальней зоны (41 02)")
    print("    [24] Порог ObSB (41 04)")
    print("    [25] Гистерезис ObSB (41 06)")
    print("    [26] Гистерезис (41 10)")
    print("    [27] Сдвиг нуля (41 12)")
    print("    [28] Alarm Hold значение (41 08)")
    print()
    print("  ═══════════════════════════════════════════════════════════════════════════")
    print("  [0]  Выход в режим измерений    |    [Q]  Выход из программы")
    print("  ═══════════════════════════════════════════════════════════════════════════")
    print("=" * 80)

def menu_measurement_mode():
    """Режим измерения (J5: 40h 04h)"""
    print("\n  Режим измерения:")
    print("    [1] 2-Pt Teach (2-точечный)")
    print("    [2] 1-Pt Teach (1-точечный)")
    print("    [3] ObSB (фон)")
    choice = input("  Выбор: ").strip()
    modes = {'1': '2pt', '2': '1pt', '3': 'obsb'}
    if choice in modes:
        status = set_measurement_mode(modes[choice])
        print(f"  {'✓' if status == 'OK' else '✗'} Результат: {status}")
    else:
        print("  Неверный выбор")

def menu_sampling_period():
    """Период дискретизации (J5: 40h 06h)"""
    print("\n  Период дискретизации:")
    print("    [1] 500 μs (2 kHz)")
    print("    [2] 1000 μs (1 kHz)")
    print("    [3] 2000 μs (500 Hz)")
    print("    [4] 4000 μs (250 Hz)")
    print("    [5] AUTO")
    choice = input("  Выбор: ").strip()
    periods = {'1': '500us', '2': '1000us', '3': '2000us', '4': '4000us', '5': 'auto'}
    if choice in periods:
        status = set_sampling_period(periods[choice])
        print(f"  {'✓' if status == 'OK' else '✗'} Результат: {status}")
    else:
        print("  Неверный выбор")

def menu_averaging():
    """Усреднение (J5: 40h 0Ah)"""
    print("\n  Усреднение:")
    print("    [1] 1 измерение")
    print("    [2] 8 измерений")
    print("    [3] 64 измерения")
    print("    [4] 512 измерений")
    choice = input("  Выбор: ").strip()
    counts = {'1': '1', '2': '8', '3': '64', '4': '512'}
    if choice in counts:
        status = set_averaging(counts[choice])
        print(f"  {'✓' if status == 'OK' else '✗'} Результат: {status}")
    else:
        print("  Неверный выбор")

def menu_output_polarity():
    """Полярность выхода (J5: 40h 08h)"""
    print("\n  Полярность выхода:")
    print("    [1] Light ON (включается при превышении)")
    print("    [2] Dark ON (включается при снижении)")
    choice = input("  Выбор: ").strip()
    polarities = {'1': 'light_on', '2': 'dark_on'}
    if choice in polarities:
        status = set_output_polarity(polarities[choice])
        print(f"  {'✓' if status == 'OK' else '✗'} Результат: {status}")
    else:
        print("  Неверный выбор")

def menu_alarm():
    """Настройка тревоги (J5: 40h 0Ch)"""
    print("\n  Режим тревоги:")
    print("    [1] Clamp (24 mA / 16 V)")
    print("    [2] Hold (последнее значение)")
    choice = input("  Выбор: ").strip()
    alarms = {'1': 'clamp', '2': 'hold'}
    if choice in alarms:
        status = set_alarm(alarms[choice])
        print(f"  {'✓' if status == 'OK' else '✗'} Результат: {status}")
    else:
        print("  Неверный выбор")

def menu_display():
    """Настройка дисплея (J5: 40h 0Eh)"""
    print("\n  Дисплей при блокировке:")
    print("    [1] ВКЛ")
    print("    [2] ВЫКЛ")
    choice = input("  Выбор: ").strip()
    displays = {'1': 'on', '2': 'off'}
    if choice in displays:
        status = set_display(displays[choice])
        print(f"  {'✓' if status == 'OK' else '✗'} Результат: {status}")
    else:
        print("  Неверный выбор")

def menu_sensitivity():
    """Чувствительность (J5: 40h 14h)"""
    print("\n  Чувствительность:")
    print("    [1] AUTO")
    print("    [2] 1 (минимальная)")
    print("    [3] 2")
    print("    [4] 3")
    print("    [5] 4")
    print("    [6] 5")
    print("    [7] 6 (максимальная)")
    choice = input("  Выбор: ").strip()
    sens = {'1': 'auto', '2': '1', '3': '2', '4': '3', '5': '4', '6': '5', '7': '6'}
    if choice in sens:
        status = set_sensitivity(sens[choice])
        print(f"  {'✓' if status == 'OK' else '✗'} Результат: {status}")
    else:
        print("  Неверный выбор")

def menu_threshold():
    """Уровень порога (J5: 40h 12h)"""
    print("\n  Уровень порога:")
    print("    [1] Base (низкий)")
    print("    [2] P400 (верхний)")
    print("    [3] P200 (средний)")
    print("    [4] P100 (низкий)")
    choice = input("  Выбор: ").strip()
    levels = {'1': 'base', '2': 'p400', '3': 'p200', '4': 'p100'}
    if choice in levels:
        status = set_threshold(levels[choice])
        print(f"  {'✓' if status == 'OK' else '✗'} Результат: {status}")
    else:
        print("  Неверный выбор")

def menu_numeric_setting(prompt, set_func, unit="мм"):
    """Меню для числовых настроек"""
    try:
        value = float(input(f"  {prompt} ({unit}): ").strip())
        status = set_func(value)
        print(f"  {'✓' if status == 'OK' else '✗'} Результат: {status}")
    except ValueError:
        print("  Неверное значение")

def handle_menu():
    """Обработка меню"""
    state.menu_active = True
    print_menu()
    
    try:
        choice = input("\n  Ваш выбор: ").strip().upper()
        
        if choice == '0':
            state.menu_active = False
            return
        elif choice == 'Q':
            state.running = False
            state.menu_active = False
            return
        
        # J4 - Чтение
        elif choice == '1':
            value, status = read_measurement()
            if value is not None:
                mm = convert_to_mm(value, state.model_type)
                print(f"  ✓ Измерение: {mm:+.3f} мм (сырое: {value})")
            else:
                print(f"  ✗ Ошибка: {status}")
        
        elif choice == '2':
            status_on, err = read_output_status()
            if err is None:
                print(f"  ✓ Выход: {'ВКЛ' if status_on else 'ВЫКЛ'}")
            else:
                print(f"  ✗ Ошибка: {err}")
        
        elif choice == '3':
            model, err = read_model_type()
            state.model_type = model
            print(f"  {'✓' if err == 'OK' else '✗'} Тип модели: OD1-{model}")
        
        elif choice == '4':
            settings = read_all_settings()
            print("\n  ═══════════════════════════════════════════════════════════════════")
            print("  Текущие настройки:")
            print("  ═══════════════════════════════════════════════════════════════════")
            for key, value in settings.items():
                print(f"    {key:15} : {value}")
            print("  ═══════════════════════════════════════════════════════════════════")
        
        # J4 - Управление
        elif choice == '5':
            status = write_to_eeprom()
            print(f"  {'✓' if status == 'OK' else '✗'} Запись в EEPROM: {status}")
        
        elif choice == '6':
            status = dismiss_setting()
            print(f"  {'✓' if status == 'OK' else '✗'} Отмена: {status}")
        
        elif choice == '7':
            status = laser_control(True)
            print(f"  {'✓' if status == 'OK' else '✗'} Лазер ВКЛ: {status}")
        
        elif choice == '8':
            status = laser_control(False)
            print(f"  {'✓' if status == 'OK' else '✗'} Лазер ВЫКЛ: {status}")
        
        elif choice == '9':
            status = zero_reset(True)
            print(f"  {'✓' if status == 'OK' else '✗'} Сброс нуля: {status}")
        
        elif choice == '10':
            status = zero_reset(False)
            print(f"  {'✓' if status == 'OK' else '✗'} Восстановление нуля: {status}")
        
        elif choice == '11':
            status = key_lock(True)
            print(f"  {'✓' if status == 'OK' else '✗'} Блокировка: {status}")
        
        elif choice == '12':
            status = key_lock(False)
            print(f"  {'✓' if status == 'OK' else '✗'} Разблокировка: {status}")
        
        elif choice == '13':
            print("  ⚠ Внимание: Инициализация сбросит все настройки!")
            confirm = input("  Продолжить? (y/n): ").strip().lower()
            if confirm == 'y':
                status = initialize_sensor()
                print(f"  {'✓' if status == 'OK' else '✗'} Инициализация: {status}")
        
        # J5 - Настройки
        elif choice == '14':
            menu_measurement_mode()
        elif choice == '15':
            menu_sampling_period()
        elif choice == '16':
            menu_averaging()
        elif choice == '17':
            menu_output_polarity()
        elif choice == '18':
            menu_alarm()
        elif choice == '19':
            menu_display()
        elif choice == '20':
            menu_sensitivity()
        elif choice == '21':
            menu_threshold()
        elif choice == '22':
            menu_numeric_setting("Порог ближней зоны", set_near_threshold)
        elif choice == '23':
            menu_numeric_setting("Порог дальней зоны", set_far_threshold)
        elif choice == '24':
            menu_numeric_setting("Порог ObSB", set_obsb_threshold)
        elif choice == '25':
            menu_numeric_setting("Гистерезис ObSB", set_obsb_hysteresis)
        elif choice == '26':
            menu_numeric_setting("Гистерезис", set_hysteresis)
        elif choice == '27':
            menu_numeric_setting("Сдвиг нуля", set_zero_shift)
        elif choice == '28':
            try:
                value = int(input("  Alarm Hold значение (0-9999): ").strip())
                status = set_alarm_hold(value)
                print(f"  {'✓' if status == 'OK' else '✗'} Результат: {status}")
            except ValueError:
                print("  Неверное значение")
        
        else:
            print("  Неверный выбор!")
    
    except Exception as e:
        print(f"  Ошибка меню: {e}")
    
    state.menu_active = False

# ============================================================================
# ПОТОК ИЗМЕРЕНИЙ
# ============================================================================

def measurement_thread():
    """Фоновый поток для непрерывных измерений"""
    counter = 0
    
    while state.running:
        if state.menu_active:
            time.sleep(0.1)
            continue
        
        try:
            value, status = read_measurement()
            
            with state.lock:
                if value is not None:
                    state.last_value = value
                    state.last_status = "OK"
                    counter += 1
                    state.counter = counter
                else:
                    state.last_status = status
            
            time.sleep(READ_INTERVAL)
            
        except Exception as e:
            with state.lock:
                state.last_status = f"Ошибка: {e}"
            time.sleep(1)

def check_keyboard_input():
    """Проверка ввода с клавиатуры"""
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        if sys.stdin in [sys.stdin]:
            import select
            if select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline().strip().upper()
                if line:
                    if line == 'M':
                        handle_menu()
                    elif line == 'Q':
                        state.running = False
                    return True
    except:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    return False

# ============================================================================
# ОСНОВНАЯ ПРОГРАММА
# ============================================================================

def main():
    global ser, state
    
    print("=" * 80)
    print(" " * 20 + "SICK OD Mini Pro - Программа управления")
    print(" " * 30 + "Плата: Rock Pi S")
    print("=" * 80)
    print("\n  Подсказки:")
    print("    • Нажмите [M] + Enter для входа в меню")
    print("    • Нажмите [Q] + Enter для выхода")
    print("    • Измерения продолжаются в фоновом режиме")
    print("    • Все команды из J4 и J5 документации поддерживаются")
    print("=" * 80)
    
    # Открытие порта
    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=TIMEOUT
        )
        print(f"\n  ✓ Порт {SERIAL_PORT} открыт")
        print(f"  ✓ Скорость: {BAUD_RATE} бод")
    except serial.SerialException as e:
        print(f"\n  ✗ Ошибка порта: {e}")
        print("    Проверьте: ls /dev/ttyUSB*")
        print("    Права: sudo usermod -a -G dialout $USER")
        return
    
    # Определение модели
    print("\n  Определение типа датчика...")
    model, err = read_model_type()
    state.model_type = model
    print(f"  ✓ Тип: OD1-{model}")
    
    # Запуск потока измерений
    thread = threading.Thread(target=measurement_thread, daemon=True)
    thread.start()
    
    print("\n" + "=" * 80)
    print(" " * 25 + "НАЧАЛО РАБОТЫ (M = меню, Q = выход)")
    print("=" * 80 + "\n")
    
    try:
        while state.running:
            if not state.menu_active:
                with state.lock:
                    mm = convert_to_mm(state.last_value, state.model_type)
                    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    
                    sys.stdout.write(f"\r  [{timestamp}] #{state.counter:04d} | "
                                   f"Значение: {mm:+8.3f} мм | "
                                   f"Сырое: {state.last_value:6d} | "
                                   f"Статус: {state.last_status:<20} ")
                    sys.stdout.flush()
            
            time.sleep(0.1)
            
            # Проверка ввода (упрощённая)
            import select
            if select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline().strip().upper()
                if line == 'M':
                    handle_menu()
                elif line == 'Q':
                    state.running = False
    
    except KeyboardInterrupt:
        print("\n")
    finally:
        state.running = False
        time.sleep(0.5)
        
        if ser and ser.is_open:
            ser.close()
            print(f"\n  ✓ Порт {SERIAL_PORT} закрыт")
        
        print("\n" + "=" * 80)
        print(" " * 30 + "ПРОГРАММА ЗАВЕРШЕНА")
        print("=" * 80)

if __name__ == '__main__':
    main()
