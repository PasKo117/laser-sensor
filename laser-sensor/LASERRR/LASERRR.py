"""
SICK OD Mini Pro — Stable Dual Sensor Version
"""

import socket
import json
import threading
import time
import sys
import os
from datetime import datetime

IS_WINDOWS = sys.platform == 'win32'
if not IS_WINDOWS:
    import termios, tty, select

# ============================================================================
# КОНФИГУРАЦИЯ
# ============================================================================
# PI_IP = '192.168.1.41'
# PI_USER = 'aptyp'
# PI_PASS = 'aptyp'
# PI_SERVER_PATH = '/home/aptyp/sensor_server.py'
# PI_TCP_PORT = 5000

#PI_IP = '192.168.1.37'
#PI_USER = 'rock'
#PI_PASS = 'rock'
#PI_SERVER_PATH = '/home/rock/sensor_server.py'
#PI_TCP_PORT = 5000

PI_IP = '192.168.2.37'
PI_USER = 'pi'
PI_PASS = 'rasprobot1'
PI_SERVER_PATH = '/home/pi/sensor_server.py'
PI_TCP_PORT = 5000

PI_SSH_PORT = 22

# ============================================================================
# СЕРВЕРНЫЙ КОД ДЛЯ RASPBERRY PI (FIXED - DUAL SENSOR)
# ============================================================================
PI_SERVER_CODE = r'''#!/usr/bin/env python3
import serial, time, struct, socket, json, threading, sys

SERIAL_PORT = '/dev/ttyUSB2'                                    #USB меняется в зависимости от порта подключения         1-свободный     2-на резаке
SERIAL_PORT_1 = '/dev/ttyUSB1'
SERIAL_PORT_2 = '/dev/ttyUSB2'
BAUD_RATE = 9600
TIMEOUT = 0.3

STX, ETX, ACK, NAK = 0x02, 0x03, 0x06, 0x15
CMD_READ_MEASUREMENT, CMD_WRITE_SETTING, CMD_READ_SETTING = 0x43, 0x57, 0x52

ser_1 = None
ser_2 = None
model_type_1 = 'B035'
model_type_2 = 'B035'
lock_1 = threading.Lock()
lock_2 = threading.Lock()
streaming = False
stream_clients = []
stream_lock = threading.Lock()
active_sensor = None  # None = оба, 1 = только первый, 2 = только второй

def calculate_bcc(data_bytes): #вычисляем BCC
    bcc = 0
    for byte in data_bytes: bcc ^= byte
    return bcc

def build_command(command, data1, data2): #сбор пакета
    bcc = calculate_bcc([command, data1, data2])
    return bytes([STX, command, data1, data2, ETX, bcc])

def open_serial(port): #открытие Serial порта
    try:
        ser = serial.Serial(port, BAUD_RATE, bytesize=serial.EIGHTBITS,
                           parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=TIMEOUT)
        print(f"[PI] Port {port} opened @ {BAUD_RATE}")
        return ser
    except Exception as e:
        print(f"[PI] Serial Error on {port}: {e}")
        return None

def send_command(ser, lock, command, data1, data2, wait_time=0.05): #сбор,  отправка команды
    if not ser or not ser.is_open: return None, "Port Closed"
    with lock:
        cmd = build_command(command, data1, data2)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        ser.write(cmd)
        time.sleep(wait_time)
        response = ser.read(20)
    return response, None

def parse_response(response): #проверка, не наебнулся ли ответ
    if not response or len(response) < 6: return None, None, "Short"
    if response[0] != STX: return None, None, "NoSTX"
    if response[1] == NAK: return None, None, f"NAK(0x{response[2]:02X})"
    if response[1] != ACK: return None, None, "NoACK"
    exp_bcc = calculate_bcc([response[1], response[2], response[3]])
    if response[5] != exp_bcc: return None, None, "BadBCC"
    if response[4] != ETX: return None, None, "NoETX"
    return response[2], response[3], "OK"

def read_measurement(ser, lock): #чтение расстояния
    resp, err = send_command(ser, lock, CMD_READ_MEASUREMENT, 0xB0, 0x01)
    if err: return None, err
    upper, lower, status = parse_response(resp)
    if status != "OK": return None, status
    val = struct.unpack('>h', bytes([upper, lower]))[0]
    return val, "OK"

def read_model_type(ser, lock): #чтение модели датчика (хотя нахуй оно нужно)
    upper, lower, status = read_setting(ser, lock, 0x01, 0x00)
    if status != "OK": return 'B035', status
    return {0x0F:'B015',0x23:'B035',0x64:'B100'}.get(upper, 'B035'), "OK"

def read_setting(ser, lock, addr_h, addr_l): #чтение настроек с подкоманд h l
    resp, err = send_command(ser, lock, CMD_READ_SETTING, addr_h, addr_l)
    if err: return None, None, err
    return parse_response(resp)

def read_output_status(ser, lock): #чтение статуса выхода 
    resp, err = send_command(ser, lock, CMD_READ_MEASUREMENT, 0xB0, 0x02)
    if err: return None, err
    upper, lower, status = parse_response(resp)
    if status != "OK": return None, status
    return (lower & 0x01) == 1, "OK"

def write_to_eeprom(ser, lock): #сохранение настроек в EEPROM, настройка без этого исчезнет
    resp, err = send_command(ser, lock, CMD_READ_MEASUREMENT, 0xA0, 0x00)
    return err or parse_response(resp)[2] if resp else "Err"

def dismiss_setting(ser, lock): #отмена настроек до предыдущего значения
    resp, err = send_command(ser, lock, CMD_READ_MEASUREMENT, 0xA0, 0x01)
    return err or parse_response(resp)[2] if resp else "Err"

def laser_control(ser, lock, on_off): #включение/выключение лазера (ну больше как режим сна)
    resp, err = send_command(ser, lock, CMD_READ_MEASUREMENT, 0xA0, 0x03 if on_off else 0x02)
    return err or parse_response(resp)[2] if resp else "Err"

def zero_reset(ser, lock, execute): #сброс настроек нуля расстояния
    resp, err = send_command(ser, lock, CMD_READ_MEASUREMENT, 0xA1, 0x00 if execute else 0x01)
    return err or parse_response(resp)[2] if resp else "Err"

def key_lock(ser, lock, execute): #а нахуя кнопки тебе, выключаем
    resp, err = send_command(ser, lock, CMD_READ_MEASUREMENT, 0xA1, 0x04 if execute else 0x05)
    return err or parse_response(resp)[2] if resp else "Err"

def initialize_sensor(ser, lock): #сборс настроек до завода (к хуям собачьим)
    resp, err = send_command(ser, lock, CMD_READ_MEASUREMENT, 0x40, 0x00)
    return err or parse_response(resp)[2] if resp else "Err"

def write_setting(ser, lock, addr_h, addr_l, val_h, val_l): #для записи настроек
    _,_,st = read_setting(ser, lock, addr_h, addr_l)
    if st != "OK": return st
    resp, err = send_command(ser, lock, CMD_WRITE_SETTING, val_h, val_l)
    return err or parse_response(resp)[2] if resp else "Err"

def _to_raw(mm, model_type): #перевод в мм
    unit = 0.001 if model_type=='B015' else 0.01
    return int(mm / unit)

def _split(val): return (val>>8)&0xFF, val&0xFF #вспомогательное, разбиение на 2 байта

def _save(ser, lock, model_type, addr_h, addr_l, mm): #вспомогательная
    raw = _to_raw(mm, model_type); u,l = _split(raw)
    st = write_setting(ser, lock, addr_h, addr_l, u, l)
    return write_to_eeprom(ser, lock) if st=="OK" else st

def set_measurement_mode(ser, lock, mode): #уст.настр. режима измерения по мануалу
    m = {'2pt':(0,0),'1pt':(0,1),'obsb':(0,2)}.get(mode)
    if m is None: return "BadMode"
    st = write_setting(ser, lock, 0x40,0x04,*m)
    return write_to_eeprom(ser, lock) if st=="OK" else st

#с мануала
def set_near_threshold(ser, lock, model_type, mm): return _save(ser, lock, model_type, 0x41,0x00,mm)
def set_far_threshold(ser, lock, model_type, mm): return _save(ser, lock, model_type, 0x41,0x02,mm)
def set_obsb_threshold(ser, lock, model_type, mm): return _save(ser, lock, model_type, 0x41,0x04,mm)
def set_obsb_hysteresis(ser, lock, model_type, mm): return _save(ser, lock, model_type, 0x41,0x06,mm)
def set_hysteresis(ser, lock, model_type, mm): return _save(ser, lock, model_type, 0x41,0x10,mm)
def set_zero_shift(ser, lock, model_type, mm): return _save(ser, lock, model_type, 0x41,0x12,mm)

def set_output_polarity(ser, lock, pol): #уст.настр. полярности выхода
    m = {'light_on':(0,0),'dark_on':(0,1)}.get(pol)
    if m is None: return "BadPol"
    st = write_setting(ser, lock, 0x40,0x08,*m)
    return write_to_eeprom(ser, lock) if st=="OK" else st

def set_sampling_period(ser, lock, per): #уст.настр. периода дискретизации
    m = {'500us':(0,0),'1000us':(0,1),'2000us':(0,2),'4000us':(0,3),'auto':(0,4)}.get(per)
    if m is None: return "BadPeriod, try 500/1000/2000/4000us OR auto"
    st = write_setting(ser, lock, 0x40,0x06,*m)
    return write_to_eeprom(ser, lock) if st=="OK" else st

def set_averaging(ser, lock, cnt): #уст.наст. установки усреднения
    m = {'1':(0,0),'8':(0,1),'64':(0,2),'512':(0,3)}.get(cnt)
    if m is None: return "BadAvg, pls try: 1/8/64/512"
    st = write_setting(ser, lock, 0x40,0x0A,*m)
    return write_to_eeprom(ser, lock) if st=="OK" else st

def set_alarm(ser, lock, alarm): #уст.наст. типа тревоги
    m = {'clamp':(0,0),'hold':(0,1)}.get(alarm)
    if m is None: return "BadAlarm, try clamp/hold"
    st = write_setting(ser, lock, 0x40,0x0C,*m)
    return write_to_eeprom(ser, lock) if st=="OK" else st

def set_alarm_hold(ser, lock, val): #уст.наст. удержания тревоги
    u,l = _split(int(val))
    st = write_setting(ser, lock, 0x41,0x08,u,l)
    return write_to_eeprom(ser, lock) if st=="OK" else st

def set_display(ser, lock, on_off): #настройки дисплея
    m = {'on':(0,0),'off':(0,1)}.get(on_off)
    if m is None: return "BadDisp, try on/off"
    st = write_setting(ser, lock, 0x40,0x0E,*m)
    return write_to_eeprom(ser, lock) if st=="OK" else st

def set_threshold(ser, lock, level): #уст.наст. порога
    m = {'base':(0,0),'p400':(0,1),'p200':(0,2),'p100':(0,3)}.get(level)
    if m is None: return "BadLevel, try base/p400/p200/p100"
    st = write_setting(ser, lock, 0x40,0x12,*m)
    return write_to_eeprom(ser, lock) if st=="OK" else st

def set_sensitivity(ser, lock, sens): #установка чувствительности
    m = {'auto':(0,0)}.get(sens)
    if m is None:
        try:
            n=int(sens)
            if 1<=n<=6: m=(0,n)
        except: pass
    if m is None: return "BadSens"
    st = write_setting(ser, lock, 0x40,0x14,*m)
    return write_to_eeprom(ser, lock) if st=="OK" else st

def read_all_settings(ser, lock): #чтение всех нахуй настроек
    s = {}
    def get(addr_h, addr_l, name, mapping=None):
        u,l,st = read_setting(ser, lock, addr_h, addr_l)
        if st=="OK" and mapping: s[name] = mapping.get((u,l), f"0x{u:02X}{l:02X}")
        elif st=="OK": s[name] = f"0x{u:02X}{l:02X}"
    get(0x40,0x04,'Mode',{(0,0):'2-Pt',(0,1):'1-Pt',(0,2):'ObSB'})
    get(0x40,0x06,'Sampling',{(0,0):'500us',(0,1):'1ms',(0,2):'2ms',(0,3):'4ms',(0,4):'AUTO'})
    get(0x40,0x0A,'Averaging',{(0,0):'1',(0,1):'8',(0,2):'64',(0,3):'512'})
    get(0x40,0x08,'Polarity',{(0,0):'LightON',(0,1):'DarkON'})
    get(0x40,0x0C,'Alarm',{(0,0):'Clamp',(0,1):'Hold'})
    get(0x40,0x0E,'Display',{(0,0):'ON',(0,1):'OFF'})
    u,l,_ = read_setting(ser, lock, 0x40,0x14)
    if u==0 and l==0: s['Sensitivity']='AUTO'
    elif u is not None: s['Sensitivity']=str(l)
    return s

def stream_sender(): #стрим... сюда лучше сильно не лезть пока.. но надо бы
    """Отдельный поток для рассылки стрим-данных"""
    global streaming, stream_clients, model_type_1, model_type_2, ser_1, ser_2, active_sensor
    while True:
        if streaming:
            try:
                sensors = []
                if active_sensor is None or active_sensor == 1:
                    sensors.append((1, ser_1, lock_1, model_type_1))
                if active_sensor is None or active_sensor == 2:
                    sensors.append((2, ser_2, lock_2, model_type_2))
                
                for sid, ser, lock, model_type in sensors:
                    if ser and ser.is_open:
                        val, st = read_measurement(ser, lock)
                        if st == "OK" and val is not None:
                            unit = 0.001 if model_type=='B015' else 0.01
                            msg = {"type":"stream","sensor_id":sid,"value":val*unit,"raw":val,"ts":time.time()}
                            data = (json.dumps(msg) + "\n").encode()
                            with stream_lock:
                                dead_clients = []
                                for client in stream_clients:
                                    try:
                                        client.send(data)
                                    except:
                                        dead_clients.append(client)
                                for client in dead_clients:
                                    stream_clients.remove(client)
            except Exception as e:
                print(f"[PI] StreamErr: {e}")
        time.sleep(0.01)

def handle_client(conn, addr): #приём JSON
    global streaming, model_type_1, model_type_2, stream_clients, ser_1, ser_2, active_sensor
    print(f"[PI] Client: {addr}")
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    buffer = ""
    try:
        while True:
            data = conn.recv(4096)
            if not data:
                break
            buffer += data.decode('utf-8', errors='ignore')

            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                if not line.strip():
                    continue
                try:
                    req = json.loads(line.strip())
                    act = req.get('action')
                    sensor_id = req.get('sensor_id', None)
                    resp = {"status":"error","data":None,"msg":"Unknown"}

                    # Выбор сенсора для операции
                    ser = None
                    lock = None
                    model_type = None
                    if sensor_id == 1:
                        ser, lock, model_type = ser_1, lock_1, model_type_1
                    elif sensor_id == 2:
                        ser, lock, model_type = ser_2, lock_2, model_type_2
                    elif sensor_id is None and active_sensor is not None:
                        # Если не указан сенсор в запросе, но активен один - используем его
                        if active_sensor == 1:
                            ser, lock, model_type = ser_1, lock_1, model_type_1
                        else:
                            ser, lock, model_type = ser_2, lock_2, model_type_2

                    if act == 'ping':
                        resp = {"status":"ok","data":"pong"}
                    elif act == 'get_model':
                        if ser and lock:
                            m,_ = read_model_type(ser, lock)
                            if sensor_id == 1:
                                model_type_1 = m
                            elif sensor_id == 2:
                                model_type_2 = m
                            resp = {"status":"ok","data":m}
                        else:
                            resp = {"status":"error","msg":"Sensor not available"}
                    elif act == 'measure':
                        if ser and lock:
                            val,st = read_measurement(ser, lock)
                            if st=="OK" and val is not None:
                                unit = 0.001 if model_type=='B015' else 0.01
                                resp = {"status":"ok","data":val*unit,"sensor_id":sensor_id}
                            else:
                                resp = {"status":st,"data":None}
                        else:
                            resp = {"status":"error","msg":"Sensor not available"}
                    elif act == 'read_status':
                        if ser and lock:
                            v,st = read_output_status(ser, lock)
                            resp = {"status":st,"data":v}
                        else:
                            resp = {"status":"error","msg":"Sensor not available"}
                    elif act == 'read_all':
                        if ser and lock:
                            resp = {"status":"ok","data":read_all_settings(ser, lock)}
                        else:
                            resp = {"status":"error","msg":"Sensor not available"}
                    elif act == 'write_eeprom':
                        if ser and lock:
                            resp = {"status":write_to_eeprom(ser, lock)}
                        else:
                            resp = {"status":"error","msg":"Sensor not available"}
                    elif act == 'dismiss':
                        if ser and lock:
                            resp = {"status":dismiss_setting(ser, lock)}
                        else:
                            resp = {"status":"error","msg":"Sensor not available"}
                    elif act == 'laser_on':
                        if ser and lock:
                            resp = {"status":laser_control(ser, lock, True)}
                        else:
                            resp = {"status":"error","msg":"Sensor not available"}
                    elif act == 'laser_off':
                        if ser and lock:
                            resp = {"status":laser_control(ser, lock, False)}
                        else:
                            resp = {"status":"error","msg":"Sensor not available"}
                    elif act == 'zero_reset':
                        if ser and lock:
                            resp = {"status":zero_reset(ser, lock, True)}
                        else:
                            resp = {"status":"error","msg":"Sensor not available"}
                    elif act == 'zero_restore':
                        if ser and lock:
                            resp = {"status":zero_reset(ser, lock, False)}
                        else:
                            resp = {"status":"error","msg":"Sensor not available"}
                    elif act == 'key_lock':
                        if ser and lock:
                            resp = {"status":key_lock(ser, lock, True)}
                        else:
                            resp = {"status":"error","msg":"Sensor not available"}
                    elif act == 'key_unlock':
                        if ser and lock:
                            resp = {"status":key_lock(ser, lock, False)}
                        else:
                            resp = {"status":"error","msg":"Sensor not available"}
                    elif act == 'init_sensor':
                        if ser and lock:
                            resp = {"status":initialize_sensor(ser, lock)}
                        else:
                            resp = {"status":"error","msg":"Sensor not available"}
                    elif act == 'set_mode':
                        if ser and lock:
                            resp = {"status":set_measurement_mode(ser, lock, req.get('mode'))}
                        else:
                            resp = {"status":"error","msg":"Sensor not available"}
                    elif act == 'set_sampling':
                        if ser and lock:
                            resp = {"status":set_sampling_period(ser, lock, req.get('period'))}
                        else:
                            resp = {"status":"error","msg":"Sensor not available"}
                    elif act == 'set_averaging':
                        if ser and lock:
                            resp = {"status":set_averaging(ser, lock, req.get('count'))}
                        else:
                            resp = {"status":"error","msg":"Sensor not available"}
                    elif act == 'set_polarity':
                        if ser and lock:
                            resp = {"status":set_output_polarity(ser, lock, req.get('polarity'))}
                        else:
                            resp = {"status":"error","msg":"Sensor not available"}
                    elif act == 'set_alarm':
                        if ser and lock:
                            resp = {"status":set_alarm(ser, lock, req.get('type'))}
                        else:
                            resp = {"status":"error","msg":"Sensor not available"}
                    elif act == 'set_display':
                        if ser and lock:
                            resp = {"status":set_display(ser, lock, req.get('on_off'))}
                        else:
                            resp = {"status":"error","msg":"Sensor not available"}
                    elif act == 'set_sensitivity':
                        if ser and lock:
                            resp = {"status":set_sensitivity(ser, lock, req.get('sens'))}
                        else:
                            resp = {"status":"error","msg":"Sensor not available"}
                    elif act == 'set_threshold':
                        if ser and lock:
                            resp = {"status":set_threshold(ser, lock, req.get('level'))}
                        else:
                            resp = {"status":"error","msg":"Sensor not available"}
                    elif act == 'set_near':
                        if ser and lock:
                            resp = {"status":set_near_threshold(ser, lock, model_type, req.get('mm'))}
                        else:
                            resp = {"status":"error","msg":"Sensor not available"}
                    elif act == 'set_far':
                        if ser and lock:
                            resp = {"status":set_far_threshold(ser, lock, model_type, req.get('mm'))}
                        else:
                            resp = {"status":"error","msg":"Sensor not available"}
                    elif act == 'set_obsb':
                        if ser and lock:
                            resp = {"status":set_obsb_threshold(ser, lock, model_type, req.get('mm'))}
                        else:
                            resp = {"status":"error","msg":"Sensor not available"}
                    elif act == 'set_obsb_hyst':
                        if ser and lock:
                            resp = {"status":set_obsb_hysteresis(ser, lock, model_type, req.get('mm'))}
                        else:
                            resp = {"status":"error","msg":"Sensor not available"}
                    elif act == 'set_hyst':
                        if ser and lock:
                            resp = {"status":set_hysteresis(ser, lock, model_type, req.get('mm'))}
                        else:
                            resp = {"status":"error","msg":"Sensor not available"}
                    elif act == 'set_zero_shift':
                        if ser and lock:
                            resp = {"status":set_zero_shift(ser, lock, model_type, req.get('mm'))}
                        else:
                            resp = {"status":"error","msg":"Sensor not available"}
                    elif act == 'set_alarm_hold':
                        if ser and lock:
                            resp = {"status":set_alarm_hold(ser, lock, req.get('value'))}
                        else:
                            resp = {"status":"error","msg":"Sensor not available"}
                    elif act == 'stream_start':
                        with stream_lock:
                            if conn not in stream_clients:
                                stream_clients.append(conn)
                            streaming = True
                        resp = {"status":"ok","data":"started"}
                    elif act == 'stream_stop':
                        with stream_lock:
                            if conn in stream_clients:
                                stream_clients.remove(conn)
                            if not stream_clients:
                                streaming = False
                        resp = {"status":"ok","data":"stopped"}
                    elif act == 'stream_set_sensor':
                        # Установка активного сенсора для стрима: 1, 2 или None (оба)
                        sid = req.get('sensor_id')
                        if sid in (1, 2, None):
                            active_sensor = sid
                            resp = {"status":"ok","data":f"active_sensor={active_sensor}"}
                        else:
                            resp = {"status":"error","msg":"Invalid sensor_id"}
                    elif act == 'stream_toggle_pause':
                        # Пауза/возобновление стрима
                        streaming = not streaming
                        resp = {"status":"ok","data":"paused" if not streaming else "resumed"}

                    conn.send((json.dumps(resp) + "\n").encode())
                except Exception as e:
                    print(f"[PI] CmdErr: {e}")
                    conn.send((json.dumps({"status":"error","msg":str(e)}) + "\n").encode())
    except Exception as e:
        print(f"[PI] ClientErr: {e}")
    finally:
        with stream_lock:
            if conn in stream_clients:
                stream_clients.remove(conn)
            if not stream_clients:
                streaming = False
        conn.close()

def main():
    global model_type_1, model_type_2, ser_1, ser_2
    ser_1 = open_serial(SERIAL_PORT_1)
    ser_2 = open_serial(SERIAL_PORT_2)
    if not ser_1 and not ser_2:
        sys.exit(1)
    if ser_1:
        m,_ = read_model_type(ser_1, lock_1)
        model_type_1 = m
        print(f"[PI] Sensor 1 Model: {model_type_1}")
    if ser_2:
        m,_ = read_model_type(ser_2, lock_2)
        model_type_2 = m
        print(f"[PI] Sensor 2 Model: {model_type_2}")

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('0.0.0.0', PI_TCP_PORT))
    srv.listen(5)
    print(f"[PI] Listening :{PI_TCP_PORT}")

    t_stream = threading.Thread(target=stream_sender, daemon=True)
    t_stream.start()

    while True:
        conn, addr = srv.accept()
        t1 = threading.Thread(target=handle_client, args=(conn, addr), daemon=True)
        t1.start()

if __name__ == '__main__':
    PI_TCP_PORT = 5000
    main()
'''


# ============================================================================
# КЛИЕНТ (ПК) - ОБНОВЛЕН ДЛЯ ДВУХ ДАТЧИКОВ
# ============================================================================

class SensorClient:
    def __init__(self):
        self.sock = None
        self.connected = False
        self.model_1 = "B035"
        self.model_2 = "B035"
        self.streaming = False
        self.stream_paused = False
        self.stream_buffer = ""
        self.last_stream_values = {1: None, 2: None}
        self.stream_lock = threading.Lock()
        self.stream_thread = None
        self.running = True
        self.active_sensor = None  # None = оба, 1 или 2 = конкретный датчик

    def _import_paramiko(self):
        try:
            import paramiko
            return paramiko
        except ImportError:
            print("\n[!] Установите paramiko: pip install paramiko")
            return None

    def deploy_to_pi(self):  # развёртывание сервера на пк
        paramiko = self._import_paramiko()
        if not paramiko:
            return False
        print(f"[*] SSH: {PI_USER}@{PI_IP}...")
        try:
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(PI_IP, username=PI_USER, password=PI_PASS, port=PI_SSH_PORT, timeout=10)

            print("[*] Проверка зависимостей...")
            ssh.exec_command("sudo apt update -qq")
            _, stdout, _ = ssh.exec_command("python3 -c 'import serial' 2>/dev/null; echo $?")
            if stdout.read().strip().decode() != "0":
                print("[*] Установка python3-serial...")
                _, out, _ = ssh.exec_command("sudo apt install python3-serial -y -qq", timeout=180)
                if out.channel.recv_exit_status() != 0:
                    print("[-] Ошибка установки")
                    ssh.close()
                    return False

            sftp = ssh.open_sftp()
            with sftp.file(PI_SERVER_PATH, 'w') as f:
                f.write(PI_SERVER_CODE)

            ssh.exec_command(f"pkill -f '{PI_SERVER_PATH}' 2>/dev/null || true")
            time.sleep(0.3)
            ssh.exec_command(f"nohup python3 {PI_SERVER_PATH} > /tmp/sensor.log 2>&1 &")

            ssh.close()
            time.sleep(1.5)
            return True
        except Exception as e:
            print(f"[-] SSH ошибка: {e}")
            return False

    def connect_tcp(self, max_attempts=5):
        """Подключение по TCP с повторными попытками"""
        for attempt in range(max_attempts):
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(2)  # быстрый таймаут для проверки
                self.sock.connect((PI_IP, PI_TCP_PORT))
                self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.sock.settimeout(None)  # дальше без таймаута
                self.connected = True
                print(f"[+] TCP: {PI_IP}:{PI_TCP_PORT}")

                # Запуск потока для приёма стрим-данных
                self.stream_thread = threading.Thread(target=self._stream_listener, daemon=True)
                self.stream_thread.start()
                return True

            except Exception as e:
                print(f"[-] Попытка {attempt + 1}/{max_attempts}: {e}")
                if attempt < max_attempts - 1:
                    time.sleep(1)
                if self.sock:
                    try:
                        self.sock.close()
                    except:
                        pass
        return False

    def _stream_listener(self):  # Фоновый поток для приёма стрим-данных //надо бы уменьшить пинг
        buffer = ""
        while self.running and self.connected:
            try:
                self.sock.settimeout(0.4)
                chunk = self.sock.recv(4096).decode('utf-8', errors='ignore')
                if not chunk:
                    continue
                buffer += chunk
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    if not line.strip():
                        continue
                    try:
                        msg = json.loads(line.strip())
                        if msg.get('type') == 'stream':
                            sid = msg.get('sensor_id', 1)
                            with self.stream_lock:
                                self.last_stream_values[sid] = msg.get('value')
                    except:
                        pass
            except socket.timeout:
                pass
            except Exception as e:
                if self.running:
                    print(f"[-] Stream listener error: {e}")
                break

    def _send_command(self, action, max_retries=5, sensor_id=None, **kw):
        """Отправка команды с автоматическими повторными попытками"""
        if not self.sock:
            return None

        for attempt in range(max_retries):
            try:
                payload = {"action": action, **kw}
                if sensor_id is not None:
                    payload['sensor_id'] = sensor_id
                req = json.dumps(payload).encode() + b"\n"
                self.sock.sendall(req)

                buffer = ""
                # Добавляем таймаут для чтения ответа (1.5 сек)
                self.sock.settimeout(1.5)

                while True:
                    chunk = self.sock.recv(4096).decode('utf-8', errors='ignore')
                    if not chunk:
                        break
                    buffer += chunk
                    if "\n" in buffer:
                        line = buffer.split("\n")[0]
                        self.sock.settimeout(None)  # сброс таймаута
                        return json.loads(line.strip())

            except socket.timeout:
                print(f"[-] Таймаут ({attempt + 1}/{max_retries}), пробую снова...")
                time.sleep(0.2)  # пауза перед повтором
                continue
            except (ConnectionResetError, BrokenPipeError, OSError) as e:
                print(f"[-] Соединение разорвано: {e}")
                self.connected = False
                # Пробуем переподключиться
                if self._reconnect():
                    print("[+] Переподключение успешно, повторяю команду...")
                    continue  # повторяем команду после реконнекта
                else:
                    return None
            except Exception as e:
                print(f"[-] Ошибка: {e}")
                self.connected = False
                if attempt < max_retries - 1:
                    time.sleep(0.3)
                    continue
                return None

        print(f"[-] Не удалось выполнить команду после {max_retries} попыток")
        return None

    def _reconnect(self, max_attempts=5):
        """Попытка переподключения к серверу"""
        print(f"[*] Попытка переподключения к {PI_IP}:{PI_TCP_PORT}...")

        for attempt in range(max_attempts):
            try:
                # Закрываем старый сокет если есть
                if self.sock:
                    try:
                        self.sock.close()
                    except:
                        pass
                    self.sock = None

                # Пробуем подключиться
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(2)
                self.sock.connect((PI_IP, PI_TCP_PORT))
                self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.sock.settimeout(None)

                self.connected = True
                print(f"[+] Переподключение успешно!")

                # Перезапускаем поток стриминга если нужно
                if self.streaming:
                    self.stream_thread = threading.Thread(target=self._stream_listener, daemon=True)
                    self.stream_thread.start()

                return True

            except Exception as e:
                print(f"[-] Попытка {attempt + 1}/{max_attempts} не удалась: {e}")
                time.sleep(1)  # ждём секунду перед следующей попыткой

        print("[-] Не удалось переподключиться")
        return self._send_command('ping')

    def get_model(self, sensor_id=1):
        r = self._send_command('get_model', sensor_id=sensor_id)
        if r and r.get('status') == 'ok':
            if sensor_id == 1:
                self.model_1 = r['data']
            else:
                self.model_2 = r['data']
        return r

    def measure(self, sensor_id=1):
        r = self._send_command('measure', sensor_id=sensor_id)
        return (r['data'], r['status']) if r and r.get('status') in ('ok', 'OK') else (
        None, r.get('status') if r else 'NoResp')

    def read_status(self, sensor_id=1):
        return self._send_command('read_status', sensor_id=sensor_id)

    def read_all(self, sensor_id=1):
        return self._send_command('read_all', sensor_id=sensor_id)

    def write_eeprom(self, sensor_id=1):
        return self._send_command('write_eeprom', sensor_id=sensor_id)

    def dismiss(self, sensor_id=1):
        return self._send_command('dismiss', sensor_id=sensor_id)

    def laser_on(self, sensor_id=1):
        return self._send_command('laser_on', sensor_id=sensor_id)

    def laser_off(self, sensor_id=1):
        return self._send_command('laser_off', sensor_id=sensor_id)

    def zero_reset(self, sensor_id=1):
        return self._send_command('zero_reset', sensor_id=sensor_id)

    def zero_restore(self, sensor_id=1):
        return self._send_command('zero_restore', sensor_id=sensor_id)

    def key_lock(self, sensor_id=1):
        return self._send_command('key_lock', sensor_id=sensor_id)

    def key_unlock(self, sensor_id=1):
        return self._send_command('key_unlock', sensor_id=sensor_id)

    def init_sensor(self, sensor_id=1):
        return self._send_command('init_sensor', sensor_id=sensor_id)

    def set_mode(self, mode, sensor_id=1):
        return self._send_command('set_mode', mode=mode, sensor_id=sensor_id)

    def set_sampling(self, period, sensor_id=1):
        return self._send_command('set_sampling', period=period, sensor_id=sensor_id)

    def set_averaging(self, count, sensor_id=1):
        return self._send_command('set_averaging', count=count, sensor_id=sensor_id)

    def set_polarity(self, pol, sensor_id=1):
        return self._send_command('set_polarity', polarity=pol, sensor_id=sensor_id)

    def set_alarm(self, atype, sensor_id=1):
        return self._send_command('set_alarm', type=atype, sensor_id=sensor_id)

    def set_display(self, on_off, sensor_id=1):
        return self._send_command('set_display', on_off=on_off, sensor_id=sensor_id)

    def set_sensitivity(self, sens, sensor_id=1):
        return self._send_command('set_sensitivity', sens=sens, sensor_id=sensor_id)

    def set_threshold(self, level, sensor_id=1):
        return self._send_command('set_threshold', level=level, sensor_id=sensor_id)

    def set_near(self, mm, sensor_id=1):
        return self._send_command('set_near', mm=mm, sensor_id=sensor_id)

    def set_far(self, mm, sensor_id=1):
        return self._send_command('set_far', mm=mm, sensor_id=sensor_id)

    def set_obsb(self, mm, sensor_id=1):
        return self._send_command('set_obsb', mm=mm, sensor_id=sensor_id)

    def set_obsb_hyst(self, mm, sensor_id=1):
        return self._send_command('set_obsb_hyst', mm=mm, sensor_id=sensor_id)

    def set_hyst(self, mm, sensor_id=1):
        return self._send_command('set_hyst', mm=mm, sensor_id=sensor_id)

    def set_zero_shift(self, mm, sensor_id=1):
        return self._send_command('set_zero_shift', mm=mm, sensor_id=sensor_id)

    def set_alarm_hold(self, val, sensor_id=1):
        return self._send_command('set_alarm_hold', value=val, sensor_id=sensor_id)

    def stream_start(self, sensor_id=None):
        # sensor_id: None = оба, 1 или 2 = конкретный
        if sensor_id is not None:
            self._send_command('stream_set_sensor', sensor_id=sensor_id)
            self.active_sensor = sensor_id
        r = self._send_command('stream_start')
        if r and r.get('status') == 'ok':
            self.streaming = True
            self.stream_paused = False
            return True
        return False

    def stream_stop(self):
        r = self._send_command('stream_stop')
        if r and r.get('status') == 'ok':
            self.streaming = False
            return True
        return False

    def stream_toggle_pause(self):
        r = self._send_command('stream_toggle_pause')
        if r and r.get('status') == 'ok':
            self.stream_paused = not self.stream_paused
            return True
        return False

    def get_stream_value(self, sensor_id=1):
        with self.stream_lock:
            val = self.last_stream_values.get(sensor_id)
            self.last_stream_values[sensor_id] = None
        return val

    def set_active_sensor_for_stream(self, sensor_id):
        """Установка активного датчика для стрима: None=оба, 1 или 2"""
        self.active_sensor = sensor_id
        return self._send_command('stream_set_sensor', sensor_id=sensor_id)


# ============================================================================
# МЕНЮ И ИНТЕРФЕЙС - ОБНОВЛЕН ДЛЯ ДВУХ ДАТЧИКОВ
# ============================================================================

def print_header():
    print("\n" + "=" * 80)
    print(" " * 20 + "SICK OD Mini Pro — Dual Sensor Version")
    print(" " * 30 + f"Плата: {PI_IP}")
    print("=" * 80)


def print_main_menu():
    print("\n" + "-" * 80)
    print("  ГЛАВНОЕ МЕНЮ")
    print("-" * 80)
    print("  [1]  Стрим с 2 датчиков (оба)")
    print("  [2]  Стрим только датчик 1 (ttyUSB1)")
    print("  [3]  Стрим только датчик 2 (ttyUSB2)")
    print("  [4]  Настройки датчика 1")
    print("  [5]  Настройки датчика 2")
    print("  [Q]  Выход")
    print("-" * 80)


def print_menu():
    print("\n" + "-" * 80)
    print("  [1]  Чтение измерения (B0 01)          [2]  Статус выхода (B0 02)")
    print("  [3]  Тип модели (01 00)                [4]  Все настройки")
    print("  [5]  Запись в EEPROM (A0 00)           [6]  Отмена настроек (A0 01)")
    print("  [7]  Лазер ВКЛ (A0 03)                 [8]  Лазер ВЫКЛ (A0 02)")
    print("  [9]  Сброс нуля (A1 00)                [10] Восст. нуля (A1 01)")
    print("  [11] Блок. кнопок (A1 04)              [12] Разблок. (A1 05)")
    print("  [13] Инициализация сенсора (40 00)")
    print("  [14] Режим измерения (40 04)           [15] Период дискрет. (40 06)")
    print("  [16] Усреднение (40 0A)                [17] Полярность (40 08)")
    print("  [18] Тревога (40 0C)                   [19] Дисплей (40 0E)")
    print("  [20] Чувствительность (40 14)          [21] Уровень порога (40 12)")
    print("  [22] Порог ближний (41 00)             [23] Порог дальний (41 02)")
    print("  [24] Порог ObSB (41 04)                [25] Гистерезис ObSB (41 06)")
    print("  [26] Гистерезис (41 10)                [27] Сдвиг нуля (41 12)")
    print("  [28] Alarm Hold (41 08)")
    print("  [S]  Стрим старт/стоп                  [P]  Пауза/возобновить стрим")
    print("  [R]  Обновить данные                   [B]  Назад в главное меню")
    print("  [0]  Выход   |   [M] Меню   |   [Q] Завершить")
    print("-" * 80)


def _input_num(prompt, min_v=None, max_v=None):  # проверка диапазона
    try:
        v = float(input(prompt).strip())
        if min_v is not None and v < min_v:
            raise ValueError
        if max_v is not None and v > max_v:
            raise ValueError
        return v
    except:
        print("  ! Неверное значение")
        return None


def menu_numeric(client, prompt, setter, unit="mm", min_v=None, max_v=None, sensor_id=1):  # проверка численных значений со ввода
    v = _input_num(f"  {prompt} ({unit}): ", min_v, max_v)
    if v is not None:
        r = setter(v, sensor_id=sensor_id)
        print(f"  {'OK' if r and r.get('status') == 'ok' else 'ERR'} Результат: {r.get('status') if r else 'NoResp'}")


def handle_menu(client, sensor_id=1):  # обработка ввода выбор ->
    """
    Возвращает:
        'back' - вернуться в главное меню
        'exit' - выход из программы
        'continue' - продолжить в текущем меню
    """
    print_menu()
    ch = input("\n  Выбор: ").strip().upper()
    if ch == '0':
        return 'exit'
    elif ch == 'Q':
        return 'exit'
    elif ch == 'B':
        return 'back'  # Возврат в главное меню
    elif ch == 'R':
        return 'continue'
    if ch == '1':
        val, st = client.measure(sensor_id=sensor_id)
        print(f"  {'OK' if st == 'ok' else 'ERR'} {val:.3f} mm" if val is not None else f"  ERR {st}")
    elif ch == '2':
        r = client.read_status(sensor_id=sensor_id)
        print(f"  {'OK' if r and r.get('status') == 'ok' else 'ERR'} Выход: {'ON' if r and r.get('data') else 'OFF'}")
    elif ch == '3':
        r = client.get_model(sensor_id=sensor_id)
        model = client.model_1 if sensor_id == 1 else client.model_2
        print(f"  {'OK' if r and r.get('status') == 'ok' else 'ERR'} Модель: OD1-{model}")
    elif ch == '4':
        r = client.read_all(sensor_id=sensor_id)
        if r and r.get('status') == 'ok':
            print("\n  +" + "-" * 40 + "+")
            for k, v in r['data'].items():
                print(f"  | {k:20} : {v:15} |")
            print("  +" + "-" * 40 + "+")
        else:
            print(f"  ERR Ошибка: {r.get('status') if r else 'NoResp'}")
    elif ch == '5':
        print(f"  {'OK' if client.write_eeprom(sensor_id=sensor_id) else 'ERR'} EEPROM")
    elif ch == '6':
        print(f"  {'OK' if client.dismiss(sensor_id=sensor_id) else 'ERR'} Отмена")
    elif ch == '7':
        print(f"  {'OK' if client.laser_on(sensor_id=sensor_id) else 'ERR'} Лазер ON")
    elif ch == '8':
        print(f"  {'OK' if client.laser_off(sensor_id=sensor_id) else 'ERR'} Лазер OFF")
    elif ch == '9':
        print(f"  {'OK' if client.zero_reset(sensor_id=sensor_id) else 'ERR'} Сброс нуля")
    elif ch == '10':
        print(f"  {'OK' if client.zero_restore(sensor_id=sensor_id) else 'ERR'} Восст. нуля")
    elif ch == '11':
        print(f"  {'OK' if client.key_lock(sensor_id=sensor_id) else 'ERR'} Блокировка")
    elif ch == '12':
        print(f"  {'OK' if client.key_unlock(sensor_id=sensor_id) else 'ERR'} Разблокировка")
    elif ch == '13':
        if input("  ! Сброс всех настроек? (y/n): ").strip().lower() == 'y':
            print(f"  {'OK' if client.init_sensor(sensor_id=sensor_id) else 'ERR'} Инициализация")
    elif ch == '14':
        print("  [1] 2-Pt  [2] 1-Pt  [3] ObSB")
        m = {'1': '2pt', '2': '1pt', '3': 'obsb'}.get(input("  Выбор: ").strip())
        if m:
            print(f"  {'OK' if client.set_mode(m, sensor_id=sensor_id) else 'ERR'} Режим")
    elif ch == '15':
        print("  [1] 500us  [2] 1ms  [3] 2ms  [4] 4ms  [5] AUTO")
        p = {'1': '500us', '2': '1000us', '3': '2000us', '4': '4000us', '5': 'auto'}.get(input("  Выбор: ").strip())
        if p:
            print(f"  {'OK' if client.set_sampling(p, sensor_id=sensor_id) else 'ERR'} Период")
    elif ch == '16':
        print("  [1] x1  [2] x8  [3] x64  [4] x512")
        c = {'1': '1', '2': '8', '3': '64', '4': '512'}.get(input("  Выбор: ").strip())
        if c:
            print(f"  {'OK' if client.set_averaging(c, sensor_id=sensor_id) else 'ERR'} Усреднение")
    elif ch == '17':
        print("  [1] Light ON  [2] Dark ON")
        p = {'1': 'light_on', '2': 'dark_on'}.get(input("  Выбор: ").strip())
        if p:
            print(f"  {'OK' if client.set_polarity(p, sensor_id=sensor_id) else 'ERR'} Полярность")
    elif ch == '18':
        print("  [1] Clamp  [2] Hold")
        a = {'1': 'clamp', '2': 'hold'}.get(input("  Выбор: ").strip())
        if a:
            print(f"  {'OK' if client.set_alarm(a, sensor_id=sensor_id) else 'ERR'} Тревога")
    elif ch == '19':
        print("  [1] ON  [2] OFF")
        d = {'1': 'on', '2': 'off'}.get(input("  Выбор: ").strip())
        if d:
            print(f"  {'OK' if client.set_display(d, sensor_id=sensor_id) else 'ERR'} Дисплей")
    elif ch == '20':
        print("  [1] AUTO  [2-7] 1...6")
        s = {'1': 'auto'}.get(input("  Выбор: ").strip())
        if not s and input("  Число (1-6): ").strip() in '123456':
            s = input("  Число: ").strip()
        if s:
            print(f"  {'OK' if client.set_sensitivity(s, sensor_id=sensor_id) else 'ERR'} Чувствительность")
    elif ch == '21':
        print("  [1] Base  [2] P400  [3] P200  [4] P100")
        l = {'1': 'base', '2': 'p400', '3': 'p200', '4': 'p100'}.get(input("  Выбор: ").strip())
        if l:
            print(f"  {'OK' if client.set_threshold(l, sensor_id=sensor_id) else 'ERR'} Порог")
    elif ch == '22':
        menu_numeric(client, "Ближний порог", client.set_near, sensor_id=sensor_id)
    elif ch == '23':
        menu_numeric(client, "Дальний порог", client.set_far, sensor_id=sensor_id)
    elif ch == '24':
        menu_numeric(client, "ObSB порог", client.set_obsb, sensor_id=sensor_id)
    elif ch == '25':
        menu_numeric(client, "ObSB гистерезис", client.set_obsb_hyst, sensor_id=sensor_id)
    elif ch == '26':
        menu_numeric(client, "Гистерезис", client.set_hyst, sensor_id=sensor_id)
    elif ch == '27':
        menu_numeric(client, "Сдвиг нуля", client.set_zero_shift, sensor_id=sensor_id)
    elif ch == '28':
        try:
            v = int(input("  Alarm Hold (0-9999): ").strip())
            print(f"  {'OK' if client.set_alarm_hold(v, sensor_id=sensor_id) else 'ERR'} Результат")
        except:
            print("  ERR Ошибка ввода")
    elif ch == 'S':
        if client.streaming:
            client.stream_stop()
            print("  STOP Стриминг остановлен")
        else:
            client.stream_start(sensor_id=sensor_id)
            print("  START Стриминг запущен (нажмите S для остановки)")
    elif ch == 'P':
        if client.streaming:
            client.stream_toggle_pause()
            state = "PAUSED" if client.stream_paused else "RESUMED"
            print(f"  {state} Стриминг")
    elif ch == 'M':
        return 'continue'
    else:
        print("  ERR Неверный выбор")
    return 'continue'


def kbhit_windows():  # проверка нажатия клавиш
    try:
        import msvcrt
        if msvcrt.kbhit():
            return msvcrt.getch().decode('utf-8', errors='ignore').strip().upper()
    except:
        pass
    return None


def _getch_linux():
    """Чтение одного символа без буферизации в Linux"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch.upper() if ch else ''


def stream_loop(client, sensor_ids):  # цикл стриминга данных
    # sensor_ids: список [1], [2] или [1, 2]
    labels = {1: "S1", 2: "S2"}
    print(f"\n  START Непрерывный режим: {', '.join(labels[sid] for sid in sensor_ids)}. [S] стоп, [P] пауза, [Q] выход, [B] назад.")
    count = 0
    start_time = time.time()

    # Сохраняем настройки терминала для Linux
    fd = None
    old_settings = None
    if not IS_WINDOWS:
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)

    try:
        while client.streaming and not client.stream_paused:
            values = {}
            statuses = {}
            for sid in sensor_ids:
                val = client.get_stream_value(sid)
                if val is None:
                    val, st = client.measure(sensor_id=sid)
                    statuses[sid] = st
                else:
                    statuses[sid] = "ok"
                values[sid] = val

            if any(v is not None for v in values.values()):
                count += 1
                ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                fps = count / (time.time() - start_time) if time.time() > start_time else 0
                parts = []
                for sid in sensor_ids:
                    val = values[sid]
                    st = statuses[sid]
                    if val is not None:
                        parts.append(f"{labels[sid]}:{val:+8.3f}mm")
                    else:
                        parts.append(f"{labels[sid]}:----")
                display = "  ".join(parts)
                sys.stdout.write(f"\r  [{ts}] {display}  |  FPS: {fps:4.1f}  ")
                sys.stdout.flush()
            time.sleep(0.05)

            # Проверка нажатий клавиш
            ch = None
            if IS_WINDOWS:
                ch = kbhit_windows()
            else:
                # Неблокирующая проверка в Linux
                if select.select([sys.stdin], [], [], 0)[0]:
                    ch = sys.stdin.read(1).strip().upper()

            if ch:
                if ch == 'S':
                    client.stream_stop()
                    break
                elif ch == 'P':
                    client.stream_toggle_pause()
                    if client.stream_paused:
                        print("\n  PAUSED Стриминг. Нажмите [P] для возобновления, [S] стоп, [Q] выход, [B] назад.")
                        # Ждём возобновления или остановки
                        while client.streaming and client.stream_paused:
                            time.sleep(0.1)
                            ch2 = None
                            if IS_WINDOWS:
                                ch2 = kbhit_windows()
                            else:
                                if select.select([sys.stdin], [], [], 0)[0]:
                                    ch2 = sys.stdin.read(1).strip().upper()
                            if ch2 == 'P':
                                client.stream_toggle_pause()
                                print(f"\n  RESUMED Стриминг: {', '.join(labels[sid] for sid in sensor_ids)}")
                                break
                            elif ch2 == 'S':
                                client.stream_stop()
                                break
                            elif ch2 == 'Q':
                                sys.exit(0)
                            elif ch2 == 'B':
                                client.stream_stop()
                                return 'back'
                elif ch == 'Q':
                    sys.exit(0)
                elif ch == 'B':
                    client.stream_stop()
                    return 'back'
    finally:
        # Восстанавливаем настройки терминала
        if not IS_WINDOWS and old_settings is not None:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    print()
    return 'continue'


def main():
    client = SensorClient()
    print_header()

    if not client.deploy_to_pi():
        print("\n[!] Не удалось подготовить Raspberry Pi")
        return

    if not client.connect_tcp():
        print("\n[!] Не удалось подключиться к серверу")
        return

    client.get_model(sensor_id=1)
    client.get_model(sensor_id=2)
    print(f"\n Подключено: OD1-{client.model_1} (ttyUSB1), OD1-{client.model_2} (ttyUSB2)")
    print("  Режим: Два датчика + фоновый стрим-поток")
    print("  Подсказки: [1-5] выбор режима, [Q] выход")

    try:
        while True:
            print_main_menu()
            ch = input("\n  Выбор: ").strip().upper()

            if ch == 'Q':
                break
            elif ch == '1':  # Стрим с 2 датчиков
                client.set_active_sensor_for_stream(None)
                if client.stream_start(sensor_id=None):
                    result = stream_loop(client, [1, 2])
                    if result == 'back':
                        continue  # Возврат в главное меню
            elif ch == '2':  # Стрим только датчик 1
                client.set_active_sensor_for_stream(1)
                if client.stream_start(sensor_id=1):
                    result = stream_loop(client, [1])
                    if result == 'back':
                        continue  # Возврат в главное меню
            elif ch == '3':  # Стрим только датчик 2
                client.set_active_sensor_for_stream(2)
                if client.stream_start(sensor_id=2):
                    result = stream_loop(client, [2])
                    if result == 'back':
                        continue  # Возврат в главное меню
            elif ch == '4':  # Настройки датчика 1
                print("\n  >> Настройки датчика 1 (ttyUSB1) <<")
                while True:
                    result = handle_menu(client, sensor_id=1)
                    if result == 'back':
                        break  # Возврат в главное меню
                    elif result == 'exit':
                        return
            elif ch == '5':  # Настройки датчика 2
                print("\n  >> Настройки датчика 2 (ttyUSB2) <<")
                while True:
                    result = handle_menu(client, sensor_id=2)
                    if result == 'back':
                        break  # Возврат в главное меню
                    elif result == 'exit':
                        return
            else:
                print("  ERR Неверный выбор")
                time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n\n  ! Прервано пользователем")
    finally:
        client.running = False
        if client.streaming:
            client.stream_stop()
        if client.sock:
            client.sock.close()
        print("\n" + "=" * 80 + "\n  Программа завершена.\n" + "=" * 80)


if __name__ == '__main__':
    main()