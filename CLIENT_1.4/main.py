import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox, Toplevel
import threading
import time
import struct
from collections import deque
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    serial = None

# ---------- Константы протокола (новые) ----------
CMD_SETPOINT = 0x70
CMD_TOGGLE_MODE = 0x40
CMD_SLAVE_ADDR = 0x30
CMD_START_ADC_CAL = 0x50
CMD_EDIT_ADC_CAL = 0x10
CMD_START_VOUT_CAL = 0x60
CMD_EDIT_VOUT_CAL = 0x20
CMD_RESET_EEPROM = 0xA0
CMD_GET_DATA = 0x30         # новая команда

# Битовые маски для статуса
MODE_BIT = 0x80
PACKET_TYPE_BIT = 0x40       # 0 = ACK, 1 = NACK
ERROR_MASK = 0x0F

# Коды ошибок NACK (биты 0-2)
NACK_WRONG_COMMAND = 0
NACK_WRONG_PARAM = 1
NACK_WRONG_PARITY = 2

# Ошибки устройства (биты 0-3)
ERR_UART = 0x01
ERR_OVERVOLTAGE = 0x02
ERR_UNDERVOLTAGE = 0x04
ERR_SETPOINT = 0x08
ERR_EEPROM = 0x10   # бит 4 (но в протоколе только 0-3? уточнить, оставим как есть)

ERROR_NAMES = {
    ERR_UART: "UART error",
    ERR_OVERVOLTAGE: "Overvoltage",
    ERR_UNDERVOLTAGE: "Undervoltage",
    ERR_SETPOINT: "Setpoint error",
    ERR_EEPROM: "EEPROM error"
}

NACK_NAMES = {
    NACK_WRONG_COMMAND: "Unknown command",
    NACK_WRONG_PARAM: "Invalid parameter",
    NACK_WRONG_PARITY: "Checksum error"
}

# CRC-16 (полином 0x8005, начальное 0xFFFF)
def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x8005
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc

class UARTProtocolApp:
    def __init__(self, root):
        self.root = root
        self.root.title("UART Control Panel - Voltage Regulator (v2)")
        self.root.geometry("1300x750")

        self.serial_port = None
        self.serial_thread_running = False
        self.tx_counter = 0
        self.rx_counter = 0

        # Данные для графика
        self.time_data = deque(maxlen=200)
        self.voltage_data = deque(maxlen=200)
        self.start_time = None

        # Событие для синхронизации ACK (калибровка)
        self.ack_event = threading.Event()

        self.setup_ui()
        self.refresh_ports()

    # ---------------------- Построение интерфейса ----------------------
    def setup_ui(self):
        main_pane = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_pane.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        left_frame = ttk.Frame(main_pane, width=400)
        main_pane.add(left_frame, weight=0)
        right_frame = ttk.Frame(main_pane)
        main_pane.add(right_frame, weight=1)

        self._build_left_panel(left_frame)
        self._build_right_panel(right_frame)

    def _build_left_panel(self, parent):
        # --- Порт ---
        port_frame = ttk.LabelFrame(parent, text="COM Port", padding=5)
        port_frame.pack(fill=tk.X, pady=5)

        ttk.Label(port_frame, text="Port:").grid(row=0, column=0)
        self.port_combo = ttk.Combobox(port_frame, width=10)
        self.port_combo.grid(row=0, column=1, padx=5)
        ttk.Button(port_frame, text="Обновить", command=self.refresh_ports).grid(row=0, column=2, padx=5)

        ttk.Label(port_frame, text="Baud:").grid(row=0, column=3, padx=5)
        self.baud_combo = ttk.Combobox(port_frame, values=[9600, 115200], width=6)
        self.baud_combo.set(9600)
        self.baud_combo.grid(row=0, column=4)

        self.connect_btn = ttk.Button(port_frame, text="Подключить", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=5, padx=10)

        self.status_label = ttk.Label(port_frame, text="Отключено", foreground="red")
        self.status_label.grid(row=0, column=6, padx=5)

        # --- Выбор команды (радиокнопки) ---
        cmd_frame = ttk.LabelFrame(parent, text="Выбор команды", padding=5)
        cmd_frame.pack(fill=tk.X, pady=5)

        self.cmd_var = tk.IntVar(value=CMD_SETPOINT)

        rb1 = ttk.Radiobutton(cmd_frame, text="Задать уставку напряжения", variable=self.cmd_var,
                              value=CMD_SETPOINT, command=self.on_command_changed)
        rb2 = ttk.Radiobutton(cmd_frame, text="Изменить режим", variable=self.cmd_var,
                              value=CMD_TOGGLE_MODE, command=self.on_command_changed)
        rb3 = ttk.Radiobutton(cmd_frame, text="Сбросить EEPROM", variable=self.cmd_var,
                              value=CMD_RESET_EEPROM, command=self.on_command_changed)
        rb4 = ttk.Radiobutton(cmd_frame, text="Изменить адрес оконечного устройства", variable=self.cmd_var,
                              value=CMD_SLAVE_ADDR, command=self.on_command_changed)
        rb5 = ttk.Radiobutton(cmd_frame, text="Запросить данные (GET_DATA)", variable=self.cmd_var,
                              value=CMD_GET_DATA, command=self.on_command_changed)   # новая кнопка

        for rb in (rb1, rb2, rb3, rb4, rb5):
            rb.pack(anchor=tk.W, pady=2)

        # --- Отдельная группа для выбора режима TEST/WORK (всегда видна) ---
        mode_frame = ttk.LabelFrame(parent, text="Режим работы (для команды 'Изменить режим')", padding=5)
        mode_frame.pack(fill=tk.X, pady=5)

        self.mode_var = tk.StringVar(value="WORK")
        self.test_rb = ttk.Radiobutton(mode_frame, text="TEST", variable=self.mode_var, value="TEST")
        self.work_rb = ttk.Radiobutton(mode_frame, text="WORK", variable=self.mode_var, value="WORK")
        self.test_rb.pack(side=tk.LEFT, padx=10)
        self.work_rb.pack(side=tk.LEFT, padx=10)

        # --- Параметры пакета ---
        param_frame = ttk.LabelFrame(parent, text="Параметры пакета", padding=5)
        param_frame.pack(fill=tk.X, pady=5)

        ttk.Label(param_frame, text="Master ID (HEX):").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.master_entry = ttk.Entry(param_frame, width=8)
        self.master_entry.insert(0, "0x02")
        self.master_entry.grid(row=0, column=1, padx=5, pady=2)
        ttk.Button(param_frame, text="Изменить", command=self.edit_master_addr).grid(row=0, column=2, padx=5)

        ttk.Label(param_frame, text="Slave ID (HEX):").grid(row=1, column=0, sticky=tk.W, pady=2)
        self.slave_entry = ttk.Entry(param_frame, width=8)
        self.slave_entry.insert(0, "0x01")
        self.slave_entry.grid(row=1, column=1, padx=5, pady=2)

        ttk.Label(param_frame, text="Номер пакета:").grid(row=2, column=0, sticky=tk.W, pady=2)
        self.pkt_num = tk.StringVar(value="0")
        self.pkt_entry = ttk.Entry(param_frame, textvariable=self.pkt_num, width=8)
        self.pkt_entry.grid(row=2, column=1, padx=5, pady=2)
        self.auto_inc = tk.BooleanVar(value=True)
        ttk.Checkbutton(param_frame, text="Автоинкремент", variable=self.auto_inc).grid(row=2, column=2, padx=5)

        ttk.Label(param_frame, text="Setpoint (30-250):").grid(row=3, column=0, sticky=tk.W, pady=2)
        self.setpoint_var = tk.StringVar(value="100")
        self.setpoint_entry = ttk.Entry(param_frame, textvariable=self.setpoint_var, width=8)
        self.setpoint_entry.grid(row=3, column=1, padx=5, pady=2)

        ttk.Label(param_frame, text="Status (HEX):").grid(row=4, column=0, sticky=tk.W, pady=2)
        self.status_readonly = ttk.Entry(param_frame, width=10, state='readonly')
        self.status_readonly.grid(row=4, column=1, padx=5, pady=2)

        ttk.Label(param_frame, text="CRC-16 (HEX):").grid(row=4, column=2, sticky=tk.W, pady=2)
        self.crc_readonly = ttk.Entry(param_frame, width=10, state='readonly')
        self.crc_readonly.grid(row=4, column=3, padx=5, pady=2)

        self.wait_ack = tk.BooleanVar(value=True)
        ttk.Checkbutton(param_frame, text="Ожидать ACK (2с)", variable=self.wait_ack).grid(row=5, column=0, columnspan=4, sticky=tk.W, pady=5)

        ttk.Button(param_frame, text="ОТПРАВИТЬ ПАКЕТ", command=self.send_packet, width=35).grid(row=6, column=0, columnspan=4, pady=10)

        # --- Сервис (калибровка) ---
        svc_frame = ttk.LabelFrame(parent, text="Калибровка", padding=5)
        svc_frame.pack(fill=tk.X, pady=5)
        ttk.Button(svc_frame, text="Калибровка АЦП (1 точка)", command=lambda: self.calibration_wizard("ADC")).pack(fill=tk.X, pady=2)
        ttk.Button(svc_frame, text="Калибровка VOUT (2 точки)", command=lambda: self.calibration_wizard("VOUT")).pack(fill=tk.X, pady=2)

        self.on_command_changed()

    def _build_right_panel(self, parent):
        notebook = ttk.Notebook(parent)
        notebook.pack(fill=tk.BOTH, expand=True)

        log_frame = ttk.Frame(notebook)
        notebook.add(log_frame, text="Журнал")
        self.log_text = scrolledtext.ScrolledText(log_frame, bg="white", fg="black", font=("Consolas", 10))
        self.log_text.pack(fill=tk.BOTH, expand=True)

        graph_frame = ttk.Frame(notebook)
        notebook.add(graph_frame, text="График")

        self.fig = Figure(figsize=(6, 4), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title("Выходное напряжение")
        self.ax.set_xlabel("Время (мм:сс)")
        self.ax.set_ylabel("Напряжение, В")
        self.ax.grid(True, linestyle='--', alpha=0.6)
        self.canvas = FigureCanvasTkAgg(self.fig, master=graph_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    # ---------------------- Логика GUI ----------------------
    def on_command_changed(self):
        cmd = self.cmd_var.get()
        # Обновляем поле status
        self.status_readonly.config(state='normal')
        self.status_readonly.delete(0, tk.END)
        self.status_readonly.insert(0, f"0x{cmd:02X}")
        self.status_readonly.config(state='readonly')

        # Управление доступностью полей
        if cmd == CMD_SETPOINT:
            self.setpoint_entry.config(state='normal')
            self.slave_entry.config(state='readonly')
        elif cmd == CMD_TOGGLE_MODE:
            self.setpoint_entry.config(state='disabled')
            self.slave_entry.config(state='readonly')
        elif cmd == CMD_SLAVE_ADDR:
            self.setpoint_entry.config(state='disabled')
            self.slave_entry.config(state='normal')
        else:   # CMD_RESET_EEPROM, CMD_GET_DATA
            self.setpoint_entry.config(state='disabled')
            self.slave_entry.config(state='readonly')

        # Радиокнопки режима (TEST/WORK) активны только для MODE_TOGGLE
        if cmd == CMD_TOGGLE_MODE:
            self.test_rb.config(state='normal')
            self.work_rb.config(state='normal')
        else:
            self.test_rb.config(state='disabled')
            self.work_rb.config(state='disabled')

        self.update_crc()

    def update_crc(self):
        try:
            master = int(self.master_entry.get(), 16)
            slave = int(self.slave_entry.get(), 16)
            pkt = int(self.pkt_num.get())
            sp = int(self.setpoint_var.get())
            if sp < 0: sp = 0
            if sp > 65535: sp = 65535
            cmd = self.cmd_var.get()
            data = bytes([0x55, master, slave, pkt & 0xFF, (sp >> 8) & 0xFF, sp & 0xFF, cmd])
            crc = crc16(data)
            self.crc_readonly.config(state='normal')
            self.crc_readonly.delete(0, tk.END)
            self.crc_readonly.insert(0, f"0x{crc:04X}")
            self.crc_readonly.config(state='readonly')
        except:
            self.crc_readonly.config(state='normal')
            self.crc_readonly.delete(0, tk.END)
            self.crc_readonly.insert(0, "Ошибка")
            self.crc_readonly.config(state='readonly')

    def edit_master_addr(self):
        win = Toplevel(self.root)
        win.title("Изменить адрес мастера")
        win.geometry("300x120")
        ttk.Label(win, text="Новый адрес (HEX, 0x00-0xFF):").pack(pady=5)
        entry = ttk.Entry(win, width=10)
        entry.pack(pady=5)

        def apply():
            try:
                val = int(entry.get(), 16)
                if 0 <= val <= 255:
                    self.master_entry.delete(0, tk.END)
                    self.master_entry.insert(0, f"0x{val:02X}")
                    self.update_crc()
                    win.destroy()
                else:
                    messagebox.showerror("Ошибка", "Адрес должен быть 0x00-0xFF")
            except:
                messagebox.showerror("Ошибка", "Некорректный HEX")
        ttk.Button(win, text="OK", command=apply).pack(pady=5)

    # ---------------------- Работа с портом ----------------------
    def refresh_ports(self):
        if SERIAL_AVAILABLE:
            ports = [p.device for p in serial.tools.list_ports.comports()]
            self.port_combo['values'] = ports
            if ports:
                self.port_combo.current(0)

    def toggle_connection(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_thread_running = False
            self.serial_port.close()
            self.status_label.config(text="Отключено", foreground="red")
            self.connect_btn.config(text="Подключить")
        else:
            try:
                self.serial_port = serial.Serial(self.port_combo.get(), int(self.baud_combo.get()), timeout=0.1)
                self.serial_thread_running = True
                threading.Thread(target=self.rx_thread, daemon=True).start()
                self.status_label.config(text="Подключено", foreground="green")
                self.connect_btn.config(text="Отключить")
                self.start_time = time.time()
            except Exception as e:
                messagebox.showerror("Ошибка", f"Порт недоступен: {e}")

    def rx_thread(self):
        buffer = bytearray()
        while self.serial_thread_running:
            if self.serial_port and self.serial_port.is_open:
                try:
                    if self.serial_port.in_waiting:
                        data = self.serial_port.read(self.serial_port.in_waiting)
                        buffer.extend(data)
                        # Ищем пакет 0xAA длиной 13 байт
                        while len(buffer) >= 13:
                            if buffer[0] == 0xAA:
                                packet = buffer[:13]
                                del buffer[:13]
                                self.process_rx_packet(packet)
                            else:
                                buffer.pop(0)
                    time.sleep(0.01)
                except:
                    break

    def process_rx_packet(self, raw):
        if len(raw) != 13:
            return
        try:
            # Распаковка полей
            master = raw[1]
            slave = raw[2]
            pkt_num = raw[3]
            voltage = struct.unpack('<f', raw[4:8])[0]
            setpoint = (raw[8] << 8) | raw[9]
            status_byte = raw[10]
            crc_recv = (raw[11] << 8) | raw[12]

            # Проверка CRC-16
            crc_calc = crc16(raw[:11])  # первые 11 байт
            if crc_calc != crc_recv:
                self.log(f"CRC ошибка: получено 0x{crc_recv:04X}, вычислено 0x{crc_calc:04X}")
                return

            # Анализ статуса
            mode = "WORK" if (status_byte & MODE_BIT) else "TEST"
            is_nack = (status_byte & PACKET_TYPE_BIT) != 0
            error_bits = status_byte & ERROR_MASK

            if is_nack:
                # NACK пакет: биты 0-2 содержат код ошибки
                nack_code = error_bits & 0x07
                err_str = NACK_NAMES.get(nack_code, f"Unknown NACK code {nack_code}")
                self.log(f"📤 NACK получен: {err_str}")
                # Не обновляем график для NACK
                return
            else:
                # ACK или телеметрический пакет
                errors = [ERROR_NAMES[b] for b in ERROR_NAMES if error_bits & b]
                err_str = ", ".join(errors) if errors else "OK"
                # Обновление графика (только для телеметрии, но и для ACK тоже можно)
                if 0 <= voltage <= 300:
                    if self.start_time is not None:
                        cur_time = time.time() - self.start_time
                        self.time_data.append(cur_time)
                        self.voltage_data.append(voltage)
                        while len(self.time_data) > 100:
                            self.time_data.popleft()
                            self.voltage_data.popleft()
                    self.root.after(0, self.update_graph)

                # Логирование
                self.root.after(0, self.log_rx, raw, slave, pkt_num, voltage, setpoint, status_byte, mode, err_str)

                # Если это ACK (бит 6 = 0) – сигналим событие для калибровки
                if not is_nack:
                    self.ack_event.set()

        except Exception as e:
            print(f"Ошибка парсинга: {e}")

    def update_graph(self):
        if not self.time_data:
            return
        self.ax.clear()
        self.ax.plot(self.time_data, self.voltage_data, 'b-', marker='o', markersize=2, linewidth=1)
        self.ax.set_title("Выходное напряжение")
        self.ax.set_xlabel("Время (мм:сс)")
        self.ax.set_ylabel("Напряжение, В")
        self.ax.grid(True, linestyle='--', alpha=0.6)
        if self.time_data:
            t_min = max(0, self.time_data[-1] - 10)
            t_max = self.time_data[-1] + 0.5
            self.ax.set_xlim(t_min, t_max)
            ticks = []
            labels = []
            step = (t_max - t_min) / 9
            for i in range(10):
                tick = t_min + i * step
                ticks.append(tick)
                minutes = int(tick // 60)
                seconds = int(tick % 60)
                labels.append(f"{minutes:02d}:{seconds:02d}")
            self.ax.set_xticks(ticks)
            self.ax.set_xticklabels(labels, rotation=0, ha='center', fontsize=8)
        self.canvas.draw()

    def log_rx(self, raw, slave, pkt_num, voltage, setpoint, status, mode, err_str):
        ts = time.strftime("%H:%M:%S")
        raw_hex = ' '.join(f"{b:02X}" for b in raw)
        self.log_text.insert(tk.END, f"─" * 70 + "\n")
        self.log_text.insert(tk.END, f"📥 ПРИЕМ [{ts}] [ {raw_hex} ]\n")
        self.log_text.insert(tk.END,
            f"   От: 0x{slave:02X} | Пакет №: {pkt_num} | Режим: {mode} | "
            f"Статус: 0x{status:02X} [{err_str}]\n")
        self.log_text.insert(tk.END, f"   Напряжение: {voltage:.2f} В | Уставка: {setpoint} В\n")
        self.log_text.see(tk.END)

    # ---------------------- Отправка пакетов (новый формат) ----------------------
    def build_packet(self, master, slave, pkt_num, value, cmd):
        # 7 байт данных: start(0x55) + master + slave + pkt_num + setpoint_H + setpoint_L + cmd
        sp_h = (value >> 8) & 0xFF
        sp_l = value & 0xFF
        data = bytes([0x55, master, slave, pkt_num & 0xFF, sp_h, sp_l, cmd])
        crc = crc16(data)
        # Добавляем CRC-16 (2 байта, старший сначала)
        return data + bytes([(crc >> 8) & 0xFF, crc & 0xFF])

    def send_packet(self, cmd_override=None, value_override=None, wait_ack=False):
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Внимание", "Порт не открыт")
            return False

        try:
            master = int(self.master_entry.get(), 16)
            slave = int(self.slave_entry.get(), 16)
            pkt_num = int(self.pkt_num.get())
            cmd = self.cmd_var.get() if cmd_override is None else cmd_override

            if cmd_override is not None:
                value = value_override
            else:
                if cmd == CMD_SETPOINT:
                    value = int(self.setpoint_var.get())
                    if value < 30 or value > 250:
                        messagebox.showwarning("Диапазон", "Уставка должна быть от 30 до 250 В")
                        return False
                elif cmd == CMD_TOGGLE_MODE:
                    value = 0 if self.mode_var.get() == "TEST" else 1
                elif cmd == CMD_SLAVE_ADDR:
                    value = 0   # не используется
                else:
                    value = 0

            packet = self.build_packet(master, slave, pkt_num, value, cmd)
            self.serial_port.write(packet)

            # Лог отправки
            ts = time.strftime("%H:%M:%S")
            cmd_names = {
                CMD_SETPOINT: "Задать уставку",
                CMD_TOGGLE_MODE: "Изменить режим",
                CMD_SLAVE_ADDR: "Изменить адрес",
                CMD_RESET_EEPROM: "Сбросить EEPROM",
                CMD_GET_DATA: "Запрос данных"
            }
            cmd_name = cmd_names.get(cmd, f"0x{cmd:02X}")
            self.log_text.insert(tk.END, "\n")
            self.log_text.insert(tk.END, f"▶ ОТПРАВКА [{ts}] Команда: {cmd_name}, Значение: {value}\n")
            self.log_text.insert(tk.END, f"   Пакет: {packet.hex().upper()}\n")
            self.log_text.see(tk.END)

            # Если меняем адрес slave – обновляем локальный адрес (но это не влияет на приём)
            if cmd == CMD_SLAVE_ADDR:
                new_slave = int(self.slave_entry.get(), 16)
                self.log_text.insert(tk.END, f"   ⚙ Адрес slave изменён на 0x{new_slave:02X} (новые пакеты будут приниматься)\n")

            # Автоинкремент
            if self.auto_inc.get():
                self.pkt_num.set(str((pkt_num + 1) & 0xFF))

            # Ожидание ACK (только для команд, кроме GET_DATA? можно и для GET_DATA)
            if wait_ack:
                self.ack_event.clear()
                if self.ack_event.wait(timeout=2.0):
                    self.log_text.insert(tk.END, "   ✅ Подтверждение (ACK) получено\n")
                    return True
                else:
                    self.log_text.insert(tk.END, "   ❌ Таймаут ACK\n")
                    return False
            return True
        except Exception as e:
            messagebox.showerror("Ошибка", str(e))
            return False

    # ---------------------- Калибровка (без изменений, но использует новый send_packet) ----------------------
    def calibration_wizard(self, mode):
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showwarning("Внимание", "Порт не открыт")
            return

        win = Toplevel(self.root)
        win.title(f"Калибровка {mode}")
        win.geometry("550x500")
        win.transient(self.root)
        win.grab_set()

        if mode == "ADC":
            desc = ("КАЛИБРОВКА АЦП (1 точка)\n\n"
                    "Шаг 1: Нажмите «Начать» – отправка START (0x50).\n"
                    "       Устройство установит скважность 50%.\n"
                    "Шаг 2: Измерьте НАПРЯЖЕНИЕ НА ВХОДЕ АЦП (канал 0) мультиметром.\n"
                    "       Можно вводить целые или дробные числа (например, 2.345).\n"
                    "Шаг 3: Введите измеренное значение и нажмите «Отправить».\n"
                    "       Будет отправлен EDIT (0x10) с setpoint = напряжение*1000 (округление до целого).\n")
        else:  # VOUT
            desc = ("КАЛИБРОВКА ВЫХОДНОГО НАПРЯЖЕНИЯ (2 точки)\n\n"
                    "ТОЧКА 1 (скважность 15%):\n"
                    "1. Нажмите «Начать калибровку» – отправка START (0x60).\n"
                    "2. Устройство установит скважность 15%.\n"
                    "3. Измерьте ВЫХОДНОЕ НАПРЯЖЕНИЕ мультиметром и введите значение (можно дробное, например, 25.3).\n"
                    "4. Нажмите «Отправить точку 1» – будет отправлен EDIT (0x20) с setpoint = напряжение*100.\n"
                    "   Устройство переключит скважность на 50%.\n\n"
                    "ТОЧКА 2 (скважность 50%):\n"
                    "5. Измерьте напряжение и введите его.\n"
                    "6. Нажмите «Отправить точку 2» – завершит калибровку.\n")

        ttk.Label(win, text=desc, justify=tk.LEFT, wraplength=500).pack(pady=10)

        self.cal_status = ttk.Label(win, text="", foreground="blue")
        self.cal_status.pack(pady=5)

        if mode == "ADC":
            self._adc_calibration(win)
        else:
            self._vout_calibration(win)

    def _adc_calibration(self, win):
        def start():
            self.cal_status.config(text="Отправка START... ожидание ACK")
            win.update()
            if self.send_packet(cmd_override=CMD_START_ADC_CAL, value_override=0, wait_ack=True):
                self.cal_status.config(text="Введите измеренное напряжение на входе АЦП (В):", foreground="black")
                entry = ttk.Entry(win, width=12)
                entry.pack(pady=5)
                entry.insert(0, "2.500")
                btn = ttk.Button(win, text="Отправить",
                                 command=lambda: self._adc_send_edit(win, entry))
                btn.pack(pady=5)
            else:
                self.cal_status.config(text="Ошибка: нет ACK на START", foreground="red")
        ttk.Button(win, text="Начать", command=start).pack(pady=10)

    def _adc_send_edit(self, win, entry):
        try:
            voltage = float(entry.get())
            setpoint_val = int(round(voltage * 1000))
            self.cal_status.config(text=f"Отправка EDIT (setpoint={setpoint_val})...")
            win.update()
            if self.send_packet(cmd_override=CMD_EDIT_ADC_CAL, value_override=setpoint_val, wait_ack=True):
                messagebox.showinfo("Успех", "Калибровка АЦП завершена")
                win.destroy()
            else:
                self.cal_status.config(text="Ошибка: нет ACK на EDIT", foreground="red")
        except ValueError:
            messagebox.showerror("Ошибка", "Введите корректное число (например, 2.500)")

    def _vout_calibration(self, win):
        self.vout_volt1 = None
        self.vout_volt2 = None

        def send_start():
            self.cal_status.config(text="Отправка START (0x60)... ожидание ACK")
            win.update()
            if self.send_packet(cmd_override=CMD_START_VOUT_CAL, value_override=0, wait_ack=True):
                self.cal_status.config(text="ТОЧКА 1 (15%): Введите измеренное напряжение (В):", foreground="black")
                entry1 = ttk.Entry(win, width=12)
                entry1.pack(pady=5)
                entry1.insert(0, "30.0")
                btn_send1 = ttk.Button(win, text="Отправить точку 1",
                                       command=lambda: send_first_edit(entry1))
                btn_send1.pack(pady=5)
            else:
                self.cal_status.config(text="Ошибка: нет ACK на START", foreground="red")

        def send_first_edit(entry1):
            try:
                volt1 = float(entry1.get())
                setpoint1 = int(round(volt1 * 100))
                self.cal_status.config(text=f"Отправка EDIT (точка 1, setpoint={setpoint1})... ожидание ACK")
                win.update()
                if self.send_packet(cmd_override=CMD_EDIT_VOUT_CAL, value_override=setpoint1, wait_ack=True):
                    self.vout_volt1 = volt1
                    self.cal_status.config(text="ТОЧКА 2 (50%): Введите измеренное напряжение (В):", foreground="black")
                    entry2 = ttk.Entry(win, width=12)
                    entry2.pack(pady=5)
                    entry2.insert(0, "100.0")
                    btn_send2 = ttk.Button(win, text="Отправить точку 2",
                                           command=lambda: send_second_edit(entry2))
                    btn_send2.pack(pady=5)
                    # Скрыть кнопку отправки первой точки
                    for child in win.winfo_children():
                        if isinstance(child, ttk.Button) and child.cget("text") == "Отправить точку 1":
                            child.pack_forget()
                else:
                    self.cal_status.config(text="Ошибка: нет ACK на EDIT (точка 1)", foreground="red")
            except ValueError:
                messagebox.showerror("Ошибка", "Введите корректное число (например, 30.0)")

        def send_second_edit(entry2):
            try:
                volt2 = float(entry2.get())
                setpoint2 = int(round(volt2 * 100))
                self.cal_status.config(text=f"Отправка EDIT (точка 2, setpoint={setpoint2})... ожидание ACK")
                win.update()
                if self.send_packet(cmd_override=CMD_EDIT_VOUT_CAL, value_override=setpoint2, wait_ack=True):
                    messagebox.showinfo("Успех", "Калибровка VOUT завершена")
                    win.destroy()
                else:
                    self.cal_status.config(text="Ошибка: нет ACK на EDIT (точка 2)", foreground="red")
            except ValueError:
                messagebox.showerror("Ошибка", "Введите корректное число (например, 100.0)")

        ttk.Button(win, text="Начать калибровку", command=send_start).pack(pady=10)

if __name__ == "__main__":
    root = tk.Tk()
    app = UARTProtocolApp(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (setattr(app, 'serial_thread_running', False), root.destroy()))
    root.mainloop()