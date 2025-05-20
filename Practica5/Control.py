import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time


class MotorControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Control de Motor DC con Encoder")
        self.root.geometry("500x600")

        self.ser = None
        self.is_connected = False
        self.reader_thread = None
        self.stop_thread = False

        # --- Estilo (opcional con ttkthemes) ---
        try:
            from ttkthemes import ThemedTk
            if isinstance(self.root, ThemedTk):  # Si ya es ThemedTk (cuando se corre como __main__)
                self.root.set_theme("arc")  # Ejemplo de tema
        except ImportError:
            print("ttkthemes no instalado, usando estilo por defecto.")
            pass  # Usar estilo por defecto de ttk

        # --- Frame de Conexión ---
        connection_frame = ttk.LabelFrame(root, text="Conexión Serial")
        connection_frame.pack(pady=10, padx=10, fill="x")

        ttk.Label(connection_frame, text="Puerto COM:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.com_port_var = tk.StringVar()
        self.com_port_combo = ttk.Combobox(connection_frame, textvariable=self.com_port_var, width=15)
        self.com_port_combo.grid(row=0, column=1, padx=5, pady=5)
        self.refresh_com_ports()

        self.connect_button = ttk.Button(connection_frame, text="Conectar", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=2, padx=5, pady=5)

        self.connection_status_label = ttk.Label(connection_frame, text="Estado: Desconectado", foreground="red")
        self.connection_status_label.grid(row=1, column=0, columnspan=3, padx=5, pady=5)

        # --- Frame de Control ---
        control_frame = ttk.LabelFrame(root, text="Control del Motor")
        control_frame.pack(pady=10, padx=10, fill="x")

        ttk.Label(control_frame, text="RPM Deseadas:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.rpm_scale = ttk.Scale(control_frame, from_=0, to=300, orient="horizontal", length=200,
                                   command=self.on_rpm_scale_change)
        self.rpm_scale.set(0)
        self.rpm_scale.grid(row=0, column=1, padx=5, pady=5)
        self.rpm_value_label = ttk.Label(control_frame, text="0 RPM")
        self.rpm_value_label.grid(row=0, column=2, padx=5, pady=5, sticky="w")
        self.send_rpm_button = ttk.Button(control_frame, text="Enviar RPM", command=self.send_rpm_command,
                                          state=tk.DISABLED)
        self.send_rpm_button.grid(row=1, column=1, padx=5, pady=5)

        ttk.Label(control_frame, text="Dirección:").grid(row=2, column=0, padx=5, pady=5, sticky="w")
        self.direction_var = tk.IntVar(value=0)  # 0: Stop, 1: Adelante, -1: Atrás
        ttk.Radiobutton(control_frame, text="Adelante", variable=self.direction_var, value=1,
                        command=self.send_direction_command).grid(row=2, column=1, sticky="w", padx=5)
        ttk.Radiobutton(control_frame, text="Atrás", variable=self.direction_var, value=-1,
                        command=self.send_direction_command).grid(row=3, column=1, sticky="w", padx=5)

        self.stop_button = ttk.Button(control_frame, text="PARADA DE EMERGENCIA", command=self.send_stop_command,
                                      style="Emergency.TButton")
        self.stop_button.grid(row=4, column=0, columnspan=3, pady=10, padx=5, sticky="ew")
        self.root.style = ttk.Style()
        self.root.style.configure("Emergency.TButton", foreground="red", font=('Helvetica', '10', 'bold'))

        # --- Frame de PID Tuning (Opcional pero útil) ---
        pid_frame = ttk.LabelFrame(root, text="Ajuste PID (Enviar individualmente)")
        pid_frame.pack(pady=10, padx=10, fill="x")

        ttk.Label(pid_frame, text="Kp:").grid(row=0, column=0, padx=5, pady=2, sticky="w")
        self.kp_entry = ttk.Entry(pid_frame, width=8)
        self.kp_entry.grid(row=0, column=1, padx=5, pady=2)
        ttk.Button(pid_frame, text="Enviar Kp", command=lambda: self.send_pid_param("KP", self.kp_entry.get())).grid(
            row=0, column=2, padx=5, pady=2)

        ttk.Label(pid_frame, text="Ki:").grid(row=1, column=0, padx=5, pady=2, sticky="w")
        self.ki_entry = ttk.Entry(pid_frame, width=8)
        self.ki_entry.grid(row=1, column=1, padx=5, pady=2)
        ttk.Button(pid_frame, text="Enviar Ki", command=lambda: self.send_pid_param("KI", self.ki_entry.get())).grid(
            row=1, column=2, padx=5, pady=2)

        ttk.Label(pid_frame, text="Kd:").grid(row=2, column=0, padx=5, pady=2, sticky="w")
        self.kd_entry = ttk.Entry(pid_frame, width=8)
        self.kd_entry.grid(row=2, column=1, padx=5, pady=2)
        ttk.Button(pid_frame, text="Enviar Kd", command=lambda: self.send_pid_param("KD", self.kd_entry.get())).grid(
            row=2, column=2, padx=5, pady=2)

        # --- Frame de Estado (Feedback de Arduino) ---
        status_frame = ttk.LabelFrame(root, text="Estado del Motor (desde Arduino)")
        status_frame.pack(pady=10, padx=10, fill="x", expand=True)

        self.current_rpm_label = ttk.Label(status_frame, text="RPM Actual: --")
        self.current_rpm_label.pack(anchor="w", padx=5, pady=2)
        self.setpoint_rpm_label = ttk.Label(status_frame, text="RPM Setpoint: --")
        self.setpoint_rpm_label.pack(anchor="w", padx=5, pady=2)
        self.pwm_output_label = ttk.Label(status_frame, text="PWM Salida: --")
        self.pwm_output_label.pack(anchor="w", padx=5, pady=2)

        self.raw_serial_text = tk.Text(status_frame, height=5, width=60)
        self.raw_serial_text.pack(pady=5, padx=5, fill="both", expand=True)
        self.raw_serial_text.config(state=tk.DISABLED)

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)  # Manejar cierre de ventana

    def refresh_com_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.com_port_combo['values'] = ports
        if ports:
            self.com_port_var.set(ports[0])

    def toggle_connection(self):
        if not self.is_connected:
            port = self.com_port_var.get()
            if not port:
                messagebox.showerror("Error", "Selecciona un puerto COM.")
                return
            try:
                self.ser = serial.Serial(port, 115200, timeout=1)
                time.sleep(2)  # Esperar a que Arduino se reinicie después de la conexión serial
                self.is_connected = True
                self.connect_button.config(text="Desconectar")
                self.connection_status_label.config(text=f"Estado: Conectado a {port}", foreground="green")
                self.send_rpm_button.config(state=tk.NORMAL)

                # Iniciar hilo para leer datos del serial
                self.stop_thread = False
                self.reader_thread = threading.Thread(target=self.read_from_serial, daemon=True)
                self.reader_thread.start()
                print(f"Conectado a {port}")

            except serial.SerialException as e:
                messagebox.showerror("Error de Conexión", f"No se pudo conectar a {port}:\n{e}")
                self.is_connected = False
        else:
            self.stop_thread = True
            if self.reader_thread and self.reader_thread.is_alive():
                self.reader_thread.join(timeout=1)  # Esperar un poco a que el hilo termine
            if self.ser and self.ser.is_open:
                try:
                    self.send_command("STOP")  # Enviar comando de parada al desconectar
                except Exception as e:
                    print(f"Error enviando STOP al desconectar: {e}")
                self.ser.close()

            self.is_connected = False
            self.connect_button.config(text="Conectar")
            self.connection_status_label.config(text="Estado: Desconectado", foreground="red")
            self.send_rpm_button.config(state=tk.DISABLED)
            self.current_rpm_label.config(text="RPM Actual: --")
            self.setpoint_rpm_label.config(text="RPM Setpoint: --")
            self.pwm_output_label.config(text="PWM Salida: --")
            print("Desconectado")

    def send_command(self, command_str):
        if self.is_connected and self.ser:
            try:
                full_command = command_str + "\n"
                self.ser.write(full_command.encode('utf-8'))
                print(f"Enviado: {command_str}")
                # Actualizar el texto de raw serial
                self.raw_serial_text.config(state=tk.NORMAL)
                self.raw_serial_text.insert(tk.END, f"TX: {command_str}\n")
                self.raw_serial_text.see(tk.END)
                self.raw_serial_text.config(state=tk.DISABLED)

            except serial.SerialException as e:
                messagebox.showerror("Error de Envío", f"Error al enviar comando: {e}")
                self.toggle_connection()  # Intentar reconectar o marcar como desconectado
            except Exception as e:
                print(f"Error inesperado al enviar: {e}")
        else:
            # messagebox.showwarning("Advertencia", "No estás conectado al Arduino.")
            print("Advertencia: No estás conectado al Arduino.")

    def on_rpm_scale_change(self, value_str):
        rpm = int(float(value_str))
        self.rpm_value_label.config(text=f"{rpm} RPM")
        # No enviar inmediatamente, esperar al botón "Enviar RPM" o cambio de dirección

    def send_rpm_command(self):
        rpm = int(self.rpm_scale.get())
        self.send_command(f"SET_RPM:{rpm}")

    def send_direction_command(self):
        direction = self.direction_var.get()
        self.send_command(f"SET_DIR:{direction}")
        if direction == 0:  # Si la dirección es Stop
            self.rpm_scale.set(0)  # Poner el slider a 0
            self.rpm_value_label.config(text="0 RPM")
            self.send_command("SET_RPM:0")  # Asegurar que el setpoint RPM sea 0
        else:  # Si es adelante o atrás, y el RPM es 0, enviar el RPM actual del slider.
            if int(self.rpm_scale.get()) == 0:  # Si el slider está en 0, no arrancar solo por dirección
                self.send_command("SET_RPM:0")
            else:  # Si ya hay un RPM en el slider, se reenvía con la nueva dirección
                self.send_rpm_command()

    def send_stop_command(self):
        self.send_command("STOP")
        self.rpm_scale.set(0)
        self.rpm_value_label.config(text="0 RPM")
        self.direction_var.set(0)  # Marcar como parado

    def send_pid_param(self, param_prefix, value_str):
        try:
            val = float(value_str)
            self.send_command(f"{param_prefix}:{val}")
        except ValueError:
            messagebox.showerror("Error", f"Valor de {param_prefix} inválido. Debe ser un número.")

    def read_from_serial(self):
        while not self.stop_thread and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='replace').strip()
                    if line:
                        # Actualizar el texto de raw serial en el hilo principal
                        self.root.after(0, self.update_raw_serial_text, f"RX: {line}\n")
                        # Parsear datos específicos
                        self.parse_and_update_status(line)
            except serial.SerialException:
                print("Error de lectura serial. Desconectando.")
                self.root.after(0, self.handle_serial_error)  # Manejar desconexión en hilo principal
                break
            except Exception as e:
                print(f"Error leyendo del serial: {e}")
                time.sleep(0.1)  # Pequeña pausa para no sobrecargar en caso de error continuo
        print("Hilo de lectura serial terminado.")

    def update_raw_serial_text(self, text_to_add):
        self.raw_serial_text.config(state=tk.NORMAL)
        self.raw_serial_text.insert(tk.END, text_to_add)
        self.raw_serial_text.see(tk.END)  # Auto-scroll
        self.raw_serial_text.config(state=tk.DISABLED)

    def parse_and_update_status(self, line):
        # Ejemplo de parsing: "R:150.23,S:160.00,P:120"
        parts = line.split(',')
        data = {}
        for part in parts:
            kv = part.split(':')
            if len(kv) == 2:
                data[kv[0].strip()] = kv[1].strip()

        if 'R' in data:
            self.current_rpm_label.config(text=f"RPM Actual: {data['R']}")
        if 'S' in data:
            self.setpoint_rpm_label.config(text=f"RPM Setpoint: {data['S']}")
        if 'P' in data:
            self.pwm_output_label.config(text=f"PWM Salida: {data['P']}")

    def handle_serial_error(self):
        """Maneja errores de serial forzando una desconexión desde el hilo principal."""
        if self.is_connected:
            messagebox.showerror("Error Serial", "Se perdió la conexión con Arduino o hubo un error.")
            self.toggle_connection()  # Esto cerrará el puerto y actualizará la UI

    def on_closing(self):
        if self.is_connected:
            self.toggle_connection()  # Intenta desconectar limpiamente
        self.root.destroy()


if __name__ == "__main__":
    # Usar ThemedTk si está disponible para mejores estilos
    try:
        from ttkthemes import ThemedTk

        root = ThemedTk(theme="arc")  # Puedes probar otros temas: "plastik", "clearlooks", etc.
    except ImportError:
        root = tk.Tk()

    app = MotorControlApp(root)
    root.mainloop()