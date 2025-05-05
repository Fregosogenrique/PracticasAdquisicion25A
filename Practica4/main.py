import tkinter as tk
from tkinter import ttk, messagebox
import sys
import os
import random
from datetime import datetime

# Import our custom GUI components
from modules.gui import HeaderFrame, ControlPanel, DataDisplayFrame, StatusBar

class MainApplication(tk.Tk):
    """Main application window for Practica4"""
    
    def __init__(self):
        super().__init__()
        self.title("Práctica 4 - Interfaz Gráfica")
        self.geometry("900x700")  # Increased size to accommodate all components
        self.minsize(800, 600)    # Set minimum window size
        
        # Configure window to be responsive
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)  # Main content area gets extra space
        
        # Create and organize components
        self.create_components()
        
        # Set up event handlers
        self.setup_events()
        
        # Initialize sample data
        self.sample_data = []
        
    def create_components(self):
        """Create and organize all GUI components"""
        # Header section at the top
        self.header = HeaderFrame(self)
        self.header.grid(row=0, column=0, sticky="ew", padx=10, pady=(10, 5))
        
        # Main content frame (will contain control panel and data display)
        self.content_frame = ttk.Frame(self)
        self.content_frame.grid(row=1, column=0, sticky="nsew", padx=10, pady=5)
        self.content_frame.columnconfigure(0, weight=1)
        self.content_frame.rowconfigure(1, weight=1)  # Data display gets extra space
        
        # Control panel (buttons and form fields)
        self.control_panel = ControlPanel(self.content_frame)
        self.control_panel.grid(row=0, column=0, sticky="ew", pady=(0, 5))
        
        # Data display area (table with sample data)
        self.data_display = DataDisplayFrame(self.content_frame)
        self.data_display.grid(row=1, column=0, sticky="nsew")
        
        # Status bar at the bottom
        self.status_bar = StatusBar(self)
        self.status_bar.grid(row=2, column=0, sticky="ew")
        
    def setup_events(self):
        """Set up event handlers for interface components"""
        # Configure button commands in the control panel
        self.control_panel.set_command("iniciar", self.start_action)
        self.control_panel.set_command("detener", self.stop_action)
        self.control_panel.set_command("reiniciar", self.reset_action)
        self.control_panel.set_command("salir", self.exit_application)
    
    def start_action(self):
        """Handler for the Start button"""
        # Get values from form fields
        nombre = self.control_panel.entries.get("nombre", ttk.Entry()).get() or "Sin nombre"
        valor = self.control_panel.entries.get("valor", ttk.Entry()).get() or "0"
        descripcion = self.control_panel.entries.get("descripción", ttk.Entry()).get() or "Sin descripción"
        
        # Generate a sample data item
        try:
            item_id = str(len(self.sample_data) + 1)
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.sample_data.append((item_id, nombre, valor, descripcion))
            
            # Add to the display
            self.data_display.add_item(item_id, [nombre, valor, descripcion])
            
            # Update status
            self.status_bar.set_status(f"Datos agregados: {nombre} - {timestamp}")
            
            # Clear form fields
            for entry in self.control_panel.entries.values():
                entry.delete(0, tk.END)
                
        except Exception as e:
            messagebox.showerror("Error", f"Error al procesar datos: {str(e)}")
    
    def stop_action(self):
        """Handler for the Stop button"""
        self.status_bar.set_status("Operación detenida")
        messagebox.showinfo("Información", "Operación detenida")
    
    def reset_action(self):
        """Handler for the Reset button"""
        # Clear all data
        self.data_display.clear_all()
        self.sample_data = []
        
        # Clear form fields
        for entry in self.control_panel.entries.values():
            entry.delete(0, tk.END)
            
        # Update status
        self.status_bar.set_status("Sistema reiniciado")
        messagebox.showinfo("Información", "Sistema reiniciado correctamente")
    
    def exit_application(self):
        """Handler for the Exit button"""
        if messagebox.askyesno("Confirmar salida", "¿Está seguro que desea salir de la aplicación?"):
            self.destroy()

def main():
    # Set up application style
    app = MainApplication()
    
    # Apply a modern theme if available
    try:
        app.tk.call("source", "azure.tcl")
        app.tk.call("set_theme", "light")
    except tk.TclError:
        # If theme not available, use default
        style = ttk.Style()
        if sys.platform.startswith('win'):
            style.theme_use('vista')
        elif sys.platform.startswith('darwin'):
            style.theme_use('aqua')
        else:
            style.theme_use('clam')
    
    # Start the application
    app.mainloop()

if __name__ == "__main__":
    main()
