"""
GUI components for Practica4
This module contains classes for different components of the interface
"""

import tkinter as tk
from tkinter import ttk, messagebox
from typing import Callable, Dict, List, Optional, Union


class HeaderFrame(ttk.Frame):
    """Frame for the header section of the application"""
    
    def __init__(self, master, **kwargs):
        super().__init__(master, **kwargs)
        self.columnconfigure(0, weight=1)
        
        # Create header components
        self.title_label = ttk.Label(
            self,
            text="Práctica 4",
            font=("Helvetica", 18, "bold")
        )
        self.title_label.grid(row=0, column=0, pady=10)
        
        # Subtitle or description
        self.subtitle_label = ttk.Label(
            self,
            text="Sistema de Interfaz Gráfica",
            font=("Helvetica", 12)
        )
        self.subtitle_label.grid(row=1, column=0, pady=5)


class ControlPanel(ttk.Frame):
    """Frame for control elements (buttons, inputs, etc.)"""
    
    def __init__(self, master, **kwargs):
        super().__init__(master, **kwargs)
        self.columnconfigure(0, weight=1)
        
        # Setup appearance
        self.configure(padding=(10, 10))
        
        # Button container
        self.button_frame = ttk.Frame(self)
        self.button_frame.grid(row=0, column=0, sticky="ew", pady=5)
        
        # Form container
        self.form_frame = ttk.Frame(self)
        self.form_frame.grid(row=1, column=0, sticky="ew", pady=5)
        
        # Create example buttons
        self.create_buttons()
        
        # Create example form fields
        self.create_form_fields()
    
    def create_buttons(self):
        """Create control buttons"""
        # Create a few example buttons
        self.buttons = {}
        button_texts = ["Iniciar", "Detener", "Reiniciar", "Salir"]
        
        for i, text in enumerate(button_texts):
            self.buttons[text.lower()] = ttk.Button(
                self.button_frame,
                text=text
            )
            self.buttons[text.lower()].grid(row=0, column=i, padx=5)
    
    def create_form_fields(self):
        """Create input fields and labels"""
        # Create some example form fields
        field_labels = ["Nombre:", "Valor:", "Descripción:"]
        self.entries = {}
        
        for i, label_text in enumerate(field_labels):
            # Label
            label = ttk.Label(self.form_frame, text=label_text)
            label.grid(row=i, column=0, sticky="w", padx=5, pady=5)
            
            # Entry field
            field_name = label_text.replace(":", "").lower()
            self.entries[field_name] = ttk.Entry(self.form_frame, width=30)
            self.entries[field_name].grid(row=i, column=1, sticky="ew", padx=5, pady=5)
    
    def set_command(self, button_name: str, command: Callable):
        """Set command for a specific button"""
        if button_name in self.buttons:
            self.buttons[button_name].configure(command=command)


class DataDisplayFrame(ttk.Frame):
    """Frame for displaying data (e.g., tabular data, graphs)"""
    
    def __init__(self, master, **kwargs):
        super().__init__(master, **kwargs)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)
        
        # Create treeview for tabular data
        self.tree = ttk.Treeview(self)
        self.tree["columns"] = ("col1", "col2", "col3")
        
        # Configure column headers
        self.tree.heading("#0", text="ID")
        self.tree.heading("col1", text="Columna 1")
        self.tree.heading("col2", text="Columna 2")
        self.tree.heading("col3", text="Columna 3")
        
        # Configure column widths
        self.tree.column("#0", width=50)
        self.tree.column("col1", width=100)
        self.tree.column("col2", width=100)
        self.tree.column("col3", width=100)
        
        # Place the treeview with scrollbars
        self.tree.grid(row=0, column=0, sticky="nsew")
        
        # Add scrollbars
        vsb = ttk.Scrollbar(self, orient="vertical", command=self.tree.yview)
        vsb.grid(row=0, column=1, sticky="ns")
        self.tree.configure(yscrollcommand=vsb.set)
        
        hsb = ttk.Scrollbar(self, orient="horizontal", command=self.tree.xview)
        hsb.grid(row=1, column=0, sticky="ew")
        self.tree.configure(xscrollcommand=hsb.set)
    
    def add_item(self, item_id: str, values: List):
        """Add an item to the treeview"""
        self.tree.insert("", "end", text=item_id, values=values)
    
    def clear_all(self):
        """Remove all items from the treeview"""
        for item in self.tree.get_children():
            self.tree.delete(item)


class StatusBar(ttk.Frame):
    """Status bar for displaying application status and messages"""
    
    def __init__(self, master, **kwargs):
        super().__init__(master, **kwargs)
        self.columnconfigure(0, weight=1)
        
        # Create status label
        self.status_label = ttk.Label(
            self,
            text="Listo",
            relief=tk.SUNKEN,
            anchor=tk.W,
            padding=(5, 2)
        )
        self.status_label.grid(row=0, column=0, sticky="ew")
    
    def set_status(self, text: str):
        """Update the status text"""
        self.status_label.configure(text=text)

