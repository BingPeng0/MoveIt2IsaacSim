#!/usr/bin/env python3

import tkinter as tk

def on_button_click():
    """Update the label text when the button is clicked."""
    label.config(text="Button Clicked!")

# Create the main application window
root = tk.Tk()
root.title("Simple Tkinter App")

# Create a label widget
label = tk.Label(root, text="Hello, Tkinter!")
label.pack(pady=20)

# Create a button widget
button = tk.Button(root, text="Click Me", command=on_button_click)
button.pack(pady=20)

# Run the Tkinter event loop
root.mainloop()
