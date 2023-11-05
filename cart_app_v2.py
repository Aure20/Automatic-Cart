import tkinter as tk
from tkinter import *
from tkinter import ttk
import json

class MyGroceryListApp:
    
    def __init__(self):
        self.root = tk.Tk()
        self.menubar = tk.Menu(self.root)
        self.grocery_list = []

        home_page = Frame(self.root)
        home_page.grid(row=0, column=0, sticky="nsew")
        home_lb = Label(home_page, text="Welcome to your Shopping Cart app")
        home_lb.pack(pady=20)

        list_page = Frame(self.root)
        list_page.grid(row=0, column=0, sticky="nsew")
        list_lb = Label(list_page, text="Your Shopping List")
        list_lb.pack(pady=20)

        route_page = Frame(self.root)
        route_page.grid(row=0, column=0, sticky="nsew")
        route_lb = Label(route_page, text="Your Route")
        route_lb.pack(pady=20)


        home_page.tkraise()

        # self.current_frame = None # store the current frame
        # self.frames = {}

        # self.show_home()
        # self.load_items()

        self.root.geometry("500x500")
        self.root.title("Supermarket Cart")
        self.root.resizable(False,False)

        self.create_menu(home_page, list_page, route_page) # create menu with buttons to other pages

        self.root.mainloop()

    def create_menu(self, page1, page2, page3):
        """Create a menu with which the user can navigate between the different pages"""
        menubar = tk.Menu(self.root)

        self.root.config(menu=menubar)

        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Menu", menu=file_menu)

        file_menu.add_command(label="Home", command=lambda: page1.tkraise()) # add menu button for Home page
        file_menu.add_command(label="Shopping List", command=lambda: page2.tkraise()) # add menu button for Shopping List page
        file_menu.add_command(label="Route", command=lambda: page3.tkraise()) # add menu button for Route page

MyGroceryListApp()