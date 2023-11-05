import tkinter as tk
from tkinter import *
from tkinter import ttk
import json

class MyGroceryListApp:

    def __init__(self):
        self.root = tk.Tk()
        self.menubar = tk.Menu(self.root)
        self.grocery_list = []

        self.home_page = Frame(self.root)
        self.home_page.grid(row=0, column=0, sticky="nsew")
        self.home_lb = Label(self.home_page, text="Welcome to your Shopping Cart app", font=('Comic Sans MS', 12))
        self.home_lb.grid(pady=20)

        self.list_page = Frame(self.root)
        self.list_page.grid(row=0, column=0, sticky="nsew")
        self.list_lb = Label(self.list_page, text="", font=('Comic Sans MS', 12))
        self.list_lb.grid(pady=0)

        self.item_entry = tk.Entry(self.list_page, width=20)
        self.item_entry.grid(row=0, column=0, padx=0, pady=0)
        self.item_entry.bind("<KeyRelease>", self.update_suggestions)

        self.suggestions_listbox = tk.Listbox(self.list_page, width=20)
        self.suggestions_listbox.grid(row=0, column=1, padx=10, pady=10)
        self.suggestions_listbox.bind("<Double-Button-1>", self.add_suggestion_to_list)

        self.listbox = tk.Listbox(self.list_page, width=30)
        self.listbox.grid(row=1, column=0, columnspan=1, padx=20, pady=10)

        self.route_page = Frame(self.root)
        self.route_page.grid(row=0, column=0, sticky="nsew")
        self.route_lb = Label(self.route_page, text="Your Route", font=('Comic Sans MS', 12))
        self.route_lb.grid(pady=20)

        self.home_page.tkraise()

        self.root.geometry("500x500")
        self.root.title("Supermarket Cart")
        self.root.resizable(False, False)

        self.create_menu(self.home_page, self.list_page, self.route_page)
        self.load_items()

        self.root.mainloop()

    def create_menu(self, page1, page2, page3):
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)
        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Menu", menu=file_menu)
        file_menu.add_command(label="Home", command=lambda: page1.tkraise())
        file_menu.add_command(label="Shopping List", command=lambda: page2.tkraise())
        file_menu.add_command(label="Route", command=lambda: page3.tkraise())

    def load_items(self):
        with open("ImageToGraph/supermarket_items.json", "r") as file:
            data = json.load(file)
            self.supermarket_items = [item for category in data.values() for item in category]

    def update_suggestions(self, event):
        user_input = self.item_entry.get()
        self.suggestions_listbox.delete(0, tk.END)
        for suggestion in self.supermarket_items:
            if user_input.lower() in suggestion.lower():
                self.suggestions_listbox.insert(tk.END, suggestion)

    def add_suggestion_to_list(self, event):
        selected_suggestion = self.suggestions_listbox.get(self.suggestions_listbox.curselection())
        if selected_suggestion:
            self.grocery_list.append(selected_suggestion)
            self.update_shopping_list()

    def update_shopping_list(self):
        self.listbox.delete(0, tk.END)
        for item in self.grocery_list:
            self.listbox.insert(tk.END, item)

MyGroceryListApp()
