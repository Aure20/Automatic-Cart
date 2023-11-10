import tkinter as tk
from tkinter import *
import json
import random # for testing purposes of the route page

class MyGroceryListApp:

    def __init__(self):
        self.root = tk.Tk()
        self.menubar = tk.Menu(self.root) # create a menu
        self.grocery_list = [] # start with an empty shopping list

        # Home page
        self.home_page = Frame(self.root)
        self.home_page.grid(row=0, column=0, sticky="nsew")
        self.home_lb = Label(self.home_page, text="Welcome to your Shopping Cart app", font=('Comic Sans MS', 10))
        self.home_lb.grid(padx=40, pady=20)

        # Shopping list page
        self.list_page = Frame(self.root)
        self.list_page.grid(row=0, column=0, sticky="nsew")
        self.list_lb = Label(self.list_page, text="")
        self.list_lb.grid(pady=0)

        self.item_entry = tk.Entry(self.list_page, width=20) # create an input field
        self.item_entry.grid(row=0, column=0, padx=0, pady=0)
        self.item_entry.bind("<KeyRelease>", self.update_suggestions) # show suggestions during typing

        self.suggestions_listbox = tk.Listbox(self.list_page, width=20)
        self.suggestions_listbox.grid(row=0, column=1, padx=10, pady=10)

        self.suggestions_scrollbar = tk.Scrollbar(self.list_page, orient=tk.VERTICAL, command=self.suggestions_listbox.yview)
        self.suggestions_scrollbar.grid(row=0, column=2, padx=0, sticky='ns')
        self.suggestions_listbox.config(yscrollcommand=self.suggestions_scrollbar.set)

        self.suggestions_listbox.bind("<Double-1>", self.add_suggestion_to_list) # double click to add item to shopping list

        self.listbox = tk.Listbox(self.list_page, width=20)
        self.listbox.grid(row=1, column=0, columnspan=1, padx=5, pady=10)

        self.listbox.bind("<Double-1>", self.delete_item) # double click to add item to shopping list

        self.save_button = tk.Button(self.list_page, text="Send Shopping List", font=('Comic Sans MS', 12), command=self.save_shopping_list)
        self.save_button.grid(row=1, column=1)


        # Route page
        self.route_page = Frame(self.root) 
        self.route_page.grid(row=0, column=0, sticky="nsew")
        self.route_lb = Label(self.route_page, text="Your Route", font=('Comic Sans MS', 10))
        self.route_lb.grid(row=0, column=0, padx=120, pady=20)

        self.next_item_label = Label(self.route_page, text="")
        self.next_item_label.grid(row=1, column=0, pady=20)

        

        self.home_page.tkraise() # start at home page

        self.root.geometry("300x500")
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

    def delete_item(self, event):
        selected_item = self.listbox.get(self.listbox.curselection())[3:] # the first part of the string is a number, a dot and a space
        if selected_item:
            self.grocery_list.remove(selected_item)
            self.update_shopping_list()

    def update_shopping_list(self):
        self.listbox.delete(0, tk.END)
        for index, item in enumerate(self.grocery_list, start=1):
            self.listbox.insert(tk.END, f"{index}. {item}")

        # Update the next item on the route page
        
        if self.grocery_list:
            next_item = random.choice(self.grocery_list)
            self.next_item_label.config(text=f"Next Item: {next_item}", font=('Comic Sans MS', 10))
        else:
            self.next_item_label.config(text="No items in the shopping list", font=('Comic Sans MS', 10))

    def save_shopping_list(self):
        with open("grocery_list.json", "w") as file:
            json.dump(self.grocery_list, file)

MyGroceryListApp()
