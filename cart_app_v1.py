import tkinter as tk
import json

class MyGroceryListApp:
    
    def __init__(self):
        self.root = tk.Tk()
        self.menubar = tk.Menu(self.root)
        self.grocery_list = []

        self.root.geometry("500x500")
        self.root.title("Supermarket Cart")

        self.current_frame = None # store the current frame

        self.create_menu()
        self.show_home()
        self.load_items()

        self.root.mainloop()

    def add_item(self):
        item = self.item_entry.get()
        if str(item) in self.suggestions:
            self.grocery_list.append(item)
            self.listbox.insert(tk.END, item)
            self.item_entry.delete(0, tk.END)

    def remove_item(self):
        selected_index = self.listbox.curselection()
        if selected_index:
            index = selected_index[0]
            item = self.listbox.get(index)
            self.grocery_list.remove(item)
            self.listbox.delete(index)

    def create_menu(self):
        menubar = tk.Menu(self.root)

        self.root.config(menu=menubar)

        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Screens", menu=file_menu)

        file_menu.add_command(label="Home", command=self.show_home)
        file_menu.add_command(label="Grocery List", command=self.show_grocery_list)
        file_menu.add_command(label="Route", command=self.show_other_screen)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.root.quit)

    def show_home(self):
        if self.current_frame:
            self.current_frame.pack_forget()
        self.current_frame = tk.Frame(self.root)
        # Add widgets for the home screen
        tk.Label(self.current_frame, text="Welcome to your Shopping List", font=('Comic Sans MS', 16), pady=100).pack()
        self.current_frame.pack()

    def show_grocery_list(self):
        if self.current_frame:
            self.current_frame.pack_forget()
        self.current_frame = tk.Frame(self.root)

                
        # Entry field for adding items
        self.item_entry = tk.Entry(self.root, width=50)
        self.item_entry.pack(pady=10)
        self.item_entry.bind("<KeyRelease>", self.update_suggestions)
        self.item_entry.bind("<KeyPress>", self.shortcut)

        self.suggestion_label = tk.Label(self.root)
        self.suggestion_label.pack()

        # Buttons for adding and removing items
        self.add_button = tk.Button(self.root, text="Add Item", font=('Comic Sans MS', 12), command=self.add_item)
        self.add_button.pack()

        self.remove_button = tk.Button(self.root, text="Remove Item", font=('Comic Sans MS', 12), command=self.remove_item)
        self.remove_button.pack()

        # Listbox to display the grocery items
        self.listbox = tk.Listbox(self.root, width=50)
        self.listbox.pack()
        
        # Add widgets for the grocery list screen
        # Example: tk.Label(self.current_frame, text="Your Grocery List:").pack()
        self.current_frame.pack()


        # ¡¡¡make sure you can't add more of the same screen!!!

    def show_other_screen(self):
        if self.current_frame:
            self.current_frame.pack_forget()
        self.current_frame = tk.Frame(self.root)
        # Add widgets for the other screen
        # Example: tk.Label(self.current_frame, text="This is another screen").pack()
        self.current_frame.pack()
    
    def load_items(self):
        with open("ImageToGraph/supermarket_items.json", "r") as file:
            data = json.load(file)
            self.supermarket_items = data.get("items", [])

    def update_suggestions(self, event):
        user_input = self.item_entry.get().strip().lower()
        self.suggestions = [item.lower() for item in self.supermarket_items if user_input in item.lower()]

        if user_input and self.suggestions:
            self.suggestion_label.config(text="Suggestions: " + ", ".join(self.suggestions))
        else:
            self.suggestion_label.config(text="")

    def shortcut(self, event):
        if event.state == 4 and event.keysym == "Return":
            self.add_item()

MyGroceryListApp()
