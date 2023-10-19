import tkinter as tk

class MyGUI:
    
    def __init__(self):
        self.root = tk.Tk()
        self.menubar = tk.Menu(self.root)
        self.grocery_list = []

        self.root.geometry("500x500")
        self.root.title("Supermarket Cart")

        self.current_frame = None # store the current frame

        self.create_menu()
        self.show_home()

        self.root.mainloop()

    def add_item(self):
        item = self.item_entry.get()
        if item:
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
        # Example: tk.Label(self.current_frame, text="Welcome to the Home Screen").pack()
        self.current_frame.pack()

    def show_grocery_list(self):
        if self.current_frame:
            self.current_frame.pack_forget()
        self.current_frame = tk.Frame(self.root)

                
        # Entry field for adding items
        self.item_entry = tk.Entry(self.root)
        self.item_entry.pack(pady=10)

        # Buttons for adding and removing items
        self.add_button = tk.Button(self.root, text="Add Item", command=self.add_item)
        self.add_button.pack()

        self.remove_button = tk.Button(self.root, text="Remove Item", command=self.remove_item)
        self.remove_button.pack()

        # Listbox to display the grocery items
        self.listbox = tk.Listbox(self.root)
        self.listbox.pack()
        
        # Add widgets for the grocery list screen
        # Example: tk.Label(self.current_frame, text="Your Grocery List:").pack()
        self.current_frame.pack()

    def show_other_screen(self):
        if self.current_frame:
            self.current_frame.pack_forget()
        self.current_frame = tk.Frame(self.root)
        # Add widgets for the other screen
        # Example: tk.Label(self.current_frame, text="This is another screen").pack()
        self.current_frame.pack()
    

MyGUI()
