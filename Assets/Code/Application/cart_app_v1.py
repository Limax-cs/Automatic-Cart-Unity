import tkinter as tk
from tkinter import *
from tkinter import ttk
import json

class MyGroceryListApp:
    
    def __init__(self):
        self.root = tk.Tk()
        self.menubar = tk.Menu(self.root)
        self.grocery_list = []

        self.root.geometry("500x500")
        self.root.title("Supermarket Cart")

        self.current_frame = None # store the current frame
        self.frames = {}

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
        file_menu.add_command(label="Route", command=self.show_route)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.root.quit)

    def show_home(self):
        self.switch_frame("Home")
        # Add widgets for the home screen
        tk.Label(self.frames["Home"], text="Welcome to your Shopping List", font=('Comic Sans MS', 16), pady=100).pack()
        # self.current_frame.pack()

    def show_grocery_list(self):
        self.switch_frame("GroceryList")
        self.current_frame = tk.Frame(self.root)
        tk.Label(self.frames["GroceryList"], text="Your Grocery List:").pack()
                
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

    def show_route(self):
        self.switch_frame("Route")
        # self.current_frame = tk.Frame(self.root)
        # Add widgets for the other screen
        tk.Label(self.frames["Route"], text="Route", font=('Comic Sans MS', 16), pady=100).pack()
        self.current_frame.pack()
    
    def load_items(self):
        with open("ImageToGraph/supermarket_items.json", "r") as file:
            data = json.load(file)
            self.supermarket_items = [item for category in data.values() for item in category]
            
    def update_suggestions(self, event):
        user_input = self.item_entry.get().strip().lower()
        self.suggestions = [item.lower() for item in self.supermarket_items if user_input in item.lower()]

        if user_input and self.suggestions:
            # Create a separate frame for the suggestion list
            if hasattr(self, "suggestion_frame"):
                self.suggestion_frame.destroy()
            self.suggestion_frame = tk.Frame(self.root)
            self.suggestion_frame.pack()

            # Use a Listbox to display clickable suggestions within the suggestion frame
            self.suggestion_listbox = tk.Listbox(self.suggestion_frame, selectmode=tk.SINGLE, height=len(self.suggestions))
            self.suggestion_listbox.pack()
            for suggestion in self.suggestions:
                self.suggestion_listbox.insert(tk.END, suggestion)
                self.suggestion_listbox.bind("<ButtonRelease-1>", self.add_suggestion)

    
    def add_suggestion(self, event):
        selected_index = self.suggestion_listbox.curselection()
        if selected_index:
            index = selected_index[0]
            suggestion = self.suggestion_listbox.get(index)
            self.item_entry.delete(0, tk.END)  # Clear the entry field
            self.item_entry.insert(0, suggestion)  # Set the selected suggestion as the input



    def shortcut(self, event):
        if event.state == 4 and event.keysym == "Return":
            self.add_item()

    def switch_frame(self, frame_name):
        new_frame = self.frames.get(frame_name)
        if self.current_frame:
            self.current_frame.pack_forget()
        if not new_frame:
            new_frame = tk.Frame(self.root)
            self.frames[frame_name] = new_frame
        self.current_frame = new_frame
        self.current_frame.pack()

MyGroceryListApp()
