import tkinter as tk
from tkinter import *
import json
import random # for testing purposes of the route page
from PIL import Image, ImageTk
from ImageToGraph.utils import *

class MyGroceryListApp:

    def __init__(self):
        self.root = tk.Tk()
        self.menubar = tk.Menu(self.root) # create a menu
        self.grocery_list = [] # start with an empty shopping list

        self.dir = 'ImageToGraph/' # set a directory for the images
        self.imagedir = '' # to be set with the use of the buttons

        # Set colors (color scheme: https://coolors.co/114b5f-1a936f-88d498-c6dabf-f3e9d2)
        app_background_color = "#F3E9D2"
        self.root.configure(bg=app_background_color)
        button_color = "#4CAF50" # green
        
        # Home page
        self.home_page = Frame(self.root, bg=app_background_color)
        self.home_page.grid(row=0, column=0, sticky="nsew")
        home_lb = Label(self.home_page, text="Select a supermarket", font=('Kozuka Gothic Pro H', 14), bg=app_background_color)
        home_lb.grid(padx=40, pady=20)
        self.selected_map = ""

        self.map1_image = PhotoImage(file=self.dir+'Model1.png') # make images applicable to the buttons
        self.map2_image = PhotoImage(file=self.dir+'Model2.png') 

        self.map1_button = tk.Button(self.home_page, image=self.map1_image, command=self.select_supermarket1)
        self.map1_button.grid()

        self.map2_button = tk.Button(self.home_page, image=self.map2_image, command=self.select_supermarket2)
        self.map2_button.grid()

        # Shopping list page
        self.list_page = Frame(self.root, bg=app_background_color)
        self.list_page.grid(row=0, column=0, sticky="nsew")

        self.entry_lb = tk.Label(self.list_page, text="Enter item:", bg=app_background_color, font=('Arial', 8))
        self.entry_lb.grid(row=0, column=0, pady=10)

        self.item_entry = tk.Entry(self.list_page, width=20) # create an input field for user to find
        self.item_entry.grid(row=1, column=0, padx=0, pady=5)
        self.item_entry.bind("<KeyRelease>", self.update_suggestions) # show suggestions during typing


        self.suggest_lb = tk.Label(self.list_page, text="Click on suggestion to add:", bg=app_background_color, font=('Arial', 8))
        self.suggest_lb.grid(row=0, column=1, pady=10)
        self.suggestions_listbox = tk.Listbox(self.list_page, width=20)
        self.suggestions_listbox.grid(row=2, column=1, padx=10, pady=10)

        # suggestions_scrollbar = tk.Scrollbar(self.list_page, orient=tk.VERTICAL, command=self.suggestions_listbox.yview)
        # suggestions_scrollbar.grid(row=0, column=2, padx=0, sticky='ns')
        # self.suggestions_listbox.config(yscrollcommand=suggestions_scrollbar.set)

        self.suggestions_listbox.bind("<Double-1>", self.add_suggestion_to_list) # double click to add suggested item to shopping list

        self.listbox = tk.Listbox(self.list_page, width=20)
        self.listbox.grid(row=2, column=0, columnspan=1, padx=5, pady=10)

        self.listbox.bind("<Double-1>", self.delete_item) # double click to delete item from shopping list

        send_button = tk.Button(self.list_page, text="Send Shopping List", font=('Kozuka Gothic Pro H', 12),  command=self.send_shopping_list, fg="white", bg="#114B5F") # send shopping list to route maker
        send_button.grid(row=3, column=1, padx=15)


        # Route page
        self.route_page = Frame(self.root, bg=app_background_color) 
        self.route_page.grid(row=0, column=0, sticky="nsew")
        route_lb = Label(self.route_page, text="Your Route", font=('Kozuka Gothic Pro H', 10), bg=app_background_color)
        route_lb.grid(row=0, column=0, padx=120, pady=20)

        self.next_item_label = Label(self.route_page, text="", bg=app_background_color)
        self.next_item_label.grid(row=1, column=0, pady=20)

        picked_button = tk.Button(self.route_page, text="Picked item", font=('Kozuka Gothic Pro H', 12), command=self.picked_item, bg=app_background_color) # user let's know they picked the current item
        picked_button.grid(row=2, column=0)

        pause_button =  tk.Button(self.route_page, text="Pause", font=('Kozuka Gothic Pro H', 12), bg=app_background_color) # pause the shopping cart
        pause_button.grid(row=3, column=0)
        continue_button =  tk.Button(self.route_page, text="Continue", font=('Kozuka Gothic Pro H', 12), bg=app_background_color) # continue the shopping cart
        continue_button.grid(row=4, column=0)


        # App startup
        self.home_page.tkraise() # start at home page

        self.root.geometry("310x550") #og size of app window: 310x550
        self.root.title("Supermarket Cart")
        self.root.resizable(False, False) # make it impossible to resize app

        self.create_menu(self.home_page, self.list_page, self.route_page)
        self.load_items() # load the supermarket items

        self.root.mainloop()

    def create_menu(self, page1, page2, page3):
        """
        Creates a menubar in which you can switch between the different pages.
        """
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)
        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Menu", menu=file_menu)
        file_menu.add_command(label="Home", command=lambda: page1.tkraise())
        file_menu.add_command(label="Shopping List", command=lambda: page2.tkraise())
        file_menu.add_command(label="Route", command=lambda: page3.tkraise())

    def select_supermarket1(self):
        """
        Selects the first supermarket by changing the image directory and updating the supermarket.
        """
        self.set_button_color(self.map1_button, '#88D498')
        self.set_button_color(self.map2_button, 'SystemButtonFace') # Reset color for the other button
        self.imagedir = self.dir + 'Model1.png'
        self.update_supermarket()

        
    def select_supermarket2(self):
        """
        Selects the second supermarket by changing the image directory and updating the supermarket.
        """
        self.set_button_color(self.map2_button, '#88D498')
        self.set_button_color(self.map1_button, 'SystemButtonFace') # Reset color for the other button
        self.imagedir = self.dir + 'Model2.png'
        self.update_supermarket()
        

    def set_button_color(self, button, color):
        """
        Sets the background color of a button.
        """
        button.config(bg=color)
        return

    def update_supermarket(self):
        """
        Reload the image and recreate the graph based on the selected supermarket
        """
        self.image = np.array(Image.open(self.imagedir).convert('RGB'))
        self.image = self.image[10:210, 10:210, :] # keep only the supermarket plan inside the frame
        self.graph = create_graph(self.image)

    def load_items(self):
        """
        Load the supermarket items from a json file
        """
        with open("ImageToGraph/supermarket_items.json", "r") as file:
            data = json.load(file)
            self.supermarket_items = [item for category in data.values() for item in category]

    def update_suggestions(self, event):
        """
        Give product suggestions based on the user input.
        """
        user_input = self.item_entry.get()
        self.suggestions_listbox.delete(0, tk.END) # delete suggestions
        for product in self.supermarket_items: # add supermarket item to the suggestion list if the user input is part of the item
            if user_input.lower() in product.lower():
                self.suggestions_listbox.insert(tk.END, product)

    def add_suggestion_to_list(self, event):
        """
        Add a suggestion to the shopping list.
        """
        selected_suggestion = self.suggestions_listbox.get(self.suggestions_listbox.curselection())
        if selected_suggestion:
            self.grocery_list.append(selected_suggestion)
            self.update_shopping_list()

    def delete_item(self, event):
        """
        Remove an existing item from the shopping list.
        """
        selected_item = self.listbox.get(self.listbox.curselection())[3:] # the first part of the string is a number, a dot and a space
        if selected_item:
            self.grocery_list.remove(selected_item)
            self.update_shopping_list()

    def update_shopping_list(self):
        """
        Define how an item should be added to the list.
        """
        self.listbox.delete(0, tk.END)
        for index, item in enumerate(self.grocery_list, start=1):
            self.listbox.insert(tk.END, f"{index}. {item}")

    def send_shopping_list(self):
        """
        Base the most optimal path on the shopping list. 
        Show which item is next on the Route Page.
        """

        print(self.grocery_list)
        items, coordinates, length = hamiltonian_path(self.graph, list(set(self.grocery_list)))
        self.grocery_list = items
        print(self.grocery_list)
        
        paths = [nx.dijkstra_path(self.graph, coordinates[i], coordinates[i+1]) for i in range(len(coordinates)-1)]
        draw_path(self.image, paths)

  
        # Update the next item on the route page
        
        if self.grocery_list:
            next_item = self.grocery_list[0]
            self.next_item_label.config(text=f"Next Item: {next_item}", font=('Kozuka Gothic Pro H', 10), bg="#F3E9D2")
        else:
            self.next_item_label.config(text="No items in the shopping list", font=('Kozuka Gothic Pro H', 10), bg="#F3E9D2")

        # Display a confirmation message
        confirmation_label = Label(self.list_page, text="Shopping list sent!", font=('Kozuka Gothic Pro H', 10), bg="#F3E9D2")
        confirmation_label.grid(row=3, column=0)

        # Schedule a function to remove the confirmation message after 3000 milliseconds (3 seconds)
        self.root.after(3000, lambda: confirmation_label.grid_forget())  

        return items
    
    def picked_item(self):
        """
        Remove the item from the shopping list when it is picked.
        Show which item is next on the Route Page.
        """

        if self.grocery_list:
            self.grocery_list.pop(0)
        print(self.grocery_list)

        # Update the next item on the route page
        
        if self.grocery_list:
            next_item = self.grocery_list[0]
            self.next_item_label.config(text=f"Next Item: {next_item}", font=('Kozuka Gothic Pro H', 10), bg="#F3E9D2")
        else:
            self.next_item_label.config(text="No items left. You're done!", font=('Kozuka Gothic Pro H', 10), bg="#F3E9D2")

        return True



        

MyGroceryListApp()
