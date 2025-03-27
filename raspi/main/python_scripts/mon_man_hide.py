#!/usr/bin/env python

import rospy
from rosgraph_msgs.msg import Log
import tkinter as tk
import threading
from PIL import Image, ImageTk
import time


class RplidarLogger:
    def __init__(self):
        self.display_window = False
        
        self.message_to_display = ""
        self.text_color = "white"  # Default text color

    def hide_window(self, root):
        self.display_window = False

    def check_messages(self):
        rospy.Subscriber('/rosout', Log, self.callback)
        rospy.spin()

    def display_popup(self):
        """Function to continuously check for the state and create a fullscreen popup window."""
        root = tk.Tk()
        root.attributes('-fullscreen', True)  # Make the window fullscreen

        # Get screen dimensions
        screen_width = root.winfo_screenwidth()
        screen_height = root.winfo_screenheight()

        # Load the background image using PIL
        try:
            original_image = Image.open('/home/walle/Pictures/currentscreen.png')
            button_image_file = Image.open('/home/walle/Pictures/off_black.png')
            
            # Scale the image
            desired_width = 110  # Set your desired width
            desired_height = 142 # Set your desired height
            button_image_file = button_image_file.resize((desired_width, desired_height))
            
            button_image = ImageTk.PhotoImage(button_image_file)
        except Exception as e:
            rospy.logerr(f"Error loading background image: {e}")
            return
        
        # Resize the image to fit the screen
        resized_image = original_image.resize((screen_width, screen_height))
        bg_image = ImageTk.PhotoImage(resized_image)

        # Create a canvas to display the background image
        canvas = tk.Canvas(root, width=screen_width, height=screen_height, bg="black")
        canvas.pack(fill="both", expand=True)

        # Add the background image to the canvas
        canvas.create_image(screen_width/2, screen_height/2, image=bg_image)

        # Actual Text
        message_text = canvas.create_text(screen_width/2, screen_height/2, text=self.message_to_display, font=("Helvetica", 24), fill=self.text_color)
        
        # Create the button using the image
        hide_button = tk.Button(root, image=button_image, command=lambda: self.hide_window(root),
        borderwidth=0, highlightthickness=1, activebackground='black', relief='flat',
        activeforeground='black', disabledforeground='black', highlightbackground='black', highlightcolor='black', state='active')
        hide_button.image = button_image  # Keep a reference to the image object to avoid garbage collection
        
        # Set the button's location
        button_x = 112  # Set your desired x-coordinate
        button_y = 457  # Set your desired y-coordinate

        # Now, position the button on top of this rectangle
        hide_button_window = canvas.create_window(button_x, button_y, anchor=tk.NW, window=hide_button)


        while not rospy.is_shutdown():
            if self.display_window:
                # Update the text and its color
                canvas.itemconfig(message_text, text=self.message_to_display, fill=self.text_color)
                root.deiconify()
            else:
                root.withdraw()

            root.update_idletasks()
            root.update()
            time.sleep(0.1)  # Sleep for 100 milliseconds

    def callback(self, msg):
        # Filtering messages from the rplidarNode and launcher
        if msg.name in ["/rplidarNode", "/launcher"]:
            self.message_to_display = "{}".format(msg.msg)
            rospy.loginfo(self.message_to_display)

            # Determine text color based on message level
            if msg.level == 2:
                self.text_color = "blue"
            elif msg.level == 8:
                self.text_color = "red"
            else:
                self.text_color = "white"
            self.display_window = True

    def run(self):
        rospy.init_node('monitor')
        # Start the message checking in a separate thread
        threading.Thread(target=self.check_messages).start()
        # Start the popup display function in the main thread
        self.display_popup()


if __name__ == '__main__':
    logger = RplidarLogger()
    try:
        logger.run()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down monitor due to keyboard interrupt.")
        rospy.signal_shutdown("Keyboard interrupt detected")
    except Exception as e:
        rospy.logerr(f"Error: {e}")
        rospy.signal_shutdown("Unexpected error")

