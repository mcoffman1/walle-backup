#!/usr/bin/env python

import rospy
from rosgraph_msgs.msg import Log
import tkinter as tk
import threading
from PIL import Image, ImageTk


class RplidarLogger:
    def __init__(self):
        self.message_event = threading.Event()
        self.message_to_display = ""
        self.text_color = "white"  # Default text color

    def display_popup(self):
        """Function to continuously check for the event and create a fullscreen popup window."""
        root = tk.Tk()
        root.attributes('-fullscreen', True)  # Make the window fullscreen

        # Get screen dimensions
        screen_width = root.winfo_screenwidth()
        screen_height = root.winfo_screenheight()

        # Load the background image using PIL
        try:
            original_image = Image.open('/home/walle/Pictures/currentscreen.png')
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

        root.withdraw()  # Initially hide the window

        while not rospy.is_shutdown():
            self.message_event.wait(5)  # Block for 5 seconds or until the event is set
            if self.message_event.is_set():
                # Update the text and its color
                canvas.itemconfig(message_text, text=self.message_to_display, fill=self.text_color)
                root.deiconify()  # Show the window
                root.update_idletasks()
                root.update()
                self.message_event.clear()  # Reset the event
            else:
                root.withdraw()  # Hide the window

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
        
            self.message_event.set()  # Trigger the popup

    def run(self):
        rospy.init_node('rplidar_logger')
        rospy.Subscriber('/rosout', Log, self.callback)
        # Start the popup display function in the main thread
        self.display_popup()


if __name__ == '__main__':
    logger = RplidarLogger()
    try:
        logger.run()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down rplidar_logger due to keyboard interrupt.")
        rospy.signal_shutdown("Keyboard interrupt detected")
    except Exception as e:
        rospy.logerr(f"Error: {e}")
        rospy.signal_shutdown("Unexpected error")

