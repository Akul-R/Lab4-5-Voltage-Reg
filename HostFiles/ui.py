from tkinter import *                                       #importing packages
import tkinter
import customtkinter
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import serial.tools.list_ports
import time
import threading
import queue


class UI(customtkinter.CTk):
    def __init__(self):  # all widgets are objects which are children of frame objects
        super().__init__()
        #serial transmitter object
        self.serial_transmitter = serial.Serial()

        #states of system, we will send these to the microcontroller
        self.start = False #whether we start recording the measurements or not
        self.target_voltage = 1.5 #target voltage
        self.pid_enabled = False #whether pid control is enabled or not
        self.p_const = 0.6 #constants for PID, fairly self explanatory
        self.i_const = 0.5
        self.d_const = 0.05

        #other constants used by the program
        self.baud_rate = 9600
        self.com_port = ""
        self.portlist = []
        self.connected = False

        #multithreading related things
        self.data_queue = queue.Queue() #allows us to exchange data between threads easily
        self.send_data = "" #the data we want to send (for commands like START or STOP)
        #events used for threading
        self.receive_event = threading.Event() #a flag to tell us when data has been received from Il matto
        self.send_event = threading.Event() #a flag to tell serial thread that we want to send smth to Il matto
        self.stop_event = threading.Event() #tell the thread to stop, used when disconnecting or closing program

        #start of actual UI code
        self.title("SPID V1.0")  # Title of the Window
        self.geometry(f"{1200}x{600}")  # Size of Window by default
        self.resizable(False, False)

        #some variables for the graph
        self.data_x = [0] #time
        self.data_y = [0] #voltage
        self.target_y = [0] #these are data used to plot a target line
        self.max_x = 25
        self.max_y = 25

        self.headerframe = customtkinter.CTkFrame(self)
        self.headerframe.place(relx=0,rely=0,relheight=0.1,relwidth=1,anchor=NW)

        self.graphframe = customtkinter.CTkFrame(self)
        self.graphframe.place(relx=0.025,rely=0.37,relwidth=0.7,relheight=0.6,anchor=NW)

        self.serialframe = customtkinter.CTkFrame(self)
        self.serialframe.place(relx=0.975, rely=0.37, relwidth=0.23, relheight=0.25, anchor=NE)

        self.statusframe = customtkinter.CTkFrame(self)
        self.statusframe.place(relx=0.975, rely=0.64, relwidth=0.23, relheight=0.33, anchor=NE)

        self.load_pid_info()

        self.load_info()
        self.load_serial()
        self.load_status()

        self.load_graph()
        self.refresh_com()
        self.disable_pid()
        self.set_init_value()

        self.after(100, self.update_graph)  # we update the graph every 100ms as data comes in every 200ms


    ########################################################################################################
    ########################################################################################################
    #Methods for PID control bit of the UI
    def load_pid_info(self): #loads the ui elements for the PID controls
            self.pidframe = customtkinter.CTkFrame(self)
            self.pidframe.place(relx=0.025, rely=0.12, relheight=0.23, relwidth=0.7, anchor=NW)

            self.sliderframe = customtkinter.CTkFrame(self.pidframe)
            self.sliderframe.place(relx=0.01, rely=0.025, relheight=0.75, relwidth=0.98, anchor=NW)

    #proportional constant
            self.kp_var = tkinter.IntVar()
            self.p_label = customtkinter.CTkLabel(self.sliderframe, text="Kp: ")
            self.p_label.place(relx=0, rely=0, relheight=0.33, relwidth=0.05, anchor=NW)
            self.p_slider = customtkinter.CTkSlider(self.sliderframe, from_=0, to=2, number_of_steps=2000, variable=self.kp_var, command=self.updatelabel_p)
            self.p_slider.place(relx=0.05, rely=0.075, relheight=0.2, relwidth=0.8, anchor=NW)
            self.p_val = customtkinter.CTkLabel(self.sliderframe, text=self.kp_var.get())
            self.p_val.place(relx=0.85, rely=0, relheight=0.33, relwidth=0.05, anchor=NW)
            self.p_entry = customtkinter.CTkEntry(self.sliderframe, placeholder_text=self.kp_var.get())
            self.p_entry.place(relx=0.92, rely=0.015, relheight=0.3, relwidth=0.08, anchor=NW)
            self.p_entry.bind(sequence="<Return>", command=self.update_from_entry_p)

    #integral constant
            self.ki_var = tkinter.IntVar()
            self.i_label = customtkinter.CTkLabel(self.sliderframe, text="Ki: ")
            self.i_label.place(relx=0, rely=0.33, relheight=0.33, relwidth=0.05, anchor=NW)
            self.i_slider = customtkinter.CTkSlider(self.sliderframe, from_=0, to=2, number_of_steps=2000, variable=self.ki_var, command=self.updatelabel_i)
            self.i_slider.place(relx=0.05, rely=0.41, relheight=0.2, relwidth=0.8, anchor=NW)
            self.i_val = customtkinter.CTkLabel(self.sliderframe, text=self.ki_var.get())
            self.i_val.place(relx=0.85, rely=0.33, relheight=0.33, relwidth=0.05, anchor=NW)
            self.i_entry = customtkinter.CTkEntry(self.sliderframe, placeholder_text=self.ki_var.get())
            self.i_entry.place(relx=0.92, rely=0.345, relheight=0.3, relwidth=0.08, anchor=NW)
            self.i_entry.bind(sequence="<Return>", command=self.update_from_entry_i)

    #derivative constant
            self.kd_var = tkinter.IntVar()
            self.d_label = customtkinter.CTkLabel(self.sliderframe, text="Kd: ")
            self.d_label.place(relx=0, rely=0.66, relheight=0.33, relwidth=0.05, anchor=NW)
            self.d_slider = customtkinter.CTkSlider(self.sliderframe, from_=0, to=2, number_of_steps=2000, variable=self.kd_var, command=self.updatelabel_d)
            self.d_slider.place(relx=0.05, rely=0.74, relheight=0.2, relwidth=0.8, anchor=NW)
            self.d_val = customtkinter.CTkLabel(self.sliderframe, text=self.kd_var.get())
            self.d_val.place(relx=0.85, rely=0.66, relheight=0.33, relwidth=0.05, anchor=NW)
            self.d_entry = customtkinter.CTkEntry(self.sliderframe, placeholder_text=self.kd_var.get())
            self.d_entry.place(relx=0.92, rely=0.675, relheight=0.3, relwidth=0.08, anchor=NW)
            self.d_entry.bind(sequence="<Return>", command=self.update_from_entry_d)

            self.send_button = customtkinter.CTkButton(self.pidframe, text="SEND", fg_color="#d94d43", hover_color="#c1433b", command=self.send_pid_const)
            self.send_button.place(relx=0.99, rely=0.8, relheight=0.2, relwidth=0.1, anchor=NE)

            self.enable_pid = customtkinter.CTkCheckBox(self.pidframe, text="Enable PID", command=self.disable_pid)
            self.enable_pid.place(relx=0.01, rely=0.8, relheight=0.2, relwidth=0.5, anchor=NW)

    def disable_pid(self):
        if(not self.enable_pid.get()):
            #PID not enabled
            self.p_slider.configure(state="disabled", button_color="#656565")
            self.i_slider.configure(state="disabled", button_color="#656565")
            self.d_slider.configure(state="disabled", button_color="#656565")

            self.p_entry.configure(state="disabled")
            self.i_entry.configure(state="disabled")
            self.d_entry.configure(state="disabled")

            self.p_label.configure(text_color="#656565")
            self.i_label.configure(text_color="#656565")
            self.d_label.configure(text_color="#656565")

            self.p_val.configure(text_color="#656565")
            self.i_val.configure(text_color="#656565")
            self.d_val.configure(text_color="#656565")

            self.pid_enabled = False
            self.pid_status.configure(text="PID Controller Inactive", text_color="#d94d43")

            self.send_data = "DIS\r\n"
            self.send_event.set()

        else:
            self.p_slider.configure(state="normal", button_color="#3a7ebf")
            self.i_slider.configure(state="normal", button_color="#3a7ebf")
            self.d_slider.configure(state="normal", button_color="#3a7ebf")

            self.p_entry.configure(state="normal")
            self.i_entry.configure(state="normal")
            self.d_entry.configure(state="normal")

            self.p_label.configure(text_color="#DCE4EE")
            self.i_label.configure(text_color="#DCE4EE")
            self.d_label.configure(text_color="#DCE4EE")

            self.p_val.configure(text_color="#DCE4EE")
            self.i_val.configure(text_color="#DCE4EE")
            self.d_val.configure(text_color="#DCE4EE")

            self.pid_enabled = True
            self.pid_status.configure(text="PID Controller Active", text_color="#08A045")

            self.send_data = "EN\r\n"
            self.send_event.set()

    def send_pid_const(self):
        p = int(float(self.p_val.cget("text"))*1000)
        i = int(float(self.i_val.cget("text"))*1000)
        d = int(float(self.d_val.cget("text"))*1000)
        t = int((float(self.targetV_val.cget("text"))/3.3)*1023)

        pid_str = f"{p} {i} {d} {t}\r\n"

        self.send_data = pid_str
        self.send_event.set()

    def set_init_value(self):
        self.p_val.configure(text=self.p_const)
        self.i_val.configure(text=self.i_const)
        self.d_val.configure(text=self.d_const)

        self.p_slider.set(self.p_const)
        self.i_slider.set(self.i_const)
        self.d_slider.set(self.d_const)



    # methods to update the label displaying value when slider is moved
    def updatelabel_p(self, value):
        self.p_val.configure(text=str(round(value, 2)))

    def updatelabel_i(self, value):
        self.i_val.configure(text=str(round(value, 2)))

    def updatelabel_d(self, value):
        self.d_val.configure(text=str(round(value, 2)))

    # methods to update value and slider when input in entry is made
    def update_from_entry_p(self, value):
        val = round(float(self.p_entry.get()), 2)
        self.p_val.configure(text=val)
        self.p_slider.set(val)

    def update_from_entry_i(self, value):
        val = round(float(self.i_entry.get()), 2)
        self.i_val.configure(text=val)
        self.i_slider.set(val)

    def update_from_entry_d(self, value):
        val = round(float(self.d_entry.get()), 2)
        self.d_val.configure(text=val)
        self.d_slider.set(val)

    ########################################################################################################
    ########################################################################################################
    #Methods for info and other control bit of the UI
    def load_info(self):
        self.infoframe = customtkinter.CTkFrame(self)
        self.infoframe.place(relx=0.975, rely=0.12, relheight=0.23, relwidth=0.23, anchor=NE)

        self.val_frame = customtkinter.CTkFrame(self.infoframe)
        self.val_frame.place(relx=0, rely=0, relheight=0.75, relwidth=1, anchor=NW)

        #displays target voltage as:
        #TARGET VOLTAGE: 0.000000 V (or something like that)
        self.targetV_label = customtkinter.CTkLabel(self.val_frame, text="TARGET VOLTAGE: ", anchor=W)
        self.targetV_label.place(relx=0.01, rely=0, relheight=0.3, relwidth = 0.45, anchor=NW)
        self.targetV_val = customtkinter.CTkLabel(self.val_frame, text=self.target_voltage, anchor=W)
        self.targetV_val.place(relx=0.5, rely=0, relheight=0.3, relwidth=0.5, anchor=NW)
        self.targetV_unit = customtkinter.CTkLabel(self.val_frame, text="V", anchor=E)
        self.targetV_unit.place(relx=0.85, rely=0, relheight=0.3, relwidth=0.1, anchor=NW)

        # displays current voltage as:
        # CURRENT VOLTAGE: 0.000000 V (or something like that)
        self.currentV_label = customtkinter.CTkLabel(self.val_frame, text="CURRENT VOLTAGE: ", anchor=W)
        self.currentV_label.place(relx=0.01, rely=0.3, relheight=0.3, relwidth=0.45, anchor=NW)
        self.currentV_val = customtkinter.CTkLabel(self.val_frame, text="0.000", anchor=W)
        self.currentV_val.place(relx=0.5, rely=0.3, relheight=0.3, relwidth=0.5, anchor=NW)
        self.currentV_unit = customtkinter.CTkLabel(self.val_frame, text="V", anchor=E)
        self.currentV_unit.place(relx=0.85, rely=0.3, relheight=0.3, relwidth=0.1, anchor=NW)

        #entry to enter new target voltage
        self.ntarget_label = customtkinter.CTkLabel(self.val_frame, text="NEW TARGET: ", anchor=W)
        self.ntarget_label.place(relx=0.01, rely=0.7, relheight=0.3, relwidth=0.45, anchor=NW)
        self.ntarget_entry = customtkinter.CTkEntry(self.val_frame, placeholder_text="ENTER NEW TARGET")
        self.ntarget_entry.place(relx=0.35, rely=0.83, relheight=0.25, relwidth=0.6, anchor=W)
        self.ntarget_entry.bind(sequence="<Return>", command= self.update_target)

        #start/stop button
        self.start_button = customtkinter.CTkButton(self.infoframe, text="START", fg_color="#08A045", command=self.start_stop, hover_color="#078E3D")
        self.start_button.place(relx=0.01, rely=0.8, relheight=0.2, relwidth=0.3, anchor=NW)

        #error msg
        self.error_txt = customtkinter.CTkLabel(self.infoframe, text="")
        self.error_txt.place(relx=0.45, rely=0.8, relheight=0.2, relwidth=0.5, anchor=NW)

    def start_stop(self):
        if(not self.start):
            self.start = True
            self.start_button.configure(text="STOP", fg_color="#d94d43", hover_color="#c1433b")
            self.reg_status.configure(text="System Started", text_color="#08A045")
            self.send_data = "START\r\n"
            self.send_event.set()
        else:
            self.start = False
            self.start_button.configure(text="START", fg_color="#08A045", hover_color="#078E3D")
            self.reg_status.configure(text="System Stopped", text_color="#d94d43")
            self.send_data = "STOP\r\n"
            self.send_event.set()

    def update_target(self, value):
        val = round(float(self.ntarget_entry.get()), 2)
        if(val > 3.3):
            self.error_txt.configure(text="TARGET TOO BIG!")
            val = 3.3
        elif(val < 0):
            self.error_txt.configure(text="TARGET TOO SMALL!")
            val = 0
        else:
            self.error_txt.configure(text="")
        self.targetV_val.configure(text=val)
        self.target_voltage = val
        self.send_pid_const()


    ########################################################################################################
    ########################################################################################################
    # Methods for serial bit of the UI
    def load_serial(self):
        self.serial_title = customtkinter.CTkLabel(self.serialframe, text="Serial Setup:")
        self.serial_title.place(relx=0, rely=0, relwidth=1, relheight=0.2, anchor=NW)

        self.baud_label = customtkinter.CTkLabel(self.serialframe, text="Baud Rate:", anchor=W)
        self.baud_label.place(relx=0.01, rely=0.25, relwidth=0.5, relheight=0.2, anchor=NW)
        self.baud_select = customtkinter.CTkComboBox(self.serialframe, values=["9600", "19200", "38400"], state="readonly")
        self.baud_select.place(relx=0.3, rely=0.34, relwidth=0.65, relheight=0.18, anchor=W)

        self.com_label = customtkinter.CTkLabel(self.serialframe, text="COM Port:", anchor=W)
        self.com_label.place(relx=0.01, rely=0.5, relwidth=0.5, relheight=0.2, anchor=NW)
        self.com_select = customtkinter.CTkComboBox(self.serialframe, values=self.portlist, state="readonly")
        self.com_select.place(relx=0.3, rely=0.59, relwidth=0.65, relheight=0.18, anchor=W)

        self.connect_button = customtkinter.CTkButton(self.serialframe, text="CONNECT", command=self.connect_serial, fg_color="#08A045", hover_color="#078E3D")
        self.connect_button.place(relx=0.01, rely=0.78, relwidth= 0.62, relheight=0.2, anchor=NW)

        self.refresh_button = customtkinter.CTkButton(self.serialframe, text="REFRESH", command=self.refresh_com)
        self.refresh_button.place(relx=0.64, rely=0.78, relwidth=0.35, relheight=0.2, anchor=NW)

    def connect_serial(self):
        if(not self.connected):
            #connecting (or trying to)
            baud = self.baud_select.get()
            com = self.com_select.get()
            self.error_status.configure(text="", text_color="#d94d43")

            #checking if the selected values are valid
            if(baud == ""):
                self.error_status.configure(text="SELECT BAUD RATE")
                return
            elif(com == ""):
                self.error_status.configure(text="SELECT COM PORT")
                return
            elif(com == "NO COM PORTS"):
                self.error_status.configure(text="SELECT COM PORT (TRY REFRESHING)")
                return

            self.baud_rate = baud
            self.com_port = com

            self.serial_transmitter.baudrate = baud
            self.serial_transmitter.port = com
            self.serial_transmitter.timeout = 0.01
            self.serial_transmitter.open()
            if(self.serial_transmitter.isOpen()):
                #checking if serial transmitter is open
                self.error_status.configure(text="CONNECTED SUCCESSFULLY :)", text_color="#08A045")
                self.connection_status.configure(text="IL MATTO CONNECTED", text_color="#08A045")
                self.connected = True
                self.connect_button.configure(text="DISCONNECT", fg_color="#d94d43", hover_color="#c1433b")

                #starting serial transmitter thread
                if(self.stop_event.is_set()):
                    self.stop_event.clear()

                self.serial_thread = threading.Thread(target=self.serial_worker, daemon=True)  # the serial thread
                #we creare a new instance of the thread every time we connect as we cant reuse the same thread
                self.serial_thread.start()
            else:
                #if its not open, connection was not succesful
                self.error_status.configure(text="COULD NOT CONNECT :(")
                self.connection_status.configure(text="IL MATTO DISCONNECTED", text_color="#d94d43")
                self.connected = False
                self.connect_button.configure(text="CONNECT", fg_color="#08A045", hover_color="#078E3D")
                return

        else:
            #disconnecting
            self.connect_button.configure(text="CONNECT", fg_color="#08A045", hover_color="#078E3D")
            self.connection_status.configure(text="IL MATTO DISCONNECTED", text_color="#d94d43")
            self.error_status.configure(text="")
            self.connected = False
            self.stop_event.set()
            self.serial_thread.join()

    def refresh_com(self):
        #searches for com ports
        ports = serial.tools.list_ports.comports()
        self.portlist = []
        for port in ports:
            self.portlist.append(port.name)

        if (len(self.portlist) == 0):
            self.portlist.append("NO COM PORTS")

        self.com_select.configure(values=self.portlist)

    def serial_worker(self):
        #running the serial transmitter stuff in a seperate thread to prevent UI from freezing up
        self.serial_transmitter.flushInput()

        while(not self.stop_event.is_set()):
            #we check if the stop flag has been raised. if not, we can continue
            #first we check if we need to send something to il matto
            if(self.send_event.is_set()):
                #the send flag has been raised so we want to send a command to il matto
                try:
                    self.serial_transmitter.write(self.send_data.encode('utf-8'))
                    print("SENT: ", self.send_data)
                except Exception as e:
                    #should print out the error if there are any
                    print("TRANSMISSION ERROR: ", e)
                    print("TYPE: ", type(e))
                finally:
                    #we clear the send flag and the place where we store the data to send
                    self.send_event.clear()
                    self.send_data = ""


            if(self.serial_transmitter.inWaiting() > 0):
                #this tells us that there are bits waiting in the input buffer
                try:
                    line = self.serial_transmitter.readline()
                    if line:
                        self.data_queue.put(line.decode('utf-8', errors="ignore").strip())
                        print(line.decode('utf-8'))
                except Exception as e:
                    print("RECEIVE ERROR: ", e)
                    print("TYPE: ", type(e))

            time.sleep(0.01)
            #i found online that if i dont include this my CPU usage would go to 100%
            #i dont really fancy my laptop exploding
        self.serial_transmitter.close()
        print("SERIAL TRANSMITTER DISCONNECTED")

    ########################################################################################################
    ########################################################################################################
    # Methods for status bit of the UI
    def load_status(self):
        self.connection_status = customtkinter.CTkLabel(self.statusframe, text="Il Matto Disconnected", anchor=W, text_color="#d94d43")
        self.connection_status.place(relx=0.5, rely=0.01, relwidth=0.95, relheight=0.1, anchor=N)

        self.reg_status = customtkinter.CTkLabel(self.statusframe, text="System Stopped", anchor=W, text_color="#d94d43")
        self.reg_status.place(relx=0.5, rely=0.15, relwidth=0.95, relheight=0.1, anchor=N)

        self.pid_status = customtkinter.CTkLabel(self.statusframe, text="PID Controller Inactive", anchor=W, text_color="#d94d43")
        self.pid_status.place(relx=0.5, rely=0.3, relwidth=0.95, relheight=0.1, anchor=N)

        self.error_status = customtkinter.CTkLabel(self.statusframe, text = "", anchor = W, text_color="#d94d43")
        self.error_status.place(relx=0.5, rely=0.45, relwidth=0.95, relheight=0.1, anchor=N)

        self.clear_graph = customtkinter.CTkButton(self.statusframe, text="CLEAR GRAPH", fg_color="#d94d43", hover_color="#c1433b", command=self.clear_graph_data)
        self.clear_graph.place(relx=0.5, rely=0.7, relwidth=0.95, relheight=0.15, anchor=N)

        self.message = customtkinter.CTkLabel(self.statusframe, text="SPID V1.0", text_color="#656565", font=("", 10), anchor=E)
        self.message.place(relx=0.5, rely=0.89, relwidth=0.95, relheight=0.1, anchor=N)

    ########################################################################################################
    ########################################################################################################
    #Methods for graph bit of the UI

    def clear_graph_data(self):
        self.data_x = [0]
        self.data_y = [0]
        self.target_x = [0]
        self.target_y = [0]
        self.line.set_data(self.data_x, self.data_y)
        self.ax.set_xlim(0, 5)
        self.canvas.draw_idle()  # updating the canvas


    def load_graph(self):
        #loading the graph
        fig = Figure(facecolor='#2C2C2C')
        t = np.arange(0, 30, .01)
        self.ax = fig.add_subplot()
        self.ax.set_facecolor('#3F3F3F')
        self.ax.set_ylim(0, 3.5)
        self.ax.set_xlim(0, 5)
        label_color = 'white'
        #the graph formatting was done with the help of Gen AI (Google Gemini 2.5)
        self.ax.set_xlabel("time [s]", color=label_color)
        self.ax.set_ylabel("Vol5tage [V]", color=label_color)
        self.ax.tick_params(axis='x', colors=label_color)
        self.ax.tick_params(axis='y', colors=label_color)
        self.ax.spines['bottom'].set_color(label_color)
        self.ax.spines['top'].set_color(label_color)
        self.ax.spines['right'].set_color(label_color)
        self.ax.spines['left'].set_color(label_color)

        self.line, = self.ax.plot(self.data_x, self.data_y, color='#7AC0E8', label="Voltage")
        self.target_line, = self.ax.plot(self.data_x, self.target_y, color ="#d94d43", linestyle="--", label="Target")

        self.ax.legend(
            loc='upper right', facecolor='#3F3F3F', edgecolor='white', labelcolor='white')
        self.ax.grid(True, color='#555555', linestyle='-', linewidth=0.5)
        self.canvas = FigureCanvasTkAgg(fig, master=self.graphframe)
        self.canvas.draw()
        self.canvas.get_tk_widget().place(relx=0, rely=0, relwidth=1, relheight=1, anchor=NW)

    def update_graph(self):
        if(self.start):
            new_data = False
            while True:
                #keeps reading from data queue until it is empty
                try:
                    #i was getting errors here so i wrapped everything in a try except block
                    #is it the best fix? probably not
                    data_string = self.data_queue.get_nowait() #getting the data from queue
                    try:
                        #i expect the data to come as (time,voltage)
                        #new method to avoid fragmentation, sent as #time,voltage#
                        #the hashtags tell us if the data is fragmented or not (i got this idea as i was going to sleep)
                        if data_string.startswith('#') and data_string.endswith('#'):
                            #get rid of # as they are just to ensure data is not fragmented
                            data_string = data_string.strip('#')
                            parts = data_string.split(',')
                            #we split data into its time and voltage parts
                            if len(parts) == 2:
                                new_time = float(parts[0]) * 0.2 #time will arrive as 1 = 200ms, 2 = 400ms
                                new_voltage = (float(parts[1])/1023)*3.3 #converting adc val to voltage

                                #adding data to graph x and y
                                if(new_time > self.data_x[-1]):
                                    #checking for error data in time
                                    self.data_x.append(new_time)
                                    self.data_y.append(new_voltage)
                                    self.target_y.append(self.target_voltage)
                                    mag = abs(new_voltage-float(self.targetV_val.cget("text"))) #how far we are from our target
                                    text_colour = "#3F3F3F"
                                    if(mag < 0.3):
                                        text_colour = "#08A045"
                                    elif(mag >= 0.3 and mag < 0.8):
                                        text_colour = "#FFB300"
                                    else:
                                        text_colour = "#d94d43"

                                    self.currentV_val.configure(text=str(round(new_voltage, 4)), text_color=text_colour)
                                    new_data = True

                            else:
                                print("ERROR: DATA NOT IN CORRECT FORMAT")
                                print(data_string)
                        else:
                            print("DATA FRAGMENTED")
                            print(data_string)

                    except:
                        print("ERROR: COULD NOT CONVERT DATA" + {data_string})

                except:
                    #this means the data queue is empty, no need to read from it
                    break

            if(new_data):
                #handles the scrolling of the x axis as time goes on. I used some Gen AI in some parts
                if(len(self.data_x) > self.max_x):
                    #Used Gen AI (Google Gemini 2.5) to debug this part
                    #trimming the data to ensure we dont use too much memory
                    trim_index = len(self.data_x) - self.max_x

                    self.data_x = self.data_x[trim_index:]
                    self.data_y = self.data_y[trim_index:]
                    self.target_y = self.target_y[trim_index:]

                self.line.set_data(self.data_x, self.data_y)
                self.target_line.set_data(self.data_x, self.target_y)

                #this bit will handle automatically scrolling the x axis
                if self.data_x: #ie is data x is not empty
                    current_max_time = self.data_x[-1] #last element in data x is the max time (so far)

                    # If the latest time is greater than the current visible window width
                    if len(self.data_x) > 24:
                        new_xmin = self.data_x[0]
                        self.ax.set_xlim(new_xmin, current_max_time) #resizing axis

                    #handle initial start-up when datax is empty. Technically doesnt ever happen since
                    #I initialised data_x to always start with 0 rather than being empty
                    else:
                        self.ax.set_xlim(0, 5)

                #Gen AI (Google Gemini 2.5) was used for these two lines of code below
                self.canvas.draw_idle() #updating the canvas
        self.after(100, self.update_graph)  # we update the graph every 100ms as data comes in every 200ms


window = UI() #creating a UI object from my class
window.mainloop() #the mainloop of the ui