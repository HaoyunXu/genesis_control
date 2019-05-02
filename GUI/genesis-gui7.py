#!/usr/bin/env python2
# -*- coding: UTF-8 -*-
#
# Created by Patrick Tomsky
#

import wx
import rospy
import threading
import time

from std_msgs.msg import UInt8
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray

# begin wxGlade: dependencies
# end wxGlade

# begin wxGlade: extracode
# end wxGlade

# Default values
int_1 = 0
int_2 = 0
int_3 = None
cont_stat = 0
left_side = 1
right_side = 1
front_side = 1


class MyFrame(wx.Frame):
    def __init__(self, *args, **kwds):
        # begin wxGlade: MyFrame.__init__
        kwds["style"] = kwds.get("style", 0) | wx.DEFAULT_FRAME_STYLE
        wx.Frame.__init__(self, *args, **kwds)
        self.SetSize((780, 864))
        self.text_ctrl_1 = wx.TextCtrl(self, wx.ID_ANY, "", size = (600,40))
	self.Timer = wx.Timer(self, wx.ID_OK)
	self.Timer.Start(100)
	self.Bind(wx.EVT_TIMER, self.onTimer, self.Timer)
        #self.text_ctrl_2 = wx.TextCtrl(self, wx.ID_ANY, "")
        #self.text_ctrl_3 = wx.TextCtrl(self, wx.ID_ANY, "")
        self.panel_1 = wx.Panel(self, wx.ID_ANY)
        self.text_ctrl_5 = wx.TextCtrl(self, wx.ID_ANY, "", size = (100,40))
        self.text_ctrl_4 = wx.TextCtrl(self, wx.ID_ANY, "", size = (300,40))
        self.button_1 = wx.Button(self, wx.ID_ANY, "CONTINUE")
	self.button_1.Bind(wx.EVT_BUTTON, self.onClick)
        self.panel_3 = wx.Panel(self, wx.ID_ANY)
        self.panel_2 = wx.Panel(self, wx.ID_ANY)
        self.panel_4 = wx.Panel(self, wx.ID_ANY)
<<<<<<< HEAD
	self.pub = rospy.Publisher('buttonvalue',UInt8, queue_size = 10)
=======
	self.pub = rospy.Publisher('/vehicle/go_cmd',Bool, queue_size = 10)
	
>>>>>>> 9af6dfae9a0e61e4c895aee6d7f2371cd71b069c



        self.__set_properties()
        self.__do_layout()
        # end wxGlade
    def onClick(self, evt):
	button_value = True
        rospy.loginfo(button_value)
        self.pub.publish(button_value)

    def onTimer(self, evt):
	global int_1
	global int_2
        global int_3

	#Top Message Window
	font1 = wx.Font(16, wx.NORMAL, wx.NORMAL, wx.NORMAL)
	self.text_ctrl_1.SetFont(font1)
	if int_1 == 0:
	    self.text_ctrl_1.SetValue("NO ERRORS")
	    self.text_ctrl_1.SetBackgroundColour((255,255,255))
	elif int_1 == 1:
	    self.text_ctrl_1.SetValue("CONTROL FAILED: TAKE OVER STEERING")
	    self.text_ctrl_1.SetBackgroundColour((255,0,0))
	#Controller Status Color Panel
	if cont_stat == 0:
	    self.panel_1.SetBackgroundColour(wx.Colour(255,0,0)) #Red
	elif cont_stat == 1:
	    self.panel_1.SetBackgroundColour(wx.Colour(0,255,0)) #Green

	#Left Message Window (Car Status)
	self.text_ctrl_5.SetFont(font1)
	if int_2 == 0:
	    self.text_ctrl_5.SetValue("OFF")
	elif int_2 == 1:
	    self.text_ctrl_5.SetValue("ON")

	#Right Message Window (Controller Status)
	self.text_ctrl_4.SetFont(font1)
	if int_3 is None:
	    self.text_ctrl_4.SetValue("OFF")
	else:
	    self.text_ctrl_4.SetValue("Cruise Control")
	

	#Radar Detection Panel (Front of Car)
	if front_side == 1:
	    self.panel_2.SetBackgroundColour(wx.Colour(255,0,0)) #Red
	elif front_side == 2:
	    self.panel_2.SetBackgroundColour(wx.Colour(255,255,0)) #Yellow
	elif front_side == 3:
	    self.panel_2.SetBackgroundColour(wx.Colour(0,255,0)) #Green

	#Lidar Detection Panel (Left Side of Car)
	if left_side == 1:
	    self.panel_3.SetBackgroundColour(wx.Colour(255,0,0)) #Red
	elif left_side == 2:
	    self.panel_3.SetBackgroundColour(wx.Colour(255,255,0)) #Yellow
	elif left_side == 3:
	    self.panel_3.SetBackgroundColour(wx.Colour(0,255,0)) #Green

	#Lidar Detection Panel (Right Side of Car)
	if right_side == 1:
	    self.panel_4.SetBackgroundColour(wx.Colour(255,0,0)) #Red
	elif right_side == 2:
	    self.panel_4.SetBackgroundColour(wx.Colour(255,255,0)) #Yellow
	elif right_side == 3:
	    self.panel_4.SetBackgroundColour(wx.Colour(0,255,0)) #Green









    def __set_properties(self):
        # begin wxGlade: MyFrame.__set_properties
        self.SetTitle("Genesis GUI")
        self.panel_1.SetBackgroundColour(wx.Colour(0, 255, 127))
        self.text_ctrl_4.SetMinSize((250, 30))
        self.button_1.SetMinSize((200, 200))
        self.button_1.SetBackgroundColour(wx.Colour(42, 255, 255))
        self.button_1.SetFont(wx.Font(16, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL, 0, ""))
        self.panel_3.SetBackgroundColour(wx.Colour(0, 255, 127))
        self.panel_2.SetBackgroundColour(wx.Colour(0, 255, 127))
        self.panel_4.SetBackgroundColour(wx.Colour(0, 255, 127))
        # end wxGlade

    def __do_layout(self):
        # begin wxGlade: MyFrame.__do_layout
        sizer_2 = wx.BoxSizer(wx.VERTICAL)
        grid_sizer_6 = wx.GridSizer(1, 4, 0, 0)
        grid_sizer_9 = wx.GridSizer(2, 1, 0, 0)
        grid_sizer_11 = wx.GridSizer(1, 2, 0, 0)
        sizer_1 = wx.BoxSizer(wx.VERTICAL)
        grid_sizer_8 = wx.GridSizer(2, 1, 0, 0)
        grid_sizer_12 = wx.GridSizer(3, 1, 0, 0)
        grid_sizer_7 = wx.GridSizer(2, 1, 0, 0)
        grid_sizer_10 = wx.GridSizer(1, 2, 0, 0)
        grid_sizer_3 = wx.GridSizer(1, 3, 0, 0)
        grid_sizer_5 = wx.GridSizer(1, 1, 0, 0)
        sizer_4 = wx.BoxSizer(wx.HORIZONTAL)
        grid_sizer_2 = wx.GridSizer(2, 1, 0, 0)
        grid_sizer_4 = wx.GridSizer(1, 2, 0, 0)
        grid_sizer_1 = wx.GridSizer(2, 1, 0, 0)
        sizer_3 = wx.BoxSizer(wx.HORIZONTAL)
        MESSAGES = wx.StaticText(self, wx.ID_ANY, "MESSAGES:")
        MESSAGES.SetFont(wx.Font(20, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL, 0, ""))
        sizer_3.Add(MESSAGES, 0, wx.LEFT, 0)
        sizer_3.Add(self.text_ctrl_1, 0, wx.LEFT, 0)
        #CONTROLLER = wx.StaticText(self, wx.ID_ANY, "CONTROLLER:")
        #CONTROLLER.SetFont(wx.Font(20, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL, 0, ""))
        #sizer_3.Add(CONTROLLER, 0, wx.LEFT, 0)
        #sizer_3.Add(self.text_ctrl_2, 0, 0, 0)
        #label_2 = wx.StaticText(self, wx.ID_ANY, "CRUISE C:")
        #label_2.SetFont(wx.Font(20, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL, 0, ""))
        #sizer_3.Add(label_2, 0, wx.LEFT, 0)
        #sizer_3.Add(self.text_ctrl_3, 0, 0, 0)
        sizer_2.Add(sizer_3, 1, wx.EXPAND, 0)
        label_1 = wx.StaticText(self, wx.ID_ANY, "CONTROLLER STATUS:")
        label_1.SetFont(wx.Font(16, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL, 0, ""))
        grid_sizer_1.Add(label_1, 0, 0, 0)
        label_3 = wx.StaticText(self, wx.ID_ANY, "CAR STATUS:")
        label_3.SetFont(wx.Font(16, wx.FONTFAMILY_DEFAULT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL, 0, ""))
        grid_sizer_1.Add(label_3, 0, 0, 0)
        sizer_4.Add(grid_sizer_1, 1, wx.EXPAND, 0)
        grid_sizer_4.Add(self.panel_1, 1, wx.EXPAND, 0)
        grid_sizer_4.Add((0, 0), 0, 0, 0)
        grid_sizer_2.Add(grid_sizer_4, 1, wx.EXPAND, 0)
        grid_sizer_2.Add(self.text_ctrl_5, 0, 0, 0)
        sizer_4.Add(grid_sizer_2, 1, wx.EXPAND, 0)
        sizer_4.Add(self.text_ctrl_4, 0, wx.LEFT, 146)
        sizer_2.Add(sizer_4, 1, wx.EXPAND, 0)
        grid_sizer_3.Add(self.button_1, 0, wx.ALIGN_CENTER, 0)
        grid_sizer_5.Add((0, 0), 0, 0, 0)
        grid_sizer_3.Add(grid_sizer_5, 1, wx.EXPAND, 0)
        grid_sizer_3.Add((0, 0), 0, 0, 0)
        sizer_2.Add(grid_sizer_3, 1, wx.EXPAND, 0)
        sizer_2.Add((0, 0), 0, 0, 0)
        grid_sizer_7.Add((0, 0), 0, 0, 0)
        grid_sizer_10.Add((0, 0), 0, 0, 0)
        grid_sizer_10.Add(self.panel_3, 1, wx.EXPAND, 0)
        grid_sizer_7.Add(grid_sizer_10, 1, wx.EXPAND, 0)
        grid_sizer_6.Add(grid_sizer_7, 1, wx.EXPAND, 0)
        grid_sizer_12.Add((0, 0), 0, 0, 0)
        grid_sizer_12.Add((0, 0), 0, 0, 0)
        grid_sizer_12.Add(self.panel_2, 1, wx.EXPAND, 0)
        grid_sizer_8.Add(grid_sizer_12, 1, wx.EXPAND, 0)
        bitmap_1 = wx.StaticBitmap(self, wx.ID_ANY, wx.Bitmap("/home/mpc/catkin_ws/src/genesis_control/GUI/blackcar.png", wx.BITMAP_TYPE_ANY))
        grid_sizer_8.Add(bitmap_1, 0, wx.ALIGN_CENTER, 0)
        sizer_1.Add(grid_sizer_8, 1, wx.EXPAND, 0)
        grid_sizer_6.Add(sizer_1, 1, wx.EXPAND, 0)
        grid_sizer_9.Add((0, 0), 0, 0, 0)
        grid_sizer_11.Add(self.panel_4, 1, wx.EXPAND, 0)
        grid_sizer_11.Add((0, 0), 0, 0, 0)
        grid_sizer_9.Add(grid_sizer_11, 1, wx.EXPAND, 0)
        grid_sizer_6.Add(grid_sizer_9, 1, wx.EXPAND, 0)
        grid_sizer_6.Add((0, 0), 0, 0, 0)
        sizer_2.Add(grid_sizer_6, 1, wx.EXPAND, 0)
        self.SetSizer(sizer_2)
        self.Layout()
        # end wxGlade

# end of class MyFrame

class MyApp(wx.App):
    def OnInit(self):
        self.frame = MyFrame(None, wx.ID_ANY, "")
        self.SetTopWindow(self.frame)
        self.frame.Show()
        return True

# end of class MyApp

def rosloop():
    rate = rospy.Rate(10) #10hz
    while not rospy.is_shutdown():
	rate.sleep()
def callback(msg):  #upper right textbox regarding controller
    global int_3
    int_3 = msg.data
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.data)
def callback0(msg2): #lower textbox regarding car
<<<<<<< HEAD
    global int_3
    int_3 = msg2.data
=======
    global int_2 
    int_2 = msg2.data
>>>>>>> 9af6dfae9a0e61e4c895aee6d7f2371cd71b069c
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg2.data)
def callback1(pnl1): #controller status
    global cont_stat
    cont_stat = pnl1.data
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", pnl1.data)
def callback2(pnl2): #radar detection
    global front_side
    front_side = pnl2.data
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", pnl2.data)
def callback3(pnl3): #left side lidar detection
    global left_side
    left_side = pnl3.data
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", pnl3.data)
def callback4(pnl4): #right side lidar detection
    global right_side
    right_side = pnl4.data
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", pnl4.data)
def callback5(msg1): #Top Textbox for Warnings
    global int_1
    int_1 = msg1.data
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg1.data)





if __name__ == "__main__":
    rospy.init_node('button', anonymous = True)
    rospy.Subscriber("/vehicle/v_acc", Float32MultiArray, callback)
    rospy.Subscriber("msgvalue2", UInt8, callback0)
    rospy.Subscriber("pnlvalue1", UInt8, callback1)
    rospy.Subscriber("pnlvalue2", UInt8, callback2)
    rospy.Subscriber("pnlvalue3", UInt8, callback3)
    rospy.Subscriber("pnlvalue4", UInt8, callback4)
    rospy.Subscriber("/vehicle/takeover", Bool, callback5)

    threading.Thread(target = rosloop)
    app = MyApp(0)
    app.MainLoop()
