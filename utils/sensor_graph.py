import os
import pprint
import random
import sys
import wx

REFRESH_INTERVAL_MS = 90

# The recommended way to use wx with mpl is with the WXAgg
# backend.
#
import matplotlib
matplotlib.use('WXAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_wxagg import \
    FigureCanvasWxAgg as FigCanvas, \
    NavigationToolbar2WxAgg as NavigationToolbar
import numpy as np
import pylab
#Data comes from here
from Arduino_Monitor import SerialData as DataGen
import pprint

class FloatControlBox(wx.Panel):
  def __init__(self, parent, ID, label, initval, datagen):
    wx.Panel.__init__(self, parent, ID)

    self.value = initval
    self.label = label
    box = wx.StaticBox(self, -1, label)
    sizer = wx.StaticBoxSizer(box, wx.VERTICAL)
    self.datagen = datagen

    self.manual_text = wx.TextCtrl(self, -1,
        size=(70,-1),
        value=str(initval),
        style=wx.TE_PROCESS_ENTER)

    self.Bind(wx.EVT_UPDATE_UI, self.on_update_manual_text, self.manual_text)
    self.Bind(wx.EVT_TEXT_ENTER, self.on_text_enter, self.manual_text)

    manual_box = wx.BoxSizer(wx.HORIZONTAL)
    manual_box.Add(self.manual_text, flag=wx.ALIGN_CENTER_VERTICAL)

    sizer.Add(manual_box, 0, wx.ALL, 10)

    self.SetSizer(sizer)
    sizer.Fit(self)


  def on_update_manual_text(self, event):
    self.manual_text.Enable

  def on_text_enter(self, event):
    self.value = self.manual_text.GetValue()
    self.datagen.write(self.label + self.value)

  def manual_value(self):
    return self.value

class BoundControlBox(wx.Panel):
    """ A static box with a couple of radio buttons and a text
box. Allows to switch between an automatic mode and a
manual mode with an associated value.
"""
    def __init__(self, parent, ID, label, initval):
        wx.Panel.__init__(self, parent, ID)
        
        self.value = initval
        
        box = wx.StaticBox(self, -1, label)
        sizer = wx.StaticBoxSizer(box, wx.VERTICAL)
        
        self.radio_auto = wx.RadioButton(self, -1,
            label="Auto", style=wx.RB_GROUP)
        self.radio_manual = wx.RadioButton(self, -1,
            label="Manual")
        self.manual_text = wx.TextCtrl(self, -1,
            size=(35,-1),
            value=str(initval),
            style=wx.TE_PROCESS_ENTER)
        
        self.Bind(wx.EVT_UPDATE_UI, self.on_update_manual_text, self.manual_text)
        self.Bind(wx.EVT_TEXT_ENTER, self.on_text_enter, self.manual_text)
        
        manual_box = wx.BoxSizer(wx.HORIZONTAL)
        manual_box.Add(self.radio_manual, flag=wx.ALIGN_CENTER_VERTICAL)
        manual_box.Add(self.manual_text, flag=wx.ALIGN_CENTER_VERTICAL)
        
        sizer.Add(self.radio_auto, 0, wx.ALL, 10)
        sizer.Add(manual_box, 0, wx.ALL, 10)
        
        self.SetSizer(sizer)
        sizer.Fit(self)
    
    def on_update_manual_text(self, event):
        self.manual_text.Enable(self.radio_manual.GetValue())
    
    def on_text_enter(self, event):
        self.value = self.manual_text.GetValue()
    
    def is_auto(self):
        return self.radio_auto.GetValue()
        
    def manual_value(self):
        return self.value


class GraphFrame(wx.Frame):
    """ The main frame of the application
"""
    title = 'Demo: dynamic matplotlib graph'
    
    def __init__(self):
        wx.Frame.__init__(self, None, -1, self.title)
        
        self.datagen = DataGen()
        self.paused = False
        
        self.create_menu()
        self.create_status_bar()
        self.create_main_panel()
        
        self.redraw_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_redraw_timer, self.redraw_timer)
        self.redraw_timer.Start(REFRESH_INTERVAL_MS)

        self.data_names = [
            'acc_raw_x',
            'acc_raw_y',
            'acc_raw_z',
            'acc_x',
            'acc_y',
            'acc_z',
            'acc_gx',
            'acc_gy',
            'acc_gz',

            'gyro_raw_x',
            'gyro_raw_y',
            'gyro_raw_z',
            'gyro_smooth_x',
            'gyro_smooth_y',
            'gyro_smooth_z',
            'gyro_x',
            'gyro_y',
            'gyro_z',

            'dcm_q0',
            'dcm_q1',
            'dcm_q2',
            'dcm_q3',
            'dcm_x',
            'dcm_y',
            'dcm_z',
        ]

        self.data = {}

        for data_name in self.data_names:
            self.data[data_name] = [0.0]

    def create_menu(self):
        self.menubar = wx.MenuBar()
        
        menu_file = wx.Menu()
        m_expt = menu_file.Append(-1, "&Save plot\tCtrl-S", "Save plot to file")
        self.Bind(wx.EVT_MENU, self.on_save_plot, m_expt)
        menu_file.AppendSeparator()
        m_exit = menu_file.Append(-1, "E&xit\tCtrl-X", "Exit")
        self.Bind(wx.EVT_MENU, self.on_exit, m_exit)
                
        self.menubar.Append(menu_file, "&File")
        self.SetMenuBar(self.menubar)

    def create_main_panel(self):
        self.panel = wx.Panel(self)

        self.init_plots()
        self.canvas = FigCanvas(self.panel, -1, self.fig)

        self.kp = FloatControlBox(self.panel, -1, "p", 0.2, self.datagen)
        self.ki = FloatControlBox(self.panel, -1, "i", 0.1, self.datagen)

        self.pause_button = wx.Button(self.panel, -1, "Pause")
        self.Bind(wx.EVT_BUTTON, self.on_pause_button, self.pause_button)
        self.Bind(wx.EVT_UPDATE_UI, self.on_update_pause_button, self.pause_button)
        
        self.cb_grid = wx.CheckBox(self.panel, -1,
            "Show Grid",
            style=wx.ALIGN_RIGHT)
        self.Bind(wx.EVT_CHECKBOX, self.on_cb_grid, self.cb_grid)
        self.cb_grid.SetValue(True)
        
        self.cb_xlab = wx.CheckBox(self.panel, -1,
            "Show X labels",
            style=wx.ALIGN_RIGHT)
        self.Bind(wx.EVT_CHECKBOX, self.on_cb_xlab, self.cb_xlab)
        self.cb_xlab.SetValue(True)
        
        self.hbox1 = wx.BoxSizer(wx.HORIZONTAL)
        self.hbox1.Add(self.pause_button, border=5, flag=wx.ALL | wx.ALIGN_CENTER_VERTICAL)
        self.hbox1.AddSpacer(20)
        self.hbox1.Add(self.cb_grid, border=5, flag=wx.ALL | wx.ALIGN_CENTER_VERTICAL)
        self.hbox1.AddSpacer(10)
        self.hbox1.Add(self.cb_xlab, border=5, flag=wx.ALL | wx.ALIGN_CENTER_VERTICAL)
        
        self.hbox2 = wx.BoxSizer(wx.HORIZONTAL)
        self.hbox2.Add(self.kp, border=5, flag=wx.ALL)
        self.hbox2.Add(self.ki, border=5, flag=wx.ALL)
        
        self.vbox = wx.BoxSizer(wx.VERTICAL)
        self.vbox.Add(self.canvas, 1, flag=wx.LEFT | wx.TOP | wx.GROW)
        self.vbox.Add(self.hbox1, 0, flag=wx.ALIGN_LEFT | wx.TOP)
        self.vbox.Add(self.hbox2, 0, flag=wx.ALIGN_LEFT | wx.TOP)
        
        self.panel.SetSizer(self.vbox)
        self.vbox.Fit(self)
    
    def create_status_bar(self):
        self.statusbar = self.CreateStatusBar()

    def init_acc_plot(self):
        self.acc_ax = self.fig.add_subplot(311)
        self.acc_ax.set_title('Acceleration data')
        pylab.setp(self.acc_ax.get_xticklabels(), fontsize=8)
        pylab.setp(self.acc_ax.get_yticklabels(), fontsize=8)
        self.acc_data = self.acc_ax.plot(
            [0],[0], [0],[0], [0],[0]
            )

    def init_gyro_plot(self):
        self.gyro_ax = self.fig.add_subplot(311)
        self.gyro_ax.set_title('Gyroscope data')
        pylab.setp(self.gyro_ax.get_xticklabels(), fontsize=8)
        pylab.setp(self.gyro_ax.get_yticklabels(), fontsize=8)
        self.gyro_data = self.gyro_ax.plot(
            [0],[0], [0],[0], [0],[0]
            )

    def init_dcm_plot(self):
        self.dcm_ax = self.fig.add_subplot(311)
        self.dcm_ax.set_title('DCM filter data')
        pylab.setp(self.dcm_ax.get_xticklabels(), fontsize=8)
        pylab.setp(self.dcm_ax.get_yticklabels(), fontsize=8)
        self.dcm_data = self.dcm_ax.plot(
            [0],[0], [0],[0], [0],[0]
            )

    def plot_xmax(self, values):
        #if self.xmax_control.is_auto():
            #xmax = len(self.data['acc_x']) if len(self.data['acc_x']) > 50 else 50
        #else:
            #xmax = int(self.xmax_control.manual_value())

        if (len(values) > 50):
            return len(values)
        else:
            return 50

    def plot_xmin(self, xmax):
            
        #if self.xmin_control.is_auto():
            #xmin = xmax - 50
        #else:
            #xmin = int(self.xmin_control.manual_value())

        return xmax - 50

    def plot_ymax(self, values):
        #if self.ymax_control.is_auto():
            #ymax = round(max(self.data['acc_x']), 0) + 1
        #else:
            #ymax = int(self.ymax_control.manual_value())

        return (round(max(values), 0) + 1)
        
    def plot_ymin(self, values):
        #if self.ymin_control.is_auto():
            #ymin = round(min(self.data['acc_x']), 0) - 1
        #else:
            #ymin = int(self.ymin_control.manual_value())

        return (round(max(values), 0) + 1)

    def draw_acc_plot(self):
        xmax = self.plot_xmax(self.data['acc_raw_x'])
        xmin = self.plot_xmin(xmax)
        ymax = 400
        ymin = -400
        self.acc_ax.set_xbound(lower=xmin, upper=xmax)
        self.acc_ax.set_ybound(lower=ymin, upper=ymax)
        self.acc_ax.grid(True, color='gray')
        pylab.setp(self.acc_ax.get_xticklabels(),
            visible=True)
        self.acc_data[0].set_xdata(np.arange(len(self.data['acc_raw_x'])))
        self.acc_data[0].set_ydata(np.array(self.data['acc_raw_x']))
        self.acc_data[1].set_xdata(np.arange(len(self.data['acc_x'])))
        self.acc_data[1].set_ydata(np.array(self.data['acc_x']))
        #self.acc_data[2].set_xdata(np.arange(len(self.data['acc_raw_z'])))
        #self.acc_data[2].set_ydata(np.array(self.data['acc_raw_z']))

    def draw_gyro_plot(self):
        xmax = self.plot_xmax(self.data['gyro_x'])
        xmin = self.plot_xmin(xmax)
        ymax = 3
        ymin = -3
        self.gyro_ax.set_xbound(lower=xmin, upper=xmax)
        self.gyro_ax.set_ybound(lower=ymin, upper=ymax)
        self.gyro_ax.grid(True, color='gray')
        pylab.setp(self.gyro_ax.get_xticklabels(),
            visible=True)
        self.gyro_data[0].set_xdata(np.arange(len(self.data['gyro_x'])))
        self.gyro_data[0].set_ydata(np.array(self.data['gyro_x']))
        self.gyro_data[1].set_xdata(np.arange(len(self.data['gyro_y'])))
        self.gyro_data[1].set_ydata(np.array(self.data['gyro_y']))
        self.gyro_data[2].set_xdata(np.arange(len(self.data['gyro_z'])))
        self.gyro_data[2].set_ydata(np.array(self.data['gyro_z']))

    def draw_dcm_plot(self):
        xmax = self.plot_xmax(self.data['dcm_x'])
        xmin = self.plot_xmin(xmax)
        ymax = 5
        ymin = -5
        self.dcm_ax.set_xbound(lower=xmin, upper=xmax)
        self.dcm_ax.set_ybound(lower=ymin, upper=ymax)
        self.dcm_ax.grid(True, color='gray')
        pylab.setp(self.dcm_ax.get_xticklabels(),
            visible=True)
        self.dcm_data[0].set_xdata(np.arange(len(self.data['dcm_x'])))
        self.dcm_data[0].set_ydata(np.array(self.data['dcm_x']))
        #self.dcm_data[1].set_xdata(np.arange(len(self.data['dcm_y'])))
        #self.dcm_data[1].set_ydata(np.array(self.data['dcm_y']))
        #self.dcm_data[2].set_xdata(np.arange(len(self.data['dcm_z'])))
        #self.dcm_data[2].set_ydata(np.array(self.data['dcm_z']))

    def init_plots(self):
        self.dpi = 100
        self.fig = Figure(figsize=(6, 4), dpi=self.dpi)
        #self.init_acc_plot()
        #self.init_gyro_plot()
        self.init_dcm_plot()

    def draw_plots(self):
        """ Redraws the plot
"""
        #self.draw_acc_plot()
        #self.draw_gyro_plot()
        self.draw_dcm_plot()

        self.canvas.draw()
    
    def on_pause_button(self, event):
        self.paused = not self.paused
    
    def on_update_pause_button(self, event):
        label = "Resume" if self.paused else "Pause"
        self.pause_button.SetLabel(label)
    
    def on_cb_grid(self, event):
        self.draw_plots()
    
    def on_cb_xlab(self, event):
        self.draw_plots()
    
    def on_save_plot(self, event):
        file_choices = "PNG (*.png)|*.png"
        
        dlg = wx.FileDialog(
            self,
            message="Save plot as...",
            defaultDir=os.getcwd(),
            defaultFile="plot.png",
            wildcard=file_choices,
            style=wx.SAVE)
        
        if dlg.ShowModal() == wx.ID_OK:
            path = dlg.GetPath()
            self.canvas.print_figure(path, dpi=self.dpi)
            self.flash_status_message("Saved to %s" % path)
    
    def on_redraw_timer(self, event):
        # if paused do not add data, but still redraw the plot
        # (to respond to scale modifications, grid change, etc.)
        #
        if not self.paused:
            self.update_data(self.datagen.next())
        
        self.draw_plots()
    
    def on_exit(self, event):
        self.Destroy()
    
    def flash_status_message(self, msg, flash_len_ms=1500):
        self.statusbar.SetStatusText(msg)
        self.timeroff = wx.Timer(self)
        self.Bind(
            wx.EVT_TIMER,
            self.on_flash_status_off,
            self.timeroff)
        self.timeroff.Start(flash_len_ms, oneShot=True)
    
    def on_flash_status_off(self, event):
        self.statusbar.SetStatusText('')

    def update_data(self, data_dict):
        for key in data_dict.keys():
            self.data[key].append(data_dict[key])

if __name__ == '__main__':
    app = wx.PySimpleApp()
    app.frame = GraphFrame()
    app.frame.Show()
    app.MainLoop()
