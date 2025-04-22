#!/usr/bin/env python3
#coding=utf-8

import time
from Rosmaster_Lib import Rosmaster
from ipywidgets import interact
import ipywidgets as widgets

bot = Rosmaster()

def run_motor(M1, M2, M3, M4):
    bot.set_motor(M1, M2, M3, M4)
    print(f"Motor speeds: M1={M1}, M2={M2}, M3={M3}, M4={M4}")
    return M1, M2, M3, M4

interact(run_motor, 
                    M1=widgets.IntSlider(min=-100, max=100, step=1, value=0),  
                    M2=widgets.IntSlider(min=-100, max=100, step=1, value=0), 
                    M3=widgets.IntSlider(min=-100, max=100, step=1, value=0), 
                    M4=widgets.IntSlider(min=-100, max=100, step=1, value=0))

bot.set_motor(0, 0, 0, 0)

del bot