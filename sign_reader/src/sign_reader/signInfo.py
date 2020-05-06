#!/usr/bin/env python
from enum import Enum

class SignNames(Enum):
    STOP = 20
    GO = 74
    LEFT = 7
    RIGHT = 6
    SLOW = 96
    FAST = 2
    NONE = -1

signDict = { 
    20 : 'STOP',    # Stop sign
    74 : 'GO',      # Traffic light sign
    7  : 'LEFT',    # One way pointing left
    6  : 'RIGHT',   # One way pointing right
    96 : 'SLOW',    # Duck crossing
    2  : 'FAST',    # Yield ~ welcome to Massachusetts
    -1 : 'NONE'
}

signNames = signDict.values()

