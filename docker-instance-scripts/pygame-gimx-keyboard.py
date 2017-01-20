import pygame, sys, time
from pygame.locals import *

import socket
import time

#PS3 DS3 Controller Maps - axis indexes
#{0, 255} means a value of 0 or 255
#[-128, 127] means a value in the -128 to 127 range 
LEFT_STICK_X =0 #left stick x [-128, 127] 
LEFT_STICK_Y =1 #left stick y [-128, 127] 
RIGHT_STICK_X =2 #right stick x [-128, 127] 
RIGHT_STICK_Y =3 #right stick y [-128, 127] 
ACC_X =4 #acc x [-512, 511] 
ACC_Y =5 #acc y [-512, 511] 	
ACC_Z =6 #acc z [-512, 511]
GYRO =7 #gyro [-512, 511] 
SELECT =8 #select {0, 255}
START =9 #start {0, 255} 
PS =10 #ps {0, 255}
UP =11 #up [0, 255] 
RIGHT =12 #right [0, 255] 
DOWN =13 #down [0, 255] 
LEFT =14 #left [0, 255]
TRIANGLE =15 #triangle [0, 255] 
CIRCLE =16 #circle [0, 255] 
CROSS =17 #cross [0, 255]
SQUARE =18 #square [0, 255]
L1 =19 #l1 [0, 255] 
R1 =20 #e1 [0, 255] 
L2 =21 #l2 [0, 255] 
R2 =22 #r2 [0, 255] 
L3 =23 #l3 [0, 255] 
R3 =24 #r3 [0, 255] 
    

'''
KeyASCII      ASCII   Common Name
K_BACKSPACE   \b      backspace
K_TAB         \t      tab
K_CLEAR               clear
K_RETURN      \r      return
K_PAUSE               pause
K_ESCAPE      ^[      escape
K_SPACE               space
K_EXCLAIM     !       exclaim
K_QUOTEDBL    "       quotedbl
K_HASH        #       hash
K_DOLLAR      $       dollar
K_AMPERSAND   &       ampersand
K_QUOTE               quote
K_LEFTPAREN   (       left parenthesis
K_RIGHTPAREN  )       right parenthesis
K_ASTERISK    *       asterisk
K_PLUS        +       plus sign
K_COMMA       ,       comma
K_MINUS       -       minus sign
K_PERIOD      .       period
K_SLASH       /       forward slash
K_0           0       0
K_1           1       1
K_2           2       2
K_3           3       3
K_4           4       4
K_5           5       5
K_6           6       6
K_7           7       7
K_8           8       8
K_9           9       9
K_COLON       :       colon
K_SEMICOLON   ;       semicolon
K_LESS        <       less-than sign
K_EQUALS      =       equals sign
K_GREATER     >       greater-than sign
K_QUESTION    ?       question mark
K_AT          @       at
K_LEFTBRACKET [       left bracket
K_BACKSLASH   \       backslash
K_RIGHTBRACKET ]      right bracket
K_CARET       ^       caret
K_UNDERSCORE  _       underscore
K_BACKQUOTE   `       grave
K_a           a       a
K_b           b       b
K_c           c       c
K_d           d       d
K_e           e       e
K_f           f       f
K_g           g       g
K_h           h       h
K_i           i       i
K_j           j       j
K_k           k       k
K_l           l       l
K_m           m       m
K_n           n       n
K_o           o       o
K_p           p       p
K_q           q       q
K_r           r       r
K_s           s       s
K_t           t       t
K_u           u       u
K_v           v       v
K_w           w       w
K_x           x       x
K_y           y       y
K_z           z       z
K_DELETE              delete
K_KP0                 keypad 0
K_KP1                 keypad 1
K_KP2                 keypad 2
K_KP3                 keypad 3
K_KP4                 keypad 4
K_KP5                 keypad 5
K_KP6                 keypad 6
K_KP7                 keypad 7
K_KP8                 keypad 8
K_KP9                 keypad 9
K_KP_PERIOD   .       keypad period
K_KP_DIVIDE   /       keypad divide
K_KP_MULTIPLY *       keypad multiply
K_KP_MINUS    -       keypad minus
K_KP_PLUS     +       keypad plus
K_KP_ENTER    \r      keypad enter
K_KP_EQUALS   =       keypad equals
K_UP                  up arrow
K_DOWN                down arrow
K_RIGHT               right arrow
K_LEFT                left arrow
K_INSERT              insert
K_HOME                home
K_END                 end
K_PAGEUP              page up
K_PAGEDOWN            page down
K_F1                  F1
K_F2                  F2
K_F3                  F3
K_F4                  F4
K_F5                  F5
K_F6                  F6
K_F7                  F7
K_F8                  F8
K_F9                  F9
K_F10                 F10
K_F11                 F11
K_F12                 F12
K_F13                 F13
K_F14                 F14
K_F15                 F15
K_NUMLOCK             numlock
K_CAPSLOCK            capslock
K_SCROLLOCK           scrollock
K_RSHIFT              right shift
K_LSHIFT              left shift
K_RCTRL               right ctrl
K_LCTRL               left ctrl
K_RALT                right alt
K_LALT                left alt
K_RMETA               right meta
K_LMETA               left meta
K_LSUPER              left windows key
K_RSUPER              right windows key
K_MODE                mode shift
K_HELP                help
K_PRINT               print screen
K_SYSREQ              sysrq
K_BREAK               break
K_MENU                menu
K_POWER               power
K_EURO                euro

The keyboard also has a list of modifier states that can be assembled bit bitwise ORing them together.

KMOD_NONE, KMOD_LSHIFT, KMOD_RSHIFT, KMOD_SHIFT, KMOD_CAPS,
KMOD_LCTRL, KMOD_RCTRL, KMOD_CTRL, KMOD_LALT, KMOD_RALT,
KMOD_ALT, KMOD_LMETA, KMOD_RMETA, KMOD_META, KMOD_NUM, KMOD_MODE
'''



UDP_IP = "127.0.0.1"
UDP_PORT = 51914

print ("UDP target IP:", UDP_IP)
print ("UDP target port:", UDP_PORT)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

def sendRecv(message):
    global sock,UDP_IP,UDP_PORT
    sock.sendto(message, (UDP_IP, UDP_PORT))
    #while True:
    data, addr = sock.recvfrom(4192) # buffer size is 1024 bytes
    print ("received message:", data)
    return data

def send(message):
    global sock,UDP_IP,UDP_PORT
    sock.sendto(message, (UDP_IP, UDP_PORT))

def update(KEY,on,value):
    global MESSAGE
    if on:
        if MESSAGE[2+KEY*4] != value:
            MESSAGE[2+KEY*4] = value
            send(MESSAGE)
    else:
        if MESSAGE[2+KEY*4] != 0:
            MESSAGE[2+KEY*4] = 0
            send(MESSAGE)

pygame.init()

FPS=30
fpsClock=pygame.time.Clock()

width=400
height=300
DISPLAYSURF=pygame.display.set_mode((width,height),0,32)
pygame.display.set_caption('Navigation')

#basic message, all axis off
MESSAGE = bytearray(157)
MESSAGE[0]=0xff
MESSAGE[1]=0x9c

#key=None

def sendEvent(key, state):
    if key == K_UP:
        print("up")
        update(UP,state,255)
    elif key == K_DOWN:
        print("down")
        update(DOWN,state,255)
    elif key == K_LEFT:
        print("left")
        update(LEFT,state,255)
    elif key == K_RIGHT:
        print("right")
        update(RIGHT,state,255)
    elif key == K_LCTRL:
        print("K_LCTRL")
        update(TRIANGLE,state,255)
    elif key == K_z:
        print("K_LCTRL")
        update(SQUARE,state,255)
    elif key == K_LALT:
        print("K_LALT")
        update(CROSS,state,255)
    elif key == K_ESCAPE:
        print("K_ESCAPE")
        update(PS,state,255)
    elif key == K_b:
        print("K_b")
        update(CIRCLE,state,255)

while True:
    for event in pygame.event.get():
        if event.type==QUIT:
            pygame.quit()
            sys.exit()

        #print("type: ",event.type)
        if event.type == KEYDOWN:
            #print("KEYDOWN: ",event.key)
            sendEvent(event.key, True)
        if event.type == KEYUP:
            #print("KEYUP: ",event.key)
            sendEvent(event.key, False)

    pygame.display.update()
    fpsClock.tick(FPS)