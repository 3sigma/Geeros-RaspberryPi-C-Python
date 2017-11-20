# -*- coding: utf-8 -*-

from websocket import create_connection
import math
import time

class geeros_api:

    def __init__(self):

        self.ws = create_connection("ws://192.168.0.199:9090/ws")
        

    def Tourner(self, vitesseRotation, duree = -1):
        tdebut = time.time()
        t = time.time() - tdebut
        while t < duree:
            psidotref = eval(str(vitesseRotation))
            # Saturations min et max
            psidotref = max(min(360, psidotref), -360)
            # Envoi de la consigne au programme principal par Websocket
            self.ws.send('{"psidotref": ' + str(psidotref) + '}')
            time.sleep(0.1)
            t = time.time() - tdebut
            
        if duree == -1:
            psidotref = eval(str(vitesseRotation))
            # Saturations min et max
            psidotref = max(min(360, psidotref), -360)
            # Envoi de la consigne au programme principal par Websocket
            self.ws.send('{"psidotref": ' + str(psidotref) + '}')
        else:
            self.ws.send('{"psidotref": 0}')
    
    
    def Avancer(self, vitesseLongitudinale, duree = -1):
        tdebut = time.time()
        t = time.time() - tdebut
        while t < duree:
            vref = eval(str(vitesseLongitudinale))
            # Saturations min et max
            vref = max(min(0.5, vref), -0.5)
            # Envoi de la consigne au programme principal par Websocket
            self.ws.send('{"vref": ' + str(vref) + '}')
            time.sleep(0.1)
            t = time.time() - tdebut
            
        if duree == -1:
            vref = eval(str(vitesseLongitudinale))
            # Saturations min et max
            vref = max(min(0.5, vref), -0.5)
            # Envoi de la consigne au programme principal par Websocket
            self.ws.send('{"vref": ' + str(vref) + '}')
        else:
            self.ws.send('{"vref": 0}')
    
    
     def Mouvement(self, vitesseLongitudinale, vitesseRotation, duree = -1):
        tdebut = time.time()
        t = time.time() - tdebut
        while t < duree:
            vref = eval(str(vitesseLongitudinale))
            psidotref = eval(str(vitesseRotation))
            # Saturations min et max
            vref = max(min(0.5, vref), -0.5)
            psidotref = max(min(360, psidotref), -360)
            # Envoi de la consigne au programme principal par Websocket
            self.ws.send('{"vref": ' + str(vref) + ', "psidotref": ' + str(psidotref) + '}')
            time.sleep(0.1)
            t = time.time() - tdebut
            
        if duree == -1:
            vref = eval(str(vitesseLongitudinale))
            psidotref = eval(str(vitesseRotation))
            # Saturations min et max
            vref = max(min(0.5, vref), -0.5)
            psidotref = max(min(360, psidotref), -360)
            # Envoi de la consigne au programme principal par Websocket
            self.ws.send('{"vref": ' + str(vref) + ', "psidotref": ' + str(psidotref) + '}')
        else:
            self.ws.send('{"vref": 0, "psidotref": 0}')
    
    
    def AngleServo(self, angle, duree = -1):
        tdebut = time.time()
        t = time.time() - tdebut
        while t < duree:
            servoref = eval(str(angle))
            # Saturations min et max
            servoref = max(min(30, servoref), -30)
            # Envoi de la consigne au programme principal par Websocket
            self.ws.send('{"servoref": ' + str(servoref) + '}')
            time.sleep(0.1)
            t = time.time() - tdebut
            
        if duree == -1:
            servoref = eval(str(angle))
            # Saturations min et max
            servoref = max(min(30, servoref), -30)
            # Envoi de la consigne au programme principal par Websocket
            self.ws.send('{"servoref": ' + str(servoref) + '}')
    
    
   
    def Terminer(self):
        self.ws.send('{"vref": 0, "psidotref": 0}')
        self.ws.close()
        
