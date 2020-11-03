# -*- coding: utf-8 -*-

from websocket import create_connection
import math
import time
import json
import sched
import threading

class geeros_api:

    def __init__(self):

        # Attente de démarrage du serveur de Websocket
        socketOK = False
        while not socketOK:
            try:
                self.ws = create_connection("ws://192.168.0.199:9090/ws")
                socketOK = True
            except:
                pass
            time.sleep(1)
        
        self.message = ""
        self.started = True
        
        # Scheduler
        self.T0 = time.time()
        self.i = 0
        self.dt = 0.01
        self.s = sched.scheduler(time.time, time.sleep)
        
        # Thread de lecture du websocket
        self._lectureWS() # Initialisation indispensable
        th = threading.Thread(None, self._loop, None, (), {})
        th.daemon = True
        th.start()
        

    def _loop(self):
        while self.started:
            self.i = self.i + 1
            self.s.enterabs(self.T0 + (self.i * self.dt), 1, self._lectureWS, ())
            self.s.run()
        
        
    def _lectureWS(self):
        self.message =  self.ws.recv()
        #print "Received '%s'" % self.message
        
        
    def Tourner(self, vitesseRotation, duree = -1):
        tdebut = time.time()
        t = time.time() - tdebut
        while t < duree:
            psidotref = eval(str(vitesseRotation))
            # Saturations min et max
            psidotref = max(min(360, psidotref), -360)
            # Envoi de la consigne au programme principal par Websocket
            self.ws.send('{"xiref": ' + str(psidotref) + '}')
            time.sleep(0.1)
            t = time.time() - tdebut
            
        if duree == -1:
            xiref = eval(str(vitesseRotation))
            # Saturations min et max
            xiref = max(min(360, xiref), -360)
            # Envoi de la consigne au programme principal par Websocket
            self.ws.send('{"xiref": ' + str(xiref) + '}')
        else:
            self.ws.send('{"xiref": 0}')
    
    
    def Avancer(self, vitesseLongitudinale, duree = -1):
        tdebut = time.time()
        t = time.time() - tdebut
        while t < duree:
            vref = eval(str(vitesseLongitudinale))
            # Saturations min et max
            vref = max(min(50, vref), -50)
            # Envoi de la consigne au programme principal par Websocket
            self.ws.send('{"vref": ' + str(vref) + '}')
            time.sleep(0.1)
            t = time.time() - tdebut
            
        if duree == -1:
            vref = eval(str(vitesseLongitudinale))
            # Saturations min et max
            vref = max(min(50, vref), -50)
            # Envoi de la consigne au programme principal par Websocket
            self.ws.send('{"vref": ' + str(vref) + '}')
        else:
            self.ws.send('{"vref": 0}')
    
    
    def Mouvement(self, vitesseLongitudinale, vitesseRotation, duree = -1):
        tdebut = time.time()
        t = time.time() - tdebut
        while t < duree:
            vref = eval(str(vitesseLongitudinale))
            xiref = eval(str(vitesseRotation))
            # Saturations min et max
            vref = max(min(50, vref), -50)
            xiref = max(min(360, xiref), -360)
            # Envoi de la consigne au programme principal par Websocket
            self.ws.send('{"vref": ' + str(vref) + ', "xiref": ' + str(xiref) + '}')
            time.sleep(0.1)
            t = time.time() - tdebut
            
        if duree == -1:
            vref = eval(str(vitesseLongitudinale))
            xiref = eval(str(vitesseRotation))
            # Saturations min et max
            vref = max(min(50, vref), -50)
            xiref = max(min(360, xiref), -360)
            # Envoi de la consigne au programme principal par Websocket
            self.ws.send('{"vref": ' + str(vref) + ', "xiref": ' + str(xiref) + '}')
        else:
            self.ws.send('{"vref": 0, "xiref": 0}')
    
    
    def AngleServo(self, angle, duree = -1):
        tdebut = time.time()
        t = time.time() - tdebut
        while t < duree:
            servoref = eval(str(angle))
            # Saturations min et max
            servoref = max(min(70, servoref), 20)
            # Envoi de la consigne au programme principal par Websocket
            self.ws.send('{"servoref": ' + str(servoref) + '}')
            time.sleep(0.1)
            t = time.time() - tdebut
            
        if duree == -1:
            servoref = eval(str(angle))
            # Saturations min et max
            servoref = max(min(70, servoref), 20)
            # Envoi de la consigne au programme principal par Websocket
            self.ws.send('{"servoref": ' + str(servoref) + '}')
    
    
   
    def LireVariable(self, variable):
        jsonMessage = json.loads(self.message)
        if jsonMessage.get(variable) != None:
            return float(jsonMessage.get(variable))
        else:
            return None
        
   
    def Terminer(self):
        self.ws.send('{"vref": 0, "xiref": 0}')
        self.ws.close()
        
