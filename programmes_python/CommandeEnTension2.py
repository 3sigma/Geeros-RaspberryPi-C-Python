#!/usr/bin/python
# -*- coding: utf-8 -*-

##################################################################################
# Programme de commande en tension des moteurs du robot Geeros Raspberry C / Python,
# disponible à l'adresse:
# http://boutique.3sigma.fr/12-robots
#
# Auteur: 3Sigma
# Version 3.0 - 01/11/2017
##################################################################################

# Import WiringPi2
import wiringpi2

import time, sched
import os
import threading
import signal
import json
import sys
import math

# Pour la détection d'adresse IP
import socket
import fcntl
import struct

# Pour le serveur de socket
import tornado.httpserver
import tornado.ioloop
from tornado.ioloop import PeriodicCallback
import tornado.web
import tornado.websocket
import tornado.template

# Imports pour la communication i2c avec la carte Pololu A-Star
from a_star import AStar
a_star = AStar()

Nmoy = 10

codeurDroitDeltaPos = 0
codeurDroitDeltaPosPrec = 0
codeurGaucheDeltaPos = 0
codeurGaucheDeltaPosPrec = 0

# Variables pour la commande en tension du moteur
vref = 0. # consigne de vitesse
vrefDroit = 0. # consigne vitesse de rotation du moteur droit
vrefGauche = 0. # consigne vitesse de rotation du moteur gauche
omegaDroit = 0. # vitesse de rotation du moteur droit
omegaGauche = 0. # vitesse de rotation du moteur gauche
commandeDroit = 0. # commande en tension pour le moteur droit
commandeGauche = 0. # commande en tension pour le moteur gauche
commande_avant_sat_Droit = 0. # valeur de la commande avant la saturation (voir ci-dessous) pour le moteur droit
commande_avant_sat_Gauche = 0. # valeur de la commande avant la saturation (voir ci-dessous) pour le moteur gauche
umax = 6. # valeur max de la tension de commande du moteur
umin = -6. # valeur min (ou max en négatif) de la tension de commande du moteur

# Déclaration des variables pour la réception des données
typeSignal = 0
offset = 0.
amplitude = 0.
frequence = 0.
moteurint = 0 # Moteur droit par défaut

# Timeout de réception des données
timeout = 2
timeLastReceived = time.time()

# Cadencement
T0 = time.time()
dt = 0.01
i = 0
tdebut = 0
# Création d'un scheduler pour exécuter des opérations à cadence fixe
s = sched.scheduler(time.time, time.sleep)

# Lecture de la tension d'alimentation
idecimLectureTension = 0
decimLectureTension = 6000
tensionAlim = 7.4
# Sécurité sur la tension d'alimentation
tensionAlimMin = 6.4;

#--- setup --- 
def setup():
    global tensionAlim
    
    wiringpi2.wiringPiSetupGpio() # For GPIO pin numbering
    
    CommandeMoteurs(0, 0, tensionAlim)

    # Mesure de la tension d'alimentation
    try:
        tensionAlimBrute = a_star.read_battery_millivolts()
        tensionAlimAvantMax = tensionAlimBrute / 1000.;
        tensionAlim = max(tensionAlimMin, tensionAlimAvantMax);
        print "Tension d'alimentation", tensionAlim
    except:
        print "Probleme lecture tension d'alimentation"
        pass
        
    
# -- fin setup -- 
 
# -- loop -- 
def loop():
    global i, T0
    i = i+1
    s.enterabs( T0 + (i * dt), 1, CalculVitesse, ())
    s.run()
# -- fin loop --

def u2s16(x):
    return x - 65536 * (x >= 32768)

def CalculVitesse():
    global started, omegaDroit, omegaGauche, timeLastReceived, timeout, \
        tdebut, ad, P_x_Droit, I_x_Droit, D_x_Droit, P_x_Gauche, I_x_Gauche, D_x_Gauche, bd, Td, Tt, \
        codeurDroitDeltaPos, codeurGaucheDeltaPos, commandeDroit, commandeGauche, vrefDroit, vrefGauche, \
        codeurGaucheDeltaPosPrec, codeurDroitDeltaPosPrec, \
        typeSignal, offset, amplitude, frequence, moteurint, \
        idecimLectureTension, decimLectureTension, tensionAlim
    
    debut = time.time()
    # Mesure de la vitesse des moteurs grâce aux codeurs incrémentaux
    try:
        codeurDroitDeltaPos = a_star.read_codeurDroitDeltaPos()
        codeurDroitDeltaPosPrec = codeurDroitDeltaPos
    except:
        #print "Erreur lecture codeur droit"
        codeurDroitDeltaPos = codeurDroitDeltaPosPrec
    
    try:
        codeurGaucheDeltaPos = a_star.read_codeurGaucheDeltaPos()
        codeurGaucheDeltaPosPrec = codeurGaucheDeltaPos
    except:
        #print "Erreur lecture codeur gauche"
        codeurGaucheDeltaPos = codeurGaucheDeltaPosPrec
    
    omegaDroit = -2 * ((2 * 3.141592 * codeurDroitDeltaPos) / 1632) / (Nmoy * dt)  # en rad/s
    omegaGauche = 2 * ((2 * 3.141592 * codeurGaucheDeltaPos) / 1632) / (Nmoy * dt)  # en rad/s
    
    tcourant = time.time() - T0
    # Calcul de la consigne en fonction des données reçues
    if typeSignal == 0: # signal carré
        if frequence > 0:
            if (tcourant - (float(int(tcourant*frequence)))/frequence < 1/(2*frequence)):
                vref = offset + amplitude
            else:
                vref = offset
        else:
            vref = offset + amplitude
    else: # sinus
        if frequence > 0:
            vref = offset + amplitude * sin(2*3.141592*frequence*tcourant)
        else:
            vref = offset + amplitude

    # Application de la consigne sur chaque moteur
    if moteurint == 0:
        vrefDroit = vref
        vrefGauche = 0.
    elif moteurint == 1:
        vrefDroit = 0.
        vrefGauche = vref
    elif moteurint == 2:
        vrefDroit = vref
        vrefGauche = vref
    else:
        vrefDroit = 0.
        vrefGauche = 0.
            
    # Si on n'a pas reçu de données depuis un certain temps, celles-ci sont annulées
    if (time.time()-timeLastReceived) > timeout:
        vrefDroit = 0
        vrefGauche = 0
        
    # Calcul de la commande avant saturation
    commande_avant_sat_Droit = vrefDroit
    commande_avant_sat_Gauche = vrefGauche

    # Application de la saturation sur la commande
    if (commande_avant_sat_Droit > umax):
        commandeDroit = umax
    elif (commande_avant_sat_Droit < umin):
        commandeDroit = umin
    else:
        commandeDroit = commande_avant_sat_Droit
    
    if (commande_avant_sat_Gauche > umax) :
        commandeGauche = umax
    elif (commande_avant_sat_Gauche < umin):
        commandeGauche = umin
    else:
        commandeGauche = commande_avant_sat_Gauche


    CommandeMoteurs(commandeDroit, commandeGauche, tensionAlim)
    
    # Lecture de la tension d'alimentation
    if idecimLectureTension >= decimLectureTension:
        try:
            tensionAlimBrute = a_star.read_battery_millivolts()
            tensionAlimAvantMax = tensionAlimBrute / 1000.;
            tensionAlim = max(tensionAlimMin, tensionAlimAvantMax);
            idecimLectureTension = 0
        except:
            print "Probleme lecture tension d'alimentation"
            pass
    else:
        idecimLectureTension = idecimLectureTension + 1
    
    #print time.time() - debut
        
        
def CommandeMoteurs(commandeDroit, commandeGauche, tensionAlim):
    # Cette fonction calcule et envoi les signaux PWM au pont en H
    # en fonction des tensions de commande et d'alimentation

    # L'ensemble pont en H + moteur pourrait ne pas être linéaire
    tensionDroit = commandeDroit
    tensionGauche = commandeGauche

    # Normalisation de la tension d'alimentation par
    # rapport à la tension d'alimentation
    tension_int_droit = int(400 * tensionDroit / tensionAlim)
    tension_int_gauche = -int(400 * tensionGauche / tensionAlim)

    # Saturation par sécurité
    if (tension_int_droit > 400):
        tension_int_droit = 400

    if (tension_int_droit < -400):
        tension_int_droit = -400

    if (tension_int_gauche > 400):
        tension_int_gauche = 400

    if (tension_int_gauche < -400):
        tension_int_gauche = -400

    # Commande PWM
    try:
        a_star.motors(tension_int_gauche, tension_int_droit)    
    except:
        #print "Erreur moteurs"
        pass

    
def emitData():
    global T0
    # Délai nécessaire pour que le serveur ait le temps de démarrer
    wiringpi2.delay(5000)
    T0 = time.time()
    while True: loop() # appelle fonction loop sans fin

    
class WSHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        global socketOK
        print 'connection opened...'
        socketOK = True
        self.callback = PeriodicCallback(self.sendToSocket, 20)
        self.callback.start()
    

    def on_message(self, message):
        global vref, vrefDroit, vrefGauche, typeSignal, offset, amplitude, frequence, Kp, Ki, Kd, moteurint, timeLastReceived, socketOK
        
        jsonMessage = json.loads(message)
        
        # Annulation du timeout de réception des données
        timeLastReceived = time.time()
        
        if jsonMessage.get('typeSignal') != None:
            typeSignal = int(jsonMessage.get('typeSignal'))
        if jsonMessage.get('offset') != None:
            offset = float(jsonMessage.get('offset'))
        if jsonMessage.get('amplitude') != None:
            amplitude = float(jsonMessage.get('amplitude'))
        if jsonMessage.get('frequence') != None:
            frequence = float(jsonMessage.get('frequence'))
        if jsonMessage.get('moteurint') != None:
            moteurint = int(jsonMessage.get('moteurint'))
                    
        if not socketOK:
            typeSignal = 0
            offset = 0.
            amplitude = 0.
            frequence = 0.


    def on_close(self):
        global socketOK, vrefDroit, vrefGauche
        print 'connection closed...'
        socketOK = False
        vrefDroit = 0.
        vrefGauche = 0.

    def sendToSocket(self):
        global codeurDroitDeltaPos, codeurGaucheDeltaPos, socketOK, commandeDroit, commandeGauche, tensionAlim
        
        tcourant = time.time() - T0
        aEnvoyer = json.dumps({ 'Temps':("%.2f" % tcourant), \
                                'Consigne':("%.2f" % vref), \
                                'omegaDroit':("%.2f" % omegaDroit), \
                                'omegaGauche':("%.2f" % omegaGauche), \
                                'TensionAlim':("%.2f" % tensionAlim), \
                                'Raw':("%.2f" % tcourant) + "," + \
                                ("%.2f" % vref) + "," + \
                                ("%.2f" % omegaDroit) + "," + \
                                ("%.2f" % omegaGauche) + "," + \
                                ("%.2f" % tensionAlim)})
        if socketOK:
            try:
                self.write_message(aEnvoyer)
            except:
                pass
            
    def check_origin(self, origin):
        # Voir http://www.tornadoweb.org/en/stable/websocket.html#tornado.websocket.WebSocketHandler.check_origin
        # et http://www.arundhaj.com/blog/tornado-error-during-websocket-handshake.html
        return True        

    
def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])
    
application = tornado.web.Application([
    (r'/ws', WSHandler)
])

def startTornado():
    http_server = tornado.httpserver.HTTPServer(application)
    http_server.listen(9090)
    tornado.ioloop.IOLoop.instance().start()


# Gestion du CTRL-C
def signal_handler(signal, frame):
    global vrefDroit, vrefGauche
    print 'You pressed Ctrl+C!'
    vrefDroit = 0.
    vrefGauche = 0.
    CommandeMoteurs(0, 0, 5)
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

#--- obligatoire pour lancement du code -- 
if __name__=="__main__": # pour rendre le code executable 

    # Test pour savoir si le firmware est présent sur la carte A-Star
    firmwarePresent = False
    for i in range(1, 11):
        time.sleep(0.1)
        print "Test presence du firmware de la carte A-Star, tentative " + str(i) + " / 10"
        try:
            firmwarePresent = a_star.firmwareOK()
            if firmwarePresent:
                break
        except:
            print "Firmware absent"
            
    if firmwarePresent:
        print "Firmware present, on continue..."
        started = False
        startedDroit = False
        startedGauche = False
        setup() # appelle la fonction setup
        print "Setup done."
        
        th = threading.Thread(None, emitData, None, (), {})
        th.daemon = True
        th.start()
        
        print "Starting Tornado."
        try:
            print "Connect to ws://" + get_ip_address('eth0') + ":9090/ws with Ethernet."
        except:
            pass
            
        try:
            print "Connect to ws://" + get_ip_address('wlan0') + ":9090/ws with Wifi."
        except:
            pass
        socketOK = False
        startTornado()
    else:
        print "Firmware absent, on abandonne ce programme."
        print "Veuillez charger le firmware sur la carte A-Star pour exécuter ce programme."


