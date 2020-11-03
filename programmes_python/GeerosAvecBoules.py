#!/usr/bin/python
# -*- coding: utf-8 -*-

##################################################################################
# Programme de pilotage du robot Geeros (avec boules stabilisatrices),
# disponible à l'adresse:
# http://boutique.3sigma.fr/12-robots
#
# Auteur: 3Sigma
# Version 3.1 - 20/02/2018
##################################################################################

# Import WiringPi2
import wiringpi2

# Imports pour le bus i2c
import smbus

import time, sched
import os
import threading
import signal
import json
import sys

# Pour la détection d'adresse IP
import socket
import fcntl
import struct

# Pour le serveur de Websocket
import tornado.httpserver
import tornado.ioloop
from tornado.ioloop import PeriodicCallback
import tornado.web
import tornado.websocket
import tornado.template

# Imports pour la communication i2c avec la carte Pololu A-Star
from a_star import AStar
a_star = AStar()

# Entete declarative
Nmoy = 10
codeurDroitDeltaPos = 0
codeurDroitDeltaPosPrec = 0
codeurGaucheDeltaPos = 0
codeurGaucheDeltaPosPrec = 0
omegaDroit = 0
omegaGauche = 0



# Les moteurs sont asservis en vitesse grâce à un régulateur de type PID
# On déclare ci-dessous les variables et paramètres nécessaires à l'asservissement et au régulateur
R = 0.045 # Rayon d'une roue
W = 0.14 # Largeur du robot
umax = 6. # valeur max de la tension de commande du moteur
umin = -6. # valeur min (ou max en négatif) de la tension de commande du moteur
vxmes = 0. # vitesse longitudinale mesurée
ximes = 0. # vitesse de rotation mesurée
Tf = 0.02 # constante de temps de filtrage de l'action dérivée du PID
Kpvx = 1. # gain proportionnel pour l'asservissement de vitesse longitudinale
Kivx = 10. # gain intégral pour l'asservissement de vitesse longitudinale
Kdvx = 0.00 # gain dérivé pour l'asservissement de vitesse longitudinale
Kpxi = 0.1 # gain proportionnel pour l'asservissement de rotation
Kixi = 1. # gain intégral pour l'asservissement de rotation
Kdxi = 0.000 # gain dérivé pour l'asservissement de rotation
commande_avant_sat_vx = 0. # commande avant la saturation pour l'asservissement de vitesse longitudinale
commande_vx = 0. # commande pour l'asservissement de vitesse longitudinale
commande_avant_sat_xi = 0. # commande avant la saturation pour l'asservissement de rotation
commande_xi = 0. # commande pour l'asservissement de rotation
P_vx = 0. # action proportionnelle pour l'asservissement de vitesse longitudinale
I_vx = 0. # action intégrale pour l'asservissement de vitesse longitudinale
D_vx = 0. # action dérivée pour l'asservissement de vitesse longitudinale
P_xi = 0. # action proportionnelle pour l'asservissement de rotation
I_xi = 0. # action intégrale pour l'asservissement de rotation
D_xi = 0. # action dérivée pour l'asservissement de rotation
commandeDroit = 0. # commande en tension calculée par le PID pour le moteur droit
commandeGauche = 0. # commande en tension calculée par le PID pour le moteur gauche
yprecvx = 0. # Mesure de la vitesse longitudinale au calcul précédent
yprecxi = 0. # Mesure de la vitesse de rotation au calcul précédent

# Variables intermédiaires
Ti = 0.
ad = 0.
bd = 0.

# Variables utilisées pour les données reçues
x1 = 0.
x2 = 0.
Kp2 = 1.
Ki2 = 1.
Kd2 = 1.
Kpxi2 = 1.
Kixi2 = 1.
Kdxi2 = 1.

# Déclarations pour les consignes de mouvement
vxref = 0.
xiref = 0.


# Time out de réception des données
timeout = 2
timeLastReceived = time.time()

T0 = time.time()
dt = 0.01
tprec = time.time()
i = 0
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
    
    # Initialisation de la position du servo
    a_star.servo(45)
    
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


def CalculVitesse():
    global started, \
        omegaDroit, omegaGauche, codeurDroitDeltaPosPrec, codeurGaucheDeltaPosPrec, \
        ad, P_vx, I_vx, D_vx, P_xi, I_xi, D_xi, bd, Ti, yprecvx, yprecxi, timeLastReceived, timeout, \
        codeurDroitDeltaPos, codeurGaucheDeltaPos, commandeDroit, commandeGauche, vxmes, ximes, vxref, xiref, dt2, tprec, \
        idecimLectureTension, decimLectureTension, tensionAlim, x1, x2
        
    
    # Mesure de la vitesse des moteurs grâce aux codeurs incrémentaux
    try:
        codeurDroitDeltaPos = a_star.read_codeurDroitDeltaPos()
        if abs(codeurDroitDeltaPos) > 1000:
            #print "Values out of range"
            codeurDroitDeltaPos = codeurDroitDeltaPosPrec
        else:
            codeurDroitDeltaPosPrec = codeurDroitDeltaPos
    except:
        #print "Erreur lecture codeur droit"
        codeurDroitDeltaPos = codeurDroitDeltaPosPrec

    try:
        codeurGaucheDeltaPos = a_star.read_codeurGaucheDeltaPos()
        if abs(codeurGaucheDeltaPos) > 1000:
            #print "Values out of range"
            codeurGaucheDeltaPos = codeurGaucheDeltaPosPrec
        else:
            codeurGaucheDeltaPosPrec = codeurGaucheDeltaPos
    except:
        #print "Erreur lecture codeur gauche"
        codeurGaucheDeltaPos = codeurGaucheDeltaPosPrec
    
    # C'est bien dt qu'on utilise ici et non pas dt2 (voir plus loin l'explication de dt2)
    # car codeurDroitDeltaPos et codeurGaucheDeltaPos sont mesurés en temps-réel par l'A*
    omegaDroit = -2 * ((2 * 3.141592 * codeurDroitDeltaPos) / 1632) / (Nmoy * dt)  # en rad/s
    omegaGauche = 2 * ((2 * 3.141592 * codeurGaucheDeltaPos) / 1632) / (Nmoy * dt)  # en rad/s

    # Si on n'a pas reçu de données depuis un certain temps, celles-ci sont annulées
    if (time.time()-timeLastReceived) > timeout:
        x1 = 0.
        x2 = 0.

    # Application de la consigne lue
    vxref = x1
    xiref = x2

    # Définition des entrées de la fonction d'asservissement
    vxmes = (omegaDroit + omegaGauche)*R/2
    ximes = -(omegaDroit - omegaGauche)*R/W

    # La suite des calculs se fait avec dt2, qui correspond au "vrai" pas de temps d'échantillonnage
    # de cette fonction (la RPi n'est pas un système temps-réel et au début de l'exécution du programme,
    # dt2 peut être jusqu'à deux fois plus petit que dt)
    dt2 = time.time() - tprec
    tprec = time.time()

    # Calcul du PID sur vx
    # Paramètres intermédiaires
    Ti = Ki2 * Kivx/(Kp2 * Kpvx + 0.01)
    ad = Tf/(Tf+dt2)
    bd = Kd2 * Kdvx/(Tf+dt2)
    
    # Terme proportionnel
    P_vx = Kpvx * Kp2 * (vxref - vxmes)

    # Terme dérivé
    D_vx = ad * D_vx - bd * (vxmes - yprecvx)
    
    # Calcul de la commande
    commande_vx = P_vx + I_vx


    # Terme intégral (sera utilisé lors du pas d'échantillonnage suivant)
    I_vx = I_vx + Kivx * Ki2 * dt2 * (vxref - vxmes)

    # Stockage de la mesure courante pour utilisation lors du pas d'échantillonnage suivant
    yprecvx = vxmes
    
    # Fin Calcul du PID sur vx

    # Calcul du PID sur xi
    # Paramètres intermédiaires
    Ti = Kixi2 * Kixi/(Kpxi2 * Kpxi + 0.01)
    ad = Tf/(Tf+dt2)
    bd = Kdxi2 * Kdxi/(Tf+dt2)
    
    # Terme proportionnel
    P_xi = Kpxi * Kpxi2 * (xiref - ximes)

    # Terme dérivé
    D_xi = ad * D_xi - bd * (ximes - yprecxi)
    
    # Calcul de la commande
    commande_xi = P_xi + I_xi + D_xi


    # Terme intégral (sera utilisé lors du pas d'échantillonnage suivant)
    I_xi = I_xi + Kixi * Kixi2 * dt2 * (xiref - ximes)

    # Stockage de la mesure courante pour utilisation lors du pas d'échantillonnage suivant
    yprecxi = ximes
    
    # Fin Calcul du PID sur xi


    # Calcul des commandes des moteurs
    commandeDroit = (commande_vx - commande_xi);
    commandeGauche = (commande_vx + commande_xi);
      
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
        print "Erreur moteurs"


            
def emitData():
    global T0
    # Délai nécessaire pour que le serveur de Websocket ait le temps de démarrer
    wiringpi2.delay(5000)
    T0 = time.time()
    while True: loop() # appelle fonction loop sans fin

    
class WSHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        global socketOK
        print 'connection opened...'
        socketOK = True
        self.callback = PeriodicCallback(self.sendToSocket, 100)
        self.callback.start()
    
    def on_message(self, message):
        global x1, x2, Kp2, Ki2, Kd2, Kpxi2, Kixi2, Kdxi2, timeLastReceived, socketOK

        jsonMessage = json.loads(message)
        
        # Annulation du timeout de réception des données
        timeLastReceived = time.time()
      
        if jsonMessage.get('vref') != None:
            x1 = float(jsonMessage.get('vref')) / 100
            #print ("x1: %.2f" % x1)
        if jsonMessage.get('xiref') != None:
            x2 = (float(jsonMessage.get('xiref'))) * 3.141592 / 180
            #print ("x2: %.2f" % x2)
        if jsonMessage.get('servoref') != None:
            servoref = int(jsonMessage.get('servoref'))
            try:
                a_star.servo(servoref)
            except:
                pass
            #print ("servoref: %d" % servoref)
        if jsonMessage.get('Kp2ref') != None:
            Kp2 = float(jsonMessage.get('Kp2ref'))
            #print ("Kp2: %.2f" % Kp2)
        if jsonMessage.get('Ki2ref') != None:
            Ki2 = float(jsonMessage.get('Ki2ref'))
            #print ("Ki2: %.2f" % Ki2)
        if jsonMessage.get('Kd2ref') != None:
            Kd2 = float(jsonMessage.get('Kd2ref'))
            #print ("Kd2: %.2f" % Kd2)
        if jsonMessage.get('Kpxi2ref') != None:
            Kpxi2 = float(jsonMessage.get('Kpxi2ref'))
            #print ("Kpxi2: %.2f" % Kpxi2)
        if jsonMessage.get('Kixi2ref') != None:
            Kixi2 = float(jsonMessage.get('Kixi2ref'))
            #print ("Kixi2: %.2f" % Kixi2)
        if jsonMessage.get('Kdxi2ref') != None:
            Kdxi2 = float(jsonMessage.get('Kdxi2ref'))
            #print ("Kdxi2: %.2f" % Kdxi2)
        
        if not socketOK:
            x1 = 0
            x2 = 0.
  

    def on_close(self):
        global socketOK, commandeDroit, commandeGauche
        print 'connection closed...'
        socketOK = False
        commandeDroit = 0.
        commandeGauche = 0.

    def sendToSocket(self):
        global started, codeurDroitDeltaPos, codeurGaucheDeltaPos, socketOK, commandeDroit, commandeGauche, vxref, xiref, \
                vxmes, ximes, T0

        tcourant = time.time() - T0
        aEnvoyer = json.dumps( {'Temps':("%.2f" % tcourant),
                                'Consigne vitesse longitudinale':("%.2f" % x1),
                                'Consigne vitesse de rotation':("%.2f" % (180 * x2/3.141592)),
                                'Vitesse longitudinale':("%.2f" % vxmes),
                                'Vitesse de rotation':("%.2f" % (180 * ximes/3.141592)),
                                'omegaDroit':("%.2f" % omegaDroit),
                                'omegaGauche':("%.2f" % omegaGauche),
                                'commandeDroit':("%.2f" % commandeDroit),
                                'commandeGauche':("%.2f" % commandeGauche)})
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
    global commandeDroit, commandeGauche
    print 'You pressed Ctrl+C!'
    commandeDroit = 0.
    commandeGauche = 0.
    CommandeMoteurs(0, 0, 5)
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# Gestion des segmentation fault
def signal_handler2(signal, frame):
    print 'Received signal ' + str(sig) + ' on line ' + str(frame.f_lineno) + ' in ' + frame.f_code.co_filename

signal.signal(signal.SIGSEGV, signal_handler2)

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



