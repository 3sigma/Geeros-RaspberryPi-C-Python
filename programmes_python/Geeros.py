#!/usr/bin/python
# -*- coding: utf-8 -*-

# Import WiringPi2
import wiringpi2

# Imports pour l'i2c
from a_star import AStar
a_star = AStar()

# Imports pour l'IMU sur bus i2c
import smbus
import FaBo9Axis_MPU9250

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

# Entete declarative
Nmoy = 10
omegaDroit = 0
codeurDroitDeltaPos = 0
omegaGauche = 0
codeurGaucheDeltaPos = 0

omega = 0.
thetames = 0.

tensionBatterie = 7.4

# Les moteurs sont asservis en vitesse grâce à un régulateur de type PID
# On déclare ci-dessous les variables et paramètres nécessaires à l'asservissement et au régulateur
R = 0.045 # Rayon d'une roue
W = 0.14 # Largeur du robot
umax = 6. # valeur max de la tension de commande du moteur
umin = -6. # valeur min (ou max en négatif) de la tension de commande du moteur
vxmes = 0. # vitesse longitudinale mesurée
xidotmes = 0. # vitesse de rotation mesurée
Tf = 0.02 # constante de temps de filtrage de l'action dérivée du PID
Kpvx1 = -18.2 # sous-gain proportionnel pour l'asservissement de vitesse longitudinale
Kpvx2 = 0.132 # sous-gain proportionnel pour l'asservissement de vitesse longitudinale
Kivx = -6.8 # gain intégral pour l'asservissement de vitesse longitudinale
Kpomega = -1.87 # gain proportionnel pour l'asservissement de verticalité
Kiomega = -23.3 # gain intégral pour l'asservissement de verticalité
Kpxidot = 0. # gain proportionnel pour l'asservissement de rotation
Kixidot = 2.14 # gain intégral pour l'asservissement de rotation
commande_vx = 0. # commande pour l'asservissement de vitesse longitudinale
commande_omega = 0. # commande pour l'asservissement de verticalité
commande_xidot = 0. # commande pour l'asservissement de rotation
P_vx = 0. # action proportionnelle pour l'asservissement de vitesse longitudinale
I_vx = 0. # action intégrale pour l'asservissement de vitesse longitudinale
P_omega = 0. # action proportionnelle pour l'asservissement de verticalité
I_omega = 0. # action intégrale pour l'asservissement de verticalité
P_xidot = 0. # action proportionnelle pour l'asservissement de rotation
I_xidot = 0. # action intégrale pour l'asservissement de rotation
thetaest = 0. # angle d'inclinaison estimé par le filtre complémentaire
tau = 1. # paramètre du filtre complémentaire
codeurDroitDeltaPosPrec = 0.
codeurGaucheDeltaPosPrec = 0.

commandeDroit = 0. # commande en tension calculée par le PID pour le moteur droit
commandeGauche = 0. # commande en tension calculée par le PID pour le moteur gauche


# Variables utilisées pour les données reçues
x1 = 0.
x2 = 0.
Kp2 = 1.0
Ki2 = 1.0
Kpxi2 = 1.
Kixi2 = 1.

# Déclarations pour les consignes de mouvement
vxref = 0.
xidotref = 0.
omegaref = 0.
thetaref = 0.

# Déclarations pour la gestion des modes "asservissement actif" et "chute"
startedGeeros = False
juststarted = True
signe_ax = 0
thetamesprec = 0
omegaprec = 0

# Time out de réception des données
timeout = 2
timeLastReceived = 0
timedOut = False

dt = 0.01
tprec = time.time()
i = 0
# Création d'un scheduler pour exécuter des opérations à cadence fixe
s = sched.scheduler(time.time, time.sleep)


#--- setup --- 
def setup():
    global imu, signe_ax
    
    wiringpi2.wiringPiSetupGpio() # For GPIO pin numbering

    # Initialisation de l'IMU
    imu = FaBo9Axis_MPU9250.MPU9250()
    
    # On démarre seulement quand le gyropode dépasse la verticale
    # Pour que le gyropode démarre tout seul quand il est sur l'avant
    # et si on le redresse quand il est sur l'arrière
    signe_ax = -1
  
    CommandeMoteurs(0, 0, tensionBatterie)
    

    
# -- fin setup -- 
 
# -- loop -- 
def loop():
    global i, T0
    i = i+1
    s.enterabs( T0 + (i * dt), 1, CalculVitesse, ())
    s.run()
# -- fin loop --


def CalculVitesse():
    global ticksCodeurDroit, ticksCodeurGauche, indiceTicksCodeurDroit, indiceTicksCodeurGauche, started, omega, thetames, \
        omegaDroit, omegaGauche, ticksCodeurDroitTab, ticksCodeurGaucheTab, thetamesprec, omegaprec, codeurDroitDeltaPosPrec, codeurGaucheDeltaPosPrec, \
        P_vx, I_vx, P_xidot, I_xidot, imu, thetaest, tau, omegaref, thetaref, timeLastReceived, timeout, timedOut, \
        codeurDroitDeltaPos, codeurGaucheDeltaPos, commandeDroit, commandeGauche, vxmes, xidotmes, vxref, xidotref, juststarted, \
        startedGeeros, signe_ax, x1, x2, dt2, tprec, commandeDroitPrec, commandeGauchePrec
        
    debut = time.time()
    
    # Tant que l'asservissement de verticalité n'est pas activé
    while not startedGeeros:

        tprec = time.time() - dt
                
        # Geeros vient de chuter
        if juststarted:

            # On réinitialise l'asservissement et on entre dans la boucle d'attente de redressement
            juststarted = False

            # Réinitilisation de l'asservissement
            I_vx = 0.
            I_omega = 0.
            I_xidot = 0.
            thetaest = 0.

        # Mesure de la pesanteur
        accel = imu.readAccel()
        ax = -accel['y'] * 9.81
        
        # Si l'accélération change de signe, cela signifie que Geeros a été redressé pour le faire démarrer
        if (ax * signe_ax) <0:

            # Donc, on démarre
            startedGeeros = True

            # Les consignes reçues sont initilisées à zéro
            x1 = 0.
            x2 = 0.


        else:
            # On garde tout à zéro
            I_vx = 0.
            I_omega = 0.
            I_xidot = 0.
            thetaest = 0.

    # Mesure de la vitesse des moteurs grâce aux codeurs incrémentaux
    try:
        codeurGaucheDeltaPos = a_star.read_codeurGaucheDeltaPos()
        codeurDroitDeltaPos = a_star.read_codeurDroitDeltaPos()
        if abs(codeurGaucheDeltaPos) > 1000 or abs(codeurDroitDeltaPos) > 1000:
            print "Values out of range"
            codeurGaucheDeltaPos = codeurGaucheDeltaPosPrec
            codeurDroitDeltaPos = codeurDroitDeltaPosPrec
        else:
            codeurGaucheDeltaPosPrec = codeurGaucheDeltaPos
            codeurDroitDeltaPosPrec = codeurDroitDeltaPos
    except:
        print "Error getting data"
        codeurGaucheDeltaPos = codeurGaucheDeltaPosPrec
        codeurDroitDeltaPos = codeurDroitDeltaPosPrec

    # C'est bien dt qu'on utilise ici et non pas dt2 (voir plus loin l'explication de dt2)
    # car codeurDroitDeltaPos et codeurGaucheDeltaPos sont mesurés en temps-réel par l'A*
    omegaDroit = -2 * ((2 * 3.141592 * codeurDroitDeltaPos) / 1632) / (Nmoy * dt)  # en rad/s
    omegaGauche = 2 * ((2 * 3.141592 * codeurGaucheDeltaPos) / 1632) / (Nmoy * dt)  # en rad/s
    
    # Lecture des infos de l'IMU
    try:
        accel = imu.readAccel()
        gyro = imu.readGyro()
        
        # Acceleration longitudinale
        thetames = -accel['y']
        # Vitesse de rotation mesurée par le gyroscope convertie en rad/s
        omega = -gyro['x'] * 3.141592 / 180.
        
        thetamesprec = thetames
        omegaprec = omega
    except:
        thetames = thetamesprec
        omega = omegaprec
        print "Problème de lecture IMU"


    # La suite des calculs se fait avec dt2, qui correspond au "vrai" pas de temps d'échantillonnage
    # de cette fonction (la RPi n'est pas un système temps-réel et au début de l'exécution du programme,
    # dt2 peut être jusqu'à deux fois plus petit que dt)
    dt2 = time.time() - tprec
    tprec = time.time()

    # Estimation de l'angle à partir de l'accélération horizontale (filtre complémentaire)
    thetaest = (tau*dt2*omega + dt2*thetames + tau*thetaest)/(dt2+tau)

    # Test pour savoir si le gyropode est toujours debout
    if startedGeeros:

        # Si l'angle d'inclinaison n'est pas trop prononcé
        if abs(thetames) < 1.:
            en = 1.

        # Sinon, cela signifie que Geeros est tombé
        else:
            # On désactive l'asservissement
            en = 0.
            startedGeeros = False
            juststarted = True

            # Réinitialisation pour détecter un futur redressement
            if thetames > 0:
                signe_ax = 1
            else:
                signe_ax = -1

    # Si Geeros n'a pas encore démarré, l'asservissement est désactivé
    else:
        en = 0.

        
    # Si on n'a pas reçu de données depuis un certain temps, celles-ci sont annulées
    if (time.time()-timeLastReceived) > timeout and not timedOut:
        timedOut = True
        x1 = 0.
        x2 = 0.

    # Application de la consigne lue
    vxref = x1 * en
    xidotref = x2 * en

    # Définition des entrées de la fonction d'asservissement
    vxmes = (omegaDroit + omegaGauche)*R/2 * en
    omegames = omega * en
    xidotmes = -(omegaDroit - omegaGauche)*R/W * en

    # Calcul du PI sur vx
    Kpvx = (Kpvx1 + Kpvx2/R) * Kp2
    
    # Terme proportionnel (la transformation de la commande par retour d'état en PI
    # conduit à une référence nulle, d'où le 0.*vxref)
    P_vx = Kpvx * (0. * vxref - vxmes)

    # Calcul de la commande
    commande_vx = P_vx + I_vx


    # Terme intégral (sera utilisé lors du pas d'échantillonnage suivant)
    I_vx = I_vx + Kivx * Ki2 * dt2 * (vxref - vxmes)

    # Fin Calcul du PI sur vx

    # Calcul du PI sur omega
    # Terme proportionnel
    P_omega = Kpomega * (omegaref - omegames)
    
    # Terme "intégral" (c'est en fait un terme proportionnel sur theta)
    I_omega = Kiomega * (thetaref - thetaest)

    # Calcul de la commande
    commande_omega = P_omega + I_omega

    # Fin Calcul du PI sur omega

    # Calcul du PI sur xidot
    
    # Terme proportionnel
    P_xidot = Kpxidot * (xidotref - xidotmes)

    # Calcul de la commande
    commande_xidot = P_xidot + I_xidot


    # Terme intégral (sera utilisé lors du pas d'échantillonnage suivant)
    I_xidot = I_xidot + Kixidot * Kixi2 * dt2 * (xidotref - xidotmes)

    # Fin Calcul du PI sur xidot


    # Calcul des commandes des moteurs
    commandeDroit = (commande_vx + commande_omega - commande_xidot) * en
    commandeGauche = (commande_vx + commande_omega + commande_xidot) * en
      
    CommandeMoteurs(commandeDroit, commandeGauche, tensionBatterie)
    
    

    
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
        global x1, x2, Kp2, Ki2, Kpxi2, Kixi2, timeLastReceived, timedOut
        jsonMessage = json.loads(message)
        
        # Annulation du timeout de réception des données
        timeLastReceived = time.time()
        timedOut = False;
      
        if jsonMessage.get('vref') != None:
            x1 = float(jsonMessage.get('vref'))
            #print ("x1: %.2f" % x1)
        if jsonMessage.get('psidotref') != None:
            x2 = (float(jsonMessage.get('psidotref'))) * 3.141592 / 180
            #print ("x2: %.2f" % x2)
        if jsonMessage.get('servoref') != None:
            servoref = int(jsonMessage.get('servoref'))
            a_star.servo(servoref)
            #print ("servoref: %d" % servoref)
        if jsonMessage.get('Kp2ref') != None:
            Kp2 = float(jsonMessage.get('Kp2ref'))
            #print ("Kp2: %.2f" % Kp2)
        if jsonMessage.get('Ki2ref') != None:
            Ki2 = float(jsonMessage.get('Ki2ref'))
            #print ("Ki2: %.2f" % Ki2)
        if jsonMessage.get('Kpxi2ref') != None:
            Kpxi2 = float(jsonMessage.get('Kpxi2ref'))
            #print ("Kpxi2: %.2f" % Kpxi2)
        if jsonMessage.get('Kixi2ref') != None:
            Kixi2 = float(jsonMessage.get('Kixi2ref'))
            #print ("Kixi2: %.2f" % Kixi2)
        

    def on_close(self):
        global socketOK, commandeDroit, commandeGauche
        print 'connection closed...'
        socketOK = False
        commandeDroit = 0.
        commandeGauche = 0.

    def sendToSocket(self):
        global started, codeurDroitDeltaPos, codeurGaucheDeltaPos, socketOK, commandeDroit, commandeGauche, vxref, xidotref, vxmes, xidotmes, omega, thetames, T0
        
        tcourant = time.time() - T0
        aEnvoyer = json.dumps({'Temps':("%.2f" % tcourant), 'commandeDroit':("%.2f" % commandeDroit), 'commandeGauche':("%.2f" % commandeGauche), 'omega':("%.3f" % omega), 'omegaDroit':("%.2f" % omegaDroit), 'omegaGauche':("%.2f" % omegaGauche), 'thetames':("%.3f" % thetames), 'Consigne vitesse longitudinale':("%.2f" % x1), 'Consigne vitesse de rotation':("%.2f" % x2), 'Vitesse longitudinale':("%.2f" % vxmes), 'Vitesse de rotation':("%.2f" % (180 * xidotmes/3.141592)), 'Raw':("%.2f" % tcourant) + "," + ("%.2f" % commandeDroit) + "," + ("%.2f" % commandeGauche) + "," + ("%.3f" % omega) + "," + ("%.2f" % omegaDroit) + "," + ("%.2f" % omegaGauche) + "," + ("%.3f" % thetames) + "," + ("%.2f" % x1) + "," + ("%.2f" % x2) + "," + ("%.2f" % vxmes) + "," + ("%.2f" % (180 * xidotmes/3.141592))})
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

#--- obligatoire pour lancement du code -- 
if __name__=="__main__": # pour rendre le code executable 
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


