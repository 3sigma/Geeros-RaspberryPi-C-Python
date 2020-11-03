#!/usr/bin/python
# -*- coding: utf-8 -*-

##################################################################################
# Programme de d'asservissement d'angle du robot Geeros disponible à l'adresse:
# http://boutique.3sigma.fr/12-robots
#
# Auteur: 3Sigma
# Version 3.1 - 02/11/2020
##################################################################################

# Import WiringPi2
import wiringpi2

# Imports pour l'IMU sur bus i2c
import smbus
from mpu9250 import MPU9250

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

omega = 0.
thetames = 0.


# Les moteurs sont asservis en vitesse grâce à un régulateur de type PID
# On déclare ci-dessous les variables et paramètres nécessaires à l'asservissement et au régulateur
R = 0.045 # Rayon d'une roue
W = 0.14 # Largeur du robot
umax = 6. # valeur max de la tension de commande du moteur
umin = -6. # valeur min (ou max en négatif) de la tension de commande du moteur
vxmes = 0. # vitesse longitudinale mesurée
ximes = 0. # vitesse de rotation mesurée par odométrie
gz = 0. # vitesse de rotation mesurée par le gyroscope
psimes = 0. # angle de rotation mesuré
Tf = 0.02 # constante de temps de filtrage de l'action dérivée du PID
Kpvx = -15.3 # gain proportionnel pour l'asservissement de vitesse longitudinale
Kivx = -6.8 # gain intégral pour l'asservissement de vitesse longitudinale
Kpomega = -1.87 # gain proportionnel pour l'asservissement de verticalité
Kiomega = -23.3 # gain intégral pour l'asservissement de verticalité
Kpxi = 0. # gain proportionnel pour l'asservissement de rotation
Kixi = 2.14 # gain intégral pour l'asservissement de rotation
Kdxi = 0. # gain dérivé pour l'asservissement de rotation
commande_vx = 0. # commande pour l'asservissement de vitesse longitudinale
commande_omega = 0. # commande pour l'asservissement de verticalité
commande_xi = 0. # commande pour l'asservissement de rotation
P_vx = 0. # action proportionnelle pour l'asservissement de vitesse longitudinale
I_vx = 0. # action intégrale pour l'asservissement de vitesse longitudinale
P_omega = 0. # action proportionnelle pour l'asservissement de verticalité
I_omega = 0. # action intégrale pour l'asservissement de verticalité
P_xi = 0. # action proportionnelle pour l'asservissement de rotation
I_xi = 0. # action intégrale pour l'asservissement de rotation
thetaest = 0. # angle d'inclinaison estimé par le filtre complémentaire
tau = 1. # paramètre du filtre complémentaire
Kpangle = 2 # gain proportionnel du PID d'asservissement d'angle
Kiangle = 0.0 # gain intégral du PID d'asservissement d'angle
Kdangle = 0.0 # gain dérivé du PID d'asservissement d'angle
Tfangle = 0.01 # constante de temps de filtrage de l'action dérivée du PID d'asservissement d'angle
I_x = [0., 0.]
D_x = [0., 0.]
erreurprec = [0., 0.]
commandeDroit = 0. # commande en tension calculée par le PID pour le moteur droit
commandeGauche = 0. # commande en tension calculée par le PID pour le moteur gauche

# Variables utilisées pour les données reçues
x1 = 0.
x2 = 0.
Kp2 = 1.
Ki2 = 1.
Kpxi2 = 1.
Kixi2 = 1.

# Déclarations pour les consignes de mouvement
vxref = 0.
xiref = 0.
omegaref = 0.
thetaref = 0.
psiref = 0.
ximax = 50. # En deg/s

# Déclarations pour la gestion des modes "asservissement actif" et "chute"
startedGeeros = False
juststarted = True
signe_ax = 0
thetamesprec = 0
omegaprec = 0

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
    global imu, signe_ax, tensionAlim
    
    wiringpi2.wiringPiSetupGpio() # For GPIO pin numbering

    # Initialisation de l'IMU
    # initIMU_OK = False
    # while not initIMU_OK:
        # try:
            # imu = MPU9250()
            # initIMU_OK = True
        # except:
            # print("Erreur init IMU")
    imu = MPU9250()
    
    # On démarre seulement quand le gyropode dépasse la verticale
    # Pour que le gyropode démarre tout seul quand il est sur l'avant
    # et si on le redresse quand il est sur l'arrière
    signe_ax = -1
  
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
    global started, omega, thetames, \
        omegaDroit, omegaGauche, thetamesprec, omegaprec, codeurDroitDeltaPosPrec, codeurGaucheDeltaPosPrec, \
        P_vx, I_vx, P_xi, I_xi, imu, thetaest, tau, omegaref, thetaref, timeLastReceived, timeout, \
        codeurDroitDeltaPos, codeurGaucheDeltaPos, commandeDroit, commandeGauche, vxmes, ximes, vxref, xiref, juststarted, \
        startedGeeros, signe_ax, x1, x2, dt2, tprec, \
        idecimLectureTension, decimLectureTension, tensionAlim, psiref, psimes, Kpangle, Kiangle, Kdangle, Tfangle, \
        Kpxi, Kixi, Kdxi, ximax, gz
    
    # Tant que l'asservissement de verticalité n'est pas activé
    while not startedGeeros:

        time.sleep(0.005)
        
        tprec = time.time() - dt
                
        # Geeros vient de chuter
        if juststarted:

            # On réinitialise l'asservissement et on entre dans la boucle d'attente de redressement
            juststarted = False

            # Réinitilisation de l'asservissement
            I_vx = 0.
            I_omega = 0.
            I_xi = 0.
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
            I_xi = 0.
            thetaest = 0.

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
    
    # Lecture des infos de l'IMU
    try:
        accel = imu.readAccel()
        gyro = imu.readGyro()
        
        # Acceleration longitudinale
        thetames = -accel['y']
        # Vitesse de rotation mesurée par le gyroscope convertie en rad/s
        omega = -gyro['x'] * 3.141592 / 180.
        gz = -gyro['z'] * 3.141592 / 180.
        
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
    if (time.time()-timeLastReceived) > timeout:
        x1 = 0.
        x2 = 0.

    # Application de la consigne lue
    vxref = x1 * en
    psiref = x2 * en

    # Définition des entrées de la fonction d'asservissement
    vxmes = (omegaDroit + omegaGauche)*R/2 * en
    omegames = omega * en
    ximes = -(omegaDroit - omegaGauche)*R/W * en
    psimes = psimes + ximes * dt

    # Calcul du PI sur vx    
    # Terme proportionnel (la transformation de la commande par retour d'état en PI
    # conduit à une référence nulle, d'où le 0.*vxref)
    P_vx = Kpvx * Kp2 * (0. * vxref - vxmes)

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

    # Calcul du PI sur xi
    
    # L'asservissement d'angle a une boucle interne en vitesse.
    xiref = PID(0, psiref, psimes, Kpangle, Kiangle, Kdangle, Tfangle, ximax * math.pi / 180, -ximax * math.pi / 180, dt2)
    commande_xi = PID(1, xiref, ximes, Kpxi, Kixi, Kdxi, Tfangle, umax, umin, dt2);

    # Calcul des commandes des moteurs
    commandeDroit = (commande_vx + commande_omega - commande_xi) * en
    commandeGauche = (commande_vx + commande_omega + commande_xi) * en
      
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

        
    
def PID(iMoteur, ref, mes, Kp, Ki, Kd, Tf, umax, umin, dt2):
    global I_x, D_x, erreurprec
    
    # Calcul du PID
    # Paramètres intermédiaires
    br = 1. / (Kp + 0.0001)
    ad = Tf / (Tf + dt2);
    bd = Kd / (Tf + dt2);

    # Calcul de la commande avant saturation
    erreur = ref - mes
    
    # Terme proportionnel
    P_x = Kp * erreur

    # Terme dérivé
    D_x[iMoteur] = ad * D_x[iMoteur] + bd * (erreur - erreurprec[iMoteur])

    # Calcul de la commande avant saturation
    if (Ki == 0):
        I_x[iMoteur] = 0.
    commande_avant_sat = P_x + I_x[iMoteur] + D_x[iMoteur]

    # Application de la saturation sur la commande
    if (commande_avant_sat > umax):
        commande = umax
    elif (commande_avant_sat < umin):
        commande = umin
    else:
        commande = commande_avant_sat
    
    # Terme intégral (sera utilisé lors du pas d'échantillonnage suivant)
    I_x[iMoteur] = I_x[iMoteur] + Ki * dt2 * (erreur + br * (commande - commande_avant_sat))
    
    # Stockage de la mesure courante pour utilisation lors du pas d'échantillonnage suivant
    erreurprec[iMoteur] = erreur
    
    return commande



    
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
        global x1, x2, psiref, Kpangle, Kiangle, Kdangle, Kpxi, Kixi, Kdxi, ximax, timeLastReceived, socketOK

        jsonMessage = json.loads(message)
        
        # Annulation du timeout de réception des données
        timeLastReceived = time.time()
      
        if jsonMessage.get('vref') != None:
            x1 = float(jsonMessage.get('vref')) / 100
            #print ("x1: %.2f" % x1)
        if jsonMessage.get('psiref') != None:
            x2 = (float(jsonMessage.get('psiref'))) * 3.141592 / 180
            #print ("x2: %.2f" % x2)
        if jsonMessage.get('servoref') != None:
            servoref = int(jsonMessage.get('servoref'))
            try:
                a_star.servo(servoref)
            except:
                pass
            #print ("servoref: %d" % servoref)
        if jsonMessage.get('Kp') != None:
            Kpangle = float(jsonMessage.get('Kp'))
        if jsonMessage.get('Ki') != None:
            Kiangle = float(jsonMessage.get('Ki'))
        if jsonMessage.get('Kd') != None:
            Kdangle = float(jsonMessage.get('Kd'))
        if jsonMessage.get('Kpv') != None:
            Kpxi = float(jsonMessage.get('Kpv'))
        if jsonMessage.get('Kiv') != None:
            Kixi = float(jsonMessage.get('Kiv'))
        if jsonMessage.get('Kdv') != None:
            Kdxi = float(jsonMessage.get('Kdv'))
        if jsonMessage.get('ximax') != None:
            ximax = float(jsonMessage.get('ximax'))
        
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
                vxmes, ximes, omega, thetames, T0, x1, x2, gz
        
        tcourant = time.time() - T0
        aEnvoyer = json.dumps({ 'Temps':("%.2f" % tcourant), \
                                'commandeDroit':("%.2f" % commandeDroit), \
                                'commandeGauche':("%.2f" % commandeGauche), \
                                'omega':("%.3f" % omega), \
                                'omegaDroit':("%.2f" % omegaDroit), \
                                'omegaGauche':("%.2f" % omegaGauche), \
                                'thetames':("%.3f" % thetames), \
                                'Consigne vitesse longitudinale':("%.2f" % x1), \
                                'Consigne vitesse de rotation':("%.2f" % (180 * xiref/3.141592)), \
                                'consigne_psi':("%.2f" % (180 * x2/3.141592)), \
                                'Vitesse longitudinale':("%.2f" % vxmes), \
                                'Vitesse de rotation':("%.2f" % (180 * ximes/3.141592)), \
                                'Vitesse de rotation gyro':("%.2f" % (180 * gz/3.141592)), \
                                'Angle':("%.2f" % (180 * psimes/3.141592))})
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



