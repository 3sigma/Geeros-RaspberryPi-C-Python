import time

from geeros_api import geeros_api
Geeros = geeros_api()

thetames = 0.
while abs(thetames) < 0.2:
	thetames = Geeros.LireVariable('thetames')
	print("thetames = %.2f rad" % thetames)
	time.sleep(0.1)
	
Geeros.Tourner(180, 1)
time.sleep(2)

Geeros.Terminer()