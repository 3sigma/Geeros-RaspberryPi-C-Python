from geeros_api import geeros_api
Geeros = geeros_api()
Geeros.Tourner("180 * math.sin(2*t)", 10)
Geeros.Terminer()