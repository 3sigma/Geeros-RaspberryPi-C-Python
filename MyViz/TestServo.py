from geeros_api import geeros_api
Geeros = geeros_api()
Geeros.AngleServo(20, 1)
Geeros.AngleServo(70, 1)
Geeros.AngleServo(45, 1)
Geeros.Terminer()