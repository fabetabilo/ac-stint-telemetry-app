import ac
import acsys
from operator import itemgetter

class RadarSystem:

    def __init__(self, radar_range=40.0):
        self.radar_range = radar_range
        self.active_drivers_cache = []

    def scan_grid(self):
        self.active_drivers_cache = []
        total_slots = ac.getCarsCount()
        # escanea desde id 1 en adelante
        for carId in range(1, total_slots):
            if ac.isConnected(carId):
                self.active_drivers_cache.append(carId)
                
    def get_nearby_cars(self, my_x, my_z, limit=0):
        
        nearby_cars = []
        radar_sq = self.radar_range * self.radar_range
        
        for carId in self.active_drivers_cache:
            try:
                # este bloque valida si esta conectado, en caso de desconexion no se rompe app lol
                if not ac.isConnected(carId): 
                    continue

                pos = ac.getCarState(carId, acsys.CS.WorldPosition)
                x = pos[0]
                z = pos[2]

                dx = x - my_x
                dz = z - my_z
                dist_sq = (dx*dx) + (dz*dz)
                
                # si esta dentro del rango cuadrado
                if dist_sq < radar_sq:
                    nearby_cars.append({
                        "id": carId,
                        "x": x,
                        "z": z,
                        "d": dist_sq
                    })
            except:
                continue
        # se ordena por distancia 'd' ascendente, los mas cercanos primero
        # esto asegura que al cortar no se elimine al que esta cerca
        nearby_cars.sort(key=itemgetter('d'))
        
        if limit > 0 and len(nearby_cars) > limit:
            nearby_cars = nearby_cars[:limit]
                
        return nearby_cars