from datetime import datetime, timedelta
from math import radians, sin, cos, tan
from tle_storage import TleStrorage
from pyorbital.orbital import Orbital
from typing import Tuple, List

class Track:
    def __init__(self,
                 lat: float, 
                 lon: float,
                 alt: float,
                 heigth: int = 0.77,
                 azimuthCorrection: int = 0) -> None:
        self.heigth = heigth
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.azimuthCorrection = azimuthCorrection
        self.tle = TleStrorage()
        self.satListLband = ["NOAA 18","NOAA 19","METEOR-M 2","METEOR-M2 2","METOP-B","METOP-C"]

    def sphericalToDecart(self, azimuth: float, elevation: float) -> Tuple[float, float]:
        if elevation == 90:
            return 0, 0     

        azimuth = radians((azimuth + self.azimuthCorrection) % 360 )
        elevation = radians(elevation)
        
        y = -(self.heigth / tan(elevation)) * cos(azimuth)
        x = -(self.heigth / tan(elevation)) * sin(azimuth)
        
        return x, y 
    
    def getOrbital(self, satellite: str) -> Orbital:
        tle = self.tle.get_tle(satellite)
        return Orbital(satellite=tle.platform, line1=tle.line1, line2=tle.line2)
    
    def getNearestPass(self, satellite: str) -> Tuple[Tuple[datetime, datetime, datetime], str]:
        return (self.getOrbital(satellite).get_next_passes(datetime.utcnow(), 48, self.lon, 
            self.lat, self.alt, horizon=55)[0],satellite)
    
    def generateTrack(self)-> Tuple[str, datetime, List[Tuple[float, float, float]]]:
        satPass, satellite = min(map(self.getNearestPass, self.satListLband))
        o = self.getOrbital(satellite)
        xCoords, yCoords, zCoords = [], [], []

        for i in range((satPass[1] - satPass[0]).seconds):
            look = o.get_observer_look(satPass[0] + timedelta(seconds=i), self.lon, self.lat, self.alt)
            x, y = self.sphericalToDecart(*look)
            xCoords.append(x)
            yCoords.append(y)
            zCoords.append(self.heigth)
        
        return satellite, satPass[0], list(zip(xCoords, yCoords, zCoords))
            
    

# t = Track(61.001389, 68.993910, 60)
# print(t.generateTrack())