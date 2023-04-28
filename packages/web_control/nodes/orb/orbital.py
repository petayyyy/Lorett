import matplotlib.pyplot as plt

from pyorbital.orbital import Orbital
from datetime import datetime, timedelta
from math import radians, tan, sin, cos
from numpy import arange, float32, float64, abs
from prettytable import PrettyTable

from os.path import join
from pathlib import Path

from logging import Logger

###
from requests import get, exceptions
from bs4 import BeautifulSoup as bs

from .exceptions import *

# TODO Проверка вменяемости конфига
# TODO сделать отрисовку пролетов не только для станций с перемещением облучателя в факальной плоскости 

__all__ = [
    "Scheduler",
    "satListLband",
    "satListAPTBand",
    "supportedStationTypes",
    "__version__"
]

__version__ = "0.0.4"

satListLband = ["NOAA 18",
                "NOAA 19",
                "METEOR-M 2",
                "METEOR-M2 2",
                "METOP-B",
                "METOP-C",
                "FENGYUN 3C"]
                
satListAPTBand = ["NOAA 15",
                  "NOAA 18",
                  "NOAA 19",
                  "METEOR-M 2"]

supportedStationTypes = {
                        'l2s': {
                                    'kinematic': 'focal',
                                    'band': 'L',
                                    'satList': satListLband,
                                    'sampleRate': 6e6,
                                    'horizon': 55,
                                    'minApogee': 65,
                                    'focus': 0.77,
                                    'radius': 0.55
                                },
                         'c4s': {
                                    'kinematic': 'focal',
                                    'band': 'L',
                                    'satList': satListLband,
                                    'sampleRate': 6e6,
                                    'horizon': 55,
                                    'minApogee': 65,
                                    'focus': 0.77,
                                    'radius': 0.55
                                },
                         'r8s': {
                                    'kinematic': 'rotate',
                                    'band': 'L',
                                    'satList': satListLband,
                                    'sampleRate': 6e6,
                                    'horizon': 15,
                                    'minApogee': 35
                                },
                         'apt': {
                                    'kinematic': 'apt',
                                    'band': 'apt',
                                    'satList': satListAPTBand,
                                    'sampleRate': 11025,
                                    'horizon': 15,
                                    'minApogee': 20
                                }
                        }


'''Класс для работы с расписанием и траекториями спутников'''
class Scheduler():
    '''
    A class for working with the schedule and trajectories of satellites    

    In:
        str stationName

        float lat

        float lon

        float alt

        str path

        str stationType

        int timeZone

        float azimuthCorrection
    '''

    def __init__(self, stationName: str,
                 lat: float = 0,
                 lon: float = 0,
                 alt: float = 0,
                 path: str = '',
                 stationType: str = 'apt',
                 timeZone: int = 0,
                 azimuthCorrection: float = 0,
                 config: dict = {}
                 ) -> None:

        # определение типа станции по названию
        if stationType != '':
            if stationType.lower() in supportedStationTypes.keys():
                self.stationType = stationType.lower()
            else:
                raise UnknownStationType(stationType)

        else:
            for type in supportedStationTypes.keys():
                if type in stationName.lower():
                    self.stationType = type
                    break
            else:
                raise UnknownStationType(stationName)

        if len(config):
            # 'apt' contains the minimum required list of parameters
            if all([x in supportedStationTypes['apt'].keys() for x in config.keys()]):
                self.stationType = 'castom'

            self.config = config
            
        else:
            self.config = supportedStationTypes[self.stationType]

        
        self.lon = round(lon, 5)
        self.lat = round(lat, 5)
        self.alt = round(alt, 5)

        self.timeZone = timeZone
        self.stationName = stationName

        self.azimuthCorrection = azimuthCorrection
        
        self.mirrorCircleColor = '#66ccff'

        self.path = path

        self._createSubDirectories()


    '''Сервисный метод для получения количества дней с начала года'''
    def _getDays(self, date: datetime) -> int:
        '''
        Service method for getting the number of days since the beginning of the year

        In:
            datetime date

        Out:
            int data
        '''
        daysForMonth = [
            0,
            31,     # January
            59,     # February
            90,     # March
            120,    # April
            151,    # May
            181,    # June
            212,    # July
            243,    # August
            273,    # September
            304,    # October
            334,    # November
            365     # December
        ]

        days = date.day
        days += daysForMonth[date.month-1]

        return days


    '''Сервисный медод для проверки координат'''
    def _checkCoordinates(self) -> None:
        """
        Service method for checking coordinates
        """
        if not all((self.lon, self.lat, self.alt)):
            raise CoordinatesAreNotGiven(self.lon, self.lat, self.alt)

        if any((abs(self.lon) > 90, abs(self.lat) > 90)):
            raise InvalidCoordinates

    
    '''Сервисный метод для создания дополнительных директорий'''
    def _createSubDirectories(self) -> None:
        """
        Service method for creating additional directories
        """

        self.tlePath = join( self.path, "tle" )
        Path( self.tlePath ).mkdir(parents=True, exist_ok=True)

        self.tracksPath = join( self.path, "tracks" )
        Path( self.tracksPath ).mkdir(parents=True, exist_ok=True)

        self.tracksSchemesPath = join( self.path, "tracksSchemes" )
        Path( self.tracksSchemesPath ).mkdir(parents=True, exist_ok=True)

        self.schedulePath = join( self.path, "schedule" )
        Path( self.schedulePath ).mkdir(parents=True, exist_ok=True)


    '''Сервисный метод для перевода из сферичиских координат в декартовы'''
    def sphericalToDecart(self, azimuth: float, elevation: float) -> tuple:
        """
        Service method for translation from spherical coordinates to Cartesian coordinates

        In:
                float azimuth

                float elevation
        Out:
                float x

                float y
        """

        if elevation == 90:
            return 0, 0

        azimuth = radians((azimuth + self.azimuthCorrection) % 360)
        elevation = radians(elevation)

        y = -(self.config['focus'] / tan(elevation)) * cos(azimuth)
        x = -(self.config['focus'] / tan(elevation)) * sin(azimuth)

        return x, y

    
    '''Метод который преобразует угловые координаты в форму градусы:минуты (a, e) --> (a:m, e:m)'''
    def degreesToDegreesAndMinutes(self, azimuth: float, elevation: float) -> tuple:
        """
        Method that translates angular coordinates to the form degrees:minutes (a, e) --> (a:m, e:m)

        In:
                float azimuth (градусы)

                float elevation (градусы)
        Out:
                str azimuth (минуты)

                str elevation (минуты)
        """
        typeAz = type(azimuth)
        if typeAz == float or typeAz == float32 or typeAz == float64:
            minutes = azimuth * 60
            degrees = minutes // 60
            minutes %= 60

            azimuthM = f"{int(degrees):03}:{int(minutes):02}"

        elif typeAz == int:
            azimuthM = f"{azimuth:03}:00"

        else:
            return False

        typeEl = type(elevation)
        if typeEl == float or typeEl == float32 or typeEl == float64:
            minutes = elevation * 60
            degrees = minutes // 60
            minutes %= 60

            elevationM = f"{int(degrees):03}:{int(minutes):02}"

        elif typeEl == int:
            elevationM = f"{elevation:03}:00"

        else:
            return False

        return azimuthM, elevationM

    
    '''Метод для обновления TLE файлов'''
    def update(self) -> bool:
        """
        Method that updates TLE files

        In:
                bool showLog
        """

        name = join(self.path, "tle", "tle.txt")
        now = datetime.utcnow()
        
        try:
            with open(name, "r") as file:
                yearInTLE, daysPassInTLE = map(int, file.readline().strip().split(' '))
        except:
            yearInTLE = now.year
            daysPassInTLE = 0

        if (yearInTLE == now.year) and (self._getDays(now) - daysPassInTLE <= 5):
            return True

        try:
            page = get("http://celestrak.com/NORAD/elements/")
            html = bs(page.content, "html.parser")
            now = datetime.utcnow()

            # Getting TLE date with server
            try:
                year = int(html.select('h3.center')[0].text.split(' ')[3])
                dayPass = int(html.select('h3.center')[
                              0].text.replace(')', '').rsplit(' ', 1)[1])

            except:
                year = now.year
                dayPass = 0
                daysPassInTLE = 0

            # if TLE is outdated then update TLE
            if (yearInTLE < year) or ((yearInTLE == year) and (daysPassInTLE < dayPass)):

                with open(name, "wb") as file:
                    file.write(
                        f"{now.year} {self._getDays(now)}\n".encode('utf-8'))
                    file.write(
                        get("http://www.celestrak.com/NORAD/elements/weather.txt").content)

        except exceptions.ConnectionError:
            #print('Error when update TLE')
            #print("No internet connection\n")
            return False

        except Exception as e:
            #print('Error when update TLE')
            #print(str(e), "\n")
            return False

        return True


    '''Метод для получения координат станции по ip-адресу.'''
    def getCoordinatesByIp(self) -> tuple:
        """
        Method that gets the station coordinates by his ip.

        Returned Altitude in kilometers.\n\n

        ATTENTION!

        THESE COORDINATES MAY BE VERY INACCURATE.

        USE IT ONLY FOR MAKING AN APPROXIMATE SCHEDULE.\n\n


        Out:
                float lat

                float lon

                float alt


        If there is an error:
                float lat=None

                float lon=None

                float alt=None

        """

        try:
            query = get("http://ip-api.com/json").json()
            
            lat = query['lat']
            lon = query['lon']

            # temporary return only elevation by coordinates
            query = get(
                f'https://api.open-elevation.com/api/v1/lookup?locations={lat},{lon}').json()
            alt = query['results'][0]['elevation']

        except exceptions.ConnectionError:
            print('Error when get coordinates')
            print("No internet connection\n")

            return 0, 0, 0

        except Exception as e:
            print('Error when get coordinates')
            print(str(e))

            return 0, 0, 0

        alt /= 1000

        return lat, lon, alt


    '''Метод который вычисляет проходы спутника по входным параметрам'''
    def getSatellitePasses(self, start: datetime, length: int, satellite: str, tol: float = 0.001, updateTLE: bool = True) -> list:
        """
        Method that calculates satellite passes by input parametres

        In:
                str satellite

                datetime start

                int length

                float tol

                bool updateTLE

        Out:
                datetime start, datetime end, datetime apogee
        """

        self._checkCoordinates()

        if updateTLE:
            self.update()

        orb = Orbital(satellite, join( self.tlePath, "tle.txt" ) )

        return orb.get_next_passes(start, length, self.lon, self.lat, self.alt, tol, self.config['horizon'])


    '''Метод который составляет расписание в соответствии с параметрами'''
    def getSchedule(self, length: int, tol: float = 0.001, saveSchedule: bool = False, returnTable: bool = False, returnNameSatellite: bool = False, updateTLE: bool = True, returnTd: bool = False) -> PrettyTable:
        """
        Method that makes up the schedule, according to the parameters.

        In:

                int length

                float tol

                bool printTable

                bool saveSchedule

                bool returnTable

                bool returnNameSatellite

                bool updateTLE

        Out:
                str table or list passesForReturn or list massout

        Not the best implementation but there is no other one yet.

        It does not take into account time zone curves, since I am too lazy to saw the implementation for the sake of several points of the planet
        """
        self._checkCoordinates()

        if updateTLE:
            self.update()

        passes = {}
        allPasses = []
        start = datetime.utcnow()

        th = ["Satellite", "DateTime", "Azimuth", "Elevation"]
        td = []
        passesForReturn = []

        # Iterating through all the passes
        for satellite in self.config['satList']:
            pas = self.getSatellitePasses(start, length, satellite, tol)
            # Flights of a specific satellite
            passes[satellite] = pas

            # All passes
            for i in pas:
                allPasses.append(i)

        # Generate table
        for onePass in sorted(allPasses):
            satName = ''
            
            # Going through the list of satellites
            for satellite in self.config['satList']:
                # If the selected span corresponds to a satellite
                if onePass in passes[satellite]:
                    satName = satellite
                    break

            name = join(self.path, 'tle', 'tle.txt')
            orb = Orbital(satellite, name)

            
            # if apogee > minApogee
            if orb.get_observer_look(onePass[2], self.lon, self.lat, self.alt)[1] >= self.config['minApogee']:
                passesForReturn.append((orb, onePass))
                td.append([satName, (onePass[0] + timedelta(hours=self.timeZone)).strftime("%Y.%m.%d %H:%M:%S"),
                            *map(lambda x: round(x, 2), orb.get_observer_look(onePass[0], self.lon, self.lat, self.alt))])

                td.append([satName, (onePass[2] + timedelta(hours=self.timeZone)).strftime("%Y.%m.%d %H:%M:%S"),
                            *map(lambda x: round(x, 2), orb.get_observer_look(onePass[2], self.lon, self.lat, self.alt))])

                td.append([satName, (onePass[1] + timedelta(hours=self.timeZone)).strftime("%Y.%m.%d %H:%M:%S"),
                            *map(lambda x: round(x, 2), orb.get_observer_look(onePass[1], self.lon, self.lat, self.alt))])

                td.append([" ", " ", " ", " "])

        table = PrettyTable(th)
        
        # Adding rows to tables
        for i in td:
            table.add_row(i)

        start += timedelta(hours=self.timeZone)
        stop = start + timedelta(hours=length) + timedelta(hours=self.timeZone)

        # Generate schedule string
        schedule = f"Satellits Schedule / LorettOrbital {__version__}\n"
        schedule += f"Coordinates of the position: {round(self.lon, 4)}° {round(self.lat, 4)}° {self.alt}km\n"
        schedule += f"Time zone: UTC {'+' if self.timeZone >= 0 else '-'}{abs(self.timeZone)}:00\n"
        schedule += f"Start: {start.strftime('%Y.%m.%d %H:%M:%S')}\n"
        schedule += f"Stop:  {stop.strftime('%Y.%m.%d %H:%M:%S')}\n"

        schedule += f"Minimum Elevation: {self.config['horizon']}°\n"
        schedule += f"Minimum Apogee: {self.config['minApogee']}°\n"

        schedule += f"Number of passes: {len(td)//4}\n\n"
        schedule += table.get_string()

        if saveSchedule:
            try:
                name = join(self.schedulePath, f'Schedule_{datetime.now().strftime("%Y-%m-%dT%H-%M")}.txt')

                with open(name, 'w') as file:
                    file.write(schedule)

            except Exception as e:
                print("ERROR:", e)
                return None

        if returnTable:
            return schedule
        
        elif returnTd:
            return td

        elif returnNameSatellite:
            massout = []
            for pas in passesForReturn:
                massout.append((pas[0].satellite_name, pas[1]))

            return massout
            
        else:
            return passesForReturn


    '''Метод для генерирации трек файла'''
    def generateTrack(self, satellite: str, satPass: list, savePlotTrack: bool = False):
        """
        Method for generating a track file

        In:
            str satellite

            list satPass

            bool  savePlotTrack

        Out:
            tuple (satellite [time, azimuth, altitude])
        """

        orb = Orbital(satellite, join(self.path, "tle", "tle.txt"))

        trackPath = join(self.path, 'tracks', f"{satellite.replace(' ', '-')}_{self.stationType.upper()}_{satPass[0].strftime('%Y-%m-%dT%H-%M')}.txt")

        with open(trackPath, "w") as file:

            times = []
            
            сoordsAZ = []
            сoordsEL = []

            if self.config['kinematic'] == 'focal':
                coordsX = []
                coordsY = []

            startTime = satPass[0].strftime('%Y-%m-%d   %H:%M:%S') + " UTC"

            metaData = f"{self.stationType.upper()} track file / LorettOrbital {__version__}\n" + \
                       f"StationName: {self.stationName}\n" +                                     \
                       f"Station Position: {self.lon}° {self.lat}° {self.alt}km\n" +              \
                       f"Satellite: {satellite}\n" +                                              \
                       f"Start date & time: {startTime}\n" +                                      \
                       f"Orbit: {orb.get_orbit_number(satPass[0])}\n"

            metaData += "Time (UTC)   Azimuth (deg:min)   Elevation (deg:min)\n\n" if self.config['kinematic'] == 'focal' \
                       else "Time (UTC)   Azimuth (deg)   Elevation (deg)\n\n"

            # Write metadata
            file.write(metaData)

            # Generating track steps
            for i in range((satPass[1] - satPass[0]).seconds):

                dateTimeForCalc = satPass[0] + timedelta(seconds=i)
                strTime = dateTimeForCalc.strftime("%H:%M:%S")

                # Convert degrees to degrees:minutes
                observerLook = orb.get_observer_look(
                    dateTimeForCalc, self.lon, self.lat, self.alt)

                sphCoords = self.degreesToDegreesAndMinutes(*observerLook)

                times.append(strTime)

                if self.config['kinematic'] == 'focal':
                    сoordsAZ.append(sphCoords[0])
                    сoordsEL.append(sphCoords[1])

                    coords = self.sphericalToDecart(*observerLook)

                    coordsX.append(coords[0])
                    coordsY.append(coords[1])

                elif self.config['kinematic'] == 'rotate':
                    сoordsAZ.append(f'{observerLook[0]:.2f}')
                    сoordsEL.append(f'{observerLook[1]:.2f}')

                string = f"{strTime}   {сoordsAZ[-1]}   {сoordsEL[-1]}\n"
                file.write(string)

        if savePlotTrack and (self.config['kinematic'] == 'focal'):
            self.SavePlotTrack(coordsX, coordsY, satellite, startTime)

        return satellite, list(zip(times, сoordsAZ, сoordsEL))


    '''Метод для отрисовки трека и сохренения в файл'''
    def SavePlotTrack(self, coordsX: list, coordsY: list, satellite: str = "Untitled", start: str = "") -> None:
        """
        Method for drawing a track and saving it to a file

        In:
                float coordsX[]

                float coordsY[]

                str satellite

                str start

                bool savePlotTrack
        """

        ax = plt.gca()
        ax.set_aspect('equal', adjustable='box')

        # Plot mirror
        circle = plt.Circle((0, 0), self.config['radius'],
                                color=self.mirrorCircleColor)
        ax.add_patch(circle)

        # Set window title
        fig = plt.figure(1)
        fig.canvas.manager.set_window_title(satellite + "   " + start)

        # Generate OX and OY Axes
        steps = list(round(i, 1) for i in arange(
            -self.config['radius'], self.config['radius'] + 0.1, 0.1))

        plt.title(satellite + "   " + start)

        # Plot OX and OY Axes
        plt.plot([0]*len(steps), steps, '--k')
        plt.plot(steps, [0]*len(steps), '--k')

        # Plot track
        plt.plot(coordsX, coordsY, '-.r')

        # Plot start and end points
        plt.plot(coordsX[0], coordsY[0], ".b")
        plt.plot(coordsX[-1], coordsY[-1], ".r")

        # Plot north
        plt.plot(0, self.config['radius'], "^r")

        fileName = join(self.path, 'tracksSchemes', f"TracksSchemes_{satellite.replace(' ', '-')}_{start.replace('   ', '-').replace(':', '-')}.png")
        plt.savefig(fileName)


    '''Метод для расчета трека ближайщего следующего пролета'''
    def nextPass(self, length = 12, printTrack: bool = False, savePlotTrack: bool = False):
        '''
        Method for calculating the track of the nearest next span

        In:
            bool  printTrack

            bool  savePlotTrack

        Out:
            tuple (Satellite [time, azimuth, altitute])'''

        
        passesList = self.getSchedule(length)


        count = 1
        td = []

        for satPass, satPasTimeList in passesList:

            td.append([count, satPass.satellite_name, *satPasTimeList, round(
                satPass.get_observer_look(satPasTimeList[2], self.lon, self.lat, self.alt)[1], 2)])
            td.append(("", "", "", "", "", ""))

            count += 1

        satPass, satPasTimeList = passesList[0]

        if self.config['kinematic']:
            outTrack =  self.generateTrack(satPass.satellite_name, satPasTimeList, savePlotTrack)

            if printTrack:
                print('         ',outTrack[0])
                for stepp in outTrack[1]:
                    print(stepp[0], stepp[1], stepp[2], sep='   ')

            return outTrack


    '''Функция для установки местоположения станции'''
    def setCoordinates(self, lat: float, lon: float, alt: float) -> None:
        '''
        Function for setting the station location

        In:

            float lat

            float lon

            float alt
        '''

        self.lon = round(lon, 5)
        self.lat = round(lat, 5)
        self.alt = round(alt, 5)

        self._checkCoordinates()


    def setTimeZone(self, timeZone: int) -> None:
        if abs(timeZone) > 12:
            raise InvalidTimeCorrection

        self.timeZone = timeZone

    
    def getStation(self):
        return {"stationName": self.stationName,
                "stationType": self.stationType,

                "lat": self.lat,
                "lon": self.lon,
                "alt": self.alt,

                'kinematic': self.config['kinematic'],
                'band': self.config['band'],
                'satList': self.config['satList'],
                'sampleRate': self.config['sampleRate'],
                'horizon': self.config['horizon'],
                'minApogee': self.config['minApogee']}


    def getCoordinates(self):
        return {"lat": self.lat,
                "lon": self.lon,
                "alt": self.alt}



class StationLogger:
    def __init__(self, stationName: str, level:int) -> None:
        self.logger = Logger(stationName, level)
    
    def scheduleMessage(self):
        pass


class lorettLBandLogger(StationLogger):
    def __init__(self, level: int) -> None:
        super().__init__(level)

    def scheduleMessage(self):
        pass


class lorettAPTBandLogger(StationLogger):
    def __init__(self, level: int) -> None:
        super().__init__(level)

    def scheduleMessage(self):
        pass


class StreamReader:
    def __init__(self, source) -> None:
        pass


class SDRReader(StreamReader):
    def __init__(self, source) -> None:
        super().__init__(source)

class Station:
    def __init__(self, scheduler: Scheduler, streamReader: StreamReader, logger: StationLogger) -> None:
        pass