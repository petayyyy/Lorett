import os
from pyorbital.tlefile import Tle
from datetime import datetime
from requests import get, exceptions
from bs4 import BeautifulSoup as bs

class TleStrorage:
    def __init__(self) -> None:
        self.tles = {}
        self.tleFilePath = os.path.join(os.path.dirname(__file__), "tle", "tle_cahce")

        self.update()
        self._load_cache()
    
    def update(self) -> None:
        rootPath = os.path.dirname(__file__)
        now = datetime.utcnow()

        if not os.path.exists(os.path.join(rootPath, "tle")):

            os.mkdir(os.path.join(rootPath, "tle"))

        if not os.path.exists(self.tleFilePath):
            open(self.tleFilePath, "a").close()
        
        try:
            with open(self.tleFilePath, "r") as file:
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

                with open(self.tleFilePath, "wb") as file:
                    file.write(
                        f"{now.year} {self._getDays(now)}\n".encode('utf-8'))
                    file.write(
                        get("http://www.celestrak.com/NORAD/elements/weather.txt").content)

        except exceptions.ConnectionError:
            return False

        except Exception as e:
            return False

        return True

    def _getDays(self, date: datetime) -> int:
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
    
    def _load_cache(self):
        self.tles = {}

        tleFile = open(self.tleFilePath, "r")
        lines = tleFile.readlines()
        tleFile.close()

        for i in range((len(lines) - 1) // 3):
            satelliteName = lines[i * 3 + 1].strip()
            line1 = lines[i * 3 + 2]
            line2 = lines[i * 3 + 3]
            self.tles[satelliteName] = (line1, line2)

    def get_tle(self, satelliteName: str) -> Tle:
        if satelliteName not in self.tles.keys():
            raise KeyError("Incorect satellite name")
        return Tle(platform=satelliteName, line1=self.tles[satelliteName][0], line2=self.tles[satelliteName][1])
