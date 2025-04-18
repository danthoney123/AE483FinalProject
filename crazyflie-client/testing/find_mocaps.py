# IMPORTS
from flight_tools import *
import asyncio

ip_address = '128.174.245.190'

obj = MarkerdeckScan()
asyncio.run(obj.markerdeck_scan(ip_address))