#!/usr/bin/env python3

import sys
import datetime
import math
import project11.geodesic
import json

positions = {}

for line in open(sys.argv[1]).readlines():
  parts = line.strip().split(',')
  if len(parts[1]) > 3 and parts[1][-3:] == 'GGA':
    lat_deg = int(parts[3][:2])
    lat_min = float(parts[3][2:])
    lat = lat_deg+lat_min/60.0
    if parts[4] == 'S':
      lat = -lat
    lon_deg = int(parts[5][:3])
    lon_min = float(parts[5][3:])
    lon = lon_deg+lon_min/60.0
    if parts[6] == 'W':
      lon = -lon
    timestamp = datetime.datetime.fromtimestamp(float(parts[0]), datetime.timezone.utc)
    #print(timestamp,lat,lon)
    positions[timestamp] = (lat, lon)

times = positions.keys()
times = sorted(times)

lines = []

current_start_time: datetime.datetime = None
current_distance = None
gap = datetime.timedelta(seconds=10)

for t in times:
  lat,lon = positions[t]
  if current_start_time is None:
    current_start_time = t
    current_distance = 0.0
    min_lat = lat
    max_lat = lat
    min_lon = lon
    max_lon = lon
    last_time = t
    last_lat = lat
    last_lon = lon
  else:
    if t - last_time > gap:
      lines.append({'from': current_start_time.isoformat(), 'to': t.isoformat(), 'distance': current_distance, 'min_lat': min_lat, 'min_lon': min_lon, 'max_lat': max_lat, 'max_lon': max_lon})
      current_start_time = None
    else:
      min_lat = min(min_lat, lat)
      max_lat = max(max_lat, lat)
      min_lon = min(min_lon, lon)
      max_lon = max(max_lon, lon)
      azimuth, distance = project11.geodesic.inverse(math.radians(last_lon), math.radians(last_lat), math.radians(lon), math.radians(lat))
      current_distance += distance
      last_lat = lat
      last_lon = lon
      last_time = t

lines.append({'from': current_start_time.isoformat(), 'to': last_time.isoformat(), 'distance': current_distance, 'min_lat': min_lat, 'min_lon': min_lon, 'max_lat': max_lat, 'max_lon': max_lon})

total_distance = 0.0
for l in lines:
  min_lat = min(min_lat, l['min_lat'])
  min_lon = min(min_lon, l['min_lon'])
  max_lat = min(max_lat, l['max_lat'])
  max_lon = min(max_lon, l['max_lon'])
  total_distance += l['distance']

results = {'lines':lines, 'total': {'from': lines[0]['from'], 'to': lines[-1]['to'], 'distance': total_distance, 'min_lat': min_lat, 'min_lon': min_lon, 'max_lat': max_lat, 'max_lon': max_lon}}

print (json.dumps(results, indent=2))
