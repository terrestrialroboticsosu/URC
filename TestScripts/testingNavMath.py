import math
earthRadius = 6371000

def convert_to_decimal(coord_str):
    # Remove symbols and split by spaces or punctuation
    for char in "°'\"":
        coord_str = coord_str.replace(char, ' ')
    parts = coord_str.split()
    
    # Extract Degrees, Minutes, Seconds
    deg = float(parts[0])
    mins = float(parts[1]) if len(parts) > 1 else 0
    secs = float(parts[2]) if len(parts) > 2 else 0
    direction = parts[-1].upper() if parts[-1].isalpha() else ""

    decimal = deg + (mins / 60) + (secs / 3600)
    
    # Flip sign for South or West
    if direction == 'W' or direction == 'S':
        decimal *= -1
    return decimal

def calculateDistance(lat1,lon1,lat2,lon2):
    lat1 = lat1*2*math.pi/360
    lat2 = lat2*2*math.pi/360
    lon1 = lon1*2*math.pi/360
    lon2 = lon2*2*math.pi/360
    theta = 2*math.asin(math.sqrt(
        math.sin((lat2-lat1)/2)**2 + 
        math.cos(lat1)*math.cos(lat2)*math.sin((lon2-lon1)/2)**2
    ))
    distance = earthRadius * theta
    return distance

def calculateHeading(lat1,lon1,lat2,lon2):
    lat1 = lat1*2*math.pi/360
    lat2 = lat2*2*math.pi/360
    lon1 = lon1*2*math.pi/360
    lon2 = lon2*2*math.pi/360
    deltaLon = lon2-lon1
    xC = math.sin(deltaLon)*math.cos(lat2)
    yC = math.cos(lat1)*math.sin(lat2)-math.sin(lat1)*math.cos(lat2)*math.cos(deltaLon)
    beta = math.atan2(xC,yC)
    betaDeg = beta*360/2/math.pi
    return (betaDeg+360) % 360

print("Enter coordinates in Degrees: ")
lat1 = convert_to_decimal(input("lat1: "))
lon1 = convert_to_decimal(input("lon1: "))
lat2 = convert_to_decimal(input("lat2: "))
lon2 = convert_to_decimal(input("lon2: "))

distance = calculateDistance(lat1,lon1,lat2,lon2)
heading = calculateHeading(lat1,lon1,lat2,lon2)

print("Report: ")
print(f"Distance: {distance:.2f} m")
print(f"Heading: {heading:.2f}°")
