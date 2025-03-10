# Input data as a multiline string
data = """##PATH-POINTS-START Path
-120.515,-79.515,120,0
-118.924,-78.304,120
-117.332,-77.092,120
-115.741,-75.881,120
-114.15,-74.669,120
-112.558,-73.458,120
-110.967,-72.246,120
-109.376,-71.035,120
-107.784,-69.823,120
-106.193,-68.612,120
-104.601,-67.401,120
-103.01,-66.189,120
-101.419,-64.978,120
-99.827,-63.766,120
-98.236,-62.555,120
-96.645,-61.343,120
-95.053,-60.132,120
-93.462,-58.92,120
-91.871,-57.709,120
-90.279,-56.498,120
-88.688,-55.286,120
-87.097,-54.075,120
-85.505,-52.863,120
-83.914,-51.652,120
-82.323,-50.44,120
-80.731,-49.229,120
-79.14,-48.017,120
-77.549,-46.806,120
-75.957,-45.595,120
-74.366,-44.383,120,0
-72.798,-44.95,120
-71.236,-46.199,120
-69.669,-47.442,120
-68.097,-48.679,120
-66.521,-49.91,120
-64.939,-51.134,120
-63.352,-52.351,120
-61.76,-53.561,120
-60.162,-54.764,120
-58.558,-55.958,120
-56.948,-57.145,120
-55.331,-58.323,120
-53.708,-59.491,120
-52.079,-60.651,120
-50.443,-61.801,120
-48.799,-62.941,120
-47.149,-64.071,120
-45.491,-65.19,120
-43.826,-66.297,120
-42.152,-67.392,120
-40.471,-68.476,120
-38.782,-69.547,120
-37.084,-70.603,120
-35.378,-71.647,120
-33.663,-72.676,120
-31.939,-73.69,120
-30.206,-74.689,120
-28.465,-75.672,120
-26.714,-76.638,120
-24.954,-77.589,120
-23.184,-78.521,120
-21.406,-79.436,120
-19.618,-80.332,120
-17.821,-81.21,120
-16.015,-82.069,120
-14.2,-82.909,120
-12.376,-83.729,120
-10.543,-84.53,120
-8.702,-85.311,120
-6.853,-86.073,120
-4.995,-86.814,120
-3.13,-87.536,120
-1.257,-88.238,120
0.622,-88.922,120
2.509,-89.586,120
4.401,-90.232,120
6.3,-90.859,120
8.205,-91.469,120
10.115,-92.063,120
12.03,-92.639,120
13.949,-93.201,120
15.874,-93.747,120
17.801,-94.281,120
19.733,-94.799,120
21.667,-95.307,120
23.605,-95.801,120
25.545,-96.286,120
27.488,-96.76,120
29.433,-97.226,120
31.381,-97.682,120
33.329,-98.132,120
35.28,-98.574,120
37.232,-99.011,120
39.185,-99.442,120
41.139,-99.868,120
43.094,-100.291,120
45.049,-100.71,120
47.006,-101.126,120
48.962,-101.539,120
50.919,-101.951,120
52.877,-102.362,120
54.834,-102.772,120
56.792,-103.181,120
59.755,-103.801,120,0
59.755,-103.801,0,0
#PATH.JERRYIO-DATA {"appVersion":"0.8.3","format":"path.jerryio v0.1","gc":{"robotWidth":30,"robotHeight":30,"robotIsHolonomic":false,"showRobot":false,"uol":1,"pointDensity":2,"controlMagnetDistance":5,"fieldImage":{"displayName":"V5RC 2025 - High Stakes (Skills)","signature":"V5RC 2025 - High Stakes (Skills)","origin":{"__type":"built-in"}},"coordinateSystem":"VEX Gaming Positioning System"},"paths":[{"segments":[{"controls":[{"uid":"cJ8WShr5kQ","x":-120.515,"y":-79.515,"lock":false,"visible":true,"heading":0,"__type":"end-point"},{"uid":"yUL9t4XcJI","x":-73.923,"y":-44.046,"lock":false,"visible":true,"heading":0,"__type":"end-point"}],"speedProfiles":[],"lookaheadKeyframes":[],"uid":"tRWmOflUIZ"},{"controls":[{"uid":"yUL9t4XcJI","x":-73.923,"y":-44.046,"lock":false,"visible":true,"heading":0,"__type":"end-point"},{"uid":"haqxzqckzG","x":-19.712854757929883,"y":-87.7838063439065,"lock":false,"visible":true,"__type":"control"},{"uid":"TtVUCPCc7m","x":6.776293823038397,"y":-92.71202003338898,"lock":false,"visible":true,"__type":"control"},{"uid":"QTCY0Y5xJU","x":59.755,"y":-103.801,"lock":false,"visible":true,"heading":0,"__type":"end-point"}],"speedProfiles":[],"lookaheadKeyframes":[],"uid":"iCmMxrZDmM"}],"pc":{"speedLimit":{"minLimit":{"value":0,"label":"0"},"maxLimit":{"value":600,"label":"600"},"step":1,"from":40,"to":120},"bentRateApplicableRange":{"minLimit":{"value":0,"label":"0"},"maxLimit":{"value":1,"label":"1"},"step":0.001,"from":0,"to":0.1}},"name":"Path","uid":"sN5ZgnVGF5","lock":false,"visible":true}]}""" 
# Split data into lines and process each line
coordinates = []
for line in data.strip().splitlines():
    # Skip lines that don't contain coordinates
    if line.startswith('#'):
        continue
    parts = line.split(',')
    # Take only the first two elements as coordinates and convert them to floats
    coordinates.append((float(parts[0]), float(parts[1])))

 

# Print or use the coordinates as needed
print(coordinates)