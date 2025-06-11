Hypack data info

Survey = Survey data from testing
Outline = ROI survey data
Path = Waypoint path


File Nomenclature:
Folder name denotes test #

"surveytype""Letter""Number"
example: outlineA2

"surveytype" denotes data type (survey,outline,path)
"Letter" denotes the data sets family corrilation
example surveyA2,pathA2,outlineA1 are all used to survey the same area

"Number" denotes the version
example surveyA1 was the first survey, surveyA2 was the second survey



Misc:
Depth1 = High frequency data (reads sea bed)
Depth2 = Low frequency data (penetrates sea bed)


New file format for Hypack Survey (units have all be converted to metric for all files);

Time		HH:MM:SS:MS
GPSLongitude	DD
GPSLatitude	DD
RawDepth1	m
RawDepth2	m
CorrDepth1	m
CorrDepth1	m
Speed		m/s
Pitch		Deg
Roll		Deg
COG 		Deg		Course over ground (heading)
GPS mode	1-5
GPS Elevation	m
Dop		0.0		Dilution of precision
DOL 		m		Distance off line (X-track error)
DBL		m		Distance from beginning of line
NumSats		0		Number of sats
Epoch				(I think its another time unit done quote me tho)
Event				(Event identifier, we dont use it)
