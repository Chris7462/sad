# RTK GNSS reading with dual-antenna heading.
# Flat equivalent of sensor_msgs/NavSatFix plus heading, matching sad::GNSS.

# Solution status, same values as GpsStatusType in the book code
int8 STATUS_OTHER=-1
int8 STATUS_NOT_EXIST=0
int8 STATUS_SINGLE_POINT=1
int8 STATUS_PSEUDO=2
int8 STATUS_FIXED=4
int8 STATUS_FLOAT=5

std_msgs/Header header

int8 status

float64 latitude        # [deg]
float64 longitude       # [deg]
float64 altitude        # [m]

float64 heading         # [deg], clockwise from north, as stored in the txt file
bool heading_valid
