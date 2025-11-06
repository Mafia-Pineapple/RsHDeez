To start, run:
ros2 run homebase homebaser

header files are found in include for root folder of repository, add ${workspaceFolder}/include/** to your vscode include path if youre using vscode (CTRL+SHIFT+P --> Edit Configurations)


        to save sightings, publish custom_msgs::msg::Sightings to /homebase/sightings
(type, name, default)
||||||||||||||||||||||||||||||||||||||||||||
uint32 sighting_count

uint16[] animal_type

#these are in global frame
float64[] x
float64[] y
float64[] z

bool save true
string save_dir /tmp/sightings.csv
||||||||||||||||||||||||||||||||||||||||||||

        For weather service, publish custom_msgs::srv::WeatherService to homebase_weather_report
(type, name, default)
||||||||||||||||||||||||||||||||||||||||||||
--- Request:
float64 lat -33.725117 (these lat long values correspond to the terrain tile's real world location)
float64 lon 150.320997
--- Response:
int32 temperature 20
float32 wind 0
uint16 direction 0
uint16 weather_type 0
string weather_message "Clear sky"
||||||||||||||||||||||||||||||||||||||||||||

(this uses REAL http requests using REAL curl with REAL c++ memory management)


