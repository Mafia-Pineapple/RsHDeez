to save sightings, publish to:

/homebase/sightings


header files are found in include for root folder of repository

add

${workspaceFolder}/include/**

to your vscode include path if youre using vscode (CTRL+SHIFT+P --> Edit Configurations)

with THIS message format
save defaults to true,
save_dir defaults to /tmp/sightings.csv
you can CHANGE THESE by specifying a custom value for them in your message.

---
uint32 sighting_count

uint16[] animal_type

#these are in global frame
float64[] x
float64[] y
float64[] z

bool save true
string save_dir /tmp/sightings.csv
---


