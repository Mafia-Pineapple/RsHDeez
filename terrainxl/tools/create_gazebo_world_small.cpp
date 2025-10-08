#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
using namespace std;

int main() {

    string terrainXL_path = "\t\t\t\tmodel://models/large_scan_obj_small\n";
    string tree_path = "\t\t\t\tmodel://models/tree_rs1\n";

    ofstream myfile;
    myfile.open("../src/worlds/earthsmall.sdf");
    myfile << "<?xml version=\"1.0\"?>\n";
    myfile << "<sdf version=\"1.8\">\n";
    myfile << "\t<world name=\"earth\">\n";

    myfile << "\t\t<plugin\n";
    myfile << "\t\t\tfilename=\"ignition-gazebo-physics-system\"\n";
    myfile << "\t\t\tname=\"gz::sim::systems::Physics\">\n";
    myfile << "\t\t</plugin>\n";

    myfile << "\t\t<plugin\n";
    myfile << "\t\t\tfilename=\"ignition-gazebo-user-commands-system\"\n";
    myfile << "\t\t\tname=\"gz::sim::systems::UserCommands\">\n";
    myfile << "\t\t</plugin>\n";

    myfile << "\t\t<plugin\n";
    myfile << "\t\t\tfilename=\"ignition-gazebo-scene-broadcaster-system\"\n";
    myfile << "\t\t\tname=\"gz::sim::systems::SceneBroadcaster\">\n";
    myfile << "\t\t</plugin>\n";

    //generate sun light
    myfile << "\t\t<light name=\"sun\" type=\"directional\">\n";
    myfile << "\t\t\t<cast_shadows>0</cast_shadows>\n";
    myfile << "\t\t\t<pose>0 0 100 0 0 0</pose>\n";
    myfile << "\t\t\t<diffuse>0.8 0.8 0.8 1</diffuse>\n";
    myfile << "\t\t\t<specular>0.8 0.8 0.8 1</specular>\n";
    myfile << "\t\t\t<direction>-0.5 0.0 -0.9</direction>\n";
    myfile << "\t\t\t<intensity>1</intensity>\n";
    myfile << "\t\t</light>\n";

    myfile << "\t\t<gravity>0 0 -9.81</gravity>\n";
    myfile << "\t\t<magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>\n";
    myfile << "\t\t<atmosphere type=\"adiabatic\" />\n";
    myfile << "\t\t<physics name=\"default_physics\" type=\"ignored\">\n";
    myfile << "\t\t\t<max_step_size>0.01</max_step_size>\n";
    myfile << "\t\t\t<real_time_factor>1</real_time_factor>\n";
    myfile << "\t\t\t<real_time_update_rate>100</real_time_update_rate>\n";
    myfile << "\t\t</physics>\n";
    myfile << "\t\t<scene>\n";
    myfile << "\t\t\t<ambient>1.0 1.0 1.0 1</ambient>\n";
    myfile << "\t\t\t<background>0.6 0.8 1.0 1</background>\n";
    myfile << "\t\t\t<shadows>0</shadows>\n";
    myfile << "\t\t\t<grid>0</grid>\n";
    myfile << "\t\t</scene>\n";

    //initialise spherical coordinates somewhere

    myfile << "\t\t<spherical_coordinates>\n";
    myfile << "\t\t\t<latitude_deg>0.0</latitude_deg>\n";
    myfile << "\t\t\t<longitude_deg>0.0</longitude_deg>\n";
    myfile << "\t\t\t<elevation>10.0</elevation>\n";
    myfile << "\t\t\t<heading_deg>0</heading_deg>\n";
    myfile << "\t\t\t<surface_model>EARTH_WGS84</surface_model>\n";
    myfile << "\t\t</spherical_coordinates>\n";



    //////////////////////////
    //Tree placer EXTREME EDITION!!
    //open trees.txt

    fstream fin;

    fin.open("smalltrees.csv", ios::in);
    vector<string> row;
    string line, word, temp;

    unsigned int tree_count = 0;

    while (fin >> temp && fin)
    {
        tree_count++;
        row.clear();

        //read row
        getline(fin, line);

        //break words up
        stringstream s(temp);

        while(getline(s,word,','))
        {
            row.push_back(word);
        }
        //write model headers
        myfile << "\t\t<include>\n";
        myfile << "\t\t\t<uri>\n";
        myfile << tree_path;
        myfile << "\t\t\t</uri>\n";
        myfile << string("\t\t\t<name>tree_") + to_string(tree_count) + "</name>\n";
        myfile << string("\t\t\t<pose>");
        double inv2 = stod(row[0]);
        inv2 = inv2 - 250;

        double inv = stod(row[1]);
        inv = inv - 7750;
        myfile << inv2;
        myfile << ' ';
        myfile << inv;
        myfile << ' ';
        myfile << row[2];
        myfile << " 0 0 0</pose>\n";
        myfile << "\t\t</include>\n\n";
    }

    //levels definition





    //////////////////////////

    myfile << "\t\t<include>\n";
    myfile << "\t\t\t<uri>\n";
    myfile << terrainXL_path;
    myfile << "\t\t\t</uri>\n";
    myfile << "\t\t</include>\n";
    myfile << "\t</world>\n";
    myfile << "</sdf>";
}

