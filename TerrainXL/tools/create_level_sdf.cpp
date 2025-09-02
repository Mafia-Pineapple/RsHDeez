#include <iostream>
#include <fstream>
using namespace std;

int main() {

    int buffersize = 2000;

    ofstream myfile;
    myfile.open("../src/models/large_scan_gen_sdf/large_scan_gen_sdf.sdf");
    myfile << "<?xml version=\"1.0\"?>\n";
    myfile << "<sdf version=\"1.8\">\n";
    myfile << "\t<model name=\"terrain_xl\">\n";

    //generate models

    for (short x = 0; x < 16; x++)
    {
        for (short y = 0; y < 16; y++)
        {
            myfile << "\t\t<model name=\"tile_" << x << "_" << y << "\">\n";
            myfile << "\t\t\t<pose>" << y*500 << " " << -x*500 << " 0 0 0 0</pose>\n";
            myfile << "\t\t\t<static>true</static>\n";
            myfile << "\t\t\t<link name=\"terrain_tile_" << x << '_' << y << "\">\n";
            myfile << "\t\t\t\t<visual name=\"visual\">\n";
            myfile << "\t\t\t\t\t<material>\n";
            myfile << "\t\t\t\t\t\t<ambient>0.1 0.1 0.1 0.1</ambient>\n";
            myfile << "\t\t\t\t\t\t<diffuse>0.1 0.1 0.1 1</diffuse>\n";
            myfile << "\t\t\t\t\t\t<specular>0 0 0 0</specular>\n";
            myfile << "\t\t\t\t\t\t<emissive>0 0 0 1</emissive>\n";
            myfile << "\t\t\t\t\t</material>\n";
            myfile << "\t\t\t\t\t<geometry>\n";
            myfile << "\t\t\t\t\t\t<mesh>\n";
            myfile << "\t\t\t\t\t\t\t<uri>merged_model_" << x << '_' << y << ".stl</uri>\n";
            myfile << "\t\t\t\t\t\t</mesh>\n";
            myfile << "\t\t\t\t\t</geometry>\n";
            myfile << "\t\t\t\t</visual>\n";

            myfile << "\t\t\t\t<collision name=\"collision\">\n";
            myfile << "\t\t\t\t\t<geometry>\n";
            myfile << "\t\t\t\t\t\t<mesh>\n";
            myfile << "\t\t\t\t\t\t\t<uri>merged_model_" << x << '_' << y << ".stl</uri>\n";
            myfile << "\t\t\t\t\t\t</mesh>\n";
            myfile << "\t\t\t\t\t</geometry>\n";
            myfile << "\t\t\t\t</collision>\n";
            myfile << "\t\t\t</link>\n";
            myfile << "\t\t</model>\n";
        }
        
    }

    myfile << "\t</model>\n";



    
    myfile << "</sdf>";
    myfile.close();
    return 0;
}   