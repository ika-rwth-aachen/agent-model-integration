#include <iostream>
#include <fstream>
#include "IkaAgent.h"
#include "osi_sensorview.pb.h"
#include "osi_trafficcommand.pb.h"
#include "osi_trafficupdate.pb.h"
#include "sl45_dynamicsrequest.pb.h"

std::string readOsiBuffer(char* fname)
{
    std::ifstream sv_file(fname, std::ifstream::binary);   
    // read the size
    int size = 0;        
    sv_file.read(reinterpret_cast<char *>(&size), sizeof(size)  );
    
    // Allocate a string, make it large enough to hold the input
    std::string buffer;
    buffer.resize(size);
    
    // read the text into the string
    sv_file.read(&buffer[0],  buffer.size() );
    sv_file.close();    
    return buffer;
}

int main(int argc, char *argv[]) 
{
    osi3::SensorView sv;
    osi3::TrafficCommand tc;
    osi3::TrafficUpdate up;
    setlevel4to5::DynamicsRequest dr;
    IkaAgent testAgent;
    
    
    std::string bufferSV = readOsiBuffer(argv[1]);
    std::string bufferTC = readOsiBuffer(argv[2]);
    //std::cout << "args: " << argv[1] << argv[2] << "\n" ;
    
     
    //bool suc_read = sv.ParseFromArray(buffer.data(), buffer.size());
    bool suc_read_sv = sv.ParseFromString(bufferSV);
    bool suc_read_tc = tc.ParseFromString(bufferTC);
    //std::cout << "\nmoving object count: " << sv.global_ground_truth().moving_object_size() << std::endl;
    //std::cout << "\nTrafficCommand: " << tc.action(0).has_acquire_global_position_action() << std::endl;
    if (suc_read_sv && suc_read_tc)
    {
        up.add_update();
        testAgent.step(0, 0.01, sv, tc, up, dr);
    }
    

    return 0;
}
