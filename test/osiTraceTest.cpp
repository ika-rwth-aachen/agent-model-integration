#include <iostream>
#include <fstream>
#include "IkaAgent.h"
#include "osi_sensorview.pb.h"
#include "osi_trafficcommand.pb.h"
#include "osi_trafficupdate.pb.h"
#include "sl45_dynamicsrequest.pb.h"


std::string getFileBuffer(char* fName)
{
    std::ifstream osifile(fName, std::ifstream::binary); 
    osifile.seekg (0, osifile.end);
    int file_size = osifile.tellg();
    osifile.seekg (0, osifile.beg);

    std::cout << "Reading: " << fName << " with file_size: " << file_size << "\n";
    std::string buffer;
    buffer.resize(file_size);
    osifile.read(&buffer[0],  buffer.size() );    
    osifile.close(); 

    return buffer;
}

template <class T>
bool nextOsiMsg(std::string &buf, T &osi_msg)
{
    unsigned long int msg_size = 0;
    if (buf.size() < sizeof(msg_size)) return false;  
    
    std::string msg_len = buf.substr(0,4);
    msg_size += (((unsigned long int)((unsigned char)msg_len[0])) << 0);
    msg_size += (((unsigned long int)((unsigned char)msg_len[1])) << 8);
    msg_size += (((unsigned long int)((unsigned char)msg_len[2])) << 16);
    msg_size += (((unsigned long int)((unsigned char)msg_len[3])) << 24);
    //std::cout << "msg_size: " << msg_size << "\n";

    bool parsed = osi_msg.ParseFromString(buf.substr(4,msg_size));

    buf.erase(0, msg_size+4);
    return parsed;
}

int main(int argc, char *argv[]) 
{
    osi3::SensorView sv;
    osi3::TrafficCommand tc;
    osi3::TrafficUpdate up;
    up.add_update(); // make sure field can be accessed
    setlevel4to5::DynamicsRequest dr;
    IkaAgent test_agent;
    
    // get complete buffers for sensor view and traffic command
    std::string buffer_sv = getFileBuffer(argv[1]);
    std::string buffer_tc = getFileBuffer(argv[2]);

    // search for first sv and tc
    bool found_sv = nextOsiMsg(buffer_sv, sv);
    bool found_tc = nextOsiMsg(buffer_tc, tc);
    double dt = 0.01; //first estimate
    double last_t;
    double t = sv.timestamp().seconds()+ double(sv.timestamp().nanos())/1000000000;
    while (found_sv)
    {
        //std::cout << "sv time stamp: " << sv.timestamp().seconds()+ double(sv.timestamp().nanos())/1000000000 << std::endl;
        if (found_tc)
            std::cout << "new traffic command with dest.: " 
                << tc.action(0).acquire_global_position_action().position().x() << ","
                << tc.action(0).acquire_global_position_action().position().y()
                << "\n";
        // perform open loop step 
        test_agent.step(t, dt, sv, tc, up, dr);     
        std::cout << "new update: " << up.update(0).base().position().x() << "," << up.update(0).base().position().y() << "\n";
        last_t = t;   
        found_sv = nextOsiMsg(buffer_sv, sv);
        if (found_sv)
        {
            t = sv.timestamp().seconds()+ double(sv.timestamp().nanos())/1000000000;
            dt = t - last_t;
        }
        found_tc = nextOsiMsg(buffer_tc, tc);
        if (!found_tc) tc.Clear();
    }
}
