#include <iostream>
#include <fstream>
#include "IkaAgent.h"
#include "osi_sensorview.pb.h"
#include "osi_trafficcommand.pb.h"
#include "osi_trafficupdate.pb.h"
#include "sl45_dynamicsrequest.pb.h"


std::string getFileBuffer(char* fName)
{
    // open file
    std::ifstream osifile(fName, std::ifstream::binary); 
    // get file length
    osifile.seekg (0, osifile.end);
    int file_size = osifile.tellg();
    osifile.seekg (0, osifile.beg);

    std::cout << "Reading: " << fName << " with file_size: " << file_size << "\n";
    // read complete trace into string byte buffer    
    std::string buffer;
    buffer.resize(file_size);
    osifile.read(&buffer[0],  buffer.size() );    
    osifile.close(); 

    return buffer;
}

/**
 * @brief Reads the next osi3 message in the remaining Byte buffer
 * 
 *  This function checks the first four Bytes for the length of the next osi 
 *  message. Then it extracts the substring of this message and parses the 
 *  Bytes in an osi3 message of type <T>. Finally, the parsed message gets
 *  deleted from the Byte buffer `buf`.
 * @tparam T an osi3 class which should be extracted from the Byte buffer
 * @param buf Byte buffer that holds the remaining trace that shall be parsed.
 * @param osi_msg The message that should be filled. Passed by reference.
 * @return true If an osi3 message could be extracted.
 * @return false If either the remaining buffer `buf` is too short, or the
 *               extraction of the osi3 message failed.
 */
template <class T>
bool nextOsiMsg(std::string &buf, T &osi_msg)
{
    // check if remaining buffer is long enough
    unsigned long int msg_size = 0;
    if (buf.size() < sizeof(msg_size)) return false;  
    
    // extract the length of the next osi3 message. This is done by little
    // Endian encoding. The first four Bytes depict the message size.
    std::string msg_len = buf.substr(0,4);
    msg_size += (((unsigned long int)((unsigned char)msg_len[0])) << 0);
    msg_size += (((unsigned long int)((unsigned char)msg_len[1])) << 8);
    msg_size += (((unsigned long int)((unsigned char)msg_len[2])) << 16);
    msg_size += (((unsigned long int)((unsigned char)msg_len[3])) << 24);

    bool parsed = osi_msg.ParseFromString(buf.substr(4,msg_size));

    // Delete the four Bytes that depict the message size and the message itself
    buf.erase(0, msg_size+4);
    return parsed;
}

int main(int argc, char *argv[]) 
{
    // Initialize required osi3 messages and driver model
    osi3::SensorView sv;
    osi3::TrafficCommand tc;
    osi3::TrafficUpdate up;
    up.add_update(); // make sure field can be accessed
    setlevel4to5::DynamicsRequest dr;
    IkaAgent test_agent;
    
    // Get complete buffers for sensor view and traffic command
    std::string buffer_sv = getFileBuffer(argv[1]);
    std::string buffer_tc = getFileBuffer(argv[2]);

    // Search for first sv and tc
    bool found_sv = nextOsiMsg(buffer_sv, sv);
    bool found_tc = nextOsiMsg(buffer_tc, tc);

    // Variables for time stamp and delta t
    double dt = 0.01; //first estimate
    double last_t;
    double t = double(sv.timestamp().seconds())
             + double(sv.timestamp().nanos())/1000000000;
    
    // Loop over the sensor view buffer as long as a sensor is found
    int sv_count = 0;
    while (found_sv)
    {
        std::cout << "Senvor view count (including empty): " << sv_count++ << "\n";
        // Perform open loop step of the driver model
        if (sv.has_global_ground_truth())
        {
            // Check if new traffic command was found. 
            if (found_tc && tc.action_size()>0)
                std::cout << "new traffic command with dest.: " 
                    << tc.action(0).acquire_global_position_action().position().x()
                    << ","
                    << tc.action(0).acquire_global_position_action().position().y()
                    << "\n";
            t = double(sv.timestamp().seconds())
              + double(sv.timestamp().nanos())/1000000000;
            test_agent.step(t, dt, sv, tc, up, dr);     
            std::cout << "new traffic update: " 
                    << up.update(0).base().position().x() << "," 
                    << up.update(0).base().position().y() << "\n";

            last_t = t; // shift processed time stamp for delta t calculation
        }
        // When another sensor view was found, make sure the loop continues
        // and calculate the delta t between new and old sensor view.
        found_sv = nextOsiMsg(buffer_sv, sv);
        if (found_sv)
        {
            t = double(sv.timestamp().seconds())
              + double(sv.timestamp().nanos())/1000000000;
            dt = t - last_t;
        }
        // When no new traffic command was found, clear the osi3 message
        found_tc = nextOsiMsg(buffer_tc, tc);
        if (!found_tc) tc.Clear();
    }
}
