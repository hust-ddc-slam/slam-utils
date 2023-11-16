#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <iostream>
#include <vector>
#include <string>

using namespace std;


int main(int argc, char **argv) {
    // The directory containing the split rosbags
    std::string directory_path = "/home/larrydong/Data/SubT-Dataset-Selected/SubT_MRS_Final_Challenge_UGV1.zip-20231116T024821Z-001/";
    
    // The final merged rosbag file
    rosbag::Bag merged_bag;
    merged_bag.open("/home/larrydong/merged.bag", rosbag::bagmode::Write);

    // Get a list of all bag files in the directory
    std::vector<std::string> bag_files;
    for(int i=0; i<40; ++i){
        string filename = directory_path + to_string(i) + ".bag";
        bag_files.push_back(filename);
    }

    // Iterate over each bag file and write its messages to the merged bag
    for (const std::string& bag_file : bag_files) {
        std::cout << "Processing " << bag_file << std::endl;
        rosbag::Bag bag;
        bag.open(bag_file, rosbag::bagmode::Read);

        rosbag::View view(bag);
        BOOST_FOREACH(rosbag::MessageInstance const m, view) {
            merged_bag.write(m.getTopic(), m.getTime(), m, m.getConnectionHeader());
        }
        
        bag.close();
    }
    
    merged_bag.close();
    
    std::cout << "Merge complete!" << std::endl;
    return 0;
}

