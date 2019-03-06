#include <tf_mapping/helper.h>


namespace tf_mapping
{

// assumees input: str=marker_id1
int getNumOfString(std::string str)
{
 return std::stoi(str.substr (9,str.length()) ); 
}

}