#include <tf_mapping/helper.h>
#include <sstream> 

namespace tf_mapping
{

int getNumOfString(std::string str)
{
    std::stringstream ss;     
  
    /* Storing the whole string into string stream */
    ss << str; 
  
    /* Running loop till the end of the stream */
    std::string temp; 
    int found; 
    while (!ss.eof()) { 
  
        /* extracting word by word from stream */
        ss >> temp; 
  
        /* Checking the given word is integer or not */
        if (std::stringstream(temp) >> found) 
            std::cout << found << " "; 
  
        /* To save from space at the end of string */
        temp = ""; 
    } 
    return found;
}

}