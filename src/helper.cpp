/**************************************************************************//**
   @author  Markus Lamprecht
   @date    March 2019
   @link    www.simact.de/about_me
   @Copyright (c) 2019 Markus Lamprecht. BSD
 *****************************************************************************/
#include <tf_mapping/helper.h>


namespace tf_mapping
{

// assumees input: str=marker_id1
int getNumOfString(std::string str)
{
 return std::stoi(str.substr (9,str.length()) ); 
}

}