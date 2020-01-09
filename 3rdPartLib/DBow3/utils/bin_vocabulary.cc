//Second step,creates the vocabulary from the set of features. It can be slow
#include <iostream>
#include <vector>

// DBoW3
#include "DBoW3.h"

// OpenCV
#include <opencv2/core/core.hpp>
using namespace DBoW3;
using namespace std;


// ----------------------------------------------------------------------------

int main(int argc,char **argv)
{


        DBoW3::Vocabulary voc ("voc_all_datasets_orb.yml");
        voc.save("voc_orb.dbow3");




}

