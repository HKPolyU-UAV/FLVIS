#include <iostream>
#include <cmath>
#include <common.h>

#include "yamlRead.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

#define YAML_PATH "/home/gh034/VO/src/VO/launch/test.yaml"

using namespace std;


int main( int argc, char** argv )
{
  Mat3x3 M33=Mat33FromYaml(YAML_PATH,"M_33");
  cout << "M33:" << endl <<M33 << endl;
  Mat4x4 M44=Mat44FromYaml(YAML_PATH,"M_44");
  cout << "M44:" << endl <<M44 << endl;
  Mat K=cameraMatrixFromYamlIntrinsics(YAML_PATH);
  cout << "K:" << endl <<K << endl;
  Mat dist=distCoeffsFromYaml(YAML_PATH);
  cout << "dist:" << endl <<dist << endl;
  double d=getDoubleVariableFromYaml(YAML_PATH,"dd");
  cout << "d:" << endl << d << endl;
  int i=getIntVariableFromYaml(YAML_PATH,"ii");
  cout << "i:" << endl << i << endl;
}


