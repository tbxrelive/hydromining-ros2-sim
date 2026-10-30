// SPDX-License-Identifier: Apache-2.0
// Typed scene inspection for confirming ambiguous create/remove acknowledgements.
#include <iostream>
#include <string>
#include <google/protobuf/util/json_util.h>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/scene.pb.h>
#include <gz/transport/Node.hh>

int main(int argc, char **argv)
{
  if (argc!=2) return 2;
  gz::transport::Node node;
  gz::msgs::Empty request;
  gz::msgs::Scene scene;
  bool result=false;
  if (!node.Request("/world/"+std::string(argv[1])+"/scene/info", request, 2000, scene, result) || !result)
  {
    std::cerr << "Scene query unavailable or timed out\n";
    return 1;
  }
  std::string json;
  if (!google::protobuf::util::MessageToJsonString(scene, &json).ok()) return 1;
  std::cout << json << '\n';
}
