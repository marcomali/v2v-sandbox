#ifndef SUMO_XML_PARSER_H
#define SUMO_XML_PARSER_H

#include <libxml/parser.h>
#include <libxml/xpath.h>
#include <libxml/xpathInternals.h>
#include <libxml/tree.h>
#include <string>

namespace ns3 {
  std::string sumo_folder = "src/automotive/examples/sumo-files/";
  std::string mob_trace = "cars.rou.xml";
  std::string sumo_config ="src/automotive/examples/sumo-files/map.sumo.cfg";

  int XML_rou_count_vehicles(xmlDocPtr doc);
}

#endif
