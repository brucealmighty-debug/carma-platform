/*
 * Copyright (C) 2020 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

/**
 * This file contains unit tests which can be used like scripts to convert lanelet2 map files.
 */

#include <gmock/gmock.h>
#include <carma_wm_ctrl/MapConformer.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_extension/traffic_rules/CarmaUSTrafficRules.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/utility/utilities.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <lanelet2_extension/io/autoware_osm_parser.h>
#include <carma_wm/CARMAWorldModel.h>
#include <unordered_set>
#include "TestHelpers.h"

using ::testing::_;
using ::testing::A;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Return;
using ::testing::ReturnArg;

namespace carma_wm_ctrl
{
/**
 * @brief Unit test for combining 2 adjacent lanelets into one.
 *
 * This script will take in a lanelet2 osm map and then try to merge the specified lanelet with its left or right
 * neighbors until either it forms a loop, it reaches an intersection where there are multiple possible directions to
 * travel, or the neighbor relationship changes. Since the update is done by assigning the bounds, routing should still
 * work, but not every edge case is covered. If the map is not a loop, it is likely there might be orphaned lanelets.
 *
 * See the UNIT TEST ARGUMENTS section below to configure this unit test.
 * The unit test is normally disabled. To enable it, removed the "DISABLED_" from the test name.
 * To run the unit test call
 *   catkin_make run_tests_carma_wm_ctrl_gtest_map-tools
 *
 * This unit test will output the new map as <map_name>.osm.combined.osm
 * Additionally, two routing graphs will be created in the test/resource folder one before the changes routing_graph.viz
 * and one after final_routing_graph.viz.
 *
 */
TEST(MapTools, DISABLED_combine_lanes)  // Remove DISABLED_ to enable unit test
{
  // Side of the lane
  enum SIDE
  {
    LEFT,
    RIGHT
  };

  ///////////
  // UNIT TEST ARGUMENTS
  ///////////

  // File to process. Path is relative to test folder
  std::string file = "resource/ATEF_pretty.osm";
  // Id of lanelet to start combing from
  lanelet::Id starting_id = 113;
  // Side to combine. If LEFT than the left lanelet left edge will be used for the left edge of the right lanelet
  // (intially the starting_id lanelet). Vice-versa for RIGHT.
  SIDE merged_side = SIDE::LEFT;

  ///////////
  // START OF LOGIC
  ///////////

  // Write new map to file
  int projector_type = 0;
  std::string target_frame;
  lanelet::ErrorMessages load_errors;
  // Parse geo reference info from the original lanelet map (.osm)
  lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);

  lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());

  lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);

  if (map->laneletLayer.size() == 0)
  {
    FAIL() << "Input map does not contain any lanelets";
  }

  carma_wm::CARMAWorldModel cwm;
  cwm.setMap(map);
  auto routing_graph = cwm.getMapRoutingGraph();
  routing_graph->exportGraphViz("resource/routing_graph.viz");

  std::unordered_set<lanelet::Id> visited_lanelets;
  std::unordered_set<lanelet::Id> lanelets_for_removal;

  // The algorithm would be better off assuming a loop.
  // Given a lanelet identify left/right relations
  // Get next lanelet based on following relation.
  lanelet::Lanelet current_lanelet;
  try
  {
    current_lanelet = map->laneletLayer.get(starting_id);  // TODO for now assume the first lanelet is the base lanelet
  }
  catch (const lanelet::NoSuchPrimitiveError& e)
  {
    FAIL() << "The specified starting lanelet Id of " << starting_id << " does not exist in the provided map.";
  }

  while (visited_lanelets.find(current_lanelet.id()) == visited_lanelets.end())
  {
    visited_lanelets.emplace(current_lanelet.id());  // Add current lanelet to set of explored lanelets

    auto left_ll = routing_graph->left(current_lanelet);  // Check routable left
    if (!left_ll)
    {
      left_ll = routing_graph->adjacentLeft(current_lanelet);  // Check non-routable left
    }
    auto right_ll = routing_graph->right(current_lanelet);
    if (!right_ll)
    {
      right_ll = routing_graph->adjacentRight(current_lanelet);
    }

    bool has_left = (!!left_ll) && (visited_lanelets.find(left_ll->id()) == visited_lanelets.end());
    bool has_right = (!!right_ll) && (visited_lanelets.find(right_ll->id()) == visited_lanelets.end());

    if (has_left && merged_side == SIDE::LEFT)
    {  // Lanelet on left

      lanelet::Lanelet mutable_left = map->laneletLayer.get(left_ll->id());
      current_lanelet.setLeftBound(mutable_left.leftBound());
      lanelets_for_removal.emplace(mutable_left.id());
    }
    else if (has_right && merged_side == SIDE::RIGHT)
    {  // Lanelet on right

      lanelet::Lanelet mutable_right = map->laneletLayer.get(right_ll->id());
      current_lanelet.setRightBound(mutable_right.rightBound());
      lanelets_for_removal.emplace(mutable_right.id());
    }
    else
    {
      std::cerr << "WARNING: " << current_lanelet.id()
                << " which was being processed does not have combinable neighbors" << std::endl;
      break;
    }

    // Get next lanelet
    auto following_set = routing_graph->following(current_lanelet, false);
    if (following_set.size() > 1)
    {
      std::cerr << "Cannot combine lanelets when there are multiple followers. Your map is not a single loop. Ending "
                   "update and saving current map state."
                << std::endl;
      break;
    }
    else if (following_set.size() == 1)
    {
      auto lanelet = following_set[0];
      current_lanelet = map->laneletLayer.get(lanelet.id());
    }
  }

  // Build new map from modified data
  std::vector<lanelet::Lanelet> new_lanelets;

  // Iterate over all lanelets and add only those not in the excluded set to the new map set.
  for (auto lanelet : map->laneletLayer)
  {
    if (lanelets_for_removal.find(lanelet.id()) != lanelets_for_removal.end())
    {
      continue;
    }
    lanelet::Lanelet mutable_ll = map->laneletLayer.get(lanelet.id());
    new_lanelets.emplace_back(mutable_ll);
  }

  std::vector<lanelet::Area> areas;
  for (auto area : map->areaLayer)
  {
    lanelet::Area mutable_area = map->areaLayer.get(area.id());
    areas.emplace_back(mutable_area);
  }

  auto new_map = lanelet::utils::createMap(new_lanelets, areas);

  auto new_routing_graph = lanelet::routing::RoutingGraph::build(*new_map, **(cwm.getTrafficRules()));
  new_routing_graph->exportGraphViz("resource/final_routing_graph.viz");

  // Write new map to file
  std::string new_file = file + ".combined.osm";
  lanelet::ErrorMessages write_errors;

  lanelet::write(new_file, *new_map, local_projector, &write_errors);

  if (write_errors.size() > 0)
  {
    std::cerr << "Errors occured while writing the map! Output file located at " << new_file << std::endl;
  }
  else
  {
    std::cerr << "Map written without errors to: " << new_file << std::endl;
  }
  for (auto msg : write_errors)
  {
    std::cerr << "Write Error: " << msg << std::endl;
  }

  // Copy over georeference tag
  pugi::xml_document doc;
  auto result = doc.load_file(new_file.c_str());
  if (!result)
  {
    std::cerr << "Failed to update georeference tag you may need to manually" << std::endl;
  }
  auto osm_node = doc.child("osm");
  auto first_osm_child = osm_node.first_child();
  auto geo_ref_node = osm_node.insert_child_before("geoReference", first_osm_child);
  geo_ref_node.text().set(target_frame.c_str());
  doc.save_file(new_file.c_str());
}

TEST(MapTools, DISABLED_split_lanes)  // Remove DISABLED_ to enable unit test
{
  ///////////
  // UNIT TEST ARGUMENTS
  ///////////

  // File to process. Path is relatice to test folder
  std::string file = "resource/ATEF_pretty.osm";
  // Id of lanelet to start combing from
  lanelet::Id starting_id = 113;
  // List of participants allowed to pass the centerline from the left
  std::vector<std::string> left_participants = { lanelet::Participants::Vehicle };
  // List of participants allowed to pass the centerline from the right
  std::vector<std::string> right_participants = { lanelet::Participants::Vehicle };
  // Default road marking. Normally this should be consistent with the left/right participants, 
  // but if you wish to decouple the road marking from regulation that will still work.
  // Line Marking Type
  std::string type_string = lanelet::AttributeValueString::LineThin;
  // Line Marking SubType
  std::string sub_type_string = lanelet::AttributeValueString::Dashed;

  ///////////
  // START OF LOGIC
  ///////////

  // Write new map to file
  int projector_type = 0;
  std::string target_frame;
  lanelet::ErrorMessages load_errors;
  // Parse geo reference info from the original lanelet map (.osm)
  lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);

  lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());

  lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);

  if (map->laneletLayer.size() == 0)
  {
    FAIL() << "Input map does not contain any lanelets";
  }

  // Overrite centerlines to ensure there is an equal number of points in the centerline and bounds
  lanelet::utils::overwriteLaneletsCenterline(map, true);

  carma_wm::CARMAWorldModel cwm;
  cwm.setMap(map);
  auto routing_graph = cwm.getMapRoutingGraph();
  routing_graph->exportGraphViz("resource/routing_graph.viz");

  std::unordered_set<lanelet::Id> visited_lanelets;
  std::unordered_set<lanelet::Id> lanelets_for_removal;
  std::unordered_set<lanelet::Id> regulations_for_removal;

  std::vector<lanelet::Lanelet> lanelets_to_add;
  std::vector<lanelet::RegulatoryElement> regulations_to_add;


  // The algorithm would be better off assuming a loop.
  // Given a lanelet identify left/right relations
  // Get next lanelet based on following relation.
  lanelet::Lanelet current_lanelet;
  try
  {
    current_lanelet = map->laneletLayer.get(starting_id);  // TODO for now assume the first lanelet is the base lanelet
  }
  catch (const lanelet::NoSuchPrimitiveError& e)
  {
    FAIL() << "The specified starting lanelet Id of " << starting_id << " does not exist in the provided map.";
  }

  while (visited_lanelets.find(current_lanelet.id()) == visited_lanelets.end())
  {
    visited_lanelets.emplace(current_lanelet.id());  // Add current lanelet to set of explored lanelets

    // Create deep copy of centerline
    lanelet::LineString3d centerline(lanelet::utils::getId()); // New ID
    if (current_lanelet.centerline3d().inverted()) { // Apply inversion
      centerline = centerline.invert();
    }
    for (auto point : current_lanelet.centerline3d().basicLineString()) { // Copy points
      centerline.push_back(lanelet::Point3d(lanelet::utils::getId(), point));
    }

    // Assign user specified attributes to centerline
    centerline.attributes()[lanelet::AttributeName::Type] = type_string;
    centerline.attributes()[lanelet::AttributeName::Subtype] = sub_type_string;
   
    // Build new lanelets
    lanelet::Lanelet left_ll(lanelet::utils::getId(), current_lanelet.leftBound3d(), centerline,
                             current_lanelet.attributes());

    lanelet::Lanelet right_ll(lanelet::utils::getId(), current_lanelet.leftBound3d(), centerline,
                              current_lanelet.attributes());

    // TODO reconsider this logic. 
    // // Remove right control line from left lanelet
    // auto left_control_lines = left_ll.regulatoryElementsAs<lanelet::PassingControlLine>();
    
    // for (auto reg : left_control_lines) {
    //   if (reg->find<lanelet::ConstLineString3d>(left_ll.rightBound3d().id())) {
    //     if (!left_ll.removeRegulatoryElement(reg)) {
    //       throw std::invalid_argument("Failed to remove regulatory element from lanelet");
    //     }
    //   }
    // }

    // // Remove left control line from right lanelet
    // auto right_control_lines = right_ll.regulatoryElementsAs<lanelet::PassingControlLine>();
    
    // for (auto reg : right_control_lines) {
    //   if (reg->find<lanelet::ConstLineString3d>(right_ll.leftBound3d().id())) {
    //     if (!right_ll.removeRegulatoryElement(reg)) {
    //       throw std::invalid_argument("Failed to remove regulatory element from lanelet");
    //     }
    //   }
    // }

    // Add a passing control line for the centerline and apply to both lanelets
    std::shared_ptr<lanelet::PassingControlLine> control_line_ptr(new lanelet::PassingControlLine(
        lanelet::PassingControlLine::buildData(lanelet::utils::getId(), { centerline }, left_participants, right_participants)));

    left_ll.addRegulatoryElement(control_line_ptr);
    right_ll.addRegulatoryElement(control_line_ptr);

    // TODO here
    // How to add the passing control lines we had previously?
    // Find all references to old lanelet
    lanelet::utils::query::findReferences();



    // Iterate over all regulatory elements in the map.
    // If they refer to the initial lanelet then replace them with a new regulatory element that references to both lanelets
    // TODO we need to first make sure that the initial passing control lines
    for (auto reg : map->regulatoryElementLayer) {
      // Check if current regulation references old lanelet
      if (reg->find<lanelet::ConstLanelet>(current_lanelet.id())) {
        // If Yes
        // -- Then mark regulation for removal and remake with new lanelets. Remember to remove it from the old lanelet
        for (auto param : reg->getParameters()) {
          auto parameter = std::get<1>(param);
        }
      }
      // If No then continue
    }
    // TODO waiting on advice from lanelet2 maintainers on best practice for element linking


    // Get next lanelet
    auto following_set = routing_graph->following(current_lanelet, false);
    if (following_set.size() > 1)
    {
      std::cerr << "Cannot combine lanelets when there are multiple followers. Your map is not a single loop. Ending "
                   "update and saving current map state."
                << std::endl;
      break;
    }
    else if (following_set.size() == 1)
    {
      auto lanelet = following_set[0];
      current_lanelet = map->laneletLayer.get(lanelet.id());
    }
  }

  // Build new map from modified data
  std::vector<lanelet::Lanelet> new_lanelets;

  // Iterate over all lanelets and add only those not in the excluded set to the new map set.
  for (auto lanelet : map->laneletLayer)
  {
    if (lanelets_for_removal.find(lanelet.id()) != lanelets_for_removal.end())
    {
      continue;
    }
    lanelet::Lanelet mutable_ll = map->laneletLayer.get(lanelet.id());
    new_lanelets.emplace_back(mutable_ll);
  }

  std::vector<lanelet::Area> areas;
  for (auto area : map->areaLayer)
  {
    lanelet::Area mutable_area = map->areaLayer.get(area.id());
    areas.emplace_back(mutable_area);
  }

  auto new_map = lanelet::utils::createMap(new_lanelets, areas);

  auto new_routing_graph = lanelet::routing::RoutingGraph::build(*new_map, **(cwm.getTrafficRules()));
  new_routing_graph->exportGraphViz("resource/final_routing_graph.viz");

  // Write new map to file
  std::string new_file = file + ".combined.osm";
  lanelet::ErrorMessages write_errors;

  lanelet::write(new_file, *new_map, local_projector, &write_errors);

  if (write_errors.size() > 0)
  {
    std::cerr << "Errors occured while writing the map! Output file located at " << new_file << std::endl;
  }
  else
  {
    std::cerr << "Map written without errors to: " << new_file << std::endl;
  }
  for (auto msg : write_errors)
  {
    std::cerr << "Write Error: " << msg << std::endl;
  }

  // Copy over georeference tag
  pugi::xml_document doc;
  auto result = doc.load_file(new_file.c_str());
  if (!result)
  {
    std::cerr << "Failed to update georeference tag you may need to manually" << std::endl;
  }
  auto osm_node = doc.child("osm");
  auto first_osm_child = osm_node.first_child();
  auto geo_ref_node = osm_node.insert_child_before("geoReference", first_osm_child);
  geo_ref_node.text().set(target_frame.c_str());
  doc.save_file(new_file.c_str());
}

}  // namespace carma_wm_ctrl