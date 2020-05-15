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
    current_lanelet = map->laneletLayer.get(starting_id); 
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

// Visitor class which builds a new parameter map that has replaced the specified lanelet id with the provided
// lanelet replacements
class ReplaceLaneletParameterVisitor : public lanelet::RuleParameterVisitor
{
public:
  explicit ReplaceLaneletParameterVisitor(lanelet::RuleParameterMap& output_map, lanelet::Id target_id,
                                          std::vector<lanelet::Lanelet> replacements, lanelet::LaneletMapPtr ll_map)
    : output_map_(output_map), target_id_(target_id), ll_map_(ll_map), replacements_(replacements)
  {
  }

  void operator()(const lanelet::ConstWeakLanelet& wll) override
  {
    if (wll.expired())
    {  // NOLINT
      return;
    }
    lanelet::ConstLanelet llt(wll.lock());
    if (llt.id() == target_id_)  // If this is the lanelet we with to replace then replace it
    {
      if (output_map_.find(role) != output_map_.end())
      {
        for (auto ll : replacements_)
          output_map_[role].push_back(ll);
      }
      else
      {
        auto variant_vector = lanelet::utils::transform(
            replacements_, [](lanelet::Lanelet ll) -> lanelet::RuleParameter { return ll; });
        output_map_.insert({ role, variant_vector });
      }
    }
    else  // If not then copy this lanelet to our output
    {
      lanelet::Lanelet mutable_ll = ll_map_->laneletLayer.get(llt.id());
      if (output_map_.find(role) != output_map_.end())
      {
        output_map_[role].push_back(mutable_ll);
      }
      else
      {
        output_map_.insert({ role, { mutable_ll } });
      }
    }
  }

  lanelet::RuleParameterMap& output_map_;

private:
  lanelet::Id target_id_;
  lanelet::LaneletMapPtr ll_map_;
  std::vector<lanelet::Lanelet> replacements_;
};

TEST(MapTools, split_lanes)  // Remove DISABLED_ to enable unit test
{
  ///////////
  // UNIT TEST ARGUMENTS
  ///////////

  // File to process. Path is relatice to test folder
  std::string file = "/workspaces/carma_ws/carma/src/carma-platform/carma_wm_ctrl/test/resource/SummitPoint_Pretty.osm";
  // Id of lanelet to start combing from
  lanelet::Id starting_id = 100;
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
  routing_graph->exportGraphViz("/workspaces/carma_ws/carma/src/carma-platform/carma_wm_ctrl/test/resource/split_lanes_starting_routing_graph.viz"); // Export the routing graph for debugging

  std::unordered_set<lanelet::Id> visited_lanelets;
  std::unordered_set<lanelet::Id> lanelets_for_removal;

  std::vector<lanelet::Lanelet> lanelets_to_add;
  std::vector<lanelet::RegulatoryElement> regulations_to_add;

  // Starting with the specified lanelet we will iterate over the 
  lanelet::Lanelet current_lanelet;
  try
  {
    current_lanelet = map->laneletLayer.get(starting_id);
  }
  catch (const lanelet::NoSuchPrimitiveError& e)
  {
    FAIL() << "The specified starting lanelet Id of " << starting_id << " does not exist in the provided map.";
  }

  while (visited_lanelets.find(current_lanelet.id()) == visited_lanelets.end())
  {
    visited_lanelets.emplace(current_lanelet.id());  // Add current lanelet to set of explored lanelets
    std::cerr << "1 " << std::endl;
    // Create deep copy of centerline
    lanelet::LineString3d centerline(lanelet::utils::getId());  // New ID
    if (current_lanelet.centerline3d().inverted())
    {  // Apply inversion
      centerline = centerline.invert();
    }
    for (auto point : current_lanelet.centerline3d().basicLineString())
    {  // Copy points
      centerline.push_back(lanelet::Point3d(lanelet::utils::getId(), point));
    }

    std::cerr << "2" << std::endl;

    // Assign user specified attributes to centerline
    centerline.attributes()[lanelet::AttributeName::Type] = type_string;
    centerline.attributes()[lanelet::AttributeName::Subtype] = sub_type_string;

    // Build new lanelets
    lanelet::Lanelet left_ll(lanelet::utils::getId(), current_lanelet.leftBound3d(), centerline,
                             current_lanelet.attributes());

    lanelet::Lanelet right_ll(lanelet::utils::getId(), centerline, current_lanelet.rightBound3d(),
                              current_lanelet.attributes());

    std::cerr << "3" << std::endl;
    // Add a passing control line for the centerline and apply to both lanelets
    std::shared_ptr<lanelet::PassingControlLine> control_line_ptr(
        new lanelet::PassingControlLine(lanelet::PassingControlLine::buildData(lanelet::utils::getId(), { centerline },
                                                                               left_participants, right_participants)));

    left_ll.addRegulatoryElement(control_line_ptr);
    right_ll.addRegulatoryElement(control_line_ptr);

    std::cerr << "4" << std::endl;
    // Find all references to old lanelet
    //lanelet::utils::query::References rf = lanelet::utils::query::findReferences(current_lanelet, map);
    auto reg_elements = map->regulatoryElementLayer.findUsages(current_lanelet);
    std::cerr << "5" << std::endl;
    // Add all regulatory elements that reference the old lanelet to the two split lanelets
    for (auto reg : reg_elements)
    {
      lanelet::RuleParameterMap output_map;
      ReplaceLaneletParameterVisitor rpv(output_map, current_lanelet.id(), { left_ll, right_ll }, map);
      reg->applyVisitor(rpv);

      std::cerr << "Rule Name: " << reg->getRuleName() << " id: " << reg->id() << std::endl;
      auto new_reg = lanelet::RegulatoryElementFactory::create(reg->getRuleName(), reg->id(), output_map, reg->attributes());
      left_ll.addRegulatoryElement(new_reg);
      right_ll.addRegulatoryElement(new_reg);
    }

    std::cerr << "6" << std::endl;

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

    std::cerr << "7" << std::endl;
  }

  std::cerr << "8" << std::endl;
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

  std::cerr << "9" << std::endl;

  std::vector<lanelet::Area> areas;
  for (auto area : map->areaLayer)
  {
    lanelet::Area mutable_area = map->areaLayer.get(area.id());
    areas.emplace_back(mutable_area);
  }

  std::cerr << "10" << std::endl;

  auto new_map = lanelet::utils::createMap(new_lanelets, areas);

  std::cerr << "11" << std::endl;
  auto new_routing_graph = lanelet::routing::RoutingGraph::build(*new_map, **(cwm.getTrafficRules()));
  new_routing_graph->exportGraphViz("/workspaces/carma_ws/carma/src/carma-platform/carma_wm_ctrl/test/resource/split_lanes_final_routing_graph.viz");

  std::cerr << "12" << std::endl;
  // Write new map to file
  std::string new_file = file + ".split.osm";
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
  std::cerr << "13" << std::endl;

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
  std::cerr << "Done" << std::endl;
}  // namespace carma_wm_ctrl

}  // namespace carma_wm_ctrl