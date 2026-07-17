#include "fixture_builders.hpp"
#include "fixture_recipe_modules.hpp"

#include <BOPAlgo_Builder.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRep_Builder.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_List.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <TopoDS_CompSolid.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Wire.hxx>
#include <array>
#include <gp_Pnt.hxx>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace cad_mesher::fixtures {
namespace {

[[nodiscard]] TopoDS_Shape require_valid_shape(TopoDS_Shape shape,
                                               const std::string_view fixture_id) {
  if (shape.IsNull()) {
    throw ConstructionFailure(std::string(fixture_id) + ": constructed a null shape");
  }
  if (!BRepCheck_Analyzer(shape, true).IsValid()) {
    throw ConstructionFailure(std::string(fixture_id) + ": constructed shape is not B-rep valid");
  }
  return shape;
}

[[nodiscard]] RoundTripContract free_wire_container_contract() {
  return RoundTripContract{
      .topology_counts = false,
      .model_topology = false,
      .accepted_normalization_codes =
          {
              "step.free_wire_container_flattened",
          },
  };
}

[[nodiscard]] RoundTripContract disconnected_face_wire_contract() {
  return RoundTripContract{
      .topology_counts = false,
      .model_topology = false,
      .accepted_normalization_codes =
          {
              "step.free_wire_container_flattened",
              "step.standalone_face_shell_wrapped",
          },
  };
}

[[nodiscard]] RoundTripContract free_orientation_contract() {
  return RoundTripContract{
      .topology_counts = false,
      .geometry_families = false,
      .concrete_types = false,
      .occurrence_graph = false,
      .model_topology = false,
      .accepted_normalization_codes =
          {
              "step.free_subshape_orientation_canonicalized",
              "step.orientation_curve_family_canonicalized",
          },
  };
}

[[nodiscard]] RoundTripContract compsolid_root_contract() {
  return RoundTripContract{
      .topology_counts = false,
      .accepted_normalization_codes =
          {
              "step.compsolid_root_canonicalized_to_compound",
          },
  };
}

[[nodiscard]] TopoDS_Shape
make_shared_interface_compsolid(const std::span<const ConstructionParameter> parameters) {
  constexpr std::string_view fixture_id = "compsolid.box_pair.shared_interface";
  const double length_x = construction_parameter(parameters, "length_x");
  const double length_y = construction_parameter(parameters, "length_y");
  const double length_z = construction_parameter(parameters, "length_z");

  BRepPrimAPI_MakeBox left_builder(length_x, length_y, length_z);
  BRepPrimAPI_MakeBox right_builder(gp_Pnt(length_x, 0.0, 0.0), length_x, length_y, length_z);
  const TopoDS_Shape left = checked_builder_shape(left_builder, fixture_id);
  const TopoDS_Shape right = checked_builder_shape(right_builder, fixture_id);

  BOPAlgo_Builder general_fuse;
  general_fuse.SetRunParallel(false);
  general_fuse.SetFuzzyValue(0.0);
  general_fuse.SetNonDestructive(true);
  general_fuse.AddArgument(left);
  general_fuse.AddArgument(right);
  general_fuse.Perform();
  if (general_fuse.HasErrors() || general_fuse.Shape().IsNull()) {
    throw ConstructionFailure(std::string(fixture_id) +
                              ": general fuse failed to create shared topology");
  }

  std::vector<TopoDS_Solid> solids;
  for (TopExp_Explorer explorer(general_fuse.Shape(), TopAbs_SOLID); explorer.More();
       explorer.Next()) {
    solids.push_back(TopoDS::Solid(explorer.Current()));
  }
  if (solids.size() != 2) {
    throw ConstructionFailure(std::string(fixture_id) +
                              ": general fuse did not retain exactly two solids");
  }

  NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>
      face_to_solids;
  TopExp::MapShapesAndAncestors(general_fuse.Shape(), TopAbs_FACE, TopAbs_SOLID, face_to_solids);
  int shared_face_count = 0;
  for (int index = 1; index <= face_to_solids.Extent(); ++index) {
    if (face_to_solids.FindFromIndex(index).Extent() == 2) {
      ++shared_face_count;
    }
  }
  if (shared_face_count != 1) {
    throw ConstructionFailure(std::string(fixture_id) +
                              ": expected exactly one face shared by both solids");
  }

  BRep_Builder builder;
  TopoDS_CompSolid compsolid;
  builder.MakeCompSolid(compsolid);
  for (const TopoDS_Solid& solid : solids) {
    builder.Add(compsolid, solid);
  }
  return require_valid_shape(compsolid, fixture_id);
}

[[nodiscard]] TopoDS_Shape
make_disconnected_face_and_wire(const std::span<const ConstructionParameter> parameters) {
  constexpr std::string_view fixture_id = "compound.mixed.disconnected_face_wire";
  const double face_x = construction_parameter(parameters, "face_length_x");
  const double face_y = construction_parameter(parameters, "face_length_y");
  const double separation = construction_parameter(parameters, "separation");
  const double wire_length = construction_parameter(parameters, "wire_length");

  BRepBuilderAPI_MakePolygon polygon;
  polygon.Add(gp_Pnt(0.0, 0.0, 0.0));
  polygon.Add(gp_Pnt(face_x, 0.0, 0.0));
  polygon.Add(gp_Pnt(face_x, face_y, 0.0));
  polygon.Add(gp_Pnt(0.0, face_y, 0.0));
  polygon.Close();
  if (!polygon.IsDone()) {
    throw ConstructionFailure(std::string(fixture_id) + ": planar face wire failed");
  }
  BRepBuilderAPI_MakeFace face_builder(polygon.Wire(), true);
  const TopoDS_Face face = TopoDS::Face(checked_builder_shape(face_builder, fixture_id));

  BRepBuilderAPI_MakeEdge first_edge_builder(gp_Pnt(separation, 0.0, 0.0),
                                             gp_Pnt(separation + wire_length, 0.0, 0.0));
  BRepBuilderAPI_MakeEdge second_edge_builder(gp_Pnt(separation + wire_length, 0.0, 0.0),
                                              gp_Pnt(separation + wire_length, wire_length, 0.0));
  const std::array edges{
      TopoDS::Edge(checked_builder_shape(first_edge_builder, fixture_id)),
      TopoDS::Edge(checked_builder_shape(second_edge_builder, fixture_id)),
  };
  const TopoDS_Wire wire = make_wire(edges);
  const std::array<TopoDS_Shape, 2> children{face, wire};
  return require_valid_shape(make_compound(children), fixture_id);
}

[[nodiscard]] TopoDS_Shape
make_free_wire_edge_vertex_variants(const std::span<const ConstructionParameter> parameters) {
  constexpr std::string_view fixture_id = "compound.free_wire_edge_vertex";
  const double side = construction_parameter(parameters, "closed_wire_side");
  const double separation = construction_parameter(parameters, "separation");

  const std::array closed_points{
      gp_Pnt(0.0, 0.0, 0.0),
      gp_Pnt(side, 0.0, 0.0),
      gp_Pnt(side, side, 0.0),
      gp_Pnt(0.0, side, 0.0),
  };
  std::array<TopoDS_Edge, 4> closed_edges;
  for (std::size_t index = 0; index < closed_edges.size(); ++index) {
    BRepBuilderAPI_MakeEdge edge_builder(closed_points[index],
                                         closed_points[(index + 1) % closed_points.size()]);
    closed_edges[index] = TopoDS::Edge(checked_builder_shape(edge_builder, fixture_id));
  }
  const TopoDS_Wire closed_wire = make_wire(closed_edges);

  BRepBuilderAPI_MakeEdge open_first_builder(gp_Pnt(separation, 0.0, 0.0),
                                             gp_Pnt(separation + side, 0.0, 0.0));
  BRepBuilderAPI_MakeEdge open_second_builder(gp_Pnt(separation + side, 0.0, 0.0),
                                              gp_Pnt(separation + side, side, 0.0));
  const std::array open_edges{
      TopoDS::Edge(checked_builder_shape(open_first_builder, fixture_id)),
      TopoDS::Edge(checked_builder_shape(open_second_builder, fixture_id)),
  };
  const TopoDS_Wire open_wire = make_wire(open_edges);

  BRepBuilderAPI_MakeEdge free_edge_builder(gp_Pnt(2.0 * separation, 0.0, 0.0),
                                            gp_Pnt(2.0 * separation, side, 0.0));
  const TopoDS_Shape free_edge = checked_builder_shape(free_edge_builder, fixture_id);
  BRepBuilderAPI_MakeVertex isolated_vertex_builder(
      gp_Pnt(3.0 * separation, 0.5 * side, 0.5 * side));
  const TopoDS_Shape isolated_vertex = checked_builder_shape(isolated_vertex_builder, fixture_id);

  const std::array<TopoDS_Shape, 4> children{
      closed_wire,
      open_wire,
      free_edge,
      isolated_vertex,
  };
  return require_valid_shape(make_compound(children), fixture_id);
}

[[nodiscard]] TopoDS_Shape
make_all_orientation_occurrences(const std::span<const ConstructionParameter> parameters) {
  constexpr std::string_view fixture_id = "compound.orientation.internal_external";
  const double length = construction_parameter(parameters, "edge_length");
  const double separation = construction_parameter(parameters, "separation");
  constexpr std::array orientations{
      TopAbs_FORWARD,
      TopAbs_REVERSED,
      TopAbs_INTERNAL,
      TopAbs_EXTERNAL,
  };

  std::array<TopoDS_Shape, orientations.size()> children;
  for (std::size_t index = 0; index < orientations.size(); ++index) {
    const double y = static_cast<double>(index) * separation;
    BRepBuilderAPI_MakeEdge edge_builder(gp_Pnt(0.0, y, 0.0), gp_Pnt(length, y, 0.0));
    TopoDS_Shape edge = checked_builder_shape(edge_builder, fixture_id);
    edge.Orientation(orientations[index]);
    children[index] = edge;
  }
  return require_valid_shape(make_compound(children), fixture_id);
}

} // namespace

void append_topology_recipes(std::vector<Recipe>& recipes) {
  recipes.push_back(procedural_shape_recipe("compound.free_wire_edge_vertex",
                                            {
                                                {"closed_wire_side", 8.0, "millimetres"},
                                                {"separation", 20.0, "millimetres"},
                                            },
                                            make_free_wire_edge_vertex_variants, StepIoProfile{},
                                            free_wire_container_contract()));
  recipes.push_back(procedural_shape_recipe("compound.mixed.disconnected_face_wire",
                                            {
                                                {"face_length_x", 12.0, "millimetres"},
                                                {"face_length_y", 9.0, "millimetres"},
                                                {"separation", 25.0, "millimetres"},
                                                {"wire_length", 7.0, "millimetres"},
                                            },
                                            make_disconnected_face_and_wire, StepIoProfile{},
                                            disconnected_face_wire_contract()));
  recipes.push_back(procedural_shape_recipe("compound.orientation.internal_external",
                                            {
                                                {"edge_length", 10.0, "millimetres"},
                                                {"separation", 5.0, "millimetres"},
                                            },
                                            make_all_orientation_occurrences, StepIoProfile{},
                                            free_orientation_contract()));
  recipes.push_back(procedural_shape_recipe("compsolid.box_pair.shared_interface",
                                            {
                                                {"length_x", 10.0, "millimetres"},
                                                {"length_y", 12.0, "millimetres"},
                                                {"length_z", 14.0, "millimetres"},
                                            },
                                            make_shared_interface_compsolid,
                                            StepIoProfile{
                                                .write_nonmanifold = true,
                                                .read_nonmanifold = true,
                                            },
                                            compsolid_root_contract()));
}

} // namespace cad_mesher::fixtures
