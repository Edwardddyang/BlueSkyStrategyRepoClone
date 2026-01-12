#pragma once

#include <iostream>
#include <nlohmann/json.hpp>

template <typename DerivedRacePlan>
void BaseRacePlan<DerivedRacePlan>::export_json(std::filesystem::path dump_path) const {
  DerivedRacePlan& derived = *static_cast<const DerivedRacePlan*>(this);
  using json = nlohmann::json;

  json j = json::array();

  // Add race plan metadata + statistics
  derived.export_metadata_json_impl(j);

  // Add segments
  derived.export_segments_json_impl(j);

  std::ofstream out(dump_path);
  RUNTIME_EXCEPTION(out.is_open(), "Could not open {} for writing", dump_path.string());
  out << std::setw(4) << j << std::endl;
}

template <typename DerivedRacePlan>
std::string BaseRacePlan<DerivedRacePlan>::get_plan_string() const {
  return static_cast<const DerivedRacePlan*>(this)->get_plan_string_impl();
}

template <typename DerivedRacePlan>
void BaseRacePlan<DerivedRacePlan>::print_plan() const {
  std::cout << get_plan_string() << std::endl;
}

template <typename DerivedRacePlan>
template <RouteType R>
bool BaseRacePlan<DerivedRacePlan>::validate_members(const R& route) const {
  return static_cast<const DerivedRacePlan*>(this)->validate_members_impl(route);
}
