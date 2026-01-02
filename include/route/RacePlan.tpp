#pragma once

template <typename DerivedRacePlan>
void BaseRacePlan<DerivedRacePlan>::export_json() const {
  DerivedRacePlan& derived = *static_cast<const DerivedRacePlan*>(this);
  using json = nlohmann::json;

  // Create dump directory
  const std::filesystem::path dump_dir = Config::get_instance().get_dump_dir();
  std::filesystem::create_directories(dump_dir);
  const std::filesystem::path path = dump_dir / dump_file_name;

  json j = json::array();

  // Add race plan metadata + statistics
  derived.export_metadata_json_impl(j);

  // Add segments
  derived.export_segments_json_impl(j);

  std::ofstream out(path);
  RUNTIME_EXCEPTION(out, "Could not open {} for writing", path.string());
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
