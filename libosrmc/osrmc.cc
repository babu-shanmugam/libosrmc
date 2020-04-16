#include <cassert>
#include <cmath>
#include <utility>
#include <string>
#include <stdexcept>
#include <Python.h>

#include <osrm/coordinate.hpp>
#include <osrm/engine_config.hpp>
#include <osrm/json_container.hpp>
#include <osrm/osrm.hpp>
#include <osrm/route_parameters.hpp>
#include <osrm/table_parameters.hpp>
#include <osrm/nearest_parameters.hpp>
#include <osrm/match_parameters.hpp>
#include <osrm/status.hpp>
#include <osrm/storage_config.hpp>

#include "osrmc.h"

/*String definitions for python version compatability*/
#if PY_MAJOR_VERSION == 2
#define PY_AS_STR PyString_AsString
#define PY_FROM_STR PyString_FromString
#define PY_STRCMP(pyobj, strchar) strcmp(PyString_AsString(pyobj), strchar)
#elif PY_MAJOR_VERSION == 3
#define PY_STRCMP PyUnicode_CompareWithASCIIString
#define PY_AS_STR PyUnicode_AsUTF8
#define PY_FROM_STR PyUnicode_FromString
#endif


/* ABI stability */

unsigned osrmc_get_version(void) { return OSRMC_VERSION; }

int osrmc_is_abi_compatible(void) { return osrmc_get_version() >> 16u == OSRMC_VERSION_MAJOR; }

/* API */

struct osrmc_error final {
  std::string code;
  std::string message;
};

static void osrmc_error_from_exception(const std::exception& e, osrmc_error_t* error) {
  *error = new osrmc_error{"Exception", e.what()};
}

static void osrmc_error_from_json(osrm::json::Object& json, osrmc_error_t* error) try {
  auto code = json.values["code"].get<osrm::json::String>().value;
  auto message = json.values["message"].get<osrm::json::String>().value;
  if (code.empty()) {
    code = "Unknown";
  }

  *error = new osrmc_error{code, message};
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

const char* osrmc_error_code(osrmc_error_t error) { return error->code.c_str(); }

const char* osrmc_error_message(osrmc_error_t error) { return error->message.c_str(); }

void osrmc_error_destruct(osrmc_error_t error) { delete error; }

osrmc_config_t osrmc_config_construct(const char* base_path, osrmc_error_t* error) try {
  auto* out = new osrm::EngineConfig;

  if (base_path)
  {
      out->storage_config = osrm::StorageConfig(base_path);
      out->use_shared_memory = false;
  }
  else
  {
      out->use_shared_memory = true;
  }

  return reinterpret_cast<osrmc_config_t>(out);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void osrmc_config_destruct(osrmc_config_t config) { delete reinterpret_cast<osrm::EngineConfig*>(config); }

osrmc_osrm_t osrmc_osrm_construct(osrmc_config_t config, osrmc_error_t* error) try {
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  auto* out = new osrm::OSRM(*config_typed);

  return reinterpret_cast<osrmc_osrm_t>(out);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void osrmc_osrm_destruct(osrmc_osrm_t osrm) { delete reinterpret_cast<osrm::OSRM*>(osrm); }

void osrmc_base_params_update(osrm::engine::api::BaseParameters *params, PyObject *in) {
    PyObject *coordinates = PyDict_GetItemString(in, "coordinates");
    for (int i = 0; i < PyList_Size(coordinates); i++) {
        PyObject *coordinate = PyList_GetItem(coordinates, i);
        params->coordinates.emplace_back(
             std::move(osrm::util::FloatLongitude{
                       PyFloat_AS_DOUBLE(PyTuple_GetItem(coordinate, 0))}),
             std::move(osrm::util::FloatLatitude{
                       PyFloat_AS_DOUBLE(PyTuple_GetItem(coordinate, 1))}));
    }
    PyObject *bearing_str = PY_FROM_STR("bearings");
    if (PyDict_Contains(in, bearing_str) == 1) {
        PyObject *bearings = PyDict_GetItem(in, bearing_str);
        for (int i = 0; i < PyList_Size(bearings); i++) {
            PyObject *bearing = PyList_GetItem(bearings, i);
            osrm::engine::Bearing bearing_typed{
                static_cast<short>(PyLong_AsLong(PyTuple_GetItem(bearing, 0))),
                static_cast<short>(PyLong_AsLong(PyTuple_GetItem(bearing, 1)))};
            params->bearings.emplace_back(std::move(bearing_typed));
        }
    }
    PyObject *radiuses_str = PY_FROM_STR("radiuses");
    if (PyDict_Contains(in, radiuses_str) == 1) {
        PyObject *radiuses = PyDict_GetItem(in, radiuses_str);
        for (int i = 0; i < PyList_Size(radiuses); i++) {
            params->radiuses.emplace_back(
                (float)PyFloat_AsDouble(PyList_GetItem(radiuses, i)));
        }
    }
    if (PyDict_Contains(in, PY_FROM_STR("generate_hints")) == 1) {
        params->generate_hints = (Py_True == PyDict_GetItemString(in, "generate_hints"));
    }
    if (PyDict_Contains(in, PY_FROM_STR("hints")) == 1) {
        PyObject *hints = PyDict_GetItemString(in, "hints");
        for (int i = 0; i < PyList_Size(hints); i++) {
            params->hints.emplace_back(
                osrm::engine::Hint::FromBase64(PY_AS_STR(PyList_GetItem(hints, i))));
        }
    }
}

void osrmc_route_params_update(osrm::engine::api::RouteParameters *params, PyObject *in) {
    osrmc_base_params_update(params, in);
    if (PyDict_Contains(in, PY_FROM_STR("alternatives")) == 1) {
        params->alternatives = (Py_True == PyDict_GetItemString(in, "alternatives"));
    }
    if (PyDict_Contains(in, PY_FROM_STR("steps")) == 1) {
        params->steps = (Py_True == PyDict_GetItemString(in, "steps"));
    }
    if (PyDict_Contains(in, PY_FROM_STR("annotations")) == 1) {
        PyObject *annotations = PyDict_GetItemString(in, "annotations");
        params->annotations = true;
        if (annotations == Py_False) {
            params->annotations_type = osrm::engine::api::RouteParameters::AnnotationsType::None;
            params->annotations = false;
        } else if (annotations == Py_True)
            params->annotations_type = osrm::engine::api::RouteParameters::AnnotationsType::All;
        else if (!PY_STRCMP(annotations, "nodes"))
            params->annotations_type = osrm::engine::api::RouteParameters::AnnotationsType::Nodes;
        else if (!PY_STRCMP(annotations, "distance"))
            params->annotations_type = osrm::engine::api::RouteParameters::AnnotationsType::Distance;
        else if (!PY_STRCMP(annotations, "duration"))
            params->annotations_type = osrm::engine::api::RouteParameters::AnnotationsType::Duration;
        else if (!PY_STRCMP(annotations, "datasources"))
            params->annotations_type = osrm::engine::api::RouteParameters::AnnotationsType::Datasources;
        else if (!PY_STRCMP(annotations, "weight"))
            params->annotations_type = osrm::engine::api::RouteParameters::AnnotationsType::Weight;
        else if (!PY_STRCMP(annotations, "speed"))
            params->annotations_type = osrm::engine::api::RouteParameters::AnnotationsType::Speed;
    }
    if (PyDict_Contains(in, PY_FROM_STR("geometries")) == 1) {
        PyObject *geometries = PyDict_GetItemString(in, "geometries");
        if (!PY_STRCMP(geometries, "polyline"))
            params->geometries = osrm::engine::api::RouteParameters::GeometriesType::Polyline;
        else if (!PY_STRCMP(geometries, "polyline6"))
            params->geometries = osrm::engine::api::RouteParameters::GeometriesType::Polyline6;
        else if (!PY_STRCMP(geometries, "geojson"))
            params->geometries = osrm::engine::api::RouteParameters::GeometriesType::GeoJSON;
    }
    if (PyDict_Contains(in, PY_FROM_STR("overview")) == 1) {
        PyObject *overview = PyDict_GetItemString(in, "overview");
        if (overview == Py_False)
            params->overview = osrm::engine::api::RouteParameters::OverviewType::False;
        else if (!PY_STRCMP(overview, "simplified"))
            params->overview = osrm::engine::api::RouteParameters::OverviewType::Simplified;
        else if (!PY_STRCMP(overview, "full"))
            params->overview = osrm::engine::api::RouteParameters::OverviewType::Full;
    }
    if (PyDict_Contains(in, PY_FROM_STR("continue_straight")) == 1) {
        PyObject *continue_straight = PyDict_GetItemString(in, "continue_straight");
        if (continue_straight == Py_True)
            params->continue_straight = true;
        else if (continue_straight == Py_False)
            params->continue_straight = false;
    }
}

void osrmc_params_add_coordinate(osrmc_params_t params, float longitude, float latitude, osrmc_error_t* error) try {
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);

  auto longitude_typed = osrm::util::FloatLongitude{longitude};
  auto latitude_typed = osrm::util::FloatLatitude{latitude};

  params_typed->coordinates.emplace_back(std::move(longitude_typed), std::move(latitude_typed));
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_params_add_coordinate_with(osrmc_params_t params, float longitude, float latitude, float radius, int bearing,
                                      int range, osrmc_error_t* error) try {
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);

  auto longitude_typed = osrm::util::FloatLongitude{longitude};
  auto latitude_typed = osrm::util::FloatLatitude{latitude};

  osrm::engine::Bearing bearing_typed{static_cast<short>(bearing), static_cast<short>(range)};

  params_typed->coordinates.emplace_back(std::move(longitude_typed), std::move(latitude_typed));
  params_typed->radiuses.emplace_back(radius);
  params_typed->bearings.emplace_back(std::move(bearing_typed));
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

osrmc_route_params_t osrmc_route_params_construct(osrmc_error_t* error) try {
  auto* out = new osrm::RouteParameters;

  return reinterpret_cast<osrmc_route_params_t>(out);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void osrmc_route_params_destruct(osrmc_route_params_t params) {
  delete reinterpret_cast<osrm::RouteParameters*>(params);
}

void osrmc_route_params_add_steps(osrmc_route_params_t params, int on) {
  auto* params_typed = reinterpret_cast<osrm::RouteParameters*>(params);
  params_typed->steps = on;
}

void osrmc_route_params_add_alternatives(osrmc_route_params_t params, int on) {
  auto* params_typed = reinterpret_cast<osrm::RouteParameters*>(params);
  params_typed->alternatives = on;
}

osrmc_route_response_t osrmc_route(osrmc_osrm_t osrm, osrmc_route_params_t params, osrmc_error_t* error) {
  auto* out = new osrm::json::Object;
  auto* params_cpp = new osrm::RouteParameters;
  try {
    auto* osrm_typed = reinterpret_cast<osrm::OSRM*>(osrm);
    auto* params_input = reinterpret_cast<PyObject *>(params);

    osrmc_route_params_update(params_cpp, params_input);
    const auto status = osrm_typed->Route(*params_cpp, *out);
    delete params_cpp;
    if (status == osrm::Status::Ok) {
        return reinterpret_cast<osrmc_route_response_t>(out);
    }

    osrmc_error_from_json(*out, error);
    return nullptr;
  } catch (const std::exception& e) {
    delete out;
    delete params_cpp;
    osrmc_error_from_exception(e, error);
    return nullptr;
  }
}

void osrmc_route_with(osrmc_osrm_t osrm, osrmc_route_params_t params, osrmc_waypoint_handler_t handler, void* data,
                      osrmc_error_t* error) try {
  auto* osrm_typed = reinterpret_cast<osrm::OSRM*>(osrm);
  auto* params_typed = reinterpret_cast<osrm::RouteParameters*>(params);

  osrm::json::Object result;
  const auto status = osrm_typed->Route(*params_typed, result);

  if (status != osrm::Status::Ok) {
    osrmc_error_from_json(result, error);
    return;
  }

  const auto& waypoints = result.values.at("waypoints").get<osrm::json::Array>().values;

  for (const auto& waypoint : waypoints) {
    const auto& waypoint_typed = waypoint.get<osrm::json::Object>();
    const auto& location = waypoint_typed.values.at("location").get<osrm::json::Array>().values;

    const auto& name = waypoint_typed.values.at("name").get<osrm::json::String>().value;
    const auto longitude = location[0].get<osrm::json::Number>().value;
    const auto latitude = location[1].get<osrm::json::Number>().value;

    (void)handler(data, name.c_str(), longitude, latitude);
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_route_response_destruct(osrmc_route_response_t response) {
  delete reinterpret_cast<osrm::json::Object*>(response);
}

float osrmc_route_response_distance(osrmc_route_response_t response, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  auto& routes = response_typed->values["routes"].get<osrm::json::Array>();
  auto& route = routes.values.at(0).get<osrm::json::Object>();

  const auto distance = route.values["distance"].get<osrm::json::Number>().value;
  return distance;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return INFINITY;
}

float osrmc_route_response_duration(osrmc_route_response_t response, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  auto& routes = response_typed->values["routes"].get<osrm::json::Array>();
  auto& route = routes.values.at(0).get<osrm::json::Object>();

  const auto duration = route.values["duration"].get<osrm::json::Number>().value;
  return duration;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return INFINITY;
}

osrmc_table_annotations_t osrmc_table_annotations_construct(osrmc_error_t* error) try {
  auto* out = new osrm::TableParameters::AnnotationsType{osrm::TableParameters::AnnotationsType::Duration};
  return reinterpret_cast<osrmc_table_annotations_t>(out);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void osrmc_table_annotations_destruct(osrmc_table_annotations_t annotations) {
  delete reinterpret_cast<osrm::TableParameters::AnnotationsType*>(annotations);
}

void osrmc_table_annotations_enable_distance(osrmc_table_annotations_t annotations, bool enable, osrmc_error_t* error) try {
  using AnnotationsType = osrm::TableParameters::AnnotationsType;
  auto* annotations_typed = reinterpret_cast<AnnotationsType*>(annotations);

  if (enable) {
    *annotations_typed |= AnnotationsType::Distance;
  } else {
    *annotations_typed = static_cast<AnnotationsType>(static_cast<int>(*annotations_typed) ^ static_cast<int>(AnnotationsType::Distance));
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

osrmc_table_params_t osrmc_table_params_construct(osrmc_error_t* error) try {
  auto* out = new osrm::TableParameters;
  return reinterpret_cast<osrmc_table_params_t>(out);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void osrmc_table_params_destruct(osrmc_table_params_t params) {
  delete reinterpret_cast<osrm::TableParameters*>(params);
}

void osrmc_table_params_add_source(osrmc_table_params_t params, size_t index, osrmc_error_t* error) try {
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);
  params_typed->sources.emplace_back(index);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_table_params_add_destination(osrmc_table_params_t params, size_t index, osrmc_error_t* error) try {
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);
  params_typed->destinations.emplace_back(index);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void osrmc_table_params_set_annotations(osrmc_table_params_t params, osrmc_table_annotations_t annotations, osrmc_error_t* error) try {
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);
  auto* annotations_typed = reinterpret_cast<osrm::TableParameters::AnnotationsType*>(annotations);
  params_typed->annotations = *annotations_typed;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

osrmc_table_response_t osrmc_table(osrmc_osrm_t osrm, osrmc_table_params_t params, osrmc_error_t* error) try {
  auto* osrm_typed = reinterpret_cast<osrm::OSRM*>(osrm);
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);

  auto* out = new osrm::json::Object;
  const auto status = osrm_typed->Table(*params_typed, *out);

  if (status == osrm::Status::Ok)
    return reinterpret_cast<osrmc_table_response_t>(out);

  osrmc_error_from_json(*out, error);
  return nullptr;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void osrmc_table_response_destruct(osrmc_table_response_t response) {
  delete reinterpret_cast<osrm::json::Object*>(response);
}

float osrmc_table_response_duration(osrmc_table_response_t response, unsigned long from, unsigned long to,
                                    osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("durations") == response_typed->values.end()) {
    *error = new osrmc_error{"NoTable", "Table request not configured to return durations"};
    return INFINITY;
  }

  auto& durations = response_typed->values["durations"].get<osrm::json::Array>();
  auto& durations_from_to_all = durations.values.at(from).get<osrm::json::Array>();
  auto nullable = durations_from_to_all.values.at(to);

  if (nullable.is<osrm::json::Null>()) {
    *error = new osrmc_error{"NoRoute", "Impossible route between points"};
    return INFINITY;
  }
  auto duration = nullable.get<osrm::json::Number>().value;

  return duration;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return INFINITY;
}

float osrmc_table_response_distance(osrmc_table_response_t response, unsigned long from, unsigned long to,
                                    osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("distances") == response_typed->values.end()) {
    *error = new osrmc_error{"NoTable", "Table request not configured to return distances"};
    return INFINITY;
  }

  auto& distances = response_typed->values["distances"].get<osrm::json::Array>();
  auto& distances_from_to_all = distances.values.at(from).get<osrm::json::Array>();
  auto nullable = distances_from_to_all.values.at(to);

  if (nullable.is<osrm::json::Null>()) {
    *error = new osrmc_error{"NoRoute", "Impossible route between points"};
    return INFINITY;
  }
  auto distance = nullable.get<osrm::json::Number>().value;

  return distance;
} catch (const std::exception& e) {
  *error = new osrmc_error{e.what()};
  return INFINITY;
}

osrmc_nearest_params_t osrmc_nearest_params_construct(osrmc_error_t* error) try {
  auto* out = new osrm::NearestParameters;
  return reinterpret_cast<osrmc_nearest_params_t>(out);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void osrmc_nearest_params_destruct(osrmc_nearest_params_t params) {
  delete reinterpret_cast<osrm::NearestParameters*>(params);
}

osrmc_match_params_t osrmc_match_params_construct(osrmc_error_t* error) try {
  auto* out = new osrm::MatchParameters;
  return reinterpret_cast<osrmc_match_params_t>(out);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void osrmc_match_params_destruct(osrmc_match_params_t params) {
  delete reinterpret_cast<osrm::MatchParameters*>(params);
}

void osrmc_nearest_set_number_of_results(osrmc_nearest_params_t params, unsigned n) {
  auto* params_typed = reinterpret_cast<osrm::NearestParameters*>(params);
  params_typed->number_of_results = n;
}

void osrmc_match_params_add_timestamp(osrmc_match_params_t params, unsigned timestamp, osrmc_error_t* error) try {
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  params_typed->timestamps.emplace_back(timestamp);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

struct JSONObject {
    explicit JSONObject(PyObject **out) : ret(out) {}

    void operator()(const osrm::util::json::String &string) const {
        *ret = PY_FROM_STR(string.value.c_str());
    }
    void operator()(const osrm::util::json::Number &number) const {
        if (number.value - long(number.value)) {
            *ret = PyFloat_FromDouble(number.value);
        } else {
            *ret = PyLong_FromDouble(number.value);
        }
    }
    void operator()(const osrm::util::json::Object &object) const {
        *ret = PyDict_New();
        for (auto it = object.values.begin(), end = object.values.end(); it != end;) {
            PyObject *r, *key;
            mapbox::util::apply_visitor(JSONObject(&r), it->second);
            key = PY_FROM_STR(it->first.c_str());
            PyDict_SetItem(*ret, key, r);
            ++it;
        }
    }
    void operator()(const osrm::util::json::Array &array) const {
        *ret = PyList_New(0);
        for (auto it = array.values.cbegin(), end = array.values.cend(); it != end;) {
            PyObject *r;
            mapbox::util::apply_visitor(JSONObject(&r), *it);
            PyList_Append(*ret, r);
            ++it;
        }
    }
    void operator()(const osrm::util::json::True &) const {
        *ret = Py_True;
    }
    void operator()(const osrm::util::json::False &) const {
        *ret = Py_False;
    }
    void operator()(const osrm::util::json::Null &) const {
        *ret = Py_None;
    }
    private:
    PyObject **ret;
};

PyObject *osrm_json_to_pyobj(osrm::util::json::Object& obj) {
    osrm::util::json::Value value = obj;
    PyObject *ret;
    mapbox::util::apply_visitor(JSONObject(&ret), value);
    return ret;
}

PyObject* osrmc_json_to_pyobj(osrmc_json_t obj) {
    auto* out = reinterpret_cast<osrm::util::json::Object*>(obj);
    return osrm_json_to_pyobj(*out);
}
