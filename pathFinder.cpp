/*
 * routeFind.cpp
 *
 *  Created on: Oct 26, 2019
 *      Author: richard
 */

#include "pathFinder.h"
#include "Cost.h"
#include "DBConnection.h"
#include "Elevation.h"
#include "Graph.h"
#include "Map.h"
#include "Point.h"
#include "Route.h"
#include "Search.h"
#include "SearchController.h"
#include "SearchLogEntry.h"
#include "configuration.h"
#include "StatusUpdate.h"
#include "JsonToNapi.h"
#include <algorithm>
#include <cmath>
#include <csignal>
#include <future>
#include <iomanip>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <list>
#include <map>
#include <pqxx/pqxx>
#include <regex>
#include <sstream>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>
#include <thread>
#include <tuple>
#include <unistd.h>
#include <vector>
#include <condition_variable>

Napi::FunctionReference PathFinder::constructor;

Napi::Object InitAll(Napi::Env env, Napi::Object exports) {
  return PathFinder::Init(env, exports);
}

NODE_API_MODULE(NODE_GYP_MODULE_NAME, InitAll)

PathFinder::PathFinder(const Napi::CallbackInfo &info)
:
  Napi::ObjectWrap<PathFinder>(info)
{
  Napi::Env env = info.Env();

  m_taskThreads.push_back(std::thread(&PathFinder::taskLoop, this));
  m_taskThreads.push_back(std::thread(&PathFinder::taskLoop, this));
  m_taskThreads.push_back(std::thread(&PathFinder::taskLoop, this));

  m_callbackFunction = Napi::ThreadSafeFunction::New(
		env,
		{},
		"callback",
		0,
		1
	);

  if (info.Length() == 1 && info[0].IsObject()) {
    auto initObject = info[0].As<Napi::Object>();

		if (initObject.Has("statusUpdate"))
		{
			std::cerr << "Has statusUpdate" << std::endl;

			auto statusUpdateCallback = initObject.Get("statusUpdate");

			auto callback = Napi::ThreadSafeFunction::New(
				env,
				statusUpdateCallback.As<Napi::Function>(),
				"statusUpdate",
				0,
				1);

      setStatusUpdateCallback(callback);
		}

    if (initObject.Has("elevationDirectory") &&
        initObject.Get("elevationDirectory").IsString()) {
      m_configuration.elevationDirectory =
          initObject.Get("elevationDirectory").As<Napi::String>();
    }

    if (initObject.Has("database") && initObject.Get("database").IsString()) {
      m_configuration.database = initObject.Get("database").As<Napi::String>();
    }

    if (initObject.Has("databaseUsername") &&
        initObject.Get("databaseUsername").IsString()) {
      m_configuration.databaseUsername =
          initObject.Get("databaseUsername").As<Napi::String>();
    }

    if (initObject.Has("databasePassword") &&
        initObject.Get("databasePassword").IsString()) {
      m_configuration.databasePassword =
          initObject.Get("databasePassword").As<Napi::String>();
    }

    if (initObject.Has("databaseHost") &&
        initObject.Get("databaseHost").IsString()) {
      m_configuration.databaseHost =
          initObject.Get("databaseHost").As<Napi::String>();
    }
  }

  std::cerr << "elevationDirectory: " << m_configuration.elevationDirectory
            << std::endl;

  elevationPathSet(m_configuration.elevationDirectory);

  configDB(m_configuration.databaseUsername,
            m_configuration.databasePassword,
            m_configuration.databaseHost, m_configuration.database);

  m_graphBuilder = std::make_unique<gb::GraphBuilder>();
  m_graphBuilder->start();
}

std::string runtimeDir;

Napi::Object PathFinder::Init(Napi::Env env, Napi::Object exports) {
  Napi::HandleScope scope(env);

  Napi::Function func = DefineClass(
      env, "Renderer",
      {
          InstanceMethod("whatIsHere", &PathFinder::whatIsHere),
          InstanceMethod("elevation", &PathFinder::elevation),
          InstanceMethod("elevationArea", &PathFinder::elevationArea),
          InstanceMethod("elevationTile", &PathFinder::elevationTile),
          InstanceMethod("getGeneratePathsQueue", &PathFinder::getGeneratePathsQueue),
          InstanceMethod("generatePaths", &PathFinder::generatePaths),
          InstanceMethod("deleteGeneratePathRequest", &PathFinder::deleteGeneratePathRequest),
          InstanceMethod("getHikeDistance", &PathFinder::getHikeDistance),
          InstanceMethod("getTrailInfo", &PathFinder::getTrailInfo),
          InstanceMethod("findRoute", &PathFinder::findRoute),
          InstanceMethod("updateRouteElevations", &PathFinder::updateRouteElevations),
          InstanceMethod("updateNavNodeElevations", &PathFinder::updateNavNodeElevations),
          InstanceMethod("updateNavEdgeCosts", &PathFinder::updateNavEdgeCosts),
          InstanceMethod("getSearchLog", &PathFinder::getSearchLog),
          InstanceMethod("getNodeCounts", &PathFinder::getNodeCounts),
          InstanceMethod("updateIntersectionCounts", &PathFinder::updateIntersectionCounts),
          InstanceMethod("addQuadrangle", &PathFinder::addQuadrangle),
          InstanceMethod("removeQuadrangle", &PathFinder::removeQuadrangle),
          InstanceMethod("search", &PathFinder::search),
          InstanceMethod("getRoutes", &PathFinder::getRoutes),
          InstanceMethod("addRouteGroup", &PathFinder::addRouteGroup),
          InstanceMethod("addRouteGroupRoutes", &PathFinder::addRouteGroupRoutes),
          InstanceMethod("getRouteGroupRoutes", &PathFinder::getRouteGroupRoutes),
          InstanceMethod("getRouteGroups", &PathFinder::getRouteGroups),
      });

  constructor = Napi::Persistent(func);
  constructor.SuppressDestruct();

  exports.Set("Finder", func);

  return exports;
}

Napi::Value PathFinder::whatIsHere(const Napi::CallbackInfo &info) {
  Napi::Env env = info.Env();

  if (info.Length() != 1 || !info[0].IsObject()) {
    Napi::TypeError::New(env, "Object expected").ThrowAsJavaScriptException();
  }

  auto request = info[0].ToObject();

  Point point(request.Get("lat").As<Napi::Number>(),
              request.Get("lng").As<Napi::Number>());

  auto result = getLineFromPoint(point);

  auto ele = Elevation::getInstance()->getElevation(point);

  auto object = Napi::Object::New(env);

  object.Set("ele", ele);
  object.Set("lineId", result.m_lineId);
  object.Set("type", result.m_highway);
  object.Set("name", result.m_name);
  object.Set("surface", result.m_surface);

  return object;
}

Napi::Value PathFinder::elevation(const Napi::CallbackInfo &info) {
  Napi::Env env = info.Env();

  if (info.Length() != 1 || !info[0].IsObject()) {
    Napi::TypeError::New(env, "Object expected").ThrowAsJavaScriptException();
  }

  auto request = info[0].ToObject();

  LatLng latlng(request.Get("lat").As<Napi::Number>(),
                request.Get("lng").As<Napi::Number>());

  auto ele = Elevation::getInstance()->getElevation(latlng);

  return Napi::Number::New(env, ele);
}

Napi::Value PathFinder::elevationArea(const Napi::CallbackInfo &info) {
  Napi::Env env = info.Env();

  if (info.Length() != 1 || !info[0].IsObject()) {
    Napi::TypeError::New(env, "Object expected").ThrowAsJavaScriptException();
  }

  auto request = info[0].ToObject();

  LatLng latlng(request.Get("lat").As<Napi::Number>(),
                request.Get("lng").As<Napi::Number>());

  auto terrain = Elevation::getInstance()->getElevationArea(
    latlng,
    request.Get("dim").As<Napi::Number>());

  auto object = Napi::Object::New(env);
  auto array = Napi::Array::New(env, terrain.points.size());

  for (size_t j = 0; j < terrain.points.size(); j++)
  {
    Napi::HandleScope scope(env);
    auto row = Napi::Array::New(env, terrain.points[j].size());

    for (size_t i = 0; i < terrain.points[j].size(); i++) {
      Napi::HandleScope scope(env);
      row[i] = terrain.points[j][i];
    }

    array[j] = row;
  }

  object.Set("points", array);

  array = Napi::Array::New(env, terrain.centers.size());

  for (size_t j = 0; j < terrain.centers.size(); j++)
  {
    Napi::HandleScope scope(env);
    auto row = Napi::Array::New(env, terrain.centers[j].size());

    for (size_t i = 0; i < terrain.centers[j].size(); i++) {
      Napi::HandleScope scope(env);
      row[i] = terrain.centers[j][i];
    }

    array[j] = row;
  }

  object.Set("centers", array);

  object.Set("sw", convertToNapiValue(env, terrain.sw));
  object.Set("ne", convertToNapiValue(env, terrain.ne));

  return object;
}

Napi::Value PathFinder::elevationTile(const Napi::CallbackInfo &info) {
  Napi::Env env = info.Env();

  if (info.Length() != 1 || !info[0].IsObject()) {
    Napi::TypeError::New(env, "Object expected").ThrowAsJavaScriptException();
  }

  auto request = info[0].ToObject();

  int x = request.Get("x").As<Napi::Number>();
  int y = request.Get("y").As<Napi::Number>();
  int z = request.Get("z").As<Napi::Number>();

  auto terrain = Elevation::getInstance()->getElevationTile(x, y, z);

  auto object = Napi::Object::New(env);
  auto array = Napi::Array::New(env, terrain.points.size());

  for (size_t j = 0; j < terrain.points.size(); j++)
  {
    Napi::HandleScope scope(env);
    auto row = Napi::Array::New(env, terrain.points[j].size());

    for (size_t i = 0; i < terrain.points[j].size(); i++) {
      Napi::HandleScope scope(env);
      row[i] = terrain.points[j][i];
    }

    array[j] = row;
  }

  object.Set("points", array);

  array = Napi::Array::New(env, terrain.centers.size());

  for (size_t j = 0; j < terrain.centers.size(); j++)
  {
    Napi::HandleScope scope(env);
    auto row = Napi::Array::New(env, terrain.centers[j].size());

    for (size_t i = 0; i < terrain.centers[j].size(); i++) {
      Napi::HandleScope scope(env);
      row[i] = terrain.centers[j][i];
    }

    array[j] = row;
  }

  object.Set("centers", array);

  object.Set("sw", convertToNapiValue(env, terrain.sw));
  object.Set("ne", convertToNapiValue(env, terrain.ne));

  object.Set("textureSW", convertToNapiValue(env, terrain.textureSW));
  object.Set("textureNE", convertToNapiValue(env, terrain.textureNE));

  return object;
}

Napi::Value PathFinder::getHikeDistance(const Napi::CallbackInfo &info) {
  Napi::Env env = info.Env();

  if (info.Length() != 1 || !info[0].IsNumber()) {
    Napi::TypeError::New(env, "Number expected").ThrowAsJavaScriptException();
  }

  int hikeId = info[0].As<Napi::Number>();

  Route route(hikeId);

  auto distance = route.getDistance();

  return Napi::Number::New(env, distance);
}

Napi::Value PathFinder::getSearchLog(const Napi::CallbackInfo &info) {
  Napi::Env env = info.Env();

  auto array = Napi::Array::New(env, 0); // searchLog.size());

  // for (size_t i = 0; i < searchLog.size(); i++) {
  //   Napi::HandleScope scope(env);
  //   array[i] = convertToNapiValue(env, searchLog[i]);
  // }

  return array;
}

std::tuple<int, int, int,  int> getLatLngBounds(
  const Napi::Value &value)
{
  int minLat {0};
  int maxLat {0};
  int minLng {0};
  int maxLng {0};

  if (value.IsArray())
  {
    auto request = value.As<Napi::Array>();
    auto point1 = request.Get(static_cast<uint32_t>(0)).As<Napi::Object>();
    auto point2 = request.Get(static_cast<uint32_t>(1)).As<Napi::Object>();

    double lat1 =  point1.Get("lat").As<Napi::Number>();
    double lat2 =  point2.Get("lat").As<Napi::Number>();

    if (lat1 > lat2) {
      std::swap(lat1, lat2);
    }

    double lng1 =  point1.Get("lng").As<Napi::Number>();
    double lng2 =  point2.Get("lng").As<Napi::Number>();

    if (lng1 > lng2) {
      std::swap(lng1, lng2);
    }

    minLng = floor(lng1);
    minLat = floor(lat1);
    maxLng = ceil(lng2);
    maxLat = ceil(lat2);
  }
  else if (value.IsObject()) {
    auto request = value.ToObject();

    double lat = request.Get("lat").As<Napi::Number>();
    double lng = request.Get("lng").As<Napi::Number>();

    minLat = floor(lat);
    maxLat = minLat + 1;

    minLng = floor(lng);
    maxLng = minLng + 1;
  }

  return {minLat, minLng, maxLat, maxLng};
}

Napi::Value latLngBounds(
  Napi::Env env,
  int minLat,
  int minLng,
  int maxLat,
  int maxLng)
{
  auto array = Napi::Array::New(env);

  {
    auto object = Napi::Object::New(env);

    object.Set("lat", minLat);
    object.Set("lng", minLng);

    array[static_cast<uint32_t>(0)] = object;
  }

  {
    auto object = Napi::Object::New(env);

    object.Set("lat", maxLat);
    object.Set("lng", maxLng);

    array[static_cast<uint32_t>(1)] = object;
  }

  return array;
}

Napi::Value PathFinder::addQuadrangle(const Napi::CallbackInfo &info)
{
  Napi::Env env = info.Env();

  if (info.Length() != 1 || !info[0].IsObject()) {
    Napi::TypeError::New(env, "Object expected").ThrowAsJavaScriptException();
  }

  auto dbConnection = std::make_shared<DBConnection>();

  PreparedStatement insertQuadrangle(
    dbConnection,
    R"%(
      INSERT INTO quad_intersection_counts (created_at, updated_at, lat, lng)
      VALUES (now(), now(), $1, $2)
      ON CONFLICT (lat, lng)
      DO NOTHING
    )%"
  );

  int minLat {0};
  int maxLat {0};
  int minLng {0};
  int maxLng {0};

  std::tie(minLat, minLng, maxLat, maxLng) = getLatLngBounds(info[0]);

  auto transaction = dbConnection->newTransaction();

  for (int y = minLat; y < maxLat; y++) {
    for (int x = minLng; x < maxLng; x++) {
      transaction.exec(insertQuadrangle, y, x);
    }
  }

  transaction.commit();

  return latLngBounds(env, minLat, minLng, maxLat, maxLng);
}

Napi::Value PathFinder::removeQuadrangle(const Napi::CallbackInfo &info)
{
  Napi::Env env = info.Env();

  if (info.Length() != 1 || !info[0].IsObject()) {
    Napi::TypeError::New(env, "Object expected").ThrowAsJavaScriptException();
  }

  auto dbConnection = std::make_shared<DBConnection>();

  PreparedStatement removeQuadrangle(
    dbConnection,
    R"%(
      DELETE FROM quad_intersection_counts
      WHERE lat = $1 and lng = $2
    )%"
  );

  int minLat {0};
  int maxLat {0};
  int minLng {0};
  int maxLng {0};

  std::tie(minLat, minLng, maxLat, maxLng) = getLatLngBounds(info[0]);

  auto transaction = dbConnection->newTransaction();

  for (int y = minLat; y < maxLat; y++) {
    for (int x = minLng; x < maxLng; x++) {
      transaction.exec(removeQuadrangle, y, x);
    }
  }

  transaction.commit();

  return latLngBounds(env, minLat, minLng, maxLat, maxLng);
}

Napi::Value PathFinder::getNodeCounts(const Napi::CallbackInfo &info)
{
  Napi::Env env = info.Env();

  auto deferred = postTask(
    env,
    [this](Napi::Promise::Deferred deferred)
    {
      auto dbConnection = std::make_shared<DBConnection>();

      auto results = dbConnection->exec(
        R"%(
          select q.lng, q.lat, COALESCE(t1.count, 0) as count, intersection_count as max
          from quad_intersection_counts as q
          left join (
                select floor(ST_X(ST_Transform(n.way, 4326))) as lng, floor(ST_Y(ST_Transform(n.way, 4326))) as lat, count(*)
                from nav_nodes AS n
                group by lng, lat
          ) as t1 on q.lat = t1.lat and q.lng = t1.lng
        )%"
      );

      m_callbackFunction.BlockingCall(
        [results, deferred](Napi::Env env, Napi::Function jsCallback)
        {
          auto array = Napi::Array::New(env, results.size());

          for (pqxx::result::size_type i = 0; i < results.size(); i++)
          {
            auto object = Napi::Object::New(env);

            object.Set("lat", results[i]["lat"].as<int>());
            object.Set("lng", results[i]["lng"].as<int>());
            object.Set("count", results[i]["count"].as<int>());

            if (results[i]["max"].is_null())
            {
              object.Set("max", env.Null());
            }
            else
            {
              object.Set("max", results[i]["max"].as<int>());
            }

            array[i] = object;
          }

          return deferred.Resolve(array);
        });
    });

    return deferred.Promise();
}

void PathFinder::updateIntersectionCounts(const Napi::CallbackInfo &info)
{
  Napi::Env env = info.Env();

  if (info.Length() != 1 || !info[0].IsObject()) {
    Napi::TypeError::New(env, "Object expected").ThrowAsJavaScriptException();
  }

  auto request = info[0].ToObject();

  double lat = request.Get("lat").As<Napi::Number>();
  double lng = request.Get("lng").As<Napi::Number>();

  postTask(
    env,
    [this, lat, lng](Napi::Promise::Deferred deferred)
    {
      auto dbConnection = std::make_shared<DBConnection>();

      int south = std::floor(lat);
      int west = std::floor(lng);
      int north = south + 1;
      int east = west + 1;

      for (int lat = south; lat < north; lat++)
      {
        for (int lng = west; lng < east; lng++)
        {
          auto transaction = m_graphBuilder->dbConnection()->newTransaction ();

          m_graphBuilder->updateIntersectionCount(transaction, lat, lng);

          transaction.commit();
        }
      }
    }
  );
}

Napi::Value PathFinder::search(const Napi::CallbackInfo &info)
{
  Napi::Env env = info.Env();

  if (info.Length() != 1) {
    Napi::TypeError::New(env, "missing parameter").ThrowAsJavaScriptException();
    return {};
  }

  std::string terms = info[0].As<Napi::String>();

  auto deferred = postTask(
    env,
    [this, terms](Napi::Promise::Deferred deferred)
    {
      auto dbConnection = std::make_shared<DBConnection>();

      PreparedStatement searchQuery(
        dbConnection,
        R"%(
          select json_agg(groups.group) AS groups
          from (
            select json_build_object('name', name, 'lines', json_agg(json_build_object('lineId', line_id, 'route', null))) as group
            from planet_osm_route
            where to_tsvector('english', name) @@ to_tsquery($1)
            and (highway in ('path', 'footway', 'track', 'bridleway', 'steps')
            or foot in ('designated', 'yes'))
            group by name
          ) AS groups
        )%"
      );

      auto result = searchQuery.exec(terms);

      m_callbackFunction.BlockingCall(
        [result, deferred](Napi::Env env, Napi::Function jsCallback)
        {
          if (result.size() > 0 && !result[0]["groups"].is_null())
          {
            Json::Value groups;
            Json::Reader reader;
            reader.parse(result[0]["groups"].as<std::string>(), groups);

            deferred.Resolve(convertToNapiValue(env, groups));
          }
          else
          {
            deferred.Resolve(Napi::Array::New(env, 0));
          }
        });
    }
  );

  return deferred.Promise();
}


Napi::Array getRoute(
  Napi::Env env,
  const pqxx::const_result_iterator::reference &row
)
{
  Json::Value route;
  Json::Reader reader;

  reader.parse(row["route"].as<std::string>(), route);

  auto coords = route["coordinates"];

  auto coordsArray = Napi::Array::New(env, coords.size());

  int j {0};
  for (const auto &coord: coords)
  {
    Napi::HandleScope scope(env);

    auto latLng = Napi::Array::New(env, 2);
    
    int k {0};
    latLng[k++] = coord[1].asDouble();
    latLng[k] = coord[0].asDouble();

    coordsArray[j++] = latLng;
  }

  return coordsArray;
}


Napi::Value PathFinder::getRoutes(const Napi::CallbackInfo &info)
{
  Napi::Env env = info.Env();

  if (info.Length() != 1) {
    Napi::TypeError::New(env, "missing parameter").ThrowAsJavaScriptException();
    return {};
  }

  auto lineIds = info[0].As<Napi::Array>();

  std::string ids;
  for (size_t i = 0; i < lineIds.Length(); i++)
  {
    int id = lineIds.Get(i).As<Napi::Number>();
    ids += std::to_string(id) + ",";
  }
  ids = ids.substr(0, ids.size() - 1);

  auto deferred = postTask(
    env,
    [this, ids](Napi::Promise::Deferred deferred)
    {
      auto dbConnection = std::make_shared<DBConnection>();

      std::string queryString = 
        R"%(
          select
            json_agg(json_build_object(
              'lineId', line_id,
              'route', ST_AsGeoJSON(ST_SwapOrdinates(ST_Force2D(ST_Transform(way2, 4326)), 'xy'))::json->'coordinates'
            )) AS lines
          from planet_osm_route
          where line_id in (
        )%" + ids + ")";

      PreparedStatement routesQuery(dbConnection, queryString);

      auto result = routesQuery.exec();

      m_callbackFunction.BlockingCall(
        [result, deferred](Napi::Env env, Napi::Function jsCallback)
        {
          if (result.size() > 0 && !result[0]["lines"].is_null())
          {
            Json::Value lines;
            Json::Reader reader;
            reader.parse(result[0]["lines"].as<std::string>(), lines);

            deferred.Resolve(convertToNapiValue(env, lines));
          }
          else
          {
            deferred.Resolve(Napi::Array::New(env, 0));
          }
        });
    }
  );

  return deferred.Promise();
}

void PathFinder::addRouteGroup(const Napi::CallbackInfo &info)
{
  Napi::Env env = info.Env();

  if (info.Length() != 1 || !info[0].IsString()) {
    Napi::TypeError::New(env, "missing parameter").ThrowAsJavaScriptException();
    return;
  }

  std::string name = info[0].As<Napi::String>();

  postTask(
    env,
    [name](Napi::Promise::Deferred deferred)
    {
      auto dbConnection = std::make_shared<DBConnection>();
    
      PreparedStatement insertRouteGroup(
        dbConnection,
        R"%(
          insert into route_groups(created_at, updated_at, name)
          values (now(), now(), $1)
        )%"
      );

      auto transaction = dbConnection->newTransaction();

      transaction.exec(insertRouteGroup, name);

      transaction.commit();
    }
  );
}

Napi::Value PathFinder::addRouteGroupRoutes(const Napi::CallbackInfo &info)
{
  Napi::Env env = info.Env();

  if (info.Length() != 1 || !info[0].IsObject()) {
    Napi::TypeError::New(env, "missing parameter").ThrowAsJavaScriptException();
    return {};
  }

  auto object = info[0].As<Napi::Object>();

  int routeGroupId = object.Get("routeGroupId").As<Napi::Number>();
  auto lineIds = object.Get("lines").As<Napi::Array>();

  std::vector<int> lines;

  for (size_t i = 0; i < lineIds.Length(); i++)
  {
    lines.push_back(lineIds.Get(i).As<Napi::Number>());
  }

  auto deferred = postTask(
    env,
    [this, routeGroupId, lines](Napi::Promise::Deferred deferred)
    {
      auto dbConnection = std::make_shared<DBConnection>();
    
      std::string insertRouteGroupRoutes =
        R"%(
          insert into route_group_routes(created_at, updated_at, route_group_id, line_id)
          values
        )%";

      for (const auto line: lines)
      {
        insertRouteGroupRoutes += "(now(), now(), "
          + std::to_string(routeGroupId) + ", "
          + std::to_string(line) + "),\n";
      }
      insertRouteGroupRoutes = insertRouteGroupRoutes.substr(0, insertRouteGroupRoutes.size() - 2);

      auto transaction = dbConnection->newTransaction();

      transaction.exec(insertRouteGroupRoutes);

      transaction.commit();

      m_callbackFunction.BlockingCall(
        [deferred](Napi::Env env, Napi::Function jsCallback)
        {
          auto value = Napi::Boolean::New(env, true);
          deferred.Resolve(value);
        }
      );
    }
  );

  return deferred.Promise();
}

Napi::Value PathFinder::getRouteGroupRoutes(const Napi::CallbackInfo &info)
{
  Napi::Env env = info.Env();

  if (info.Length() != 2 || !info[0].IsNumber() || !info[1].IsBoolean()) {
    Napi::TypeError::New(env, "invalid parameters").ThrowAsJavaScriptException();
    return {};
  }

  int routeGroupId = info[0].As<Napi::Number>();
  bool full = info[1].As<Napi::Boolean>();

  auto deferred = postTask(
    env,
    [this, routeGroupId, full](Napi::Promise::Deferred deferred)
    {
      auto dbConnection = std::make_shared<DBConnection>();

      if (full)
      {
        PreparedStatement selectFullRoutes(
          dbConnection,
          R"%(
            select json_build_object(
              'name', name,
              'lines', json_agg(
                json_build_object(
                  'lineId', rgr.line_id,
                  'route', ST_AsGeoJSON(ST_SwapOrdinates(ST_Force2D(ST_Transform(way2, 4326)), 'xy'))::json->'coordinates'
                )
              )
            ) AS group
            from route_group_routes rgr
            join planet_osm_route AS route on route.line_id = rgr.line_id
            where route_group_id = $1
            group by name
          )%"
        );

        auto result = selectFullRoutes.exec(routeGroupId);

        m_callbackFunction.BlockingCall(
          [deferred, result](Napi::Env env, Napi::Function jsCallback)
          {
            auto array = Napi::Array::New(env, result.size());

            int i {0};
            for (const auto &row: result)
            {
              Json::Value group;
              Json::Reader reader;
              reader.parse(row["group"].as<std::string>(), group);
              array[i++] = convertToNapiValue(env, group);
            }

            deferred.Resolve(array);
          }
        );
      }
      else
      {
        PreparedStatement selectRoutes(
          dbConnection,
          R"%(
            select json_build_object(
              'name', route.name,
              'lines', json_agg(
                json_build_object(
                  'lineId', rgr.line_id,
                  'route', null
                )
              )
            ) AS group
            from route_group_routes rgr
            join planet_osm_route AS route on route.line_id = rgr.line_id
            where route_group_id = $1
            group by route.name
          )%"
        );

        auto result = selectRoutes.exec(routeGroupId);

        m_callbackFunction.BlockingCall(
          [deferred, result](Napi::Env env, Napi::Function jsCallback)
          {
            auto array = Napi::Array::New(env, result.size());

            int i {0};
            for (const auto &row: result)
            {
              Json::Value group;
              Json::Reader reader;
              reader.parse(row["group"].as<std::string>(), group);
              array[i++] = convertToNapiValue(env, group);
            }

            deferred.Resolve(array);
          }
        );
      }
    }
  );

  return deferred.Promise();
}

Napi::Value PathFinder::getRouteGroups(const Napi::CallbackInfo &info)
{
  Napi::Env env = info.Env();

  if (info.Length() != 0) {
    Napi::TypeError::New(env, "too many parameters").ThrowAsJavaScriptException();
    return {};
  }

  auto deferred = postTask(
    env,
    [this](Napi::Promise::Deferred deferred)
    {
      auto dbConnection = std::make_shared<DBConnection>();

      auto results = dbConnection->exec(
        R"%(
          select id, name from route_groups
        )%"
      );

      m_callbackFunction.BlockingCall(
        [results, deferred](Napi::Env env, Napi::Function jsCallback)
        {
          auto array = Napi::Array::New(env, results.size());

          int i {0};
          for (const auto &row: results)
          {
            auto object = Napi::Object::New(env);

            object.Set("id", row["id"].as<int>());
            object.Set("name", row["name"].as<std::string>());
            object.Set("subgroups", env.Null());

            array[i++] = object;
          }

          deferred.Resolve(array);
        }
      );
    }
  );

  return deferred.Promise();
}

Napi::Value PathFinder::getTrailInfo(const Napi::CallbackInfo &info) {
  Napi::Env env = info.Env();

  if (info.Length() != 1 || !info[0].IsObject()) {
    Napi::TypeError::New(env, "Object expected").ThrowAsJavaScriptException();
  }

  auto point = info[0].As<Napi::Object>();
  auto trailInfo = getTrailFromPoint({point.Get("lat").As<Napi::Number>(),
                                      point.Get("lng").As<Napi::Number>()});

  auto object = convertToNapiValue(env, trailInfo);

  return object;
}

void PathFinder::updateRouteElevations(const Napi::CallbackInfo &info) {
  if (info.Length() != 0) {
    int lineId = info[0].As<Napi::Number>();

    ::updateRouteElevation(lineId);
  }
  else {
    ::updateRouteElevations();
  }
}

void PathFinder::updateNavNodeElevations(const Napi::CallbackInfo &info) {
  ::updateNavNodeElevations();
}

void PathFinder::updateNavEdgeCosts(const Napi::CallbackInfo &info) {
  Napi::Env env = info.Env();

  if (info.Length() != 1 || !info[0].IsArray()) {
    Napi::TypeError::New(env, "Object expected").ThrowAsJavaScriptException();
  }

  auto request = info[0].As<Napi::Array>();

  std::vector<std::vector<int>> areas;

  for (size_t i = 0; i < request.Length(); i++)
  {
    auto point = request.Get(i).As<Napi::Array>();
    int lat = point.Get(static_cast<uint32_t>(0)).As<Napi::Number>();
    int lng = point.Get(static_cast<uint32_t>(1)).As<Napi::Number>();

    areas.push_back({lat, lng});
  }

  postTask(
    env,
    [areas](Napi::Promise::Deferred deferred)
    {
      ::updateNavEdgeCosts(areas);
    }
  );
}

Napi::Value PathFinder::getGeneratePathsQueue(const Napi::CallbackInfo &info) {
  Napi::Env env = info.Env();

  if (info.Length() != 0) {
    Napi::TypeError::New(env, "No parameters expected").ThrowAsJavaScriptException();
  }

  auto queue = m_graphBuilder->getQueue();

  auto array = Napi::Array::New(env, queue.size());

  for (size_t i = 0; i < queue.size(); i++) {
    Napi::HandleScope scope(env);
    array[i] = convertToNapiValue(env, queue[i]);
  }

  return array;
}

void PathFinder::generatePaths(const Napi::CallbackInfo &info) {
  Napi::Env env = info.Env();

  if (info.Length() != 1 || !info[0].IsArray()) {
    Napi::TypeError::New(env, "Object expected").ThrowAsJavaScriptException();
  }

  auto request = info[0].As<Napi::Array>();

  LatLngBounds bounds;

  auto latlng = request.Get(static_cast<uint32_t>(0)).As<Napi::Array>();
  bounds.m_southWest.m_lat = latlng.Get(static_cast<uint32_t>(0)).As<Napi::Number>();
  bounds.m_southWest.m_lng = latlng.Get(static_cast<uint32_t>(1)).As<Napi::Number>();

  latlng = request.Get(static_cast<uint32_t>(1)).As<Napi::Array>();
  bounds.m_northEast.m_lat = latlng.Get(static_cast<uint32_t>(0)).As<Napi::Number>();
  bounds.m_northEast.m_lng = latlng.Get(static_cast<uint32_t>(1)).As<Napi::Number>();

  m_graphBuilder->postRequest(bounds);
}

void PathFinder::deleteGeneratePathRequest(const Napi::CallbackInfo &info) {
  Napi::Env env = info.Env();

  if (info.Length() != 1 || !info[0].IsArray()) {
    Napi::TypeError::New(env, "Object expected").ThrowAsJavaScriptException();
  }

  auto request = info[0].As<Napi::Array>();

  LatLngBounds bounds;

  auto latlng = request.Get(static_cast<uint32_t>(0)).As<Napi::Array>();
  bounds.m_southWest.m_lat = latlng.Get(static_cast<uint32_t>(0)).As<Napi::Number>();
  bounds.m_southWest.m_lng = latlng.Get(static_cast<uint32_t>(1)).As<Napi::Number>();

  latlng = request.Get(static_cast<uint32_t>(1)).As<Napi::Array>();
  bounds.m_northEast.m_lat = latlng.Get(static_cast<uint32_t>(0)).As<Napi::Number>();
  bounds.m_northEast.m_lng = latlng.Get(static_cast<uint32_t>(1)).As<Napi::Number>();

  m_graphBuilder->deleteRequest(bounds);
}

Json::Value AnchorsToJson(const std::vector<std::vector<Anchor>> &anchors) {
  Json::Value jsonAnchors = Json::arrayValue;

  for (auto &anchorArray : anchors) {
    Json::Value jsonAnchorArray = Json::arrayValue;

    for (auto &anchor : anchorArray) {
      Json::Value a;

      a["point"] = anchor.m_point;

      if (!anchor.m_type.empty()) {
        a["type"] = anchor.m_type;
      }

      if (anchor.m_lineEndpoint[0].m_lineId != -1) {
        a["prev"]["line_id"] = anchor.m_lineEndpoint[0].m_lineId;
        a["prev"]["fraction"] = anchor.m_lineEndpoint[0].m_fraction;
      }

      if (anchor.m_lineEndpoint[1].m_lineId != -1) {
        a["next"]["line_id"] = anchor.m_lineEndpoint[1].m_lineId;
        a["next"]["fraction"] = anchor.m_lineEndpoint[1].m_fraction;
      }

      jsonAnchorArray.append(a);
    }

    jsonAnchors.append(jsonAnchorArray);
  }

  return jsonAnchors;
}

std::shared_ptr<Graph> graph;
ThreadPool threadPool(4);

Json::Value routeFindRequest(
  const std::vector<Point> &points,
  bool returnLog,
  const std::string &searchAlgorithm)
{
  if (!graph)
  {
    graph = std::make_shared<Graph>();
  }

  std::vector<std::vector<Anchor>> anchors;

  Json::Value log;

  // Start each of the search pairs.
  for (size_t i = 0; i < points.size() - 1; i++)
  {
    // Generate nodeIds for the points.
    SearchController controller(graph, searchAlgorithm, points[i], points[i + 1], returnLog);

    auto searchPairAnchors = controller.search(threadPool);

    anchors.push_back(searchPairAnchors[0]);

    if (returnLog)
    {
      log = controller.getSearchLog();
    } 
  }

  graph->m_loadNodes.printResults();
  
  Json::Value result;

  result["anchors"] = AnchorsToJson(anchors);
  result["log"] = log;

  return result;
}

Napi::Value PathFinder::findRoute(const Napi::CallbackInfo &info) {
  Napi::Env env = info.Env();

  if (info.Length() != 2 || !info[0].IsArray() || !info[1].IsObject()) {
    Napi::TypeError::New(env, "Array expected").ThrowAsJavaScriptException();
    return {};
  }

  std::vector<Point> points;

  auto pointArray = info[0].As<Napi::Array>();

  for (size_t i = 0; i < pointArray.Length(); i++) {
    auto point = pointArray.Get(i).As<Napi::Object>();
    points.emplace_back(Point{point.Get("lat").As<Napi::Number>(),
                              point.Get("lng").As<Napi::Number>()});
  }

  auto options = info[1].As<Napi::Object>();
  bool returnLog = false;
  if (options.Has("returnLog"))
  {
    returnLog = options.Get("returnLog").As<Napi::Boolean>();
  }

  std::string searchAlgorithm = "BiDiA*";
  if (options.Has("searchAlgorithm"))
  {
    searchAlgorithm = options.Get("searchAlgorithm").As<Napi::String>();
  }

  auto deferred = postTask(
    env,
    [this, points, returnLog, searchAlgorithm](Napi::Promise::Deferred deferred)
    {
      try
      {
        auto result = routeFindRequest(points, returnLog, searchAlgorithm);

        m_callbackFunction.BlockingCall(
          [result, deferred](Napi::Env env, Napi::Function jsCallback)
          {
            deferred.Resolve(convertToNapiValue(env, result));
          });
      }
      catch (...)
      {
        m_callbackFunction.BlockingCall(
          [deferred](Napi::Env env, Napi::Function jsCallback)
          {
            deferred.Reject(Napi::String::New(deferred.Env(), "Error"));
          });
      }
    }
  );

  return deferred.Promise();
}

void PathFinder::taskLoop()
{
  for (;;)
  {
    {
      std::unique_lock<std::mutex> lock(m_queueMutex);

      if (m_queue.size() > 0)
      {
        auto func = m_queue.front();
        m_queue.pop_front();

        lock.unlock();

        try
        {
          func();
        }
        catch(const std::exception& e)
        {
          std::cerr << e.what() << '\n';
        }
      }
    }

    {
      std::unique_lock<std::mutex> lock(m_queueMutex);

      if (m_queue.size() == 0)
      {
        if (m_stopThreads)
        {
          break;
        }

        m_queueCondition.wait(lock, [this]{ return m_queue.size() != 0; });
      }
    }
  }
}

Napi::Promise::Deferred PathFinder::postTask(
  Napi::Env env, std::function<void(Napi::Promise::Deferred)> task
)
{
  Napi::Promise::Deferred deferred = Napi::Promise::Deferred::New(env);

  std::unique_lock<std::mutex> lock(m_queueMutex);
  m_queue.push_back(
    [this, deferred, task]
    {
      try
      {
        task(deferred);
      }
      catch (const pqxx::pqxx_exception &e) {
        std::cerr << e.base().what() << std::endl;
        const pqxx::sql_error *s = dynamic_cast<const pqxx::sql_error *>(&e.base());
        if (s) {
          std::cerr << "Query was: " << s->query() << std::endl;
        }

        m_callbackFunction.BlockingCall(
          [deferred](Napi::Env env, Napi::Function jsCallback)
          {
            deferred.Reject(Napi::String::New(deferred.Env(), "Error"));
          });
      }
      catch (std::exception &e)
      {
        std::cerr << e.what() << std::endl;

        m_callbackFunction.BlockingCall(
          [deferred](Napi::Env env, Napi::Function jsCallback)
          {
            deferred.Reject(Napi::String::New(deferred.Env(), "Error"));
          });
      }
      catch(...) {
        std::cerr << "unknown error" << std::endl;
      }
    }
  );

  m_queueCondition.notify_all();

  return deferred;
}
