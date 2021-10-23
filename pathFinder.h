#pragma once

#include "Map.h"
#include <map>
#include <memory>
#include <condition_variable>
#include <mutex>
#include <deque>
#include <thread>
#include <atomic>
#include <napi.h>

class PathFinder : public Napi::ObjectWrap<PathFinder> {
public:
  static Napi::Object Init(Napi::Env env, Napi::Object exports);

  PathFinder(const Napi::CallbackInfo &info);

private:
  struct Configuration {
    std::string elevationDirectory{"."};
    std::string databaseUsername;
    std::string databasePassword;
    std::string database;
    std::string databaseHost;
    std::string eleUsername;
    std::string elePassword;
  };

  void initialize();

  static Napi::FunctionReference constructor;

  Napi::Value whatIsHere(const Napi::CallbackInfo &info);
  Napi::Value elevation(const Napi::CallbackInfo &info);
  Napi::Value elevationArea(const Napi::CallbackInfo &info);
  Napi::Value elevationTile(const Napi::CallbackInfo &info);
  void generatePaths(const Napi::CallbackInfo &info);
  void generatePathsInArea(const Napi::CallbackInfo &info);
  Napi::Value getHikeDistance(const Napi::CallbackInfo &info);
  Napi::Value getTrailInfo(const Napi::CallbackInfo &info);
  Napi::Value findRoute(const Napi::CallbackInfo &info);
  void updateRouteElevations(const Napi::CallbackInfo &info);
  void updateNavNodeElevations(const Napi::CallbackInfo &info);
  void updateNavEdgeCosts(const Napi::CallbackInfo &info);
  Napi::Value getSearchLog(const Napi::CallbackInfo &infO);
  Napi::Value getNodeCounts(const Napi::CallbackInfo &info);
  void updateIntersectionCounts(const Napi::CallbackInfo &info);
  Napi::Value addQuadrangle(const Napi::CallbackInfo &info);
  Napi::Value removeQuadrangle(const Napi::CallbackInfo &info);
  Napi::Value search(const Napi::CallbackInfo &info);
  Napi::Value getRoutes(const Napi::CallbackInfo &info);
  void addRouteGroup(const Napi::CallbackInfo &info);
  Napi::Value addRouteGroupRoutes(const Napi::CallbackInfo &info);
  Napi::Value getRouteGroupRoutes(const Napi::CallbackInfo &info);
  Napi::Value getRouteGroups(const Napi::CallbackInfo &info);
  Napi::Value addSegment(const Napi::CallbackInfo &info);
  
  Napi::ThreadSafeFunction m_callbackFunction;

  std::vector<std::thread> m_taskThreads;
  std::deque<std::function<void()>> m_queue;
  std::mutex m_queueMutex;
  std::condition_variable m_queueCondition;
  std::atomic<bool> m_stopThreads {false};
  void taskLoop();
  Napi::Promise::Deferred postTask(
    Napi::Env env, std::function<void(Napi::Promise::Deferred)> task
  );

  Configuration m_configuration;
};
