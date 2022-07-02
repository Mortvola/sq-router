#pragma once
#include <napi.h>
#include <json/json.h>

extern const std::string StatusRoutesUpdating;
extern const std::string StatusUpdatingRoutes;
extern const std::string StatusRoutesUpdated;
extern const std::string StatusGenRequestRemoved;
extern const std::string StatusUpdateIntersectionCount;
extern const std::string StatusSearchStarted;
extern const std::string StatusSearchCompleted;

void setStatusUpdateCallback(Napi::ThreadSafeFunction &callback);

void statusUpdate(Json::Value value);

