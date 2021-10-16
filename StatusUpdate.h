#pragma once
#include <napi.h>
#include <jsoncpp/json/json.h>

extern const std::string StatusRoutesUpdating;
extern const std::string StatusUpdatingRoutes;
extern const std::string StatusRoutesUpdated;
extern const std::string StatusUpdateIntersectionCount;

void setStatusUpdateCallback(Napi::ThreadSafeFunction &callback);

void statusUpdate(Json::Value value);

