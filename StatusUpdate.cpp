#include "StatusUpdate.h"
#include "JsonToNapi.h"
#include <string>

const std::string StatusRoutesUpdating {"routesUpdating"};
const std::string StatusUpdatingRoutes {"updatingRoutes"};
const std::string StatusRoutesUpdated {"routesUpdated"};
const std::string StatusUpdateIntersectionCount {"updateIntersectionCount"};
const std::string StatusSearchStarted {"searchStarted"};
const std::string StatusSearchCompleted {"searchCompleted"};

static Napi::ThreadSafeFunction statusUpdateCallback {};

void setStatusUpdateCallback(Napi::ThreadSafeFunction &callback)
{
  statusUpdateCallback = callback;
}

void statusUpdate(Json::Value value)
{
  if (statusUpdateCallback)
  {
    auto result = statusUpdateCallback.Acquire();

    if (result == napi_ok)
    {
      statusUpdateCallback.BlockingCall(
        [value](Napi::Env env, Napi::Function jsCallback)
        {
          jsCallback.Call({ convertToNapiValue(env, value) });
        }
      );

      statusUpdateCallback.Release();
    }
  }
}

