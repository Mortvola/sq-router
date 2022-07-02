#pragma once

#include <napi.h>
#include <json/json.h>

Napi::Value convertToNapiValue(Napi::Env env, const Json::Value &value);
