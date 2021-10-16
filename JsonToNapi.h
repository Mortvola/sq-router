#pragma once

#include <napi.h>
#include <jsoncpp/json/json.h>

Napi::Value convertToNapiValue(Napi::Env env, const Json::Value &value);
