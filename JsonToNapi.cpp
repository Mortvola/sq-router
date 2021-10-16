#include "JsonToNapi.h"

Napi::Value convertToNapiValue(Napi::Env env, const Json::Value &value)
{
  if (value.isArray()) {
    auto array = Napi::Array::New(env, value.size());

    for (size_t i = 0; i < value.size(); i++) {
      Napi::HandleScope scope(env);
      array[i] = convertToNapiValue(
          env, value[static_cast<Json::Value::ArrayIndex>(i)]);
    }

    return array;
  }

  if (value.isObject()) {
    auto object = Napi::Object::New(env);

    for (const auto &name : value.getMemberNames()) {
      Napi::HandleScope scope(env);
      object.Set(name, convertToNapiValue(env, value[name]));
    }

    return object;
  }

  if (value.isNumeric()) {
    return Napi::Number::New(env, value.asDouble());
  }

  if (value.isString()) {
    return Napi::String::New(env, value.asString());
  }

  if (value.isBool()) {
    return Napi::Boolean::New(env, value.asBool());
  }

  return env.Null();
}

