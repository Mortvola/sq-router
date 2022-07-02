{
  'variables': {
      'pkg-config': 'pkg-config',
  },
  "targets": [{
    "target_name": "sqrouter",
    "cflags!": [
      "-fno-exceptions",
      ],
    "cflags_cc": [
        '-std=c++17',
        '-Wno-shadow',
        '-fPIC',
        '-O2',
      ],
    'cflags_cc!': [
      '-fno-rtti',
      "-fno-exceptions",
    ],
    "sources": [
        "pathFinder.cpp",
        "Elevation.cpp",
        "ElevationFile.cpp",
        "./Database/DBConnection.cpp",
        "./Database/DBTransaction.cpp",
        "./Database/PreparedStatement.cpp",
        "GraphBuilder/GraphBuilder.cpp",
        "Cost.cpp",
        "Map.cpp",
        "Route.cpp",
        "Graph.cpp",
        "Edge.cpp",
        "Node.cpp",
        "SearchAlgs/Search.cpp",
        "SearchAlgs/AStarSearch.cpp",
        "SearchAlgs/BiDiAStarSearch.cpp",
        "SearchNode.cpp",
        "SearchController.cpp",
        "StatusUpdate.cpp",
        "JsonToNapi.cpp",
        "SearchLog/SearchLog.cpp",
        "SearchLog/SearchLogEntry.cpp",
        "ThreadPool.cpp"
    ],
    'include_dirs': [
        "<!@(node -p \"require('node-addon-api').include\")",
      ".",
      "./SearchAlgs",
      "./SearchLog",
    ],
    "libraries": [
      '-ljsoncpp',
      '-lpqxx',
      '-lpq',
    ],        
    'dependencies': [
        "<!(node -p \"require('node-addon-api').gyp\")"
    ],
    'defines': [
      'NAPI_CPP_EXCEPTIONS',
      'NODE_API_SWALLOW_UNTHROWABLE_EXCEPTIONS'
    ],
    'conditions': [
      ['OS == "linux"', {
        'include_dirs': [
          "/usr/include/jsoncpp",
        ],
      },
      {
        "xcode_settings": {
          'CLANG_CXX_LANGUAGE_STANDARD': 'c++17',
          'GCC_ENABLE_CPP_EXCEPTIONS': 'YES',
          'GCC_ENABLE_CPP_RTTI': 'YES',
          'MACOSX_DEPLOYMENT_TARGET': '12.0',
          'OTHER_LDFLAGS': [
            '-L/opt/homebrew/lib',
          ],
        },
        'include_dirs': [
          "/opt/homebrew/include",
        ],
      }]
    ],
  }],
}
