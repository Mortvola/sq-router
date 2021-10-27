{
    'variables': {
        'pkg-config': 'pkg-config',
        'mapnik-config': 'mapnik-config'
    },
    "targets": [{
        "target_name": "sqrouter",
        "cflags!": [
        	"-fno-exceptions",
        	],
        "cflags_cc": [
            '<!@(<(mapnik-config) --cflags --cxxflags)',
            '-std=c++17',
            '-Wno-shadow',
            '-fPIC',
            '-O2'
        	],
        'cflags_cc!': [
          '-fno-rtti',
        	"-fno-exceptions",
        ],
        "sources": [
            "pathFinder.cpp",
            "Elevation.cpp",
            "ElevationFile.cpp",
            "DBConnection.cpp",
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
            '<!@(<(mapnik-config) --libs --dep-libs)',
            '-ljsoncpp',
            '-lgdal',
            '-lpqxx',
            '-lpq',
          ],        
        'dependencies': [
            "<!(node -p \"require('node-addon-api').gyp\")"
        ],
        'defines': [
          'NAPI_CPP_EXCEPTIONS',
          'NODE_API_SWALLOW_UNTHROWABLE_EXCEPTIONS'
         ]
    }]
}
