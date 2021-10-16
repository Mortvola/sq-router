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
            '-O0',
            '-g'
        	],
        'cflags_cc!': [
            '-fno-rtti',
        	"-fno-exceptions",
        ],
        "sources": [
            "pathFinder.cpp",
            "Elevation.cpp",
            "DBConnection.cpp",
            "graphBuilder.cpp",
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
        	'NAPI_DISABLE_CPP_EXCEPTIONS',
         ]
    }]
}