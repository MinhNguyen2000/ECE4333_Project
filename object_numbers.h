// #define BEAR 0
// #define BULDOZER 1
// #define CONE_RED 2
// #define CONE_BLUE 3
// #define CONE_YELLOW 4
// #define CONE_GREEN 5
// #define CONE_ORANGE 6

enum object_type {
    NO_OBJECT = 0,
    BEAR = 1,
    BULLDOZER = 2,
    CONE_RED = 3,
    STOP_SIGN = 4,
    CONE_BLUE = 5,
    CONE_YELLOW = 6,
    CONE_GREEN = 7,
    CONE_ORANGE = 8
};

struct camera_classification {
    uint8_t label_encode;
    uint32_t x;
    uint32_t y;
    uint32_t width;
    uint32_t height;
};


