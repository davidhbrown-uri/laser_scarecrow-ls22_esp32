#pragma once
enum ls_event_types {
    // fire when the magnet on the rotating arm enter/leave the detection area of the Hall-effect sensor
    LSEVT_MAGNET_ENTER, LSEVT_MAGNET_LEAVE // <! no value for magnet events
};
typedef struct ls_event {
    ls_event_types type;
    void *value;    
};
