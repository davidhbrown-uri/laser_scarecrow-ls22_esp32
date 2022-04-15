enum ls_laser_mode_t {
    LS_LASER_OFF,
    LS_LASER_ON,
    LS_LASER_MAPPED,
    LS_LASER_PULSE,
};

enum ls_laser_mode_t IRAM_ATTR ls_laser_mode;

void ls_laser_set_mode(ls_laser_mode_t requested_mode);