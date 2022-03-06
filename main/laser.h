enum ls_laser_mode {
    LS_LASER_OFF,
    LS_LASER_PULSE,
    LS_LASER_MAPPED,
    LS_LASER_ON
};

void ls_laser_set_mode(ls_laser_mode requested_mode);