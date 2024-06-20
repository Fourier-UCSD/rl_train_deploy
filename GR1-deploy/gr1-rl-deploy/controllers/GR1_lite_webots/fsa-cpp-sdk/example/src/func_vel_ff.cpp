int TestVelff(std::vector<double> &setpos, std::vector<double> &getpos, std::vector<double> &getpos_velff) {
    std::vector<double> setposition;
    std::vector<double> getposition;
    std::vector<double> getposition_vel_ff;
    double pos1;
    double vel1;
    double cur1;
    double max_t = 1;
    double dt = 0.0025;
    double dt_count = round(10 / dt);

    for (int i = 0; i < 400; i++) {

        fsa1.GetPVC(pos1, vel1, cur1);
        double set_position = 90 * cos(2 * M_PI / max_t * double(i) * dt) - 90;
        double set_velocity = -2 * M_PI / max_t * 90 * sin(2 * M_PI / max_t * double(i) * dt);
        fsa1.SetPosition(set_position, 0, 0);
        setposition.push_back(abs(set_position));
        getposition.push_back(abs(pos1));

        usleep(2500);
    }

    sleep(2);

    for (int i = 0; i < 400; i++) {

        fsa1.GetPVC(pos1, vel1, cur1);

        double set_position = 90 * cos(2 * M_PI / max_t * double(i) * dt) - 90;
        double set_velocity = -2 * M_PI / max_t * 90 * sin(2 * M_PI / max_t * double(i) * dt);
        fsa1.SetPosition(set_position, set_velocity, 0);

        getposition_vel_ff.push_back(abs(pos1));

        usleep(2500);
    }

    auto max_element_setposition = std::max_element(setposition.begin(), setposition.end());
    auto max_element_getposition = std::max_element(getposition.begin(), getposition.end());
    auto max_element_getposition_vel_ff = std::max_element(getposition_vel_ff.begin(), getposition_vel_ff.end());

    int maxValue_setposition;
    int maxIndex_setposition;

    int maxValue_getposition;
    int maxIndex_getposition;

    int maxValue_getposition_vel_ff;
    int maxIndex_getposition_vel_ff;

    if (max_element_setposition != setposition.end()) {
        maxValue_setposition = *max_element_setposition;
        maxIndex_setposition = std::distance(setposition.begin(), max_element_setposition);
    }

    if (max_element_getposition != getposition.end()) {
        maxValue_getposition = *max_element_getposition;
        maxIndex_getposition = std::distance(getposition.begin(), max_element_getposition);
    }

    if (max_element_getposition_vel_ff != getposition_vel_ff.end()) {
        maxValue_getposition_vel_ff = *max_element_getposition_vel_ff;
        maxIndex_getposition_vel_ff = std::distance(getposition_vel_ff.begin(), max_element_getposition_vel_ff);
    }

    setpos = setposition;
    getpos = getposition;
    getpos_velff = getposition_vel_ff;

    if (maxIndex_getposition_vel_ff > maxIndex_setposition && maxIndex_getposition_vel_ff < maxIndex_setposition + 5) {

        std::cout << "correct" << std::endl;
        return 1
    } else {
        std::cout << "wrong" << std::endl;
        return -1
    }
}
