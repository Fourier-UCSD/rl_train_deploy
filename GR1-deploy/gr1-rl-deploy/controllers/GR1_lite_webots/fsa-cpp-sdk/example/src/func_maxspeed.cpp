void TestMaxSpeed(std::vector<double> &vel, std::vector<double> &cur) {
    std::vector<double> setvelocity;
    std::vector<double> getcurrent;
    double pos1;
    double vel1;
    double cur1;
    for (int i = 0; i < 1000; i++) {

        fsa1.GetPVC(pos1, vel1, cur1);
        fsa1.SetVelocity(0, 0);

        setvelocity.push_back(vel1);
        setvelocity.push_back(cur1);
        // fsa3.SetVelocity(2, 0);
        // fsa4.SetVelocity(2, 0);

        // fsa5.SetVelocity(2, 0);
        // fsa6.SetVelocity(2, 0);
        // long long res1 = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        // std::cout << "end: " << res1 - res << std::endl;
        // std::cout << "pos: " << pos1 << "vel: " << vel1 << "cur: " << cur1 << std::endl;

        // usleep(2000);

        usleep(2500)
    }

    for (int i = 0; i < 4000; i++) {

        fsa1.GetPVC(pos1, vel1, cur1);
        fsa1.SetVelocity(600, 0);

        setvelocity.push_back(vel1);
        setvelocity.push_back(cur1);

        usleep(2500);
    }

    for (int i = 0; i < 1000; i++) {

        fsa1.GetPVC(pos1, vel1, cur1);
        fsa1.SetVelocity(0, 0);

        setvelocity.push_back(vel1);
        setvelocity.push_back(cur1);

        usleep(2500);
    }

    vel = setvelocity;
    cur = getcurrent;
}
