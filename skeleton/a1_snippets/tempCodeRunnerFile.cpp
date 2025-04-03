pfms::nav_msgs::Odometry odo;
    pfms::PlatformType type = pfms::PlatformType::ACKERMAN;

    unsigned long i = 0;
    /* produce messages */
    for(i = 0; i < atoi(argv[1]); i ++) {
        UGV ugv {
                    i,
                    atof(argv[2]),
                    atof(argv[3]),
                    atof(argv[4])
                 };
        pipesPtr->send(ugv);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        cout << "wrote:" << i << endl;