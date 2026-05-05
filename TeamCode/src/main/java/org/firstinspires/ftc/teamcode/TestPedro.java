package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;

public class TestPedro {
    public static void test(Follower follower) {
        follower.startTeleopDrive();
        follower.setTeleOpDrive(0, 0, 0, false);
    }
}
