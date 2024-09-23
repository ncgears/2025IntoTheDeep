package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.hwMecanumFtclib;

@Autonomous(name="Do Nothing", group="JRB")
//@Disabled
public class
autonDoNothing extends OpMode {
    hwMecanumFtclib robot = new hwMecanumFtclib(this);

    // driver presses init
    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    // repeatedly until driver presses play
    @Override
    public void init_loop() {
    }

    // driver presses start
    @Override
    public void start() {
    }

    // repeatedly until driver presses stop or interrupted
    @Override
    public void loop() {
    }

    // driver presses stop
    @Override
    public void stop() {
    }

}
