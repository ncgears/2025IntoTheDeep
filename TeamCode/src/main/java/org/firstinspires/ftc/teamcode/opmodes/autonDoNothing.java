package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Size;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.hardware.hwMecanumFtclib;
import org.firstinspires.ftc.teamcode.pidcontrollers.pidDriveControllerFtclib;
import org.firstinspires.ftc.teamcode.pidcontrollers.pidElevatorController;
import org.firstinspires.ftc.teamcode.pidcontrollers.pidTiltController;
import org.firstinspires.ftc.teamcode.pidcontrollers.pidTurnControllerFtclib;
import org.firstinspires.ftc.teamcode.processors.tseSaturationProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

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
