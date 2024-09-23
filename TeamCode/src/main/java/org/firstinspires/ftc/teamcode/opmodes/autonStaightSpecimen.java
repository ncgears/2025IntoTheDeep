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
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="Straight Specimen Auton", group="JRB")
//@Disabled
public class
autonStaightSpecimen extends OpMode {
    boolean m_long_auton = false; //set true if this is the long auton
    hwMecanumFtclib robot = new hwMecanumFtclib(this);
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime elapsed = new ElapsedTime();
    StateMachine machine = null;
    Constants.Manipulator.Positions m_manip_pos = Constants.Manipulator.Positions.START;
    Constants.Commands m_last_command = Constants.Commands.NONE;
    double m_last_command_time = 0.0;
    double m_turn_multiplier = 1.0;

    double drive_fwd, drive_strafe, drive_turn = 0.0; //used for holding requested drive values
    double pid_drive_target = 0; //target ticks for pid drive
    double pid_turn_target = 0; //target degrees for pid turn
    boolean pid_driving, pid_turning = false; //tracking if we are using these pid controllers
    pidDriveControllerFtclib drivepid = new pidDriveControllerFtclib(this, pid_drive_target, Constants.Drivetrain.driveController.kP, Constants.Drivetrain.driveController.kI, Constants.Drivetrain.driveController.kD, Constants.Drivetrain.driveController.kF, Constants.Drivetrain.driveController.kIZone);
    pidTurnControllerFtclib turnpid = new pidTurnControllerFtclib(this, pid_turn_target, Constants.Drivetrain.turnController.kP, Constants.Drivetrain.turnController.kI, Constants.Drivetrain.turnController.kD, Constants.Drivetrain.turnController.kF, Constants.Drivetrain.turnController.kIZone);
    pidTiltController tiltpid = new pidTiltController(this, m_manip_pos.getTilt(), Constants.Manipulator.tiltController.kP, Constants.Manipulator.tiltController.kI, Constants.Manipulator.tiltController.kD, Constants.Manipulator.tiltController.kF, Constants.Manipulator.tiltController.kIZone);
    pidElevatorController elevpid = new pidElevatorController(this, m_manip_pos.getElevator(), Constants.Manipulator.elevatorController.kP, Constants.Manipulator.elevatorController.kI, Constants.Manipulator.elevatorController.kD, Constants.Manipulator.elevatorController.kF, Constants.Manipulator.elevatorController.kIZone);

    boolean strafing = false;

    boolean isDone_speciment1 = false;
    boolean needSample1, needSample2, needSample3 = false;
    boolean isAtSample1, isAtSample2, isAtSample3 = false;

    // States for the finite state machine
    enum States {
        DETERMINE_TEAM, //Determine Alliance and set robot.alliance
        MANIP_TRANSPORT1, //Move manipulator to transport
        MANIP_SUB_SPECIMEN_HIGH, //Move manipulator to high specimen bar
        DRIVE_SUB_SPECIMEN_HIGH, //Drive to the submersible high specimen bar
        MANIP_SUB_SPECIMEN_LOW, //Move manipulator to low specimen bar
        DRIVE_SUB_SPECIMENT_LOW, //Drive to the submersible low specimen bar
        MANIP_TRANSPORT2, //Move manipulator to transport
        RESTING //Doing nothing
    }

    // driver presses init
    @Override
    public void init() {
        robot.init(hardwareMap);

        machine = new StateMachineBuilder()
                /** Identify which alliance we are */
                .state(States.DETERMINE_TEAM)
                .onEnter( () -> {
                    if(robot.alliance == Constants.Alliance.NONE) runCommand(Constants.Commands.DETERMINE_TEAM);
                })
                .onExit( () -> {
                    m_turn_multiplier = (robot.alliance == Constants.Alliance.RED) ? -1.0 : 1.0; //If red alliance, turns are reversed
                    robot.playAudio(String.format("%s", robot.alliance.toString()),500);
                })
//                .transitionWithPointerState( () -> (robot.alliance != Constants.Alliance.NONE), States.STRAFE_CLEAR)
                .transition( () -> (robot.alliance != Constants.Alliance.NONE))
                /** Move manipulator to transport position */
                .state(States.MANIP_TRANSPORT1)
                .onEnter( () -> {
                    m_manip_pos = Constants.Manipulator.Positions.TRANSPORT;
                })
                .transition(() -> (true))
                /** Move manipulator to specimen high position */
                .state(States.MANIP_SUB_SPECIMEN_HIGH)
                .onEnter( () -> {
                    m_manip_pos = Constants.Manipulator.Positions.SPECIMEN_HIGH;
                })
                .transition(() -> (true))
                /** Drive to the high specimen bar */
                .state(States.DRIVE_SUB_SPECIMEN_HIGH)
                .onEnter( () -> {
                    elapsed.reset();

                    double distance = 22;
                    driveInchesPID(distance);
                })
                .onExit( () -> {
                    pid_driving = false;
                })
                .transition( () -> (pid_driving && drivepid.atTarget()) )
                /** Move manipulator to transport position */
                .state(States.MANIP_TRANSPORT2)
                .onEnter( () -> {
                    m_manip_pos = Constants.Manipulator.Positions.TRANSPORT;
                })
                .transition(() -> (true))
                /** Wait until end of auton */
                .state(States.RESTING)
                .onEnter( () -> {
                    pid_driving = false;
                    pid_turning = false;
                    strafing = false;
                    robot.playAudio("Resting",500);
                })
                .build();
    }

    // repeatedly until driver presses play
    @Override
    public void init_loop() {
        if(m_last_command == Constants.Commands.NONE || robot.alliance == Constants.Alliance.NONE) runCommand(Constants.Commands.DETERMINE_TEAM); //determine team
        if(robot.driverOp.getButton(GamepadKeys.Button.BACK) && runtime.seconds() - m_last_command_time > 1) runCommand(Constants.Commands.GYRO_RESET); //listen for gyro reset request
        //if(robot.driverOp.getButton(GamepadKeys.Button.START) && runtime.seconds() - m_last_command_time > 1) runCommand(Constants.Commands.TOGGLE_PIXEL);
        if(runtime.seconds() - m_last_command_time > 2.0) { //reset imu every 2 seconds during init
            robot.imu.resetYaw();
        }
        if(m_last_command != Constants.Commands.NONE && runtime.seconds() - m_last_command_time > 2) { //reset the last command after 2 seconds
            runCommand(Constants.Commands.NONE);
        }
        // Update all telemetry data
        telem(true);
    }

    // driver presses start
    @Override
    public void start() {
        runtime.reset();
        machine.start();
    }

    // repeatedly until driver presses stop or interrupted
    @Override
    public void loop() {
        machine.update();

        // PID Driving
        drivepid.setTarget(pid_drive_target);
        turnpid.setTarget(pid_turn_target);
        drive_fwd = (pid_driving) ? drivepid.update(robot.getDriveAvgPosition()) : 0.0;
        drive_strafe = (strafing) ? -Constants.Auton.autonStrafeSpeed * m_turn_multiplier : 0.0;
        //in auton, drivestraight is easy since our target is always set by the auton and never the driver
        drive_turn = (pid_turning || Constants.Auton.autonDriveStraight) ? -turnpid.update(robot.getRobotYaw()) : 0.0;
        autonDrive(drive_fwd, drive_strafe, drive_turn, robot.getRobotYaw());

//this isnt working.. not sure why
//        if (pid_turning && turnpid.atTarget(robot.getRobotYaw())) pid_turning = false;
//        if (pid_driving && drivepid.atTarget()) pid_driving = false;

        if(m_last_command != Constants.Commands.NONE && runtime.seconds() - m_last_command_time > 2) { //reset the last command after 2 seconds
            runCommand(Constants.Commands.NONE);
        }

        //handle moving the manipulator
        moveElevator();
        moveTilt();

        // Update all telemetry data
        telem(false);
    }

    // driver presses stop
    @Override
    public void stop() {
        runCommand(Constants.Commands.ROBOT_RESET);
    }

    public void autonDrive(double fwd, double strafe, double turn, double headingDegrees) {
//        robot.drive.driveFieldCentric(strafe,fwd,turn,headingDegrees);
        robot.drive.driveRobotCentric(strafe,fwd,turn);
    }

    public boolean isPidDriving() { return pid_driving; }
    public boolean isPidTurning() { return pid_turning; }

    public void runCommand(Constants.Commands command) {
        m_last_command = command;
        m_last_command_time = runtime.seconds();
        switch (command) {
            case GYRO_RESET:
                robot.imu.resetYaw();
                robot.playAudio("Reset Gyro", 500);
                break;
            case INTAKE_IN:
                robot.setIntakeDirection(Constants.Intake.Directions.IN);
            case INTAKE_OUT:
                robot.setIntakeDirection(Constants.Intake.Directions.OUT);
            case INTAKE_STOP:
                robot.setIntakeDirection(Constants.Intake.Directions.STOP);
            case DETERMINE_TEAM:
                robot.alliance = robot.determineAlliance();
                break;
            case ROBOT_RESET:
                break;
            case NONE:
            default:
        }
    }

    public void turnPID(double degrees) {
        turnToPID(degrees + robot.getRobotYaw());
    }

    public void turnToPID(double targetAngle) {
        pid_turning = true;
        pid_turn_target = targetAngle;
    }

    public void driveInchesPID(double targetInches) {
        pid_driving = true;
        robot.resetAllDriveEncoder();
        pid_drive_target = targetInches * Constants.Drivetrain.driveController.ticksPerInch + robot.getDriveAvgPosition();
    }

    public void moveTilt() {
        tiltpid.setTargetPosition(m_manip_pos);
        double power = tiltpid.update(robot.getTiltPosition());
        robot.setTiltPower(power);
    }

    public void moveElevator() {
        elevpid.setTargetPosition(m_manip_pos);
        double power = elevpid.update(robot.getElevatorPosition());
        robot.setElevatorPower(power);
    }

    private void telem(boolean idle) {
        telemetry.addData("Alliance", robot.alliance.toString());
        telemetry.addData("Last Command", m_last_command.toString());
        telemetry.addData("Robot Heading", "%.2f", robot.getRobotYaw());
        telemetry.addData("Obstacle Distance", "%.2f Inches", robot.getDistance());
        telemetry.addData("Manipulator Position", m_manip_pos.toString());
        telemetry.addData("Auton State", machine.getState().toString());
        robot.getDriveAvgPosition(); //put the encoder data on the telem
        telemetry.addData("Tilt", "lim=%s, tgt=%.0f, pos=%d, pwr=%.2f", robot.getTiltLimitString(), tiltpid.getTarget(), robot.getTiltPosition(), robot.getTiltPower());
        telemetry.addData("Elev", "lim=%s, tgt=%.0f, pos=%d, pwr=%.2f", robot.getElevatorLimitString(), elevpid.getTarget(), robot.getElevatorPosition(), robot.getElevatorPower());
        if(idle) { //items that are only in idle
        } else {
            telemetry.addData("OpMode", "Run Time: %.2f", runtime.seconds());
            telemetry.addData("Robot Drive", "fwd=%.2f, str=%.2f, turn=%.2f", drive_fwd, drive_strafe, drive_turn);
            if(pid_driving) telemetry.addData("PID Drive", "target=%.0f, error=%.0f", pid_drive_target, drivepid.getLastError());
            if(pid_turning) telemetry.addData("PID Turn", "target=%.0f, error=%.0f", pid_turn_target, turnpid.getLastError());
        }
        //telemetry.update(); //this is called automatically every loop
    }
}
