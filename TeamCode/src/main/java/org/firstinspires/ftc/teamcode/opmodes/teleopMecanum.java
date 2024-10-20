package org.firstinspires.ftc.teamcode.opmodes;

//import com.arcrobotics.ftclib.command.InstantCommand;
import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.hardware.hwMecanumFtclib;
import org.firstinspires.ftc.teamcode.pidcontrollers.pidTurnControllerFtclib;
import org.firstinspires.ftc.teamcode.pidcontrollers.pidElevatorController;
import org.firstinspires.ftc.teamcode.pidcontrollers.pidTiltController;
import org.firstinspires.ftc.teamcode.StateMachines;

@SuppressWarnings({"unused"})
@TeleOp(name="Mecanum Drive", group="JRB")
public class teleopMecanum extends OpMode {
    hwMecanumFtclib robot = new hwMecanumFtclib(this);
    OpMode opmode = this;
    StateMachine specimenMachine, sampleMachine, climbMachine;
    StateMachine globalMachine;

    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime elapsed = new ElapsedTime();
//    Constants.Manipulator.Positions m_manip_pos = Constants.Manipulator.Positions.START;
    Constants.Manipulator.Positions m_manip_prev_pos = Constants.Manipulator.Positions.START;
    boolean m_manip_momentary = false;
    Constants.Manipulator.Positions m_last_manip_pos = Constants.Manipulator.Positions.SPECIMEN_HIGH;
    boolean m_manip_manual = false;
    boolean m_climbing = false;
    String m_last_command = Constants.Commands.NONE.toString();
    double m_last_command_time = 0.0;
    double m_turn_multiplier = 1.0;

    double drive_fwd, drive_strafe, drive_turn = 0.0; //used for holding requested drive values
    double ds_heading = 0; //locked heading for drivestraight
    boolean ds_locked = false; //used for drivestraight tracking
    double pid_turn_target = 0; //target degrees for pid turn
    boolean pid_turning = false; //tracking if we are using these pid controllers
    pidTurnControllerFtclib turnpid = new pidTurnControllerFtclib(this, pid_turn_target, Constants.Drivetrain.turnController.kP, Constants.Drivetrain.turnController.kI, Constants.Drivetrain.turnController.kD, Constants.Drivetrain.turnController.kF, Constants.Drivetrain.turnController.kIZone);
    pidTiltController tiltpid = new pidTiltController(this, robot.getManipulatorPosition().getTilt(), Constants.Manipulator.tiltController.kP, Constants.Manipulator.tiltController.kI, Constants.Manipulator.tiltController.kD, Constants.Manipulator.tiltController.kF, Constants.Manipulator.tiltController.kIZone);
    pidElevatorController elevpid = new pidElevatorController(this, robot.getManipulatorPosition().getElevator(), Constants.Manipulator.elevatorController.kP, Constants.Manipulator.elevatorController.kI, Constants.Manipulator.elevatorController.kD, Constants.Manipulator.elevatorController.kF, Constants.Manipulator.elevatorController.kIZone);
    boolean tilt_low_limit, tilt_high_limit = false;

    boolean d_a, d_b, d_x, d_y, d_lb, d_lt = false; //for debouncing driver button presses
    boolean o_rb, o_lb, o_up, o_dn, o_lt, o_rt, o_tl, o_tr = false; //for debouncing operator button presses

    public enum States {
        INIT,
        TRANSPORT,
        SPECIMEN_PICKUP,
        SAMPLE_PICKUP,
        CLIMB,
        IDLE
    }

    @Override
    public void init() {
        robot.init(hardwareMap);

        /*
         * The globalMachine is a state machine that handles creating child state machines for
         * semi-automated tasks.  The IDLE state waits for conditions and starts other state
         * machines as needed.
         */
        specimenMachine = StateMachines.getSpecimenPickupMachine(robot, opmode);
        sampleMachine = StateMachines.getSamplePickupMachine(robot, opmode);
        climbMachine = StateMachines.getClimbMachine(robot, opmode);
        globalMachine = new StateMachineBuilder()
            /* INIT prepares the manipulator as needed */
            .state(States.INIT)
                .transition( () -> (true), States.IDLE)
            /* IDLE waits for transition conditions, defines buttons */
            .state(States.IDLE)
                .transition(() -> robot.operOp.getButton(GamepadKeys.Button.X), States.SPECIMEN_PICKUP)
                .transition(() -> robot.operOp.getButton(GamepadKeys.Button.B), States.SAMPLE_PICKUP)
                .transition(() -> !m_climbing && robot.driverOp.getButton(GamepadKeys.Button.DPAD_LEFT), States.CLIMB)
            /* SAMPLE_PICKUP starts the sample state machine */
            .state(States.SAMPLE_PICKUP)
                .onEnter(sampleMachine::start)
                .loop(sampleMachine::update)
                .onExit( () -> {
                    sampleMachine.reset();
                    sampleMachine.stop();
                })
                .transition( () -> sampleMachine.getState() == StateMachines.SamplePickup.DONE, States.IDLE)
            /* SPECIMEN_PICKUP starts the specimen state machine */
            .state(States.SPECIMEN_PICKUP)
                .onEnter(specimenMachine::start)
                .loop(specimenMachine::update)
                .onExit( () -> {
                    specimenMachine.reset();
                    specimenMachine.stop();
                })
                .transition( () -> specimenMachine.getState() == StateMachines.SpecimenPickup.DONE, States.IDLE)
            /* CLIMB starts the climbing state machine */
            .state(States.CLIMB)
                .onEnter(() -> {
                    m_climbing = true;
                    climbMachine.start();
                })
                .loop(climbMachine::update)
                .onExit( () -> {
                    m_climbing = false;
                    climbMachine.reset();
                    climbMachine.stop();
                })
                .transition( () -> climbMachine.getState() == StateMachines.Climb.DONE, States.IDLE)
            /* Build the state machine */
            .build();
    }

    // repeatedly until driver presses play
    @Override
    public void init_loop() {
        if(m_last_command.equals("NONE") || robot.alliance == Constants.Alliance.NONE) telemCommand("DETERMINE TEAM"); //determine team and store it
        // always listen for gyro reset button
        if(robot.driverOp.getButton(GamepadKeys.Button.BACK) && runtime.seconds() - m_last_command_time > 0.5) {
            robot.imu.resetYaw();
            robot.playAudio("Reset Gyro", 500);
            telemCommand("RESET GYRO");
        }
        // command name updates for telemetry
        if(!m_last_command.equals("NONE") && runtime.seconds() - m_last_command_time > 2) { //reset the last command after 2 seconds
            telemCommand("NONE");
        }
        // Update all telemetry data
        telem(true);
    }

    // driver presses start
    @Override
    public void start() {
        runtime.reset();
        globalMachine.start();
        m_turn_multiplier = (robot.alliance == Constants.Alliance.RED) ? -1.0 : 1.0;
//        robot.setIntakeDirection(Constants.Intake.Directions.STOP);
    }

    @Override
    public void loop() {
        globalMachine.update();
        /* /this prevents "commented out code" warning
        drive_fwd = (pid_turning) ? 0.0 : robot.driverOp.getLeftY(); //if pid turning, no throttle
        drive_strafe = (pid_turning) ? 0.0 : robot.driverOp.getLeftX(); //if pid turning, no strafing
        drive_turn = (pid_turning) ? turnpid.update(robot.getRobotYaw()) : 0.0;
         */
        drive_fwd = distanceCorrectedPower(stickDeadband(robot.driverOp.getLeftY()));
        drive_strafe = distanceCorrectedPower(stickDeadband(robot.driverOp.getLeftX()));
        drive_turn = stickDeadband(robot.driverOp.getRightX());
        if (drive_turn != 0.0) { //we have requested a turn using the joystick
            pid_turning = false; //disable pid turning
            ds_locked = false; //unlock drivestraight heading
        }

        if (robot.driveStraight) {
            /* useDriveStraight:
             * If we are not requesting a turn and locked to a heading,
             * lock the current robot heading
             */
            if(drive_turn == 0.0 && !ds_locked) {
                ds_heading = robot.getRobotYaw();
                ds_locked = true;
            }
        }

        if (pid_turning) { //set new values for joysticks if we requested pid turning
            ds_locked = false; //always unlock the driveStraight when using pid turning
            // update the pid controller
            turnpid.setTarget(pid_turn_target);
            if (turnpid.atTarget(robot.getRobotYaw())) {
                drive_turn = 0.0;
                pid_turning = false;
            } else {
                drive_turn = -turnpid.update(robot.getRobotYaw());
            }
        } else if (robot.driveStraight && ds_locked) { //not pid turning, but useDriveStraight enabled and we have a heading locked
            pid_turn_target = ds_heading; //set a new heading
            turnpid.setTarget(pid_turn_target);
            drive_turn = -turnpid.update(robot.getRobotYaw());
        }

        // perform the drive
        if (robot.getRobotYaw() == 0.0 || !robot.fieldCentric) { //exactly 0 from the imu is unlikely, fall back to robot centric
            robot.drive.driveRobotCentric(drive_strafe, drive_fwd, drive_turn);
        } else {
            robot.drive.driveFieldCentric(drive_strafe, drive_fwd, drive_turn, robot.getRobotYaw());
        }

        /* Driver Controls */
        // always listen for gyro reset button
        if (robot.driverOp.getButton(GamepadKeys.Button.BACK)) {
            robot.imu.resetYaw();
            robot.playAudio("Reset Gyro", 500);
            telemCommand("RESET GYRO");
        } else if (robot.driverOp.getButton(GamepadKeys.Button.START)) {
            robot.setManipulatorPosition(Constants.Manipulator.Positions.START);
            m_manip_manual = true;
            try {
                moveElevator(false);
                wait(3000);
                moveTilt(false);
            } catch (Exception e) {
                //do nothing
            } finally {
                m_manip_manual = false;
            }
            telemCommand("STARTING CONFIG");
        } else if (!d_y && robot.driverOp.getButton(GamepadKeys.Button.Y)) {
            d_y = true;
            turnToPID(0);
            telemCommand("PID TURN FC 0");
        } else if (!d_b && robot.driverOp.getButton(GamepadKeys.Button.B)) {
            d_b = true;
            turnToPID(-90 * m_turn_multiplier);
            telemCommand("PID TURN FC -90");
        } else if (!d_a && robot.driverOp.getButton(GamepadKeys.Button.A)) {
            d_a = true;
            turnToPID(180);
            telemCommand("PID TURN FC 180");
        } else if (!d_x && robot.driverOp.getButton(GamepadKeys.Button.X)) {
            d_x = true;
            turnToPID(90 * m_turn_multiplier);
            telemCommand("PID TURN FC 90");
        } else if (d_y && !robot.driverOp.getButton(GamepadKeys.Button.Y)) { //released the button
            d_y = false;
        } else if (d_b && !robot.driverOp.getButton(GamepadKeys.Button.B)) { //released the button
            d_b = false;
        } else if (d_a && !robot.driverOp.getButton(GamepadKeys.Button.A)) { //released the button
            d_a = false;
        } else if (d_x && !robot.driverOp.getButton(GamepadKeys.Button.X)) { //released the button
            d_x = false;
        }

        // toggle field centric drive
        if (!d_lb && robot.driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            d_lb = true;
            robot.fieldCentric = !robot.fieldCentric;
            robot.playAudio((robot.fieldCentric) ? "Field Centric" : "Robot Centric", 500);
        }
        else if (d_lb && !robot.driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER)) { d_lb = false; }
        if (d_lt && robot.driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < 0.5) {
            d_lt = false;
        } else if (!d_lt && robot.driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.5) {
            d_lt = true;
            robot.driveStraight = !robot.driveStraight;
            robot.playAudio((robot.driveStraight) ? "Heading Lock Yes" : "Heading Lock No", 500);
        }

        // automated field-relative turn functions for d-pad
        if (robot.driverOp.getButton(GamepadKeys.Button.DPAD_UP) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_LEFT) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
            //up, but not left or right
            robot.setManipulatorPosition(Constants.Manipulator.Positions.CLIMB_READY);
            telemCommand("CLIMB READY");
        } else if (robot.driverOp.getButton(GamepadKeys.Button.DPAD_RIGHT) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_DOWN) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_UP)) {
            //right, but not up or down
            robot.setManipulatorPosition(Constants.Manipulator.Positions.CLIMB_UP);
            telemCommand("CLIMB READY");
        } else if (robot.driverOp.getButton(GamepadKeys.Button.DPAD_DOWN) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_LEFT) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
            //down, but not left or right
            robot.setManipulatorPosition(Constants.Manipulator.Positions.CLIMB_LIFT);
            telemCommand("CLIMB LIFT");
        }
        /* else if (robot.driverOp.getButton(GamepadKeys.Button.DPAD_UP) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_LEFT) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
            //up, but not left or right
            m_manip_pos = Constants.Manipulator.Positions.CLIMB_READY;
            telemCommand("CLIMB READY");
        }
        */
        /* End Driver Controls */

        /* Operator Controls */
        if (o_tr && robot.operOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.5) { //release intake button
            telemCommand("INTAKE STOP");
            o_tr = false;
            robot.setIntakeDirection(Constants.Intake.Directions.STOP);
        } else if (!o_tr && robot.operOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.5) { //press intake button
            telemCommand("INTAKE IN");
            o_tr = true;
            robot.setIntakeDirection(Constants.Intake.Directions.IN);
        }
        if (robot.operOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < 0.5 && m_manip_manual) { //release manual control button
            m_manip_manual = false;
            elevpid.setTarget(robot.getElevatorPosition());
            tiltpid.setTarget(robot.getTiltPosition());
            telemCommand("PID MANIP");
        } else if (robot.operOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.5 && !m_manip_manual) { //press manual control button
            m_manip_manual = true;
            telemCommand("MANUAL MANIP");
        } else if (!o_rb && robot.operOp.getButton(GamepadKeys.Button.RIGHT_BUMPER)) { //position up
            o_rb = true;
            switch (robot.getManipulatorPosition()) {
                case SAMPLE_LOW:
                    robot.setManipulatorPosition(Constants.Manipulator.Positions.SAMPLE_HIGH);
                    m_last_manip_pos = robot.getManipulatorPosition();
                    telemCommand("SAMPLE HIGH");
                    break;
                case SPECIMEN_LOW:
                    robot.setManipulatorPosition(Constants.Manipulator.Positions.SPECIMEN_HIGH);
                    m_last_manip_pos = robot.getManipulatorPosition();
                    telemCommand("SPECIMEN HIGH");
                    break;
                default:
//                        telemCommand("NOTHING");
            }
        } else if (!o_lb && robot.operOp.getButton(GamepadKeys.Button.LEFT_BUMPER)) { //position down
            o_lb = true;
            switch (robot.getManipulatorPosition()) {
                case SAMPLE_HIGH:
                    robot.setManipulatorPosition(Constants.Manipulator.Positions.SAMPLE_LOW);
                    m_last_manip_pos = robot.getManipulatorPosition();
                    telemCommand("SAMPLE LOW");
                    break;
                case SPECIMEN_HIGH:
                    robot.setManipulatorPosition(Constants.Manipulator.Positions.SPECIMEN_LOW);
                    m_last_manip_pos = robot.getManipulatorPosition();
                    telemCommand("SPECIMEN LOW");
                    break;
                default:
//                    telemCommand("NOTHING");
            }
        } else if (robot.operOp.getButton(GamepadKeys.Button.Y)) { //specimen high
            robot.setManipulatorPosition(Constants.Manipulator.Positions.SPECIMEN_HIGH);
            telemCommand("SPECIMEN HIGH");
        } else if (robot.operOp.getButton(GamepadKeys.Button.X)) { //transport
            robot.setManipulatorPosition(Constants.Manipulator.Positions.TRANSPORT);
            telemCommand("TRANSPORT POSITION");
        } else if (robot.operOp.getButton(GamepadKeys.Button.A)) { //sample pickup
            robot.setManipulatorPosition(Constants.Manipulator.Positions.SAMPLE_PICKUP);
            telemCommand("SAMPLE PICKUP");
        } else if (robot.operOp.getButton(GamepadKeys.Button.B)) { //specimen pickup
            robot.setManipulatorPosition(Constants.Manipulator.Positions.SPECIMEN_PICKUP);
            telemCommand("SPECIMEN PICKUP");
        } else if (robot.operOp.getButton(GamepadKeys.Button.START)) { //sample high
            robot.setManipulatorPosition(Constants.Manipulator.Positions.SAMPLE_HIGH);
            telemCommand("SAMPLE HIGH");
        } else if (o_rb && !robot.operOp.getButton(GamepadKeys.Button.RIGHT_BUMPER)) { //released the button
            o_rb = false;
        } else if (o_lb && !robot.operOp.getButton(GamepadKeys.Button.LEFT_BUMPER)) { //released the button
            o_lb = false;
        }

        /* DPAD_UP and DPAD_DOWN handles running the intake */
        if (!o_up && robot.operOp.getButton(GamepadKeys.Button.DPAD_UP)) { //intake out
            telemCommand("INTAKE OUT");
            robot.setIntakeDirection(Constants.Intake.Directions.OUT);
            o_up = true;
        } else if (!o_dn && robot.operOp.getButton(GamepadKeys.Button.DPAD_DOWN)) { //intake in
            telemCommand("INTAKE IN");
            robot.setIntakeDirection(Constants.Intake.Directions.IN);
            o_dn = true;
        } else if ((o_up || o_dn) && !robot.operOp.getButton(GamepadKeys.Button.DPAD_UP) && !robot.operOp.getButton(GamepadKeys.Button.DPAD_DOWN)) { //released the button
            telemCommand("INTAKE STOP");
            robot.setIntakeDirection(Constants.Intake.Directions.STOP);
            o_up = false;
            o_dn = false;
        }

        /* DPAD_LEFT and DPAD_RIGHT handles adjusting the elevator offset */
        /*
        if (!o_rt && robot.operOp.getButton(GamepadKeys.Button.DPAD_RIGHT)) { //elev offset up
            o_rt = true;
            elevpid.increaseOffset();
        } else if (!o_lt && robot.operOp.getButton(GamepadKeys.Button.DPAD_LEFT)) { //elev offset down
            o_lt = true;
            elevpid.decreaseOffset();
        } else if (o_lt && !robot.operOp.getButton(GamepadKeys.Button.DPAD_LEFT)) { //released the button
            o_lt = false;
        } else if (o_rt && !robot.operOp.getButton(GamepadKeys.Button.DPAD_RIGHT)) { //released the button
            o_rt = false;
        }
        */
        /* End Operator Controls */

        // Update the manipulator - these should be called every loop to make the manipulator move to target position
        moveElevator(m_manip_manual);
        moveTilt(m_manip_manual);

        // command name updates for telemetry
        if(!m_last_command.equals("NONE") && runtime.seconds() - m_last_command_time > 2) { //reset the last command after 2 seconds
            telemCommand("NONE");
        }
        // Update all telemetry data
        telem(false);
    }

    public void telemCommand(@NonNull String command) {
        m_last_command = command;
        m_last_command_time = runtime.seconds();
        switch (command) {
            case "DETERMINE TEAM":
                robot.alliance = robot.determineAlliance();
                break;
            case "RESET ROBOT":
                break;
            case "NONE":
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

    public double stickDeadband(double value) {
        if (Math.abs(value) <= Constants.Global.stickDeadbandMin) return 0.0;
        if (Math.abs(value) >= Constants.Global.stickDeadbandMax) return Math.signum(value);
        return (Constants.Drivetrain.useInputSquaring) ? value * value * Math.signum(value) : value;
    }

    public double distanceCorrectedPower(double power) {
        if (robot.ignoreDistance || !Constants.Drivetrain.woahNelly.enabled) return power;
        double yaw = robot.getRobotYaw();
        if (yaw >= ((Constants.Drivetrain.woahNelly.direction - Constants.Drivetrain.woahNelly.directionThreshold) * m_turn_multiplier)
        && yaw <= ((Constants.Drivetrain.woahNelly.direction + Constants.Drivetrain.woahNelly.directionThreshold) * m_turn_multiplier)) {
            if (robot.getDistance() <= Constants.Drivetrain.woahNelly.distance) {
                telemetry.addData("Speeding", "WOAH NELLY!");
                power *= Constants.Drivetrain.woahNelly.multiplier;
            }
        }
//        if (robot.getDistance() <= Constants.Drivetrain.woahNelly) return power * Constants.Drivetrain.woahNellyMultiplier;
        return power;
    }

    public double getTiltManualPower() {
        //get requested power from oper stick
        double power = stickDeadband(robot.operOp.getRightY()) * -1.0; //inverted
        /* this is handled in setPower
        //prevent moving if we are at limits
        if(power < 0 && robot.getTiltLowLimit()) power = 0;
        if(power > 0 && robot.getTiltHighLimit()) power = 0;
         */
        return power;
    }

    public void moveTilt(boolean manual) {
        if(Constants.Manipulator.tiltController.disabled) return;
        if(manual) {
            robot.setManipulatorPosition(Constants.Manipulator.Positions.MANUAL);
            //move the tilt
            double power = getTiltManualPower();
            if(power==0) {
                power = tiltpid.update(robot.getTiltPosition());
            } else {
                tiltpid.setTarget(robot.getTiltPosition());
            }
            robot.setTiltPower(power);
        } else {
            if (tilt_low_limit) {
                tiltpid.setTarget(robot.getTiltPosition());
                tilt_low_limit = false;
            } else if (tilt_high_limit) {
                tiltpid.setTarget(robot.getTiltPosition());
                tilt_high_limit = false;
            } else {
                tiltpid.setTargetPosition(robot.getManipulatorPosition());
            }
            double power = tiltpid.update(robot.getTiltPosition());
            if (power < 0 && robot.getTiltLowLimit()) {
                tiltpid.setTarget(robot.getTiltPosition());
                tilt_low_limit = true;
            }
            if (power > 0 && robot.getTiltHighLimit()) {
                tiltpid.setTarget(robot.getTiltPosition());
                tilt_high_limit = true;
            }
            robot.m_tilt_atTarget = tiltpid.atTarget();
            robot.setTiltPower(power);
        }
    }

    public double getElevatorManualPower() {
        //get requested power from oper stick
        double power = stickDeadband(robot.operOp.getLeftY()); //non-inverted
        /* this is handled in setPower
        //prevent moving if we are at limits
        if(power < 0 && robot.getElevatorLowLimit()) power = 0;
        if(power > 0 && robot.getElevatorHighLimit()) power = 0;
         */
        return power;
    }

    public void moveElevator(boolean manual) {
        if(Constants.Manipulator.elevatorController.disabled) return;
        if(manual) {
            //move the elevator
            robot.setElevatorPower(getElevatorManualPower());
            robot.setManipulatorPosition(Constants.Manipulator.Positions.MANUAL);
            elevpid.setTarget(robot.getElevatorPosition());
        } else {
            elevpid.setTargetPosition(robot.getManipulatorPosition());
            double power = elevpid.update(robot.getElevatorPosition());
            robot.m_elev_atTarget = elevpid.atTarget();
            robot.setElevatorPower(power);
        }
    }

    private void telem(boolean idle) { //items that are visible in all modes
        telemetry.addData("Alliance", robot.alliance.toString());
        telemetry.addData("Last Command", m_last_command);
        telemetry.addData("Robot Drive", "%s Centric", (robot.fieldCentric) ? "Field" : "Robot");
        telemetry.addData("Heading Lock", (robot.driveStraight) ? "YES" : "NO");
        telemetry.addData("Robot Heading", "%.2f", robot.getRobotYaw());
        telemetry.addData("Obstacle Distance", "%.2f Inches", robot.getDistance());
        telemetry.addData("Robot State", globalMachine.getState().toString());
        telemetry.addData("Specimen Pickup State", specimenMachine.getState().toString());
        telemetry.addData("Sample Pickup State", sampleMachine.getState().toString());
        telemetry.addData("Climb State", climbMachine.getState().toString());
        telemetry.addData("Manipulator Position", robot.getManipulatorPosition().toString());
        telemetry.addData("Scoop", "up=%s, full=%s",robot.getScoopUpString(), robot.getScoopFullString());
        if(!Constants.Intake.disabled) {
            telemetry.addData("Intake Direction", robot.getIntakeDirection().toString());
        } else {
            telemetry.addData("Intake Direction", "DISABLED");
        }
        if(!Constants.Manipulator.tiltController.disabled) {
            telemetry.addData("Tilt", "lim=%s, tgt=%.0f, pos=%d, pwr=%.2f", robot.getTiltLimitString(), tiltpid.getTarget(), robot.getTiltPosition(), robot.getTiltPower());
        } else {
            telemetry.addData("Tilt","%s","DISABLED");
        }
        if(!Constants.Manipulator.elevatorController.disabled) {
            telemetry.addData("Elev", "lim=%s, tgt=%.0f, pos=%d, pwr=%.2f", robot.getElevatorLimitString(), elevpid.getTarget(), robot.getElevatorPosition(), robot.getElevatorPower());
        } else {
            telemetry.addData("Elev","%s","DISABLED");
        }
        if(m_manip_manual) telemetry.addData("Manual", "tilt=%.2f, elev=%.2f", getTiltManualPower(), getElevatorManualPower());
        if(idle) { //items that are only in idle
            robot.noop();
        } else { //items that are only while running
            telemetry.addData("OpMode", "Run Time: %.2f", runtime.seconds());
            telemetry.addData("Robot Drive", "fwd=%.2f, str=%.2f, turn=%.2f", drive_fwd, drive_strafe, drive_turn);
//        if(pid_driving) telemetry.addData("PID Drive", "target=%.0f, error=%.0f", pid_drive_target, drivepid.getLastError());
            if(pid_turning) telemetry.addData("PID Turn", "target=%.0f, error=%.0f", pid_turn_target, turnpid.getLastError());
        }
        //telemetry.update(); //this is called automatically every loop
    }

}
