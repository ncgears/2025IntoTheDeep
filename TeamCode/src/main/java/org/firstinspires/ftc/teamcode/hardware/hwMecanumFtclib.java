/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.Constants.Alliance;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;

/*
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or... In OnBot Java, add a new file named RobotHardware.java, select this sample, and select Not an OpMode.
 * Also add a new OpMode, select the sample ConceptExternalHardwareClass.java, and select TeleOp.
 *
 */

public class hwMecanumFtclib {

    /* Public OpMode members. */

    // Define Motor and Servo objects
    public Motor m_motor_fl, m_motor_fr, m_motor_rl, m_motor_rr = null;
//    public Motor[] m_motors = null;
    public GamepadEx driverOp, operOp = null;
    public MecanumDrive drive = null;
    public ServoEx m_pixelservo = null;
    public ServoEx m_droneservo = null;

    public IMU imu = null;

    // Alliance Flag related stuff
    public DigitalChannel m_flag_a, m_flag_b = null;
    public Alliance alliance = Alliance.NONE;
    public boolean fieldCentric = true;
    public boolean driveStraight = true;

    // Manipulator related stuff
    public Motor m_tilt_motor, m_elev_motor = null;
    public DigitalChannel m_tilt_lim_low, m_tilt_lim_high, m_elev_lim_low, m_elev_lim_high = null;

    // Pixel Dropper related stuff
    public Constants.PixelDropper.Positions m_pixel_position = null;

    // Drone Launcher related stuff
    public Constants.DroneLauncher.Positions m_drone_position = null;

    // Distance sensor
    public SensorRevTOFDistance m_distance = null;
    public boolean ignoreDistance = false;

    // Audio Telemetry
    public boolean isSpeaking = false;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode

    /* Local OpMode members. */
    HardwareMap hwMap = null;
    OpMode myOpMode = null;   // gain access to methods in the calling OpMode.
    boolean teleop = false;
//    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public hwMecanumFtclib(OpMode opmode) {
        myOpMode = opmode;
    }
    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        // Define and Initialize Motors
        imu = hwMap.get(IMU.class, "imu"); //imu is BNO055 in EH
        //imu = hwMap.get(IMU.class, "imu2"); //imu2 is BHI260AP in CH
        //This is used for deciding to reset the elevator and tilt
        teleop = (myOpMode.toString().indexOf("teleopMecanum") > 0); //"org.firstinspires.ftc.teamcode.opmodes.teleopMecanum"

        /* //Get reference to the relative layout so we can change the color of the robot controller app to match the alliance color
        int relativeLayoutId = hwMap.appContext.getResources().getIdentifier("RelativeLayout","id", hwMap.appContext.getPackageName());
        relativeLayout = ((Activity) hwMap.appContext).findViewById(relativeLayoutId);
        */

        m_motor_fl = new Motor(hwMap, "fl drive");
        m_motor_fr = new Motor(hwMap, "fr drive");
        m_motor_rl = new Motor(hwMap, "rl drive");
        m_motor_rr = new Motor(hwMap, "rr drive");
        Motor[] m_motors = {m_motor_fl, m_motor_fr, m_motor_rl, m_motor_rr};
        drive = new MecanumDrive(false, m_motor_fl, m_motor_fr, m_motor_rl, m_motor_rr); //do not invert right, we handle this later to fix encoders too
        fieldCentric = Constants.Drivetrain.useFieldCentric;
        driveStraight = Constants.Drivetrain.useDriveStraight;
        ignoreDistance = false;

        // Gamepads
        driverOp = new GamepadEx(myOpMode.gamepad1);
        operOp = new GamepadEx(myOpMode.gamepad2);

        try {
            // Alliance Flag switches
            m_flag_a = hwMap.get(DigitalChannel.class, "flag sw a");
            m_flag_b = hwMap.get(DigitalChannel.class, "flag sw b");
            m_flag_a.setMode(DigitalChannel.Mode.INPUT);
            m_flag_b.setMode(DigitalChannel.Mode.INPUT);
            alliance = determineAlliance();
        } catch(Exception e) {
            myOpMode.telemetry.addLine("ERROR: Could not init flag switches!");
        }

        try {
            // Tilt (tilt)
            m_tilt_lim_low = hwMap.get(DigitalChannel.class, "tilt sw low"); //normally closed
            m_tilt_lim_high = hwMap.get(DigitalChannel.class, "tilt sw high"); //normally closed
            m_tilt_motor = new Motor(hwMap, "tilt motor");
            m_tilt_motor.setRunMode(Motor.RunMode.RawPower);
            m_tilt_motor.setInverted(false);
//            if(!teleop) m_tilt_motor.resetEncoder();
            m_tilt_motor.resetEncoder();
        } catch(Exception e) {
            myOpMode.telemetry.addLine("ERROR: Could not init Tilt");
        }

        try {
            // Elevator
            m_elev_lim_low = hwMap.get(DigitalChannel.class, "elev sw low"); //normally closed
            m_elev_lim_high = hwMap.get(DigitalChannel.class, "elev sw high"); //normally closed
            m_elev_motor = new Motor(hwMap, "elev motor");
            m_elev_motor.setRunMode(Motor.RunMode.RawPower);
            m_elev_motor.setInverted(false);
//            if(!teleop) m_elev_motor.resetEncoder();
            m_elev_motor.resetEncoder();
        } catch(Exception e) {
            myOpMode.telemetry.addLine("ERROR: Could not init Elevator");
        }

        try {
            // Pixel Dropper
            m_pixelservo = new SimpleServo(hwMap, "pixel servo", 0, 30);
            m_pixelservo.setInverted(false);
            setPixelPosition(Constants.PixelDropper.Positions.DOWN);
        } catch (Exception e) {
            myOpMode.telemetry.addLine("ERROR: Could not init Pixel Servo");
        }

        try {
            // Drone Launcher
            m_droneservo = new SimpleServo(hwMap, "drone servo", 0, 30);
            m_droneservo.setInverted(false);
            setDronePosition(Constants.DroneLauncher.Positions.ARMED);
        } catch (Exception e) {
            myOpMode.telemetry.addLine("ERROR: Could not init Pixel Servo");
        }

        try {
            // Distance Sensor
            m_distance = new SensorRevTOFDistance(hwMap, "distance");
        } catch (Exception e) {
            myOpMode.telemetry.addLine("ERROR: Could not init Distance Sensor");

        }

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        setMotorInverted(false,true,false,true);

        // Set the runmode for each motor
        for (Motor m : m_motors) {
            m.setRunMode(Motor.RunMode.RawPower);
        }

        // Reset the encoders for each motor
        for (Motor m : m_motors) {
            m.resetEncoder();
        }

        // Adjust the orientation parameters of the IMU
        // 2023 - imu2 is LEFT, imu is RIGHT
        IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(imuParams);

        myOpMode.telemetry.addData("Robot", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    public void setMotorInverted(boolean fl, boolean fr, boolean rl, boolean rr) {
        m_motor_fl.setInverted(fl);
        m_motor_rl.setInverted(rl);
        m_motor_fr.setInverted(fr);
        m_motor_rr.setInverted(rr);
    }

    public void setAllDrivePower(double p) {
        setDrivePower(p,p,p,p);
    }

    public void setDrivePower(double fl, double fr, double rl, double rr) {
        m_motor_fl.set(fl);
        m_motor_fr.set(fr);
        m_motor_rl.set(rl);
        m_motor_rr.set(rr);
    }

    public void resetAllDriveEncoder() {
        Motor[] m_motors = {m_motor_fl, m_motor_fr, m_motor_rl, m_motor_rr};
        for (Motor m : m_motors) {
            m.resetEncoder();
        }
    }

    public int getDriveAvgPosition() {
        return getDriveAvgPosition(false);
    }
    public int getDriveAvgPosition(boolean frontOnly) {
        int fl = m_motor_fl.getCurrentPosition();
        int fr = m_motor_fr.getCurrentPosition();
        int rl = m_motor_rl.getCurrentPosition();
        int rr = m_motor_rr.getCurrentPosition();

        //right
        double right = (frontOnly) ? fr : Math.min(Math.abs(fr),Math.abs(rr)) * Math.signum(fr);
        //left
        double left = (frontOnly) ? fl : Math.min(Math.abs(fl),Math.abs(rl)) * Math.signum(fl);
        //average
        int avg = (int) ((right + left) / 2);
//        myOpMode.telemetry.addData("drive enc", "fl=%d, rl=%d, fr=%d, rr=%d, avg=%d", fl, rl, fr, rr, avg);
        return avg;
    }

    public double getRobotYaw() {
        try {
            return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        } catch(Exception e) {
            return 0.0;
        }
    }

    public Alliance determineAlliance() {
        try {
            //digital channels have pull-up resistor so they are high normally
            if(m_flag_a.getState() && m_flag_b.getState()) return Alliance.RED; //if neither switches, red flag
            if(m_flag_a.getState() || m_flag_b.getState()) return Alliance.BLUE; //if one switch, blue flag
            return Alliance.NONE; //indeterminate, either flag not installed or a wiring fault
        } catch(Exception e) {
            myOpMode.telemetry.addLine("ERROR: Unable to determine alliance");
            return Alliance.NONE;
        }
    }

    // Tilt Methods
    public int getTiltPosition() { return m_tilt_motor.getCurrentPosition(); }
    public double getTiltPower() { return m_tilt_motor.get(); }
    public boolean getTiltLowLimit() { return m_tilt_lim_low.getState(); }
    public boolean getTiltHighLimit() { return m_tilt_lim_high.getState(); }
    public String getTiltLimitString() {
        String lim = "";
        lim += (getTiltLowLimit()) ? "L" : "";
        lim += (getTiltHighLimit()) ? "H" : "";
        return (lim=="") ? "O" : lim;
    }
    public void setTiltPower(double power) {
        if(power < 0 && getTiltLowLimit()) {
            myOpMode.telemetry.addLine("ERROR: Tilt is at low limit");
//            playAudio("Tilt Low Limit", 500);
            power = 0;
        }
        if(power > 0 && getTiltHighLimit()) {
            myOpMode.telemetry.addLine("ERROR: Tilt is at high limit");
//            playAudio("Tilt High Limit", 500);
            power = 0;
        }
        m_tilt_motor.set(power);
    }
    public void homeTilt() {
        if(!getTiltLowLimit()) m_tilt_motor.set(-Constants.Manipulator.tiltController.homingSpeed);
        else {
            m_tilt_motor.stopAndResetEncoder();
        }
    }

    // Elevator Methods
    public int getElevatorPosition() { return m_elev_motor.getCurrentPosition(); }
    public double getElevatorPower() { return m_elev_motor.get(); }
    public boolean getElevatorLowLimit() { return !m_elev_lim_low.getState(); }
    public boolean getElevatorHighLimit() { return !m_elev_lim_high.getState(); }
    public String getElevatorLimitString() {
        String lim = "";
        lim += (getElevatorLowLimit()) ? "L" : "";
        lim += (getElevatorHighLimit()) ? "H" : "";
        return (lim=="") ? "O" : lim;
    }
    public void homeElevator() {
        if(!getElevatorLowLimit()) m_elev_motor.set(-Constants.Manipulator.elevatorController.homingSpeed);
        else {
            m_elev_motor.stopAndResetEncoder();
        }
    }
    public void setElevatorPower(double power) {
        if(power < 0 && getElevatorLowLimit()) {
            myOpMode.telemetry.addLine("ERROR: Elevator is at low limit");
            power = 0;
        }
        if(power > 0 && getElevatorHighLimit()) {
            myOpMode.telemetry.addLine("ERROR: Elevator is at high limit");
            power = 0;
        }
        m_elev_motor.set(power);
    }

    // Pixel Dropper Methods
    public void setPixelPosition(Constants.PixelDropper.Positions position) {
        m_pixel_position = position;
        m_pixelservo.turnToAngle(position.getAngle());
    }
    public Constants.PixelDropper.Positions getPixelPosition() {
        return m_pixel_position;
    }

    // Drone Launcher Methods
    public void setDronePosition(Constants.DroneLauncher.Positions position) {
        m_drone_position = position;
        m_droneservo.turnToAngle(position.getAngle());
    }
    public Constants.DroneLauncher.Positions getDronePosition() {
        return m_drone_position;
    }

    // Distance Sensor Methods
    public double getDistance() {
        if(m_distance.getDistance(DistanceUnit.INCH) > 40) return -1.0;
        return m_distance.getDistance(DistanceUnit.INCH);
    }

    // Driver Station Audio
    public void playAudio(String text, int delayMs) {
        try {
            myOpMode.telemetry.speak(text);
            wait(delayMs);
        } catch (Exception e) {}
    }

}
