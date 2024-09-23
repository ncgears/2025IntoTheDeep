package org.firstinspires.ftc.teamcode;

@SuppressWarnings("unused")
public class Constants {
    public enum Commands { NONE, ROBOT_RESET, GYRO_RESET, TOGGLE_PIXEL, DETERMINE_TEAM }
    public enum Alliance { RED, BLUE, NONE }
    public enum TSELocation { LEFT, MIDDLE, RIGHT, NONE }
    public static class Global {
        public static double stickDeadbandMin = 0.1; //10%, anything less than this is considered to be 0%
        public static double stickDeadbandMax = 0.95; //95%, anything over this is considered to be 100%
    }
    public static class Auton {
        public static double autonDriveSpeed = 0.5;
        public static double autonStrafeSpeed = 0.6;
        public static boolean autonDriveStraight = true;
    }
    public static class Drivetrain {
        public static boolean useFieldCentric = true; //try to use FC if gyro has value
        public static boolean useDriveStraight = false; //use DriveStraight correction
        public static class woahNelly {
            public static boolean enabled = false; // use woahNelly system to slow down at backstage
            public static double distance = 10.0;//Inches from obstacle to adjust throttle when aimed at backstage
            public static double multiplier = 0.4;//Speed reduction
            public static double direction = 90; //direction for woahNelly
            public static double directionThreshold = 10.0; //degrees within direction for throttling
        }
        public static class turnController {
            public static double targetThreshold = 1.0; //how many degrees is close enough
            public static double kF = 0.00; //0.0 //minimum power to move the robot
            public static double kP = 0.055; //0.04
            public static double kI = 0.00011; //0.0006;
            public static double kIZone = 5.0; //I Zone for accumulating error
            public static double kD = 0.45; //0.004;
            public static class limits {
                public static double maxOutput = 0.55; //maximum output power
            }
        }
        public static class driveController {
            public static double ticksPerRev = 28;
            public static double gearReduction = 20;
            public static double wheelDiamMM = 75;
            public static double wheelCircumferenceMM = wheelDiamMM * Math.PI;
            public static double ticksPerMM = ticksPerRev * gearReduction / wheelCircumferenceMM;
            public static double ticksPerInch = ticksPerMM * 25.4;
            public static double targetThreshold = 1.0; //how many inches is close enough
            public static double targetThresholdTicks = targetThreshold * ticksPerInch;
            public static double kF = 0.15; //0.0 //minimum power to move the robot
            public static double kP = 0.0005;
            public static double kI = 0.00002;
            public static double kIZone = 5.0 * ticksPerInch; //I Zone for accumulating error
            public static double kD = 0.003;
            public static class limits {
                public static double maxOutput = 0.55; //maximum output power
            }
        }
    }

    public static class PixelDropper {
        public enum Positions {
            DOWN(0.0),
            UP(20.0);
            private final double angle;
            Positions(double angle) { this.angle = angle; }
            public double getAngle() { return this.angle; }
        }
    }
    public static class DroneLauncher {
        public enum Positions {
            ARMED(0.0),
            LAUNCH(10);
            private final double angle;
            Positions(double angle) { this.angle = angle; }
            public double getAngle() { return this.angle; }
        }
    }

    /**
     * The Manipulator constants relate to the top-end system above the drivetrain.
     * The manipulator contains 2 major parts, the elevator and the extend.
     * tilt - This controls the angle of the delivery "bucket" and ranges from ~-19 degrees (floor pickup) to ~+58 degrees (hanger) relative to the floor plane
     * elevator - This controls the length of the extended arm with the delivery bucket and ranges from 0 inches to 12.3 inches
     */
    public static class Manipulator {
        public enum Positions {
            //NAME(angle,length,distance)
            //tilt = (double) position of the tilt, in encoder counts, from the low limit switch reference
            //elevator = (double) length of the elevator, in inches
            //distance = (double) robot distance from backstage (-1 if not used)
            ZERO(0,0,-1.0),
            START(30,0.0,-1.0),
            TRANSPORT(1750,0.0,-1.0),
            FLOOR_CLOSE(50,0.0,-1.0),
            FLOOR_FAR(275,3.0,-1.0),
            FLOOR_DESTACK(525,3.0,-1.0),
            SCORE_AUTO(2925,4.25,0.0),
            SCORE_AUTODROP(2400,4.5,0.0),
            SCORE_ROW1(2925,10,0.0),
            SCORE_DROP1(2400,10,0.0),
            SCORE_ROW2(2925,13,0.0),
            SCORE_DROP2(2400,13,0.0),
            SCORE_ROW3(2925,16,0.0),
            SCORE_DROP3(2400,16,0.0),
            //SCORE_ROWX(2925,12,0.0),
            //SCORE_DROPX(1950,12,0.0),
            CLIMB_READY(3500,7.0,-1.0),
            CLIMB_UP(3500,22,-1.0),
            CLIMB_LIFT(3500,1.5,-1.0),
            CLIMB_VERT(150, 1.5, -1.0);
            final double tilt, elevator, distance;
            Positions(double tilt, double elevator, double distance) {
                this.tilt = tilt;
                this.elevator = elevator;
                this.distance = distance;
            }
            public double getTilt() { return this.tilt; }
            public double getElevator() { return this.elevator; }
            public double getDistance() { return this.distance; }
        }
        public static class tiltController {
            public static double offsetStepSize = 90; //amount to change offset per request
            public static double homingSpeed = 0.1; //speed for homing to limit
            public static double targetThresholdTicks = 30; //how many encoder ticks is close enough
            public static double kF = 0.0; //0.0 //minimum power to move the motor
            public static double kP = 0.0015; //0.0015
            public static double kI = 0.0000010; //0.0000015
            public static double kIZone = 500.0; //I Zone for accumulating error
            public static double kD = 0.002; //0.002
            public static class limits {
                public static double maxOutput = 0.75; //maximum output power
                public static double minTicks = -5000.0; //Minimum encoder ticks of target (at limit sw)
                public static double maxTicks = 9000.0; //Maximum encoder ticks of target (at limit sw)
            }
        }
        public static class elevatorController {
            public static double offsetStepSize = 200; //amount to change offset per request
            public static double homingSpeed = 0.1; //speed for homing to limit
            public static double ticksPerRev = 28;
            public static double gearReduction = 100;
            public static double drumDiamInches = 1.3;
            public static double drumCircumferenceInches = drumDiamInches * Math.PI;
            public static double ticksPerInch = (ticksPerRev * gearReduction / drumCircumferenceInches) / 2; //2 stage elevator makes 2:1 reduction
            public static double targetThreshold = 0.2; //how many inches is close enough
            public static double targetThresholdTicks = 50; //targetThreshold * ticksPerInch;
            public static double kF = 0.0; //0.0 //minimum power to move the motor
            public static double kP = 0.0010; //0025
            public static double kI = 0.000004; //000002
            public static double kIZone = 2.0 * ticksPerInch; //I Zone for accumulating error
            public static double kD = 0.002; //002
            public static class limits {
                public static double maxOutput = 1.00; //maximum output power
                public static double minLength = 0.0; //Minimum extended length of arm
                public static double maxLength = 24.6; //Maximum extended length of arm
                public static double maxTicks = 16100;
            }
        }
    }
}
