package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.hardware.hwMecanumFtclib;

@SuppressWarnings({"unused"})
public class StateMachines {
    public static ElapsedTime elapsed = new ElapsedTime();
    OpMode myOpMode;

    public enum SpecimenPickup {
        IDLE,
        EXTEND,
        REBOUND,
        LIFT,
        REVERSE,
        DONE
    }
    public static StateMachine getSpecimenPickupMachine(hwMecanumFtclib robot, OpMode opmode) {
        return new StateMachineBuilder()
                /* Make sure we stopped driving */
                .state(SpecimenPickup.IDLE)
                //.onEnter()
                .transitionTimed(0.5)
                /* Extend the elevator to the grab position */
                .state(SpecimenPickup.EXTEND)
                //.onEnter()
                .transitionTimed(0.5)
                /* Done with pickup operation */
                .state(SpecimenPickup.DONE)
                //.onEnter()
                .build();
    }

    public enum SamplePickup {
        IDLE,
        LEVEL,
        INTAKE,
        EXTEND,
        LIFT,
        DONE
    }
    public static StateMachine getSamplePickupMachine(hwMecanumFtclib robot, OpMode opmode) {
        return new StateMachineBuilder()
                /* Make sure we stopped driving */
                .state(SamplePickup.IDLE)
                //.onEnter()
                .transitionTimed(0.5)
                /* Move the manipulator to the intake position */
                .state(SamplePickup.LEVEL)
                //.onEnter()
                .transitionTimed(0.5)
                /* Extend the elevator */
                .state(SamplePickup.EXTEND)
                //.onEnter()
                .transitionTimed(0.5)
                /* Done with pickup operation */
                .state(SamplePickup.DONE)
                //.onEnter()
                .build();
    }

    public enum Climb {
        IDLE,
        READY,
        HOOK,
        LIFT,
        DONE
    }
    public static StateMachine getClimbMachine(hwMecanumFtclib robot, OpMode opmode) {
        return new StateMachineBuilder()
            .state(Climb.IDLE)
                .transitionTimed(0.25)
            .state(Climb.READY)
                .onEnter( () -> {
                    elapsed.reset();
                    robot.setManipulatorPosition(Constants.Manipulator.Positions.CLIMB_READY);
                })
                .transition( () -> elapsed.seconds() >= 1.0 && robot.getManipulatorAtTarget())
            .state(Climb.HOOK)
                .onEnter( () -> {
                    elapsed.reset();
                    robot.setManipulatorPosition(Constants.Manipulator.Positions.CLIMB_UP);
                })
                .transition( () -> elapsed.seconds() >= 1.0 && robot.getManipulatorAtTarget())
            .state(Climb.LIFT)
                .onEnter( () -> {
                    elapsed.reset();
                    robot.setManipulatorPosition(Constants.Manipulator.Positions.CLIMB_LIFT);
                })
                .transition( () -> elapsed.seconds() >= 1.0 && robot.getManipulatorAtTarget())
            .state(Climb.DONE)
            .build();
    }
}
