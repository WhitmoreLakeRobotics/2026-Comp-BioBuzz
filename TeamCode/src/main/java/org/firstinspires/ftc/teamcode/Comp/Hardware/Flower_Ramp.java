package org.firstinspires.ftc.teamcode.Comp.Hardware;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable

public class Flower_Ramp extends BaseHardware{
    private Servo servoRamp;
    private Servo RightRamp;
    public Mode CurrentMode = Mode.Down;
    //public static double Down_POSITION = 0.30;  //left
    //public static double Up_POSITION = 0.60;//No position right now //left
    //public static double UP = 0.65; // right
    //public static double DOWN = 0.40; //right
    public static double Down_POSITION = 0.85;  // MJD: center 0.5 minus 0.25, used by BOTH servos (right is reversed)
    public static double Up_POSITION = 0.30;    // MJD: center 0.5 plus 0.25, used by BOTH servos (right is reversed)

    @Override
    public void init() {
        servoRamp = hardwareMap.get(Servo.class, "Flower_Ramp");
        RightRamp = hardwareMap.get(Servo.class, "Flower_Ramp_R"); // MJD: right servo, name must match the robot configuration
        RightRamp.setDirection(Servo.Direction.REVERSE); // MJD: mounted opposite, so the same position number moves both the same way

        cmdUp();    //was Down VAM


    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        telemetry.addData("Down Position", servoRamp.getPosition());
        telemetry.addData("Up State", CurrentMode);
    }

    @Override
    public void stop() {

    }
    public void cmdDown() {

        CurrentMode = Mode.Down;

        servoRamp.setPosition(Down_POSITION);
        //RightRamp.setPosition(DOWN);
        RightRamp.setPosition(Down_POSITION); // MJD
    }

    public void cmdUp() {

        CurrentMode = Mode.Up;

        servoRamp.setPosition(Up_POSITION);
        //RightRamp.setPosition(UP);
        RightRamp.setPosition(Up_POSITION); // MJD
    }
    public enum Mode {
        Down,
        Up
    }
}
