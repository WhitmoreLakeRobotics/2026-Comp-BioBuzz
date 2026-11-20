package org.firstinspires.ftc.teamcode.Comp.Hardware;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable

public class Flower_Ramp extends BaseHardware{
    private Servo servoRamp;
    private Servo RightRamp;
    public Mode CurrentMode = Mode.Down;
    public static double Down_POSITION = 0.30;  //left
    public static double Up_POSITION = 0.60;//No position right now //left
    public static double UP = 0.65; // right
    public static double DOWN = 0.40; //right

    @Override
    public void init() {
        servoRamp = hardwareMap.get(Servo.class, "Flower_Ramp");

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
        RightRamp.setPosition(DOWN);
    }

    public void cmdUp() {

        CurrentMode = Mode.Up;

        servoRamp.setPosition(Up_POSITION);
        RightRamp.setPosition(UP);
    }
    public enum Mode {
        Down,
        Up
    }
}
