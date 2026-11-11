package org.firstinspires.ftc.teamcode.Comp.Hardware;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable

public class Flower_Ramp extends BaseHardware{
    private Servo servoRamp;
    public Mode CurrentMode = Mode.Down;
    public static double Down_POSITION = 0.00;
    public static double Up_POSITION = 0.00;//No position right now
    @Override
    public void init() {
        servoRamp = hardwareMap.get(Servo.class, "Flower_Ramp");

        cmdDown();


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
    }

    public void cmdUp() {

        CurrentMode = Mode.Up;

        servoRamp.setPosition(Up_POSITION);
    }
    public enum Mode {
        Down,
        Up
    }
}
