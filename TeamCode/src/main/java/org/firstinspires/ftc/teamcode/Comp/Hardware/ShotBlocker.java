package org.firstinspires.ftc.teamcode.Comp.Hardware;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;
public class ShotBlocker extends BaseHardware{
    public Servo ShotBlocker;

    private static double UP = 0.80;
    private static double Down = 0.30;

    public Mode CurrentMode = Mode.Down;


    @Override
    public void init() {

        ShotBlocker = hardwareMap.get(Servo.class,"shot_blocker");

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

    }

    @Override
    public void stop() {

    }


    public void cmdDown(){
        CurrentMode = Mode.Down;
        ShotBlocker.setPosition(Down);

    }

    public void cmdUp(){
        CurrentMode = Mode.Up;
        ShotBlocker.setPosition(UP);
    }

    public enum Mode{
        Up,
        Down,
    }
}
