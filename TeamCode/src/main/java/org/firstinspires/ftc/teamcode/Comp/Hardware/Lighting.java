package org.firstinspires.ftc.teamcode.Comp.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Lighting extends BaseHardware{

    private Servo photo;
    private Servo photo2; // MJD: second light, shows intake spinning/stopped
    private int intakeLightState = -1; // MJD: -1 unknown, 0 stopped, 1 spinning (so the servo is only written on change)

    private static final String Photo = "Lighting";
    private static double Off = 0.0;
    private static double Green = 0.5;
    private double currentColor = Off;
    private double Red = 0.277;// MJD
    private double Blue = 0.6111;
    private double Yellow = 0.388;
    private double Orange = 0.333;
    private double Purple = 0.722;

    @Override
    public void init(){
        photo = hardwareMap.get(Servo.class, "Lighting");
        setColor(Red);
        photo2 = hardwareMap.tryGet(Servo.class, "Lighting2"); // MJD: name must match the robot configuration, ignored if missing

    }

    @Override
    public void init_loop(){

    }

    @Override
    public void start(){

    }

    @Override
    public void loop(){
        telemetry.addData("Lighting", currentColor);


    }

    @Override
    public void stop(){

        if(photo != null){
            setColor(Off);
        }
        if (photo2 != null) photo2.setPosition(Off); // MJD

    }



    public void cmdInLaunchPosition(boolean InPosition){
        setColor(InPosition ?Green:Red);

    }
    /** MJD: second light. Yellow while the intake is spinning, red when it has stopped. */
    public void cmdIntakeSpinning(boolean spinning) { // MJD
        int state = spinning ? 1 : 0; // MJD
        if (photo2 == null || state == intakeLightState) return; // MJD
        intakeLightState = state; // MJD
        photo2.setPosition(spinning ? Yellow : Red); // MJD
    } // MJD

    /** MJD: second light with reverse. Yellow = forward, purple = reverse, red = stopped. */
    public void cmdIntakeLight(boolean on, boolean reverse) { // MJD
        int state = !on ? 0 : (reverse ? 2 : 1); // MJD
        if (photo2 == null || state == intakeLightState) return; // MJD
        intakeLightState = state; // MJD
        photo2.setPosition(state == 0 ? Red : (state == 2 ? Purple : Yellow)); // MJD
    } // MJD

    private void setColor(double position){
        currentColor = position;
        photo.setPosition(position);
    };


}
