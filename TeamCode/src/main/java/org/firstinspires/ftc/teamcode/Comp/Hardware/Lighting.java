package org.firstinspires.ftc.teamcode.Comp.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Lighting extends BaseHardware{

    private Servo photo;
    private Servo photo2; // MJD: second light, shows intake spinning/stopped
    private int intakeLightState = 0; // MJD: 0 stopped (the default, red), 1 forward (yellow), 2 reverse (purple)

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
        if (photo2 != null) photo2.setPosition(Red); // MJD: start red (intake not running) so it is lit at INIT

    }

    @Override
    public void init_loop(){
        writeIntakeLight(); // MJD: keep the intake light on (red) the whole time INIT is waiting
    }

    @Override
    public void start(){
        writeIntakeLight(); // MJD
    }

    @Override
    public void loop(){
        writeIntakeLight(); // MJD: red unless the intake is running, written every loop
        telemetry.addData("Lighting", currentColor);
        telemetry.addData("Intake light: found / state (0 stop, 1 fwd, 2 rev) / last servo pos", "%b / %d / %.3f", photo2 != null, intakeLightState, photo2 != null ? photo2.getPosition() : -1.0); // MJD


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
        //if (photo2 == null || state == intakeLightState) return; // MJD: old, only wrote when the color changed
        if (photo2 == null) return; // MJD
        intakeLightState = state; // MJD
        writeIntakeLight(); // MJD
    } // MJD

    /** MJD: red by default, yellow while the intake runs forward, purple in reverse. */
    private void writeIntakeLight() { // MJD
        if (photo2 == null) return; // MJD
        photo2.setPosition(intakeLightState == 0 ? Red : (intakeLightState == 2 ? Purple : Yellow)); // MJD
    } // MJD

    private void setColor(double position){
        currentColor = position;
        photo.setPosition(position);
    };


}
