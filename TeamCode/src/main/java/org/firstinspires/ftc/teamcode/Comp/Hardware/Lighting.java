package org.firstinspires.ftc.teamcode.Comp.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Lighting extends BaseHardware{

    private Servo photo;

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

    }



    public void cmdInLaunchPosition(boolean InPosition){
        setColor(InPosition ?Green:Red);

    }
    private void setColor(double position){
        currentColor = position;
        photo.setPosition(position);
    };


}
