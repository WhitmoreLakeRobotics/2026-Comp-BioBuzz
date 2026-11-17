/*package org.firstinspires.ftc.teamcode.Comp.Hardware;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@Configurable
public class Sensors extends BaseHardware{

    public ColorSensor NTKPD1; //may be a range sensor
    public ColorSensor NTKPD2;
    public ColorSensor NTKPD3;

    public double NTKPD1distance;
    public double NTKPD2distance;
    public double NTKPD3distance;



    @Override
    public void init() {

        NTKPD3 = hardwareMap.get(ColorSensor.class, "NTKPD3");
        NTKPD2 = hardwareMap.get(ColorSensor.class, "NTKPD2");
        NTKPD1 = hardwareMap.get(ColorSensor.class, "NTKPD1");
        
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
  /*  private void getDistNTKAP2() {
        NTKPD2distance = NTKPD2.getDistance(DistanceUnit.CM);
    }
    private void getDistNTKAP3() {
        NTKPD3distance = NTKPD3.getDistance(DistanceUnit.CM);
    }
    private void getDistNTKAP1() {
        NTKPD1distance = NTKPD1.getDistance(DistanceUnit.CM);
    }

    public enum Distance3 {
        FILLED3,
        MISSING3
    }

    public enum Distance2 {
        FILLED2,
        MISSING2
    }

    public enum Distance1 {
        FILLED1,
        MISSING1
    }


}
*/