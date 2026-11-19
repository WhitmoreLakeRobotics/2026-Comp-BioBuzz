package org.firstinspires.ftc.teamcode.Comp.Hardware;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * MJD: two pollen sensors. NTKPD1 is at the top of the feeder, NTKPD2 is just inside the intake.
 * "Full" = both see pollen (for a few loops in a row). Robot uses that to stop the intake.
 * If a sensor is missing from the robot configuration it is just treated as "no pollen" (no crash).
 */
@Configurable
public class Sensors extends BaseHardware {

    // ---------------- TUNABLE CONSTANTS ----------------
    public static double PRESENT_CM = 3.0;   // closer than this = pollen is there (tune in Panels)
    public static int CONFIRM_LOOPS = 3;     // must see pollen this many loops in a row (stops one noisy reading)

    private static final String FEEDER_SENSOR_NAME = "NTKPD1";   // top of the feeder, must match the robot configuration
    private static final String INTAKE_SENSOR_NAME = "NTKPD2";   // just inside the intake

    private DistanceSensor feederSensor;
    private DistanceSensor intakeSensor;

    private double feederCm = Double.NaN;
    private double intakeCm = Double.NaN;
    private int feederLoops = 0;
    private int intakeLoops = 0;

    @Override
    public void init() {
        feederSensor = hardwareMap.tryGet(DistanceSensor.class, FEEDER_SENSOR_NAME);
        intakeSensor = hardwareMap.tryGet(DistanceSensor.class, INTAKE_SENSOR_NAME);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        feederCm = (feederSensor != null) ? feederSensor.getDistance(DistanceUnit.CM) : Double.NaN;
        intakeCm = (intakeSensor != null) ? intakeSensor.getDistance(DistanceUnit.CM) : Double.NaN;

        feederLoops = isNear(feederCm) ? feederLoops + 1 : 0;
        intakeLoops = isNear(intakeCm) ? intakeLoops + 1 : 0;

        telemetry.addData("Feeder sensor (cm)", feederCm);
        telemetry.addData("Intake sensor (cm)", intakeCm);
        telemetry.addData("Pollen feeder / intake / full", "%b / %b / %b", hasFeederPollen(), hasIntakePollen(), isFull());
        telemetry.addData("Sensors found", "%b / %b", feederSensor != null, intakeSensor != null);
    }

    @Override
    public void stop() {
    }

    private boolean isNear(double cm) {
        return !Double.isNaN(cm) && cm <= PRESENT_CM;
    }

    public boolean hasFeederPollen() { return feederLoops >= CONFIRM_LOOPS; }

    public boolean hasIntakePollen() { return intakeLoops >= CONFIRM_LOOPS; }

    /** Both sensors see pollen. */
    public boolean isFull() { return hasFeederPollen() && hasIntakePollen(); }
}

// MJD: old version (3 color sensors, all commented out), kept for reference:
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