package org.firstinspires.ftc.teamcode.Comp.Hardware;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.ElapsedTime; // MJD

/**
 * MJD: two pollen sensors. NTKPD1 is at the top of the feeder, NTKPD2 is just inside the intake.
 * "Full" = both see pollen (for a few loops in a row). Robot uses that to stop the intake.
 * If a sensor is missing from the robot configuration it is just treated as "no pollen" (no crash).
 */
@Configurable
public class Sensors extends BaseHardware {

    // ---------------- TUNABLE CONSTANTS ----------------
    public static double PRESENT_CM = 4.5;   // closer than this = pollen is there (tune in Panels)
    //public static int CONFIRM_LOOPS = 3;     // must see pollen this many loops in a row (stops one noisy reading)
    public static double STABLE_SEC = 0.2;  // MJD: BOTH sensors must see pollen this long, continuously, before isFull()

    private static final String FEEDER_SENSOR_NAME = "NTKPD1";   // top of the feeder, must match the robot configuration
    private static final String INTAKE_SENSOR_NAME = "NTKPD2";   // just inside the intake

    private DistanceSensor feederSensor;
    private DistanceSensor intakeSensor;

    private double feederCm = Double.NaN;
    private double intakeCm = Double.NaN;
    private int feederLoops = 0;
    private int intakeLoops = 0;
    private final ElapsedTime bothTimer = new ElapsedTime(); // MJD: time since both sensors started seeing pollen
    private boolean bothNear = false; // MJD

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
        boolean nowBoth = isNear(feederCm) && isNear(intakeCm); // MJD
        if (nowBoth && !bothNear) bothTimer.reset(); // MJD: both just became true, start the clock
        bothNear = nowBoth; // MJD: any dropout makes this false, so the clock restarts next time

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

    //public boolean hasFeederPollen() { return feederLoops >= CONFIRM_LOOPS; }
    public boolean hasFeederPollen() { return isNear(feederCm); } // MJD: raw reading now

    //public boolean hasIntakePollen() { return intakeLoops >= CONFIRM_LOOPS; }
    public boolean hasIntakePollen() { return isNear(intakeCm); } // MJD: raw reading now

    /** Both sensors see pollen. */
    //public boolean isFull() { return hasFeederPollen() && hasIntakePollen(); }
    public boolean isFull() { return bothNear && bothTimer.seconds() >= STABLE_SEC; } // MJD: both seen, stable for STABLE_SEC
}


