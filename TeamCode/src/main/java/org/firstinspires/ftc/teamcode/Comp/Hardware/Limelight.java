package org.firstinspires.ftc.teamcode.Comp.Hardware;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.hardware.limelightvision;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.bylazar.configurables.annotations.Configurable; // MJD
import com.pedropathing.math.Pose; // MJD
import com.qualcomm.hardware.limelightvision.LLResultTypes; // MJD
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@Configurable // MJD
public class Limelight extends BaseHardware{

    // ---------------- HIVE STATE (MJD) ----------------
    // The hive is judged by where its AprilTag is on the FIELD (camera reading + robot pose).
    // armHiveBaseline() remembers the hive position at the first launch. After that:
    //   same spot (within HIVE_MOVE_TOL_IN)  -> UPRIGHT
    //   moved                                -> TIPPED
    //   tag not seen / no baseline yet       -> UNKNOWN
    public enum HiveState { UNKNOWN, UPRIGHT, TIPPED } // MJD

    public static int HIVE_TAG_ID = -1;          // MJD: -1 = use the first tag seen, otherwise only this tag ID
    public static double HIVE_MOVE_TOL_IN = 3.0; // MJD: tag moved more than this (inches) = tipped
    public static int HIVE_CONFIRM_FRAMES = 3;   // MJD: a new state must hold this many loops before it counts
    // MJD: where the camera sits on the robot (MEASURE THESE) and which way it points
    public static double CAM_FORWARD_IN = 10.25;   // MJD: camera ahead of robot center (negative = behind)
    public static double CAM_LEFT_IN = 8.0;      // MJD: camera left of robot center (negative = right)
    public static double CAM_YAW_DEG = 45;      // MJD: 0 = camera looks straight ahead, positive = turned left

    private HiveState hiveState = HiveState.UNKNOWN; // MJD
    private HiveState rawHiveState = HiveState.UNKNOWN; // MJD
    private int rawHiveFrames = 0; // MJD
    private double[] hiveBaseline = null; // MJD: field x,y of the hive tag at the first launch
    private boolean baselinePending = false; // MJD
    private double[] hiveNow = null; // MJD: latest field x,y of the hive tag
    private Pose robotPose = null; // MJD
    private ElapsedTime runtime = new ElapsedTime();
    /**
     * The {@link #telemetry} field contains an object in which a user may accumulate data which
     * is to be transmitted to the driver station. This data is automatically transmitted to the
     * driver station on a regular, periodic basis.
     */
    public Telemetry telemetry = null;
    //private ColorRangeSensor IntakeSensor;
    //private DistanceSensor RearLeftSensor

    //private com.qualcomm.hardware.limelightvision Limelight3A;

    private Limelight3A Limey;
    private LLStatus status;
    private LLResult result;
    private int AprilTagID = -1;
    private double ty = 1000;
    private double tx = 1000;
    private List<List<Double>> corners;
    private Pose3D TagPose = new Pose3D(new Position(DistanceUnit.METER,0,0,0,0),
            new YawPitchRollAngles(AngleUnit.DEGREES,0,0,0,0));
    private double TagAngle = 0;
    private double TagDistance = 0;

    private boolean cmdComplete = true;
    private Mode CurrentMode = Mode.STOP;



    /**
     * Hardware Mappings
     */
    public HardwareMap hardwareMap = null; // will be set in Child class


    /**
     * BaseHardware constructor
     * <p>
     * The op mode name should be unique. It will be the name displayed on the driver station. If
     * multiple op modes have the same name, only one will be available.
     */
    /*public Swing_Arm_And_Lift() {

    }*/

    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    public void init(){
        //DeliverySensor = hardwareMap.get(ColorSensor.class, "DeliveryS");

        Limey  = hardwareMap.get(Limelight3A.class, "limelight");

        //telemetry.setMsTransmissionInterval(11);

        Limey.pipelineSwitch(1);
        Limey.start();
        Limey.setPollRateHz(100);

    }

    /**
     * User defined init_loop method
     * <p>
     * This method will be called repeatedly when the INIT button is pressed.
     * This method is optional. By default this method takes no action.
     */
    public void init_loop() {



        /**
         * User defined init_loop method
         * <p>
         * This method will be called repeatedly when the INIT button is pressed.
         * This method is optional. By default this method takes no action.
         */

//         telemetry.addData("FLDS1 Pos " , FLDS1.getDistance(DistanceUnit.INCH)) ;
    }

    /**
     * User defined start method.
     * <p>
     * This method will be called once when the PLAY button is first pressed.
     * This method is optional. By default this method takes not action.
     * Example usage: Starting another thread.
     */
    public void start(){

    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    public void loop(){
        //  status = Limey.getStatus();
        result = Limey.getLatestResult();

        //if(result.isValid()) {   // MJD: old, crashed when result was null
        // MJD: result can be null, and a valid result can still have no tags in the list
        if (result != null && result.isValid() && result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) { // MJD
            AprilTagID = result.getFiducialResults().get(0).getFiducialId();
            ty = result.getFiducialResults().get(0).getTargetYDegrees();
            tx = result.getFiducialResults().get(0).getTargetXDegrees();
            corners = result.getFiducialResults().get(0).getTargetCorners();
            TagPose = result.getFiducialResults().get(0).getTargetPoseCameraSpace();
            TagAngle = TagPose.getOrientation().getYaw();
            TagDistance = TagPose.getPosition().z;

            //Rich telemetry when tag is visible
            telemetry.addData("Limelight", "VALID TARGET");
            telemetry.addData("ApirlTag ID", "%d", AprilTagID);
            telemetry.addData("tx (horizontal offset", "%.2f\u00B0",tx);
            telemetry.addData("ty (vertical offset)", "%.2f\u00B0", ty);
            telemetry.addData("Distance to Tag", TagDistance);
            telemetry.addData("Yaw to Tag", "%.2f\u00B0", TagAngle);
            telemetry.addData("Latency", "%.1f ms", result.getTargetingLatency());

            //Optional: Bot pose if you're using 3D solve (robot pose in field space)
            if (result.getBotpose() != null) {
                Pose3D botPose = result.getBotpose();
                telemetry.addData("Bot Pose (X,Y,Z)", "%.2f, %.2f, %.2f",
                        botPose.getPosition().x, botPose.getPosition().y, botPose.getPosition().z);

            }

        }else {
            AprilTagID = -1;
            telemetry.addData("Limelight", "NO TARGET");
            if (result == null) {
                telemetry.addData("Status", "No data received");
            }else{
                telemetry.addData("Valid", result.isValid() ? "Yes" : "No");
                telemetry.addData("Latency", "%.1f ms", result.getTargetingLatency());
            }
        }
        updateHive(); // MJD
        telemetry.addData("TagDistance (m)", TagDistance); //disable after tuning
        telemetry.addData("Hive State", hiveState); // MJD

        //Always show connection status
        // status  = Limey.getStatus();
        // telemetry.addData("LL Conected", status.isconnected() ? "YES" : "NO");
        // telemetry.addData("LL Temperature", "%.1f\u00B0C", status.getTemp());
        // telemetry.addData("Pineline", Limey.getActivePipeline());

        // telemetry.update(); // Important! Forces immediate send

    }

    // MJD: Robot calls this every loop so the tag can be placed on the field.
    public void setRobotPose(Pose pose) { // MJD
        this.robotPose = pose; // MJD
    } // MJD

    // MJD: remember where the hive is RIGHT NOW (first tag frame after this call). Call it at the first launch.
    public void armHiveBaseline() { // MJD
        hiveBaseline = null; // MJD
        baselinePending = true; // MJD
    } // MJD

    public HiveState getHiveState() { return hiveState; } // MJD
    public boolean isHiveUpright() { return hiveState == HiveState.UPRIGHT; } // MJD
    public boolean isHiveTipped() { return hiveState == HiveState.TIPPED; } // MJD
    public boolean hasHiveBaseline() { return hiveBaseline != null; } // MJD

    // MJD: camera-space tag position -> field x,y (inches), using the robot pose and camera mount.
    // Limelight camera space: +X right, +Y down, +Z forward.
    private double[] tagFieldPosition(Pose3D camPose) { // MJD
        Position pos = camPose.getPosition().toUnit(DistanceUnit.INCH); // MJD
        double phi = Math.toRadians(CAM_YAW_DEG); // MJD
        double fwd = pos.z * Math.cos(phi) + pos.x * Math.sin(phi) + CAM_FORWARD_IN; // MJD
        double left = pos.z * Math.sin(phi) - pos.x * Math.cos(phi) + CAM_LEFT_IN; // MJD
        double th = robotPose.heading(); // MJD
        return new double[]{ // MJD
                robotPose.x() + fwd * Math.cos(th) - left * Math.sin(th), // MJD
                robotPose.y() + fwd * Math.sin(th) + left * Math.cos(th)}; // MJD
    } // MJD

    private void updateHive() { // MJD
        hiveNow = null; // MJD
        if (robotPose != null && result != null && result.isValid() && result.getFiducialResults() != null) { // MJD
            for (LLResultTypes.FiducialResult f : result.getFiducialResults()) { // MJD
                if (HIVE_TAG_ID < 0 || f.getFiducialId() == HIVE_TAG_ID) { // MJD
                    hiveNow = tagFieldPosition(f.getTargetPoseCameraSpace()); // MJD
                    break; // MJD
                } // MJD
            } // MJD
        } // MJD

        HiveState raw = HiveState.UNKNOWN; // MJD
        if (hiveNow != null) { // MJD
            if (baselinePending) { // MJD
                hiveBaseline = hiveNow; // MJD
                baselinePending = false; // MJD
            } // MJD
            if (hiveBaseline != null) { // MJD
                double moved = Math.hypot(hiveNow[0] - hiveBaseline[0], hiveNow[1] - hiveBaseline[1]); // MJD
                raw = (moved <= HIVE_MOVE_TOL_IN) ? HiveState.UPRIGHT : HiveState.TIPPED; // MJD
                telemetry.addData("Hive moved (in)", "%.1f", moved); // MJD
            } // MJD
        } // MJD

        // a new state has to hold for HIVE_CONFIRM_FRAMES loops before it counts (one noisy frame won't flip it)
        if (raw == rawHiveState) { // MJD
            rawHiveFrames++; // MJD
        } else { // MJD
            rawHiveState = raw; // MJD
            rawHiveFrames = 1; // MJD
        } // MJD
        if (rawHiveFrames >= HIVE_CONFIRM_FRAMES) { // MJD
            hiveState = raw; // MJD
        } // MJD
    } // MJD

    public  double getTy(){
        return  ty;
    }

    public  double getTx(){
        return  tx;
    }
    public double getTagID(){
        return AprilTagID;
    }

    public double getTagDistance(){
        return TagDistance;
    }

    public double getTagAngle(){
        return TagAngle;
    }

    public void doStop(){
        CurrentMode = Mode.STOP;
        cmdComplete = true;
    }



    /**
     * User defined stop method
     * <p>
     * This method will be called when this op mode is first disabled
     * <p>
     * The stop method is optional. By default this method takes no action.
     */

    public void setTelemetry(Telemetry LLtelemetry) {
        this.telemetry = LLtelemetry;

    }



    public void stop(){
        if (Limey != null) { // MJD: release the camera when the OpMode ends
            Limey.stop(); // MJD
        } // MJD
    }

    public enum Mode{
        STOP
    }



}
