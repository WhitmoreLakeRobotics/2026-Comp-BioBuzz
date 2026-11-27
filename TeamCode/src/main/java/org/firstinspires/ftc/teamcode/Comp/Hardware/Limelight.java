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

    public static int LIMELIGHT_PIPELINE = 0;     // MJD: which Limelight pipeline to run (applies when the OpMode starts)
    public static int HIVE_TAG_ID = -1;          // MJD: -1 = use the first tag seen, otherwise only this tag ID
    public static double HIVE_MOVE_TOL_IN = 3.0; // MJD: tag moved more than this (inches) = tipped
    public static int HIVE_CONFIRM_FRAMES = 3;   // MJD: a new state must hold this many loops before it counts
    // MJD: where the camera sits on the robot (MEASURE THESE) and which way it points
    //public static double CAM_FORWARD_IN = 10.25;   // MJD: camera ahead of robot center (negative = behind)   // MJD: old value, measured from the intake in the wrong frame
    public static double CAM_FORWARD_IN = 0.5;   // MJD: lens is 8 in from the back edge; the center is 7.5 in from the back, so 0.5 in toward the intake
    //public static double CAM_LEFT_IN = 8.0;      // MJD: camera left of robot center (negative = right)   // MJD: old value
    public static double CAM_LEFT_IN = 0.0;      // MJD: lens is centered side to side
    //public static double CAM_YAW_DEG = 45;      // MJD: 0 = camera looks straight ahead, positive = turned left   // MJD: old value, camera was assumed to look toward the intake
    //public static double CAM_YAW_DEG = 225;      // MJD: old, aimed 45 deg off. A tag centered in the camera is also centered on the launcher.
    public static double CAM_YAW_DEG = 180;      // MJD: 0 = toward the intake, + = left. 180 = looks straight along the launcher axis

    // ---------------- AUTO ALIGN (MJD) ----------------
    // Remembers the closest AprilTag with a positive ID. The turning itself moved to the AutoAlign class.
    //public static double ALIGN_KP = 0.02;          // MJD: moved to AutoAlign
    //public static double ALIGN_MIN_POWER = 0.08;   // MJD: moved to AutoAlign
    //public static double ALIGN_MAX_POWER = 0.6;    // MJD: moved to AutoAlign
    //public static double ALIGN_TOL_DEG = 1.5;      // MJD: moved to AutoAlign
    //public static double ALIGN_AIM_OFFSET_DEG = 0; // MJD: moved to AutoAlign
    public static double ALIGN_MAX_STALE_MS = 100; // MJD: ignore camera frames older than this
    private double[] alignTagField = null;         // MJD: field x,y (in) of the AIM point (the tag, or the spot behind it), last seen
    // MJD: aim BEHIND the tag. The point is moved along the tag's own direction (into the wall), which only changes
    // the aim angle when the robot is off to the side of the tag. 0 = off (aim at the tag itself).
    public static double AIM_BEHIND_IN = 4;          // MJD: inches behind the tag face to aim at (0 = off, tune in Panels, try 4-8)
    public static int NORMAL_ANGLE_SOURCE = 1;       // MJD: which tag angle is the left/right turn of the tag: 0 = yaw, 1 = pitch, 2 = roll (check the telemetry)
    public static double NORMAL_SIGN = 1;            // MJD: flip to -1 if the "behind" point lands on the wrong side
    public static double BEHIND_MAX_TURN_DEG = 60;   // MJD: tag turned more than this = reading too steep to trust, aim at the tag itself
    private int alignTagsSeen = 0;                   // MJD: tags used for the aim point, 1 or 2 (telemetry)
    private double[] alignTagRaw = null;             // MJD: field x,y of the tag itself (telemetry)
    private double lastTagYawDeg = 0;                // MJD: the tag angle that was used (telemetry)
    private double lastBehindDirDeg = 0;             // MJD: field angle (deg) of the "behind the tag" direction (telemetry)
    //private double alignErrDeg = 0;                // MJD: moved to AutoAlign

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

        //Limey.pipelineSwitch(1); // MJD: old, pipeline 1
        Limey.pipelineSwitch(LIMELIGHT_PIPELINE); // MJD: pipeline 0
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
        updateAlignTag(); // MJD
        telemetry.addData("Align tag known", alignTagField != null); // MJD
        telemetry.addData("Align tags used (1 or 2)", alignTagsSeen); // MJD
        if (alignTagField != null && alignTagRaw != null) { // MJD
            telemetry.addData("Aim point vs tag (in)", "%.1f,%.1f vs %.1f,%.1f", alignTagField[0], alignTagField[1], alignTagRaw[0], alignTagRaw[1]); // MJD
            telemetry.addData("Tag turn used (deg) / behind dir (field deg)", "%.1f / %.1f", lastTagYawDeg, lastBehindDirDeg); // MJD
        } // MJD
        // MJD: debug, shows which step is failing (remove when auto align works)
        telemetry.addData("LL result / valid", "%b / %b", result != null, result != null && result.isValid()); // MJD
        telemetry.addData("LL staleness (ms)", result != null ? result.getStaleness() : -1); // MJD
        telemetry.addData("LL tags seen", (result != null && result.getFiducialResults() != null) ? result.getFiducialResults().size() : -1); // MJD
        telemetry.addData("LL robotPose set", robotPose != null); // MJD
        //telemetry.addData("Align error (deg)", "%.1f", alignErrDeg); // MJD: now shown by Telop from AutoAlign
        telemetry.addData("TagDistance (m)", TagDistance); //disable after tuning
        telemetry.addData("Hive State", hiveState); // MJD
        telemetry.addData("HiveBaseline:", hiveBaseline); //naj

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
    public boolean isHiveUnknown() { return hiveState == HiveState.UNKNOWN; } // MJD
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

    // MJD: the point AIM_BEHIND_IN inches behind the tag face. "Behind" is the tag's own direction away from the camera,
    // worked out from how far the tag is turned left/right as the camera sees it.
    private double[] aimPointBehindTag(Pose3D camPose, double[] tagField) { // MJD
        if (AIM_BEHIND_IN == 0 || tagField == null || camPose.getOrientation() == null) { // MJD
            return tagField; // MJD
        } // MJD
        YawPitchRollAngles o = camPose.getOrientation(); // MJD
        double turnDeg = (NORMAL_ANGLE_SOURCE == 0) ? o.getYaw(AngleUnit.DEGREES) // MJD
                : (NORMAL_ANGLE_SOURCE == 1) ? o.getPitch(AngleUnit.DEGREES) // MJD
                : o.getRoll(AngleUnit.DEGREES); // MJD
        turnDeg = Math.atan2(Math.sin(Math.toRadians(turnDeg)), Math.cos(Math.toRadians(turnDeg))) * 180.0 / Math.PI; // MJD: wrap to +-180
        lastTagYawDeg = turnDeg; // MJD
        if (Math.abs(turnDeg) > BEHIND_MAX_TURN_DEG) { // MJD
            return tagField; // MJD: too steep, aim at the tag
        } // MJD
        double psi = Math.toRadians(turnDeg) * NORMAL_SIGN; // MJD
        // the tag's direction in camera space (x right, z forward). A tag facing the camera straight on (psi = 0) points straight away from it.
        double cx = Math.sin(psi); // MJD
        double cz = Math.cos(psi); // MJD
        if (cz < 0) { cx = -cx; cz = -cz; } // MJD: always the way that points away from the camera
        // camera space -> robot forward/left (rotation only, same math as tagFieldPosition) -> field
        double phi = Math.toRadians(CAM_YAW_DEG); // MJD
        double fwd = cz * Math.cos(phi) + cx * Math.sin(phi); // MJD
        double left = cz * Math.sin(phi) - cx * Math.cos(phi); // MJD
        double th = robotPose.heading(); // MJD
        double dx = fwd * Math.cos(th) - left * Math.sin(th); // MJD
        double dy = fwd * Math.sin(th) + left * Math.cos(th); // MJD
        lastBehindDirDeg = Math.toDegrees(Math.atan2(dy, dx)); // MJD
        return new double[]{tagField[0] + AIM_BEHIND_IN * dx, tagField[1] + AIM_BEHIND_IN * dy}; // MJD
    } // MJD

    // MJD: remember where the closest tag with a positive ID is on the field (kept until a newer one is seen)
    //private void updateAlignTag() { // MJD
    //    if (robotPose == null || result == null || !result.isValid() // MJD
    //            || result.getStaleness() > ALIGN_MAX_STALE_MS || result.getFiducialResults() == null) { // MJD
    //        return; // MJD
    //    } // MJD
    //    double best = Double.MAX_VALUE; // MJD
    //    for (LLResultTypes.FiducialResult f : result.getFiducialResults()) { // MJD
    //        if (f.getFiducialId() <= 0) { // MJD
    //            continue; // MJD: positive tag IDs only
    //        } // MJD
    //        Position p = f.getTargetPoseCameraSpace().getPosition(); // MJD
    //        double d = Math.hypot(p.x, p.z); // MJD: closest = smallest distance
    //        if (d < best) { // MJD
    //            best = d; // MJD
    //            //alignTagField = tagFieldPosition(f.getTargetPoseCameraSpace()); // MJD: old, aimed at the tag itself
    //            alignTagRaw = tagFieldPosition(f.getTargetPoseCameraSpace()); // MJD
    //            alignTagField = aimPointBehindTag(f.getTargetPoseCameraSpace(), alignTagRaw); // MJD
    //        } // MJD
    //    } // MJD
    //} // MJD
    // MJD: hive center tag pairs that sit together. Tag 31 pairs with 32, 35 with 36, 39 with 40, 43 with 44.
    private static final int[] PAIR_IDS = {31, 32, 35, 36, 39, 40, 43, 44}; // MJD

    private static int partnerOf(int id) { // MJD: the other tag in the pair, or -1 if this tag is not in a pair
        for (int k = 0; k < PAIR_IDS.length; k++) { // MJD
            if (PAIR_IDS[k] == id) { // MJD
                return (k % 2 == 0) ? PAIR_IDS[k + 1] : PAIR_IDS[k - 1]; // MJD
            } // MJD
        } // MJD
        return -1; // MJD
    } // MJD

    // MJD: old version (above) aimed at the one closest tag. Now: find the closest tag. If its pair partner is also in view,
    // aim AIM_BEHIND_IN inches behind the middle of the two. With one tag, aim AIM_BEHIND_IN inches behind that tag.
    private void updateAlignTag() { // MJD
        if (robotPose == null || result == null || !result.isValid() // MJD
                || result.getStaleness() > ALIGN_MAX_STALE_MS || result.getFiducialResults() == null) { // MJD
            return; // MJD
        } // MJD
        double best = Double.MAX_VALUE; // MJD
        LLResultTypes.FiducialResult closest = null; // MJD
        for (LLResultTypes.FiducialResult f : result.getFiducialResults()) { // MJD
            if (f.getFiducialId() <= 0) { // MJD
                continue; // MJD: positive tag IDs only
            } // MJD
            Position p = f.getTargetPoseCameraSpace().getPosition(); // MJD
            double d = Math.hypot(p.x, p.z); // MJD: closest = smallest distance
            if (d < best) { // MJD
                best = d; // MJD
                closest = f; // MJD
            } // MJD
        } // MJD
        if (closest == null) { // MJD
            alignTagsSeen = 0; // MJD
            return; // MJD: no tag this frame, keep the last aim point
        } // MJD
        int partnerId = partnerOf(closest.getFiducialId()); // MJD
        LLResultTypes.FiducialResult partner = null; // MJD
        if (partnerId > 0) { // MJD
            for (LLResultTypes.FiducialResult f : result.getFiducialResults()) { // MJD
                if (f.getFiducialId() == partnerId) { // MJD
                    partner = f; // MJD
                    break; // MJD
                } // MJD
            } // MJD
        } // MJD
        double[] t1 = tagFieldPosition(closest.getTargetPoseCameraSpace()); // MJD
        if (partner != null) { // MJD
            double[] t2 = tagFieldPosition(partner.getTargetPoseCameraSpace()); // MJD
            double lx = t2[0] - t1[0], ly = t2[1] - t1[1]; // MJD: the line along the hive face between the two tags
            double len = Math.hypot(lx, ly); // MJD
            if (len > 1.0) { // MJD: tags farther than 1 in apart, otherwise treat as one tag
                double mx = (t1[0] + t2[0]) / 2.0, my = (t1[1] + t2[1]) / 2.0; // MJD: middle of the two tags
                double px = -ly / len, py = lx / len; // MJD: perpendicular to that line = into the hive
                if (px * (mx - robotPose.x()) + py * (my - robotPose.y()) < 0) { // MJD
                    px = -px; py = -py; // MJD: pick the side that points away from the robot
                } // MJD
                alignTagsSeen = 2; // MJD
                alignTagRaw = new double[]{mx, my}; // MJD
                alignTagField = new double[]{mx + AIM_BEHIND_IN * px, my + AIM_BEHIND_IN * py}; // MJD
                lastBehindDirDeg = Math.toDegrees(Math.atan2(py, px)); // MJD
                return; // MJD
            } // MJD
        } // MJD
        alignTagsSeen = 1; // MJD
        alignTagRaw = t1; // MJD: only one tag
        alignTagField = aimPointBehindTag(closest.getTargetPoseCameraSpace(), t1); // MJD: behind it by AIM_BEHIND_IN, using the tag's own direction
    } // MJD

    public boolean alignHasTarget() { return alignTagField != null; } // MJD
    public boolean hasClosestTag() { return alignTagField != null; } // MJD: same thing, clearer name
    public double[] getClosestTagField() { return alignTagField; } // MJD: field x,y (in) of the closest positive tag, or null

    // MJD: the turn controller (getAlignTurnPower) moved to the AutoAlign class. Old version:
    //public double getAlignTurnPower(Pose robot) {
    //    if (alignTagField == null) { return 0; }
    //    double desired = Math.atan2(alignTagField[1] - robot.y(), alignTagField[0] - robot.x())
    //            + Math.PI + Math.toRadians(ALIGN_AIM_OFFSET_DEG);
    //    double err = Math.atan2(Math.sin(desired - robot.heading()), Math.cos(desired - robot.heading()));
    //    alignErrDeg = Math.toDegrees(err);
    //    if (Math.abs(alignErrDeg) <= ALIGN_TOL_DEG) { return 0; }
    //    double power = ALIGN_KP * alignErrDeg;
    //    return Math.copySign(Math.max(ALIGN_MIN_POWER, Math.min(ALIGN_MAX_POWER, Math.abs(power))), power);
    //}

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
