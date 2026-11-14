package org.firstinspires.ftc.teamcode.Comp.Hardware;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.hardware.limelightvision;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
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

public class Limelight extends BaseHardware{
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
        telemetry.addData("TagDistance (m)", TagDistance); //disable after tuning

        //Always show connection status
        // status  = Limey.getStatus();
        // telemetry.addData("LL Conected", status.isconnected() ? "YES" : "NO");
        // telemetry.addData("LL Temperature", "%.1f\u00B0C", status.getTemp());
        // telemetry.addData("Pineline", Limey.getActivePipeline());

        // telemetry.update(); // Important! Forces immediate send

    }

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
