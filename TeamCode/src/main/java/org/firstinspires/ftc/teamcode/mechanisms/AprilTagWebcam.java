package org.firstinspires.ftc.teamcode.mechanisms;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
@Autonomous

public class AprilTagWebcam extends LinearOpMode {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    private Telemetry telemetry;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private static final double DESIRED_DISTANCE = 12.0;       //  this is how close the camera should get to the target (inches)
    private static final int DESIRED_TAG_ID = -1;       // Choose the tag you want to approach or set to -1 for ANY tag.
    private AprilTagProcessor aprilTag;                 // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag;        // Used to hold the data for a detected AprilTag
    protected boolean targetFound = false;    // Set to true when an AprilTag target is detected
    protected int turretTarget = 45;


    protected DcMotorEx turretmotor = null;
    private static final int TURRET_MOTOR_GEAR_COUNT = 50;
    private static final int TURRET_GEAR_COUNT = 200;


    public void runOpMode() {


        turretmotor.setTargetPosition(turretTarget);//int type. Set target before setting RunMode.
        turretmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);



        waitForStart();
        turretmotor.setPower(1);// specifies max available power to motor

        visionPortal.close();

        detectAprilTag();



        while(opModeIsActive()) {

//            TelemetryPacket packet = new TelemetryPacket();
            aimAtTarget();

        }

        if (isStopRequested()) {
            return;
        }
    }
    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
    }

    public void update() {
        detectedTags = aprilTagProcessor.getDetections();

    }
    private void aimAtTarget() {
        if (targetFound) {
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double  headingError    = desiredTag.ftcPose.bearing;
            double  yawError        = desiredTag.ftcPose.yaw;
            turretmotor.setTargetPosition((int) (headingError*TURRET_GEAR_COUNT/TURRET_MOTOR_GEAR_COUNT*1.49444444444));//ticks multiply?

//            turretTarget = 45 + headingError;
            turretmotor.setTargetPosition((int) turretTarget);

        }
    }

    private void detectAprilTag() {
        targetFound = false;

        desiredTag  = null;

        // Step through the list of detected tags and look for a matching tag

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {

            // Look to see if we have size info on this tag.

            if (detection.metadata != null) {

                //  Check to see if we want to track towards this tag.

                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {

                    // Yes, we want to use this tag.

                    targetFound = true;

                    desiredTag = detection;

                    break;  // don't look any further.

                } else {

                    // This tag is in the library, but we do not want to track it right now.

                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);

                }

            } else {

                // This tag is NOT in the library, so we don't have enough information to track to it.

                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);

            }

        }

        // Tell the driver what we see, and what to do.

        if (targetFound) {

            // telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");

            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);

            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);

            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);

            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);

        } else {

            telemetry.addData("\n>","Drive using joysticks to find valid target\n");

        }
    }

    public List<AprilTagDetection> getDetectedTags() {
        return  detectedTags;
    }

    public void displayDetectionTelemetry(AprilTagDetection detectedId) {
        if (detectedId == null) {return;}
        if (detectedId.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detectedId.id, detectedId.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (cm)", detectedId.ftcPose.x, detectedId.ftcPose.y, detectedId.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detectedId.ftcPose.pitch, detectedId.ftcPose.roll, detectedId.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detectedId.ftcPose.range, detectedId.ftcPose.bearing, detectedId.ftcPose.elevation));
        } else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detectedId.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detectedId.center.x, detectedId.center.y));
        }
    }

    public AprilTagDetection getTagBySpecificId(int id) {
        for (AprilTagDetection detection : detectedTags) {
            if (detection.id == id){
                return detection;
            }
        }
        return null;

    }

//    public void stop() {
//        if (visionPortal != null) {
//            visionPortal.close();
//        }
//    }


}
