package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public abstract class AbstractAprilTag extends LinearOpMode {
    protected static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    protected VisionPortal visionPortal;
    protected FtcDashboard dashboard = FtcDashboard.getInstance();
    private AprilTagProcessor aprilTagProcessor;                 // Used for managing the AprilTag detection process.
    private boolean targetFound = false;    // Set to true when an AprilTag target is detected
    private AprilTagDetection detectedAprilTag;        // Used to hold the data for a detected AprilTag

    protected abstract int getDesiredTagID();

    protected void detectAprilTag() {
        telemetry.addData("Inside opModeIsActive loop", "");

        this.doAprilDetection();

        this.displayDetectionTelemetry(detectedAprilTag);
        this.logTelemetryToDashBoard(detectedAprilTag);

    }

    protected void initAprilTag() {

//        aprilTagProcessor = new AprilTagProcessor.Builder()
//                .setDrawTagID(true)
//                .setDrawTagOutline(true)
//                .setDrawAxes(true)
//                .setDrawCubeProjection(true)
//                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
//                .build();
//
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        builder.setCameraResolution(new Size(640, 480));
//        builder.addProcessor(aprilTagProcessor);
//
//        visionPortal = builder.build();

        // Create the AprilTag processor the easy way.
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTagProcessor);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTagProcessor);
        }
    }


    private void doAprilDetection() {
        targetFound = false;
        detectedAprilTag = null;

        // Step through the list of detected tags and look for a matching tag

        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

        if (currentDetections == null || currentDetections.isEmpty()) {
            telemetry.addData("Detection result", "No tag is detected");
            return;
        }

        for (AprilTagDetection detection : currentDetections) {

            // Look to see if we have size info on this tag.

            if (detection.metadata != null) {

                //  Check to see if we want to track towards this tag.

                if ((detection.id == getDesiredTagID())) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    detectedAprilTag = detection;
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

//        if (targetFound) {
//
//            // telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
//
//            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
//
//            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
//
//            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
//
//            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
//
//        } else {
//
//            telemetry.addData("\n>","Drive using joysticks to find valid target\n");
//
//        }
    }

    protected void displayDetectionTelemetry(AprilTagDetection detectedId) {
        if (detectedId == null) {
            telemetry.addLine("detectedId: is null");
            return;
        }
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

    protected void logTelemetryToDashBoard(AprilTagDetection detectedId) {
        TelemetryPacket packet = new TelemetryPacket();
        if (detectedId == null) {
            packet.put("detected id:", null);

        }
        else {
            packet.put("detected id:", detectedId.id);
            packet.put("detectedId.ftcPose.range:", detectedId.ftcPose.range);
            packet.put("detectedId.ftcPose.bearing:", detectedId.ftcPose.bearing);
            packet.put("detectedId.ftcPose.elevation:", detectedId.ftcPose.elevation);

        }
        // Send packet to dashboard
        dashboard.sendTelemetryPacket(packet);
    }

    protected void stopVisionPortal() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
    protected AprilTagDetection getDetectedAprilTag() {
        return detectedAprilTag;
    }


    protected AprilTagProcessor getAprilTagProcessor() {
        return aprilTagProcessor;
    }

    protected boolean isTargetFound() {
        return targetFound;
    }

    protected VisionPortal getVisionPortal() {
        return visionPortal;
    }
}


