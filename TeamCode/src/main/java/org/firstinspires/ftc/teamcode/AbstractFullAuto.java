package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public abstract class AbstractFullAuto extends LinearOpMode {
    private double home = 0, kick = 0.3;
    private double kickCycleTime = 3;
    private static final int TURRET_MOTOR_GEAR_COUNT = 50;
    private static final int TURRET_GEAR_COUNT = 200;

    private ElapsedTime kickTimer = new ElapsedTime();

    protected FtcDashboard dashboard = FtcDashboard.getInstance();

    // Below are hardware
    private Servo kicker;
    //    private Servo outtakeservo = null;
    protected MecanumDrive drive;
    protected DcMotorEx intakemotor = null;
    protected DcMotorEx outtakemotor1 = null;
    protected DcMotorEx outtakemotor2 = null;
    protected DcMotorEx turretmotor = null;

    protected DcMotorEx transfermotor = null;


    // Below are for AprilTag
    private VisionPortal visionPortal;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private static final double DESIRED_DISTANCE = 12.0;       //  this is how close the camera should get to the target (inches)
    private static final int DESIRED_TAG_ID = -1;       // Choose the tag you want to approach or set to -1 for ANY tag.
    private AprilTagProcessor aprilTag;                 // Used for managing the AprilTag detection process.
    protected AprilTagDetection desiredTag;        // Used to hold the data for a detected AprilTag
    protected boolean targetFound = false;    // Set to true when an AprilTag target is detected
    protected double turretTarget = 0;
    protected double headingError;

    private static final double NEW_P = 2.5;
    private static final double NEW_I = 0.1;
    private static final double NEW_D = 0.2;
    private static final double NEW_F = 0.5;

    @Override
    public void runOpMode() {


        turretmotor.setTargetPosition((int) turretTarget);//int type. Set target before setting RunMode.
        turretmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

//        initHardware();

        //TODO: instantiate your MecanumDrive at a particular pose.
        //63 is the edge of tile minus half the length of the robot
        Pose2d initialPose = getInitialPose();

        drive = new MecanumDrive(hardwareMap, initialPose);
        initAprilTag();

        // Wait for the DS start button to be touched.

        kicker.setPosition(0.075);
//        outtakeservo.setPosition(0.475);
        kickTimer.reset();

        waitForStart();
        turretmotor.setPower(1);// specifies max available power to motor

        visionPortal.close();

        detectAprilTag();


        // First run
        Action pathAction = getPathAction();

        while(opModeIsActive()) {

            TelemetryPacket packet = new TelemetryPacket();
            aimAtTarget();
            if (!pathAction.run(packet)) {
                break;
            }
            drawAndLogTelemetry(packet);
        }

        if (isStopRequested()) {
            return;
        }
    }

    protected double aimAtTarget() {
        if (targetFound) {
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            headingError = desiredTag.ftcPose.bearing;
            double  yawError        = desiredTag.ftcPose.yaw;
            turretmotor.setTargetPosition((int) (headingError*TURRET_GEAR_COUNT/TURRET_MOTOR_GEAR_COUNT*1.49444444444));//ticks multiply?
        }

        return headingError;
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

    private void drawAndLogTelemetry(TelemetryPacket packet) {
        Pose2d pose = getCurrentPos(drive);
        drawRobot(packet, pose);   // Draw robot on overlay

// === Log basic pose ===
        packet.put("x", pose.position.x);
        packet.put("y", pose.position.y);
        packet.put("headingDeg", Math.toDegrees(pose.heading.toDouble()));

// === Log motor velocity ===


        // === Log PID error (your custom controller) ===
        // Example:
//            packet.put("lift_error", drive.leftFront.getPositionError());


        // Send packet to dashboard
        dashboard.sendTelemetryPacket(packet);
    }

    protected void drawRobot(TelemetryPacket packet, Pose2d pose) {
        Canvas field = packet.fieldOverlay();

        double x = pose.position.x;
        double y = pose.position.y;
        double heading = pose.heading.toDouble();

        // === ROBOT DIMENSIONS (adjust to your bot!) ===
        final double ROBOT_LENGTH = 18; // inches
        final double ROBOT_WIDTH  = 18;

        double halfL = ROBOT_LENGTH / 2.0;
        double halfW = ROBOT_WIDTH / 2.0;

        // Draw center
        field.strokeCircle(x, y, 1);

        // Draw heading arrow
        double arrowLength = 9;
        double arrowX = x + arrowLength * Math.cos(heading);
        double arrowY = y + arrowLength * Math.sin(heading);
        field.strokeLine(x, y, arrowX, arrowY);

        // Draw rotated rectangle robot body
        field.strokePolygon(
                new double[]{x - halfL, x + halfL, x + halfL, x - halfL},
                new double[]{y - halfW, y - halfW, y + halfW, y + halfW}

        );

        // Draw wheels
        field.strokeCircle(x + halfL, y + halfW, 1);
        field.strokeCircle(x + halfL, y - halfW, 1);
        field.strokeCircle(x - halfL, y + halfW, 1);
        field.strokeCircle(x - halfL, y - halfW, 1);
    }

    protected void logPidTelemetry(Pose2d targetPose, TelemetryPacket packet){

        Pose2d curr = getCurrentPos(drive);
        double xError = targetPose.position.x - curr.position.x;
        double yError = targetPose.position.y - curr.position.y;
        double hError = AngleUtil.normDelta(targetPose.heading.toDouble() - curr.heading.toDouble());

        packet.put("xError", xError);
        packet.put("yError", yError);
        packet.put("headingErrorDeg", Math.toDegrees(hError));
    }


    protected abstract Action getPathAction();

    protected abstract Action getLaunchAction();

    protected Action getIntakeAction() {
        return telemetryPacket -> {
            intakemotor.setPower(0.3);
            transfermotor.setPower(-0.3);

            return false;
        };
    }

    public abstract Pose2d getInitialPose();



//    private void initHardware() {
//        outtakemotorright = hardwareMap.get(DcMotorEx.class, "outtakemotorright");
////        outtakeservo = hardwareMap.get(Servo.class, "outtakeservo");
//        transfermotor = hardwareMap.get(DcMotorEx.class, "transfermotor");
//        outtakemotorleft = hardwareMap.get(DcMotorEx.class,"outtakemotorleft");
//        intakemotor = hardwareMap.get(DcMotorEx.class,"intakemotor");
//
//        kicker = hardwareMap.get(Servo.class,"kickservo");
//    }


    public void kickBalls() {

        intakemotor.setPower(0);
        transfermotor.setPower(0); // stop intake and transfer
        kickAuto(1); // 1st ball
        waitForTime(kickCycleTime*0.6); //wait for 2nd / 3rd ready

        intakemotor.setPower(1);
        transfermotor.setPower(-0.6);  // Push 2nd ball forward
        waitForTime(kickCycleTime*0.5); //wait for 2nd / 3rd ready

        transfermotor.setPower(0);
        waitForTime(kickCycleTime*0.2); //wait for motor stop

        kickAuto(2); // 2nd ball

        transfermotor.setPower(-0.8);  // Push 3rd ball forward
        waitForTime(kickCycleTime* 0.5); //wait for the intake stop,and out take back to speed completely

        kickAuto(3); // kick 3rd ball
    }


    protected void kickAuto(int ballNumber) {
        telemetry.addData("outtake motor left speed:", outtakemotor2.getVelocity());
        telemetry.addData("outtake motor right speed:", outtakemotor1.getVelocity());

        kicker.setPosition(kick);
        waitForTime(kickCycleTime*0.25);

        kicker.setPosition(0.055);

        if (ballNumber<3) {
            //So kicker has time to go back to position 0
            waitForTime(kickCycleTime*0.25);

        }
        telemetry.addData("outtake motor left speed:", outtakemotor2.getVelocity());
        telemetry.addData("outtake motor right speed:", outtakemotor1.getVelocity());

        telemetry.update();
    }

    protected void waitForTime(double waitTime) {
        kickTimer.reset();

        while (this.opModeIsActive() && kickTimer.seconds() < waitTime) {// && kickTimer.seconds() < waitTime
            //do nothing, just wait
            logInfo();
            //telemetry.addData("waiting to kick: ", kickTimer.time());
        }
    }

    private void logInfo() {
        telemetry.addData("outtake motor left speed:", outtakemotor2.getVelocity());
        telemetry.addData("outtake motor right speed:", outtakemotor1.getVelocity());
        telemetry.addData("getcurrentpos:", this.getCurrentPos(drive));

        telemetry.update();
    }

    protected Pose2d getCurrentPos(MecanumDrive drive) {
        return drive.localizer.getPose();
    }


    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }

    private void telemetryAprilTag () {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()
}