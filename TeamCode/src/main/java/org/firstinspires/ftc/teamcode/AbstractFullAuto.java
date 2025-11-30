package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;

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

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    protected DcMotorEx intakemotor = null;
    private ElapsedTime kickTimer = new ElapsedTime();
    private double kickCycleTime = 3;
    private Servo kicker;
    protected DcMotorEx outtakemotorright = null;
    protected DcMotorEx outtakemotorleft = null;
    protected DcMotorEx transfermotor = null;
    private Servo outtakeservo = null;
    private double home = 0, kick = 0.3;

    @Override
    public void runOpMode() {

        initHardware();

        //TODO: instantiate your MecanumDrive at a particular pose.
        //63 is the edge of tile minus half the length of the robot
        Pose2d initialPose = getInitialPose();

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        initAprilTag();


        // Wait for the DS start button to be touched.


        kicker.setPosition(0.075);
        outtakeservo.setPosition(0.475);
        kickTimer.reset();

        waitForStart();
        visionPortal.close();
        setOuttakePower();

        // First run


        runFirstPath(drive, initialPose);

        waitForTime(1); //wait for the outtake to slow down
        kickBalls();


        // 2nd run
//        intakemotor.setPower(0);
//        transfermotor.setPower(0);
        runSecondPath(drive);

        setOuttakePower();
        kickBalls();
        parkOutsideLaunch(drive);

        if (isStopRequested()) {
            return;

            }


    }



    public abstract Pose2d getInitialPose();
    public abstract void setOuttakePower();
    public abstract void runFirstPath(MecanumDrive drive, Pose2d initialPose);
    public abstract void runSecondPath(MecanumDrive drive);
    public abstract void parkOutsideLaunch(MecanumDrive drive);



    private void initHardware() {
        outtakemotorright = hardwareMap.get(DcMotorEx.class, "outtakemotorright");
        outtakeservo = hardwareMap.get(Servo.class, "outtakeservo");
        transfermotor = hardwareMap.get(DcMotorEx.class, "transfermotor");
        outtakemotorleft = hardwareMap.get(DcMotorEx.class,"outtakemotorleft");
        intakemotor = hardwareMap.get(DcMotorEx.class,"intakemotor");

        kicker = hardwareMap.get(Servo.class,"kickservo");
    }


    public void kickBalls() {
        kickAuto(1); // 1st ball
        waitForTime(kickCycleTime*0.3); //wait for 2nd / 3rd ready

        intakemotor.setPower(1);
        transfermotor.setPower(-0.5);  // Push 2nd ball forward
        waitForTime(kickCycleTime*0.3); //wait for 2nd / 3rd ready

        transfermotor.setPower(0);
        waitForTime(kickCycleTime*0.2); //wait for motor stop

        kickAuto(2); // 2nd ball

        transfermotor.setPower(-0.5);  // Push 3rd ball forward
        waitForTime(kickCycleTime* 0.5); //wait for the intake stop,and out take back to speed completely

        kickAuto(3); // kick 3rd ball
    }


    protected void kickAuto(int ballNumber) {
        telemetry.addData("outtake motor left speed:", outtakemotorleft.getVelocity());
        telemetry.addData("outtake motor right speed:", outtakemotorright.getVelocity());

        kicker.setPosition(kick);
        waitForTime(kickCycleTime*0.25);

        kicker.setPosition(0.055);

        if (ballNumber<3) {
            //So kicker has time to go back to position 0
            waitForTime(kickCycleTime*0.25);

        }
        telemetry.addData("outtake motor left speed:", outtakemotorleft.getVelocity());
        telemetry.addData("outtake motor right speed:", outtakemotorright.getVelocity());

        telemetry.update();
    }

    private void waitForTime(double waitTime) {
        kickTimer.reset();

        while (kickTimer.seconds() < waitTime) {
            //do nothing, just wait
            telemetry.addData("outtake motor left speed:", outtakemotorleft.getVelocity());
            telemetry.addData("outtake motor right speed:", outtakemotorright.getVelocity());

            telemetry.update();
            //telemetry.addData("waiting to kick: ", kickTimer.time());
        }
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