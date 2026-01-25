package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public abstract class AbstractFullAuto extends LinearOpMode {
//    private double home = 0, kick = 0.3;
//    private double kickCycleTime = 3;
    private static final int TURRET_MOTOR_GEAR_COUNT = 50;
    private static final int TURRET_GEAR_COUNT = 200;

    private ElapsedTime kickTimer = new ElapsedTime();

    protected FtcDashboard dashboard = FtcDashboard.getInstance();

    // Below are hardware
    private Servo kicker;
    protected MecanumDrive drive;
    //    private Servo outtakeservo = null;
    protected DcMotorEx intakeMotor = null;
    protected DcMotorEx outtakeMotor1 = null;
    protected DcMotorEx outtakeMotor2 = null;
    protected DcMotorEx turretMotor = null;
    protected Servo hoodServo = null;
    protected Servo blockServo;
    protected GoBildaPinpointDriver driver;


    // Below are for AprilTag
    protected static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    protected VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;                 // Used for managing the AprilTag detection process.
    private boolean targetFound = false;    // Set to true when an AprilTag target is detected
    private AprilTagDetection detectedAprilTag;        // Used to hold the data for a detected AprilTag

    private static final double DESIRED_DISTANCE = 12.0;       //  this is how close the camera should get to the target (inches)
    protected static final int DESIRED_TAG_ID_RED = 24;       // Choose the tag you want to approach or set to -1 for ANY tag.
    protected static final int DESIRED_TAG_ID_BLUE = 20 ;       // Choose the tag you want to approach or set to -1 for ANY tag.

    private static final double MAX_TURRET_TURN_POWER = 0.3;
    private double lastTargetPositionToMove = 0.0;
    protected static final double NEW_P_CLOSE = 40;
    protected static final double NEW_F_CLOSE = 15.3; //TODO: NEED TO TUNE P AND F FOR CLOSE SIDE
    protected static final double NEW_P_FAR = 90;
    protected static final double NEW_F_FAR = 14.3;

    // Hood Constants
    protected static final double HOOD_MIN_POSITION = 0.18;   // lowest angle
    protected static final double HOOD_MAX_POSITION = 0.62;   // highest angle
    protected static final double HOOD_INITIAL_TARGET_POSITION_CLOSE_SIDE = 0.55;
    protected static final double HOOD_INITIAL_TARGET_POSITION_FAR_SIDE = HOOD_MAX_POSITION;

    // Linear model (range → hood)
    private static final double HOOD_K = 0.007;   // position per inch
    private static final double HOOD_B = 0.12;    // base position

    private int ballCount;
    protected double lowVelocity = 1250d;// 1450 for far side
    protected double highVelocity = 1500d;// 1450 for far side
    // 84 = Tower height 99 - Robot height 35 + Goal height 20
    public static final double TARGET_HEIGHT = 84d;

    protected boolean useAprilTag = true;
    protected boolean shouldOpenGate = false;

    @Override
    public void runOpMode() {

        initAprilTag();
        initHardware();
        PIDFCoefficients pidfCoefficients = getPidfCoefficients();
        outtakeMotor1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        outtakeMotor2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        //TODO: instantiate your MecanumDrive at a particular pose.
        //63 is the edge of tile minus half the length of the robot
        Pose2d initialPose = getInitialPose();

        drive = new MecanumDrive(hardwareMap, initialPose);

        // Wait for the DS start button to be touched.
//        outtakeservo.setPosition(0.475);

        telemetry.update();
        waitForStart();
        //TODO: check if we need to move to init
        //TODO: robot must be stationary!! and init when stationary
        driver.recalibrateIMU();

        Action pathAction = getPathAction();

        if (isStopRequested()) {
            return;
        }
        while(opModeIsActive()) {
            outtakeMotor1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            outtakeMotor2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);this.detectAprilTag();
            this.logInfo();

            TelemetryPacket packet = new TelemetryPacket();
            if (!pathAction.run(packet)) {
                break;
            }
            drawAndLogTelemetry(packet);

        }
        stopVisionPortal();
    }

    protected TrajectoryActionBuilder strafeToOpenGate(TrajectoryActionBuilder actionBuilder, FieldSide fieldSide) {

        if (!shouldOpenGate) {
            //just return the actionBuilder since we don't need open gate
            return actionBuilder;
        }

        Vector2d gatePosition = null;
        switch (fieldSide) {
            case BLUE:
                gatePosition = new Vector2d(0, -63);
                break;
            case RED:
                gatePosition = new Vector2d(0, 63);
                break;
        };
        return actionBuilder.strafeToConstantHeading(gatePosition);   //to open gate
    }


    protected abstract PIDFCoefficients getPidfCoefficients();

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

    protected void reverseOuttake() {
        outtakeMotor1.setVelocity(900);
        outtakeMotor2.setVelocity(-900);
        //ensure the outtake has some time to spin up
        sleep(100);
    }

    protected int convertToTicks(double degree) {
        // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/?srsltid=AfmBOooipd93693DUvUrrENlLrLOl9bLTH1eXlhTHmXHPDSyLkckPCNS
        // Encoder Resolution:  537.7 pulses per revolution (PPR)
        //   537.7 pulses per revolution (PPR)  / 360 = 1.49361111111 ticks/degree
        return (int) (degree * 1.49361111111 * 200 / 50); // incorporates turret.
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

    protected Action getLaunchAction() {

//        Action launchAction = new Action() {
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                this.setOuttakePowerForClose();
//                this.kickBalls();
//                return false;
//            }
//        };
//        return launchAction;

        return telemetryPacket -> {

            this.sleep(350);
            this.intakeMotor.setPower(1);
            this.sleep(2000);
//            this.intakeMotor.setPower(0);

            return false;
        };
    }

    protected void setOuttakeSpeed(double outtakeSpeed) {
        outtakeMotor1.setVelocity(-1 * outtakeSpeed);
        outtakeMotor2.setVelocity(outtakeSpeed);

        this.sleep(200);
    }

    protected Action getAimAction(Double turretInitialTargetDegree, Double hoodInitialTargetPosition, boolean runAprilTagAim) {
        return telemetryPacket -> {

            boolean needToWaitForTurretOrHoodServo = false;

            //First move the turret and hood to a ballpark target position/angle
            if (turretInitialTargetDegree != null) {
                this.moveTurret(convertToTicks(turretInitialTargetDegree));
                needToWaitForTurretOrHoodServo = true;
            }

            if (hoodInitialTargetPosition != null) {
                this.moveHoodServo(hoodInitialTargetPosition);
                needToWaitForTurretOrHoodServo = true;
            }

            if (this.useAprilTag && runAprilTagAim) {
                if (needToWaitForTurretOrHoodServo) {
                    // Give the turret and hoodSevo some time run to the initial target position before we detect april tag
                    sleep(300);
                }
                sleep(500);
                //Further adjust the turret and hood angles by AprilTag detection and calculation
                this.detectAprilTag();
                this.aimAtTarget();  //aimAtTarget will move both turret and hood
            }
            return false;
        };
    }

    protected Action getIntakeAction() {
        return telemetryPacket -> {
            intakeMotor.setPower(1);

            return false;
        };
    }

    public abstract Pose2d getInitialPose();

    private void initHardware() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretmotor");
        outtakeMotor1 = hardwareMap.get(DcMotorEx.class,"outtakemotor1");
        outtakeMotor2 = hardwareMap.get(DcMotorEx.class,"outtakemotor2");
        intakeMotor = hardwareMap.get(DcMotorEx.class,"intakemotor");
        hoodServo = hardwareMap.get(Servo.class, "hoodservo");
        blockServo = hardwareMap.get(Servo.class, "blockservo");
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        //TODO: remember to check if this assumes start pos is 0,0

        driver.resetPosAndIMU();

        blockServo.setPosition(1);//0.5 is down, 1 is up


        resetMotorPosition();
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
            this.setManualExposure(1,100);  // Use low exposure time to reduce motion blur

        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTagProcessor);
        }
    }

    private void resetMotorPosition() {

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);//int type. Set target before setting RunMode.
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(MAX_TURRET_TURN_POWER);
    }

    private void logInfo() {
        telemetry.addData("apriltagstatus", this.useAprilTag);

        telemetry.addData("outtake motor left speed:", outtakeMotor2.getVelocity());
        telemetry.addData("outtake motor right speed:", outtakeMotor1.getVelocity());
        telemetry.addData("getcurrentpos:", this.getCurrentPos(drive));


//        telemetry.addData("hoodpos", hoodServo.getPosition());
        telemetry.addData("turretpos", turretMotor.getCurrentPosition());
        telemetry.addData("turrettargetpos", turretMotor.getTargetPosition());

        telemetry.addData("estimated pos:", driver.getPosition());

        telemetry.update();
    }
    protected void detectAprilTag() {

        this.doAprilDetection();

        this.displayDetectionTelemetry(detectedAprilTag);
        this.logTelemetryToDashBoard(detectedAprilTag);

    }
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls
        if (visionPortal == null) {
            return;
        }
        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    protected abstract int getDesiredTagID();

    protected void aimAtTarget() {
        int currentPosition = turretMotor.getCurrentPosition();
        telemetry.addData("Current motor pos", currentPosition);

        if (isTargetFound()) {//&& !turretMotor.isBusy()
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double rangeError = (this.getDetectedAprilTag().ftcPose.range - DESIRED_DISTANCE);
            double headingError = this.getDetectedAprilTag().ftcPose.bearing;
            double yawError = this.getDetectedAprilTag().ftcPose.yaw;

            int deltaPosition = convertToTicks(headingError);
            //cap the delta to maximum 2 clicks
//            deltaPosition = Range.clip(deltaPosition, 0, 2);
            int targetPosition = deltaPosition + currentPosition;

            telemetry.addLine("HeadingError: " + headingError);
            telemetry.addLine("Moving turret");
            telemetry.addData("Target motor pos", targetPosition);

            //move turret if the heading error > 3 degree
            if (Math.abs(headingError) > 3) {// && Math.abs(targetPosition) < convertToTicks(70)
                int offsetPosition = 10;
                this.moveTurret(targetPosition + offsetPosition);
            }
            else {
                telemetry.addLine("Target aimed, no need to move, stop the motor");
                turretMotor.setPower(0);
            }

            //move hood servo
            double servoPosition = this.calculateHoodPositionByAprilTagRange(this.getDetectedAprilTag().ftcPose.range);
            this.moveHoodServo(servoPosition);
        } else {
            telemetry.addLine("Target not found or turretMotor is busy");


        }
    }

    protected void moveTurret(int targetPosition) {
        turretMotor.setTargetPosition(targetPosition);
        turretMotor.setPower(MAX_TURRET_TURN_POWER);

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
    }
    protected Pose2d getCurrentPos(MecanumDrive drive) {
        return drive.localizer.getPose();
    }

    // Calculate the hood sevo position by liner model. We can't really calculate the position via geometry based on the
    // current position as the sevo doesn't have real current position returned from hardware.
    private double calculateHoodPositionByAprilTagRange(double range) {
        return HOOD_K * range + HOOD_B;
    }


    protected void moveHoodServo(double targetPosition) {
        telemetry.addData("HoodServo", "Trying to set position %f", targetPosition);

        double thePosition = Range.clip(targetPosition, HOOD_MIN_POSITION, HOOD_MAX_POSITION);
        telemetry.addData("HoodServo", "Setting position %f", thePosition);
        hoodServo.setPosition(thePosition);
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