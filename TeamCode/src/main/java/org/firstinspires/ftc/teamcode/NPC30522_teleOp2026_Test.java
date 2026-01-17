package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**

* This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs
* in either the autonomous or the TeleOp period of an FTC match. The names of OpModes appear on
* the menu of the FTC Driver Station. When an selection is made from the menu, the corresponding
* OpMode class is instantiated on the Robot Controller and executed.
*
* Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
* Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
* added to the Driver Station.
*/

@TeleOp

public class NPC30522_teleOp2026_Test extends LinearOpMode {
   
// Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)
    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
   
    /* =====================
       Vision
     ===================== */
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    private boolean targetFound = false;
    private double targetVelocity = 0;
    private double distanceToTarget = 0;
    
    /* =====================
       Turret tuning
     ===================== */
    private static final double TURRET_KP = 0.015;
    private static final double TURRET_DEADBAND = 3;
    private static final double MAX_TURRET_POWER = 0.3;
    private static final int TURRET_MIN_TICKS = -900;
    private static final int TURRET_MAX_TICKS = 900;
    private double headingError;
    
    /* =====================
       Hood tuning
     ===================== */
    //hood servo up hoodservo.setPosition(1), hoodservo.setPosition(0.9)(5 turn servo)
   //Safe mechanical limits (tune these!)
    private static final double HOOD_MIN = 0.18;   // lowest angle
    private static final double HOOD_MAX = 0.62;   // highest angle
    private double hoodManualOffset = 0.0;

   /*Preset shooting angles
    private static final double HOOD_CLOSE = 0.25;
    private static final double HOOD_MID   = 0.38;
    private static final double HOOD_FAR   = 0.52;*/
    
   // Linear model (distance → hood)
   private static final double HOOD_K = 0.007;   // position per inch
   private static final double HOOD_B = 0.12;    // base position

    //Constants for outtakemotors
    final double MIN_VELOCITY = 1000;
    final double MAX_VELOCITY = 2000;
    final double SPINUP_TIME = 0.35;          // seconds
    final double VELOCITY_READY_RATIO = 0.95;
    final double SHOOTER_K = 7.5;   // velocity per inch
    final double SHOOTER_B = 865; // base velocity

    private DcMotorEx frontleft,frontright,backleft,backright,outtakemotor1,outtakemotor2,intakemotor,turretmotor;  //  Used to control the left front drive wheel
    private Servo blockservo,hoodservo,RGB;

    /* =====================
       Robot moving
     ===================== */
    private IMU imu;
    private double  botHeading, rotX, rotY, denominator, x, y, rx;
    private double  drive           = 0;        // Desired forward power/speed (-1 to +1)
    private double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
    private double  turn            = 0;        // Desired turning power/speed (-1 to +1)
   
    private ElapsedTime shooterTimer = new ElapsedTime();
   
    // =========================
    // Shooter State Machine
    // =========================
    enum ShooterState {
        IDLE,
        BLOCKUP,
        SPINUP,
        FEED,
        FINISH
    }
    ShooterState shooterState = ShooterState.IDLE;


    @Override public void runOpMode()
    {
        /*telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );
        telemetry.addLine("Dashboard connected");
        telemetry.update();*/

        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        //Gamepad currentGamepad1 = new Gamepad();
        //Gamepad previousGamepad1 = new Gamepad();
    
        // Initialize the Apriltag Detection process
        initAprilTag();
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        frontleft = hardwareMap.get(DcMotorEx.class, "frontleft");
        frontright = hardwareMap.get(DcMotorEx.class, "frontright");
        backleft = hardwareMap.get(DcMotorEx.class, "backleft");
        backright = hardwareMap.get(DcMotorEx.class, "backright");
        
        outtakemotor1 = hardwareMap.get(DcMotorEx.class,"outtakemotor1");
        outtakemotor2 = hardwareMap.get(DcMotorEx.class, "outtakemotor2");      
        intakemotor = hardwareMap.get(DcMotorEx.class,"intakemotor");
        turretmotor = hardwareMap.get(DcMotorEx.class,"turretmotor");
        
        blockservo = hardwareMap.get(Servo.class, "blockservo");  
        hoodservo =    hardwareMap.get(Servo.class, "hoodservo");
        RGB = hardwareMap.get(Servo.class, "RGB");
        
       //PIDFCoefficients settings: this configures the internal velocity PID controller (Motor output power to hit target velocity); it persists for the entire OpMode
       //F does ~90% of the work; P cleans up error caused by ball contact
        PIDFCoefficients shooterPIDF =  new PIDFCoefficients(
                12.0,    //P (Proportional) -- Corrects velocity error instantly,increase P until spin-up is fast but stable
                0.0,     //I (Integral) -- Fixes long-term undershoot (usually 0 for shooters)
                0.0,     //D (Derivative) -- Dampens oscillation (often 0 for flywheels)
                15  //F (Feedforward) -- Base power needed to spin at target speed
        );
        outtakemotor1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,shooterPIDF);
        outtakemotor2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,shooterPIDF);

       
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.FORWARD);
        backright.setDirection(DcMotor.Direction.FORWARD);
        turretmotor.setDirection(DcMotor.Direction.FORWARD);
        turretmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakemotor1.setDirection(DcMotor.Direction.REVERSE);
        outtakemotor2.setDirection(DcMotor.Direction.FORWARD);        
                 
        if (USE_WEBCAM)
            setManualExposure(1,100);  // Use low exposure time to reduce motion blur
        // Wait for driver to press start
        // telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        // telemetry.addData(">", "Touch START to start OpMode");
        // telemetry.update();

        blockservo.setPosition(0.6);//0.6? is down, 0.45 is the flat position. // Austin 1/15/2026 - block servo 0.5 is down
        hoodservo.setPosition(0.25);
        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
          //previousGamepad1.copy(currentGamepad1);
          //currentGamepad1.copy(gamepad1);

         boolean shooterActive = (shooterState != ShooterState.IDLE); //Avoid Intake conflict: manual intake VS shooter in the SHOOT STATE MACHINE
         
          if (!shooterActive) {
             if (gamepad1.right_bumper)
                intakemotor.setPower(0.67);
             else
                intakemotor.setPower(0);
          }        
           
            updateAprilTag();
            updateTurret();
            updateHood();
            updateShooter();          


            if (gamepad1.options) {
                imu.resetYaw();
            }
           
            updateDriveFromGamepad();
            //assistedAutoAim();
            moveRobot(drive, strafe, turn);
            //sleep(10);
         
            // ---------------------------------
            // Telemetry
            // ---------------------------------
            telemetry.addData("outtakemotor1 velocity differece: ", Math.abs(outtakemotor1.getVelocity() - targetVelocity)); //must be less than 100
            telemetry.addData("outtakemotor2 velocity differece: ", Math.abs(outtakemotor2.getVelocity() - targetVelocity)); //must be less than 100
            telemetry.addData("Shooter State: ", shooterState);
            telemetry.addData("Target Found: ", targetFound);
            telemetry.addData("Distance: ", distanceToTarget);
            telemetry.addData("Target Velocity: ", targetVelocity);
            telemetry.addData("Turret heading error: ", Math.abs(headingError));
            telemetry.addData("Hood Pos", hoodservo.getPosition());
            telemetry.addData("Hood Offset", hoodManualOffset);

            telemetry.update();        
        }  //the big WHILE loop end
        visionPortal.close();
    } //runOpMode() end

///////////////////Class scope methods//////////////////////////
    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor by using a builder.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        //aprilTag = new AprilTagProcessor.Builder().build();
        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(2);
        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            /*visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();*/
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"),
                    aprilTag
            );

        } else {
            /*visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();*/
             visionPortal = VisionPortal.easyCreateWithDefaults(
                            BuiltinCameraDirection.BACK,
                            aprilTag
                            );
        }
    }
    /* =====================
       Vision update
       ===================== */
  private void updateAprilTag() {
             //targetFound = true;    
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
               //automatic outtakemotor velocity caculation
               distanceToTarget = desiredTag.ftcPose.range;
               //distanceToTarget = 24.0 //tuning for testing
               targetVelocity = SHOOTER_K * (distanceToTarget) + SHOOTER_B; //we use linear model to describe the relationship between velocity and distance
               targetVelocity = Range.clip(
                        targetVelocity,
                        MIN_VELOCITY,
                        MAX_VELOCITY
                );//It limits shooter speed so it never goes too slow or too fast.
                //telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                //telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                //telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                // telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                // telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {
                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
            }
  }
 
      /* =====================
       Turret control
       ===================== */
    private void updateTurret() {
            // RESOLVED: Use D-pad for manual override to prevent conflict with Robot Turning
    double manual = 0;
    if (gamepad1.dpad_left) {
        manual = -MAX_TURRET_POWER;
    } else if (gamepad1.dpad_right) {
        manual = MAX_TURRET_POWER;
    }
    // Manual override check
    if (manual != 0) {
        turretmotor.setPower(manual);
        return;
    }
    // If no manual input and no target, stop motor
    if (!targetFound) {
        turretmotor.setPower(0);
        return;
    }
    headingError = desiredTag.ftcPose.bearing;
    // Deadband check to prevent "jittering"
    if (Math.abs(headingError) < TURRET_DEADBAND) {
        turretmotor.setPower(0);
        return;
    }
    // P-Control for Auto-Aim
    double power = Range.clip(
            headingError * TURRET_KP,
            -MAX_TURRET_POWER,
            MAX_TURRET_POWER
    );
    // Software Limits (Check encoder ticks)
    int pos = turretmotor.getCurrentPosition();
    if ((pos <= TURRET_MIN_TICKS && power < 0) ||
        (pos >= TURRET_MAX_TICKS && power > 0)) {
        power = 0;
    }
    turretmotor.setPower(power); // Verify if '-' is needed based on your direction settings
    }
 
       /* =====================
       Hood control
       ===================== */
      private void updateHood() {
        if (!targetFound) return;

        // Manual adjusting
       if (gamepad1.dpad_up)   hoodManualOffset += 0.002;
       if (gamepad1.dpad_down) hoodManualOffset -= 0.002;

       hoodManualOffset = Range.clip(hoodManualOffset, -0.08, 0.08);

       double hoodPos = HOOD_K * distanceToTarget + HOOD_B + hoodManualOffset;
       hoodPos = Range.clip(hoodPos, HOOD_MIN, HOOD_MAX);

       hoodservo.setPosition(hoodPos);
}

      /* =====================
       Shooter control
       ===================== */
    private void updateShooter() {
        /*getVelocity() is measured by motor encoders; Changes every loop;
         Affected by:Battery voltage,Ball contact,Friction, Motor wear
         This is the velocity that launches the ball*/
        boolean readyforshooting = Math.abs(outtakemotor1.getVelocity() - targetVelocity) < 100 && Math.abs(outtakemotor2.getVelocity() - targetVelocity) < 100;
        
        if (shooterState == ShooterState.IDLE) {
          if (gamepad1.x && targetFound) {
             //blockservo.setPosition(0.55);
            outtakemotor1.setVelocity(targetVelocity);
            outtakemotor2.setVelocity(targetVelocity);
             shooterState = ShooterState.BLOCKUP;
             //shooterState = ShooterState.SPINUP;
             shooterTimer.reset();
          }
        }
            // ---------------------------------
            // Shooter State Machine
            // ---------------------------------
            switch (shooterState) {
                case IDLE:
                     outtakemotor1.setVelocity(0); //setVelocity() uses RUN_USING_ENCODER PIDF
                     outtakemotor2.setVelocity(0); //setVelocity() uses RUN_USING_ENCODER PIDF
                     break;
                     
                case BLOCKUP:
                    outtakemotor1.setVelocity(targetVelocity);
                     outtakemotor2.setVelocity(targetVelocity);
                     if (shooterTimer.seconds()>= 0.5) 
                    {
                        //blockservo.setPosition(0.55);//block UP
                        shooterState = ShooterState.SPINUP;
                        shooterTimer.reset();
                    }
                    break;
                    
                case SPINUP:
                     //updateHood();
                     outtakemotor1.setVelocity(targetVelocity);
                     outtakemotor2.setVelocity(targetVelocity);
                     blockservo.setPosition(1);//block UP
                    if ((shooterTimer.seconds() >= 0.6)) 
                    {  
                        //intakemotor.setPower(0.8);
                        shooterState = ShooterState.FEED;
                        shooterTimer.reset();
                    }
                    break;

                case FEED:
                    outtakemotor1.setVelocity(targetVelocity);
                    outtakemotor2.setVelocity(targetVelocity);
                    intakemotor.setPower(0.8);

                 if (shooterTimer.seconds() >= 1 && readyforshooting) {
                    shooterState = ShooterState.FINISH;
                    shooterTimer.reset();
                 }
                    break;
                    
                case FINISH:
                     if (shooterTimer.seconds() >= 0.6){
                        blockservo.setPosition(0.6); // blocker DOWN
                        intakemotor.setPower(0);
                        outtakemotor1.setVelocity(0);
                        outtakemotor2.setVelocity(0);
                        shooterState = ShooterState.IDLE;
                     }
                     break;
            }        
    }
   
    /* =====================
       Update drive
      ===================== */  
    private void updateDriveFromGamepad() {
          y = -gamepad1.left_stick_y;
          x = gamepad1.left_stick_x;
          rx = gamepad1.right_stick_x;
          botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
          rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
          rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
          rotX *= 1.1;
          drive  = rotY;
          strafe = rotX;
          turn   = rx;
   }
    /* =====================
       Assisted auto-aim
      ===================== */    
    private void assistedAutoAim() {

    if (!(gamepad1.left_bumper && targetFound)) {
        return; // do nothing, driver stays in full control
    }
               if (gamepad1.left_bumper && targetFound) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;

                // drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = -Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                // strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                drive = drive*0.5;
                strafe = strafe*0.5;
                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            //telemetry.update();    
    }
    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double frontLeftPower    =  x + y + yaw;
        double frontRightPower   =  x - y - yaw;
        double backLeftPower     =  x - y + yaw;
        double backRightPower    =  x + y - yaw;
        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Send powers to the wheels.
        frontleft.setPower(frontLeftPower);
        frontright.setPower(frontRightPower);
        backleft.setPower(backLeftPower);
        backright.setPower(backRightPower);
    }

   /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls
        if (visionPortal == null) {
            return;
        }
        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
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
}