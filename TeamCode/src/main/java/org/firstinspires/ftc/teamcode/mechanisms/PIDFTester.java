package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AbstractAprilTag;

public class PIDFTester extends AbstractAprilTag {
    private static final double DESIRED_DISTANCE = 12.0;       //  this is how close the camera should get to the target (inches)
    protected static final int DESIRED_TAG_ID = 24;       // Choose the tag you want to approach or set to -1 for ANY tag.
    private static final int TURRET_GEAR_COUNT = 200;
    private static final int TURRET_MOTOR_GEAR_COUNT = 50;
    private static final double MAX_TURRET_TURN_POWER = 0.3;

    // 84 = Tower height 99 - Robot height 35 + Goal height 20
    public static final double TARGET_HEIGHT = 84d;
    private DcMotorEx turretMotor;
    private Servo hoodServo;

    private double lastTargetPositionToMove = 0.0;

    public void runOpMode() {
        initServo();
        initAprilTag();
        telemetry.update();
        waitForStart();



        if (isStopRequested()) {
            stopVisionPortal();
            return;
        }
        while (opModeIsActive()) {

            this.detectAprilTag();
            this.aimAtTarget();
            telemetry.addData("Last target pos to move", lastTargetPositionToMove);
            telemetry.update();


        }

    }

    private double calculateHoodDegreeToChange() {
        double degreeBasedOnStartPosition = this.calculateHoodDegreeBasedOnStartedPosition();
        double hoodServoPos = hoodServo.getPosition();
        double servoPosition = hoodServoPos / 300;

        double hoodToChange;
        hoodToChange = degreeBasedOnStartPosition - servoPosition;
        return hoodToChange;
    }
    private double calculateHoodDegreeBasedOnStartedPosition() {
        double closestRange = Math.sqrt(60d * 60d + TARGET_HEIGHT * TARGET_HEIGHT);
        double furthestRange = Math.sqrt(290d * 290d + TARGET_HEIGHT * TARGET_HEIGHT);
        double range = this.getDetectedAprilTag().ftcPose.range;
        if (range <closestRange){
            range = closestRange;
        }
        if (range > furthestRange){
            range = furthestRange;
        }

        double theta = Math.asin(TARGET_HEIGHT / range);

        double thetaMax = Math.asin(TARGET_HEIGHT/closestRange);
        double thetaMin = Math.asin(TARGET_HEIGHT/furthestRange);
        double thetaProportion = (thetaMax - theta) / (thetaMax - thetaMin);
        double hoodChangeDegree = thetaProportion * 154.2857;   // 154.2857 = 360/ 14 * 6
        if (hoodChangeDegree < 0) {
            hoodChangeDegree = 0;
        }
        if (hoodChangeDegree > 154.2857) {
            hoodChangeDegree = 154.2857;
        }
        return hoodChangeDegree;
    }

    private void moveServoAngle() {
        hoodServo.getPosition();
        double servoPosition = calculateHoodDegreeBasedOnStartedPosition() / 180;
        hoodServo.setPosition(servoPosition);
    }


    private void initServo() {

        turretMotor = hardwareMap.get(DcMotorEx.class, "turretmotor");
        //TODO: Tune the direction
        turretMotor.setDirection(DcMotorEx.Direction.REVERSE);
        resetMotorPosition();

    }
    private void resetMotorPosition() {

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);//int type. Set target before setting RunMode.
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(MAX_TURRET_TURN_POWER);

        telemetry.addData("Current position after reset", turretMotor.getCurrentPosition());

    }

    private void aimAtTarget() {
        telemetry.addData("Current motor pos", turretMotor.getCurrentPosition());
        telemetry.addData("is busy status: ", turretMotor.isBusy());


        if (isTargetFound()) {//&& !turretMotor.isBusy()

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double rangeError = (this.getDetectedAprilTag().ftcPose.range - DESIRED_DISTANCE);
            double headingError = this.getDetectedAprilTag().ftcPose.bearing;
            double yawError = this.getDetectedAprilTag().ftcPose.yaw;
            int targetPosition = convertToTicks(headingError) + turretMotor.getCurrentPosition();
            moveServoAngle();
            telemetry.addLine("HeadingError: " + headingError);
            telemetry.addLine("Moving turret");
            telemetry.addData("Target motor pos", targetPosition);

            if (Math.abs(headingError) >= 3) {// && Math.abs(targetPosition) < convertToTicks(70)
                lastTargetPositionToMove = targetPosition;
                turretMotor.setTargetPosition(-targetPosition);
                turretMotor.setPower(MAX_TURRET_TURN_POWER);
            }
            else {
                telemetry.addLine("Target aimed, no need to move, stop the motor");
                turretMotor.setPower(0);
            }

        } else {
            telemetry.addLine("Target not found or turretMotor is busy");
        }
    }

    private int convertToTicks(double degree) {
        // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/?srsltid=AfmBOooipd93693DUvUrrENlLrLOl9bLTH1eXlhTHmXHPDSyLkckPCNS
        // Encoder Resolution:  537.7 pulses per revolution (PPR)
        //   537.7 pulses per revolution (PPR)  / 360 = 1.49361111111 ticks/degree
        return (int) (degree * 1.49361111111);

    }


    @Override
    protected int getDesiredTagID() {
        return DESIRED_TAG_ID;
    }
}


