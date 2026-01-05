package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.AbstractAprilTag;

public class HoodTester extends AbstractAprilTag {
    private static final double DESIRED_DISTANCE = 12.0;       //  this is how close the camera should get to the target (inches)
    protected static final int DESIRED_TAG_ID = 24;       // Choose the tag you want to approach or set to -1 for ANY tag.
    private static final int TURRET_GEAR_COUNT = 200;
    private static final int TURRET_MOTOR_GEAR_COUNT = 50;
    private static final double MAX_TURRET_TURN_POWER = 0.3;
    private DcMotorEx hoodServo;

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

    private void initServo() {

        hoodServo = hardwareMap.get(DcMotorEx.class, "turretmotor");
        //TODO: Tune the direction
        hoodServo.setDirection(DcMotorEx.Direction.REVERSE);
        resetMotorPosition();

    }

    private void resetMotorPosition() {

        hoodServo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hoodServo.setTargetPosition(0);//int type. Set target before setting RunMode.
        hoodServo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hoodServo.setPower(MAX_TURRET_TURN_POWER);

        telemetry.addData("Current position after reset", hoodServo.getCurrentPosition());

    }

    private void aimAtTarget() {
        telemetry.addData("Current motor pos", hoodServo.getCurrentPosition());
        telemetry.addData("is busy status: ", hoodServo.isBusy());


        if (isTargetFound()) {//&& !turretMotor.isBusy()
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double rangeError = (this.getDetectedAprilTag().ftcPose.range - DESIRED_DISTANCE);
            double headingError = this.getDetectedAprilTag().ftcPose.bearing;
            double yawError = this.getDetectedAprilTag().ftcPose.yaw;
            int targetPosition = convertToTicks(headingError) + hoodServo.getCurrentPosition();

            telemetry.addLine("HeadingError: " + headingError);
            telemetry.addLine("Moving turret");
            telemetry.addData("Target motor pos", targetPosition);

            if (Math.abs(headingError) >= 3) {// && Math.abs(targetPosition) < convertToTicks(70)
                lastTargetPositionToMove = targetPosition;
                hoodServo.setTargetPosition(-targetPosition);
                hoodServo.setPower(MAX_TURRET_TURN_POWER);
            }
            else {
                telemetry.addLine("Target aimed, no need to move, stop the motor");
                hoodServo.setPower(0);
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


