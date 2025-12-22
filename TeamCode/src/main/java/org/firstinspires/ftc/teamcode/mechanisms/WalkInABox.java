package org.firstinspires.ftc.teamcode.mechanisms;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@Autonomous
public class WalkInABox extends LinearOpMode {
    static final double DRIVE_POWER = 1;
    static final double TURN_POWER  = 1;
    static final long DRIVE_FORWARD_MS = 1800; // adjust!
    static final long TURN_90_DEG_MS   = 700;  // adjust!
    private final ElapsedTime driveTimer = new ElapsedTime();
    private DcMotorEx frontLeft, backLeft, frontRight, backRight;


    @Override
    public void runOpMode() {
        //TODO: instantiate your MecanumDrive at a particular pose.
        // Wait for the DS start button to be touched.
        telemetry.addData("Starting WalkInABox", "");
        telemetry.update();

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backleft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontright");
        backRight = hardwareMap.get(DcMotorEx.class, "backright");

        //TODO: adjust direction
//        frontLeft.setDirection(DcMotor.Direction.REVERSE);
//        backLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        // Run the square: forward + turn, 4 times

        //Option 1
        runInSquareOption1();

        //Option 2
//        runInSquareOption2();

    }


    private void runInSquareOption1() {
        for (int i = 0; i < 4 && opModeIsActive(); i++) {

            driveForward(DRIVE_POWER, DRIVE_FORWARD_MS);
            stopMotors();
            sleep(250);

            turnRight(TURN_POWER, TURN_90_DEG_MS);
            stopMotors();
            sleep(250);
        }

        stopMotors();
    }

    private void runInSquareOption2() {
        for (int i = 0; i < 4 && opModeIsActive(); i++) {
            driveTimer.reset();
            while (driveTimer.milliseconds() < DRIVE_FORWARD_MS) {
                frontLeft.setPower(DRIVE_POWER);
                frontRight.setPower(DRIVE_POWER);
                backLeft.setPower(DRIVE_POWER);
                backRight.setPower(DRIVE_POWER);
            }

            double currentMilliSecs = driveTimer.milliseconds();
            while (driveTimer.milliseconds() < currentMilliSecs + TURN_90_DEG_MS) {
                frontLeft.setPower(TURN_POWER);
                backLeft.setPower(TURN_POWER);
                frontRight.setPower(-TURN_POWER);
                backRight.setPower(-TURN_POWER);
            }
        }
    }

    private void driveForward(double power, long timeMs) {
        setMotorPowers(power, power, power, power);
        sleep(timeMs);
    }

    private void turnRight(double power, long timeMs) {
        setMotorPowers(power, -power, power, -power);
        sleep(timeMs);
    }

    private void stopMotors() {
        setMotorPowers(0, 0, 0, 0);
    }

    private void setMotorPowers(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

}