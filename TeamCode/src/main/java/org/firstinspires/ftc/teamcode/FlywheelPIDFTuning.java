package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
@TeleOp
public class FlywheelPIDFTuning extends OpMode {
    private DcMotorEx outtakeMotor;
    public static double NEW_P = 0;
    public static double NEW_F = 0;

    double highVelocity = 1500;
    double lowVelocity = 600;
    double curTargetVelocity = highVelocity;

    // for 1150 1 motor F = 10.684 P = 12.12
    // for 6000 1 motor F = 11.656 P = 12.7
//    for 6000 rpm F = 16.72 for 1500 P = 90


    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001};

    int stepIndex = 1;

    @Override
    public void init() {
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakemotor1");
        outtakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(NEW_P, 0, 0, NEW_F);
        outtakeMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        outtakeMotor.setDirection(DcMotorEx.Direction.REVERSE);

        telemetry.addData("Init status", "initialized");
    }

    @Override
    public void loop() {
        outtakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;

            } else {
                curTargetVelocity = highVelocity;
            }
        }
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            NEW_F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            NEW_F += stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            NEW_P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            NEW_P -= stepSizes[stepIndex];

        }

        //set new PIDF Coefficients

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(NEW_P, 0, 0, NEW_F);
        outtakeMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        //set velocity
        outtakeMotor.setVelocity(curTargetVelocity);

        double curVelocity = outtakeMotor.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("------------------------------");
        telemetry.addData("Tuning P", "%.4f", NEW_P);
        telemetry.addData("Tuning F", "%.4f", NEW_F);
        telemetry.addData("Step size", "%.4f", stepSizes[stepIndex]);



    }
}
