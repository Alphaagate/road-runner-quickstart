package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

//public class FlywheelPIDFTuning extends OpMode {
//    private DcMotorEx outtakeMotor;
//    public static final double NEW_P = 2.5;
//    public static final double NEW_F = 0.5;
//
//    double highVelocity = 1100;
//    double lowVelocity = 800;
//    double curTargetVelocity = highVelocity;
//
//    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
//
//    int stepIndex = 1;
//
//    @Override
//    public void init() {
//        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakemotor1");
//        outtakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(NEW_P, 0, 0, F)
//    }

//}
