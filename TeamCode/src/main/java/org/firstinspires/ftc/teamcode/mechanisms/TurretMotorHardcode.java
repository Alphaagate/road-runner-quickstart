package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class TurretMotorHardcode extends LinearOpMode {
    private static final double MAX_TURRET_TURN_POWER = 0.002;
    private DcMotorEx turretMotor;


    public void runOpMode() {
        telemetry.addData("Starting TurretMotorHardcode ", "");

        initMotor();

        telemetry.update();
        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
//            turretMotor.setTargetPosition(20);
            turretMotor.setPower(1);
            telemetry.addData("vel", turretMotor.getVelocity());
            telemetry.update();
        }

    }

    private void initMotor() {

        turretMotor = hardwareMap.get(DcMotorEx.class, "turretmotor");
        //TODO: Tune the direction
        turretMotor.setDirection(DcMotorEx.Direction.REVERSE);

    }

    private void resetMotorPosition() {

//        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setTargetPosition(0);//int type. Set target before setting RunMode.
        turretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turretMotor.setPower(MAX_TURRET_TURN_POWER);

    }


}
