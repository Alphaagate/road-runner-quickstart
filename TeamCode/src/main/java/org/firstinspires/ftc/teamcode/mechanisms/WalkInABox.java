package org.firstinspires.ftc.teamcode.mechanisms;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@Autonomous
public class WalkInABox extends LinearOpMode {
    private int power = 1;
        private ElapsedTime driveTimer = new ElapsedTime();



    @Override
    public void runOpMode() {
        //TODO: instantiate your MecanumDrive at a particular pose.
        // Wait for the DS start button to be touched.
        telemetry.addData("Starting WalkInABox", "");
        telemetry.update();

        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "frontleft");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "backleft");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "frontright");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "frontleft");


        waitForStart();

        // above is the code for first movement, below is the code for the square movement
        for (int i = 1; i <= 16; i++) {
            driveTimer.reset();
            while (driveTimer.seconds() < 1) {
                leftFront.setPower(power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(power);
            }
            while (driveTimer.seconds() > 1 && driveTimer.seconds() < 2) {
                leftFront.setPower(power);
                leftBack.setPower(power);
                leftFront.setPower(-power);
                rightFront.setPower(-power);
                driveTimer.reset();
            }
        }

        if (isStopRequested()) {
            return;
        }

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


}