package org.firstinspires.ftc.teamcode.mechanisms;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous
public class WalkInABox extends LinearOpMode {

    private MecanumDrive drive;
    private FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void runOpMode() {
        //TODO: instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(180));
        drive = new MecanumDrive(hardwareMap, initialPose);
        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();


        waitForStart();
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(0, 24), Math.toRadians(0))
                // turn is relative to robot check?
                .turn(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(24, 24), Math.toRadians(-90))
                .turn(Math.toRadians(-90));

        Action trajectoryActionChosen = tab1.build();
        Actions.runBlocking(trajectoryActionChosen);

        // above is the code for first movement, below is the code for the square movement
        for (int i = 1; i <= 100; i++) {
            Pose2d currentPos = drive.localizer.getPose();

            TrajectoryActionBuilder tab2 = drive.actionBuilder(currentPos)
                    .splineToConstantHeading(new Vector2d(24, -24), Math.toRadians(180))
                    .turn(Math.toRadians(-90))
                    .splineToConstantHeading(new Vector2d(-24, -24), Math.toRadians(90))
                    .turn(Math.toRadians(-90))
                    .splineToConstantHeading(new Vector2d(-24, 24), Math.toRadians(0))
                    .turn(Math.toRadians(-90))
                    .splineToConstantHeading(new Vector2d(24, 24), Math.toRadians(-90))
                    .turn(Math.toRadians(-90));
            Action trajectoryActionChosen2 = tab2.build();
            Actions.runBlocking(trajectoryActionChosen2);
        }

        if (isStopRequested()) {
            return;
        }

    }

    private void drawAndLogTelemetry(TelemetryPacket packet) {
        Pose2d pose = getCurrentPos(drive);
        drawRobot(packet, pose);   // Draw robot on overlay

// === Log basic pose ===
        packet.put("x", pose.position.x);
        packet.put("y", pose.position.y);
        packet.put("headingDeg", Math.toDegrees(pose.heading.toDouble()));

// === Log motor velocity ===
//        packet.put("outtake velocity left", drive.outtakemotorleft.getVelocity());
//        packet.put("outtake velocity right", drive.outtakemotorright.getVelocity());


        // Send packet to dashboard
        dashboard.sendTelemetryPacket(packet);
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
    private Pose2d getCurrentPos(MecanumDrive drive) {
        return drive.localizer.getPose();
    }

}