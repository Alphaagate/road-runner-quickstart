package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class PathTester extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        //TODO: instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);


        waitForStart();
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(180)))

                .strafeToConstantHeading(new Vector2d(-24, 0))
                .turn(Math.toRadians(-90))
                .strafeToConstantHeading(new Vector2d(-24, 24))
                .turn(Math.toRadians(-90))
                .strafeToConstantHeading(new Vector2d(24, 24))
                .turn(Math.toRadians(-90))
                .strafeToConstantHeading(new Vector2d(24, -24))
                .turn(Math.toRadians(-90))
                .strafeToConstantHeading(new Vector2d(-24, -24))
                .turn(Math.toRadians(-90))
                .strafeToConstantHeading(new Vector2d(-24, 24))
                .turn(Math.toRadians(-90))
                .strafeToConstantHeading(new Vector2d(24, 24))
                .turn(Math.toRadians(-90))
                .strafeToConstantHeading(new Vector2d(24, -24))
                .turn(Math.toRadians(-90))
                .strafeToConstantHeading(new Vector2d(-24, -24))
                .turn(Math.toRadians(-90))
                .strafeToConstantHeading(new Vector2d(-24, 24))
                .turn(Math.toRadians(-90))
                .strafeToConstantHeading(new Vector2d(24, 24))
                .turn(Math.toRadians(-90))
                .strafeToConstantHeading(new Vector2d(24, -24))
                .turn(Math.toRadians(-90))
                .strafeToConstantHeading(new Vector2d(-24, -24))
                .turn(Math.toRadians(-90));
        Action trajectoryActionChosen2 = tab2.build();
        Actions.runBlocking(trajectoryActionChosen2);
    }

}
