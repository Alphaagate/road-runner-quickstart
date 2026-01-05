package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@Autonomous(name = "BLUE_TEST_FAR_AUTO_PIXEL", group = "Autonomous")
public class BlueSideFarTestAuto extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        //TODO: instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);



        waitForStart();

/**
 //Note:
 //Tangent -- Means the body moving direction
 //Heading -- Means the facing direction. For MecanumDrive, it doesn't have to be the same as the Tangent (body moving direction)
 TrajectoryActionBuilder tabTest = drive.actionBuilder(initialPose)
 //Start moving the body to 0 direction, i.e. X axis direction
 .setTangent(0)
 // splineTo() -  The last parameter is the robot's ending tangent when it arrives (48, 48), not the heading.
 // In fact, the 'Math.PI / 2' parameter in all these methods is the Tangent angle, not heading angle.
 // We don't need to specify the heading for splineTo() method,
 // it implies the heading will keep changing along with the spline path.
 .splineTo(new Vector2d(48, 48), Math.PI / 2)

 .setTangent(0)
 // strafeToConstantHeading() - We don't need to specify the heading in this method,
 // it implies the heading will not change at all, it keeps the heading same as the previous moving segment.
 .strafeToConstantHeading(new Vector2d(48, 48), Math.PI / 2)

 .setTangent(0)
 //splineToLinearHeading() - the 0 inside the Post2d() is heading, so facing to X axis direction, again  Math.PI / 2 is the body moving direction (i.e. tangent) when it arrives (48, 48).
 //The linearHeading here means the heading is changing in a linear/static speed when robot moving along the spline path.
 .splineToLinearHeading(new Pose2d(48, 48, 0), Math.PI / 2)

 .setTangent(0)
 //splineToSplineHeading() - the 0 inside the Post2d() is heading, so facing to X axis direction, again  Math.PI / 2 is the body moving direction (i.e. tangent) when it arrives (48, 48).
 //The SplineHeading here means the heading is changing in a Spline speed when robot is moving along the spline path. The heading change should be more smooth.
 // But if moving in a short path, you may not see obvious difference comparing with splineToLinearHeading() method.
 .splineToSplineHeading(new Pose2d(48, 48, 0), Math.PI / 2);

 */


//
//        Path 1
//        -----------------------------------------------
//        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
//                //.lineToYSplineHeading(24, Math.toRadians(0))
//                .splineTo(new Vector2d(0, 24), 0)
//                .splineTo(new Vector2d(24, 24), -Math.PI / 2);
//
//        //.strafeTo(new Vector2d(44.5, 30))
//
//        Action trajectoryActionChosen = tab1.build();
//        Actions.runBlocking(trajectoryActionChosen);
//
//        // above is the code for first movement, below is the code for the square movement
//        for (int i = 1; i <= 4; i++) {
//            Pose2d currentPos = drive.localizer.getPose();
//
//            TrajectoryActionBuilder tab2 = drive.actionBuilder(currentPos)
//                    .splineTo(new Vector2d(24, -24), Math.PI)
//                    .splineTo(new Vector2d(-24, -24), Math.PI / 2)
//                    .splineTo(new Vector2d(-24, 24), 0)
//                    .splineTo(new Vector2d(24, 24), -Math.PI / 2);
//            Action trajectoryActionChosen2 = tab2.build();
//            Actions.runBlocking(trajectoryActionChosen2);
//        }
//        ---------------------------------------------------------
        // path 2 better

//        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
//                .strafeToConstantHeading(new Vector2d(0, 24), Math.toRadians(0))
//                // turn is relative to robot check?
//                .turn(Math.toRadians(-90))
//                .strafeToConstantHeading(new Vector2d(24, 24), Math.toRadians(-90))
//                .turn(Math.toRadians(-90));
//
//        Action trajectoryActionChosen = tab1.build();
//        Actions.runBlocking(trajectoryActionChosen);
//
//        // above is the code for first movement, below is the code for the square movement
//        for (int i = 1; i <= 4; i++) {
//            Pose2d currentPos = drive.localizer.getPose();
//
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

//
 // NOTE: below code is optimized for our 4x6 playing field, not the official 6x6 playing field




                //launch code here
                //.strafeToConstantHeading(new Vector2d(0, -12), Math.toRadians(0))

                // ignore this bit below
//                .turn(Math.toRadians(-90))
//                .strafeToConstantHeading(new Vector2d(24, 0), Math.toRadians(-90))
//                .turn(Math.toRadians(-90))
//                .strafeToConstantHeading(new Vector2d(0, 0), Math.toRadians(180))
//                .turn(Math.toRadians(-90))
//                .strafeToConstantHeading(new Vector2d(0, 24), Math.toRadians(90))
//                .turn(Math.toRadians(-90))
//                .strafeToConstantHeading(new Vector2d(24, 24), Math.toRadians(0))
//                .turn(Math.toRadians(-90))
//                .strafeToConstantHeading(new Vector2d(24, 0), Math.toRadians(-90))
//                .turn(Math.toRadians(-90))
//
//                .strafeToConstantHeading(new Vector2d(0, 0), Math.toRadians(180))
//                .turn(Math.toRadians(-90))
//                .strafeToConstantHeading(new Vector2d(0, 24), Math.toRadians(90))
//                .turn(Math.toRadians(-90))
//                .strafeToConstantHeading(new Vector2d(24, 24), Math.toRadians(0))
//                .turn(Math.toRadians(-90))
//                .strafeToConstantHeading(new Vector2d(24, 0), Math.toRadians(-90))
//                .turn(Math.toRadians(-90))
//                .strafeToConstantHeading(new Vector2d(0, 0), Math.toRadians(180))
//                .turn(Math.toRadians(-90));


        //.strafeTo(new Vector2d(44.5, 30))



//        if (isStopRequested()) {
//            return;
//        }

    }


}