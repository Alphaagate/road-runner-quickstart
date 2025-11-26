

package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name = "FULL_AUTO_RED_ClOSE_PIXEL", group = "Autonomous")
public class FullAutoRedSideClose extends AbstractFullAuto {

    @Override
    public Pose2d getInitialPose() {
        return new Pose2d(-60.04, 43.655, Math.toRadians(-69.1));
    }

    @Override
    public void setOuttakePower() {
        outtakemotorright.setPower(-0.42);
        outtakemotorleft.setPower(0.42);

    }


    @Override
    public void runFirstPath(MecanumDrive drive, Pose2d initialPose) {
        TrajectoryActionBuilder goToLaunchSpot = drive.actionBuilder(initialPose)
                //.lineToYSplineHeading(24, Math.toRadians(0))
                .strafeToConstantHeading(new Vector2d(-9, 23));

        Action trajectoryActionChosen = goToLaunchSpot.build();
        Actions.runBlocking(trajectoryActionChosen);
    }

    @Override
    public void runSecondPath(MecanumDrive drive) {
        Action trajectoryActionChosen;
        TrajectoryActionBuilder goToIntake = drive.actionBuilder(getCurrentPos(drive))
                .turn(Math.toRadians(135+24.1))    //24.1 = 69.1 - 45
                .strafeToConstantHeading(new Vector2d(-12, 48));
        trajectoryActionChosen = goToIntake.build();
        Actions.runBlocking(trajectoryActionChosen);

        intakemotor.setPower(1);
        transfermotor.setPower(1);

        TrajectoryActionBuilder goToLaunchSpot2 = drive.actionBuilder(getCurrentPos(drive))
                .turn(Math.toRadians(-135-24.1))
                .strafeToConstantHeading(new Vector2d(-12, 23));
        trajectoryActionChosen = goToLaunchSpot2.build();
        Actions.runBlocking(trajectoryActionChosen);
    }
    @Override
    public void parkOutsideLaunch(MecanumDrive drive) {
        Action trajectoryActionChosen;
        TrajectoryActionBuilder goToPark = drive.actionBuilder(getCurrentPos(drive))
                .splineToConstantHeading(new Vector2d(-20, 48), Math.toRadians(-90));
        trajectoryActionChosen = goToPark.build();
        Actions.runBlocking(trajectoryActionChosen);

    }

}