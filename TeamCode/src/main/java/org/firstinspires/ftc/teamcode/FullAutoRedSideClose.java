

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
        return new Pose2d(-58.3, 44.5, Math.toRadians(-55));
    }

    @Override
    public void setOuttakePowerForClose() {
        //        outtakemotorright.setPower(-0.4);
//        outtakemotorleft.setPower(0.4);
        outtakemotorright.setVelocity(-965);
        outtakemotorleft.setVelocity(965);
    }
    public void setOuttakePowerForFar() {
    }




    @Override
    public void runFirstPath(MecanumDrive drive, Pose2d initialPose) {
        TrajectoryActionBuilder goToLaunchSpot = drive.actionBuilder(initialPose)
                //.lineToYSplineHeading(24, Math.toRadians(0))
//                .strafeToConstantHeading(new Vector2d(-14, 22))
                .strafeToSplineHeading(new Vector2d(-12, 12), Math.toRadians(-42));   //to launch spot


        Action trajectoryActionChosen = goToLaunchSpot.build();
        Actions.runBlocking(trajectoryActionChosen);
    }

    @Override
    public void runSecondPath(MecanumDrive drive) {
        Action trajectoryActionChosen;
        TrajectoryActionBuilder goToIntake = drive.actionBuilder(getCurrentPos(drive))
                .turn(Math.toRadians(135+24.1))    //24.1 = 69.1 - 45
                .strafeToConstantHeading(new Vector2d(-24, 40));
        trajectoryActionChosen = goToIntake.build();
        Actions.runBlocking(trajectoryActionChosen);

        intakemotor.setPower(0.3);
//        transfermotor.setPower(-0.15);

        TrajectoryActionBuilder goToLaunchSpot2 = drive.actionBuilder(getCurrentPos(drive))
//                .turn(Math.toRadians(-135-24.1))
//                .strafeToConstantHeading(new Vector2d(-12, 23));
                .strafeToSplineHeading(new Vector2d(-24, 0), Math.toRadians(-45));   //to launch spot

        trajectoryActionChosen = goToLaunchSpot2.build();
        Actions.runBlocking(trajectoryActionChosen);
    }
    @Override
    public void parkOutsideLaunch(MecanumDrive drive) {
        Action trajectoryActionChosen;
        TrajectoryActionBuilder goToPark = drive.actionBuilder(getCurrentPos(drive))
                .strafeToConstantHeading(new Vector2d(-20, 48));
        trajectoryActionChosen = goToPark.build();
        Actions.runBlocking(trajectoryActionChosen);

    }

}