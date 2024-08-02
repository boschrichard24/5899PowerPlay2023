package org.firstinspires.ftc.teamcode.AutoCode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "NEWCone")
public class NEWCone extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(36, 0, Math.toRadians(90)))
                .build();

        drive.followTrajectory(traj);

        sleep(2000);

        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineToLinearHeading(new Pose2d(56, -16, Math.toRadians(180)), Math.toRadians(0))
                        .build()
        );
    }
}