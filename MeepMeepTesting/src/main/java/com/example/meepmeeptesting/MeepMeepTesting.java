package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 45, Math.toRadians(90), Math.toRadians(70), 10.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11.83, 62.16, Math.toRadians(270.00)))
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, Math.toRadians(90), 10.5))
                                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(45))

                                .addTemporalMarker(()->{
                                    //intake.transferPosition();
                                    //intake.setTarget(40);
                                })
                                .lineTo(new Vector2d(26, 42))
                                .setReversed(true)
                                .UNSTABLE_addDisplacementMarkerOffset(10, ()->{
                                    //intake.stay(0);
                                })
                                .UNSTABLE_addDisplacementMarkerOffset(20, ()->{
                                    //outtake.depositPosition(0, 0);
                                    //outtake.setPixelLatch(true);
                                })
                                .splineTo(new Vector2d(48.5, 44), Math.toRadians(0.00))
                                .setReversed(false)
                                .addTemporalMarker(()->{
                                    //outtake.setPixelLatch(false);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                                    //outtake.transferPosition();
                                })
                                .waitSeconds(2)
                                .splineToLinearHeading(new Pose2d(30, 9, Math.toRadians(180)), Math.toRadians(180))
                                .UNSTABLE_addDisplacementMarkerOffset(0, ()->{
                                    //intake.intakePosition4th( 0);
                                    //intake.setPower(1);
                                })
                                .lineToConstantHeading(new Vector2d(-23, 9))//first pickup
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}