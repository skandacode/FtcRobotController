package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(120), Math.toRadians(120), 10.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36.11, 62.17, Math.toRadians(270.00)))
                                .lineToConstantHeading(new Vector2d(-49, 39.31))
                                .setReversed(true)
                                .UNSTABLE_addDisplacementMarkerOffset(10, ()->{
                                    //intake.intakePosition5th(0);
                                })
                                .lineToLinearHeading(new Pose2d(-49, 49, 230))
                                .addDisplacementMarker(()->{
                                    //intake.setTarget(600);
                                })
                                .waitSeconds(2)
                                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                                    //intake.transferPosition();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(1.3, ()->{
                                    //intake.setPower(-0.2);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(2, ()->{
                                    //intake.setPower(0);
                                    //intake.intakePosition5th(0);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(2.5, ()->{
                                    //outtake.depositPosition(0, 0);
                                    //outtake.setPixelLatch(true);
                                })
                                .setReversed(true)
                                .splineTo(new Vector2d(-9, 58), Math.toRadians(0))
                                .splineTo(new Vector2d(43, 43), Math.toRadians(0.00))
                                .setReversed(false)
                                //change this
                                .lineTo(new Vector2d(46, 26))
                                .UNSTABLE_addDisplacementMarkerOffset(0.5, ()->{
                                    //outtake.setPixelLatch(false);
                                })
                                .waitSeconds(1)
                                .splineTo(new Vector2d(-9, 58), Math.toRadians(180))
                                .splineTo(new Vector2d(-49, 49), Math.toRadians(220))
                                .UNSTABLE_addDisplacementMarkerOffset(-5, ()->{
                                    //intake.intakePosition3rd(600);
                                })
                                .waitSeconds(2)
                                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                                    //intake.transferPosition();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(1.3, ()->{
                                    //intake.setPower(-0.2);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(2, ()->{
                                    //intake.setPower(0);
                                    //intake.intakePosition5th(0);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(2.5, ()->{
                                    //outtake.depositPosition(0, 0);
                                    //outtake.setPixelLatch(true);
                                })
                                .setReversed(true)
                                .splineTo(new Vector2d(-9, 58), Math.toRadians(0))
                                .splineTo(new Vector2d(43, 43), Math.toRadians(0.00))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}