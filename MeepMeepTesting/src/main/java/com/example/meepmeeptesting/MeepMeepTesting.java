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
                .setConstraints(35, 35, Math.toRadians(70), Math.toRadians(70), 10.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11.83, 62.16, Math.toRadians(270.00)))
                                .lineTo(new Vector2d(27, 40))
                                .setReversed(true)
                                .UNSTABLE_addDisplacementMarkerOffset(10, ()->{
                                    //intake.intakePosition5th(0);
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
                                .lineTo(new Vector2d(30, 15))
                                .splineToConstantHeading(new Vector2d(12, 12), Math.toRadians(180))
                                .UNSTABLE_addDisplacementMarkerOffset(0, ()->{
                                    //intake.intakePosition4th(900);
                                    //intake.setPower(1);
                                })
                                .lineToSplineHeading(new Pose2d(-27, 12, Math.toRadians(180)))
                                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                                    //intake.transferPosition();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(1.1, ()->{
                                    //intake.setPower(-0.2);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(2, ()->{
                                    //intake.setPower(0);
                                    //intake.intakePosition5th(60);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(2.5, ()->{
                                    //outtake.depositPosition(300, 0);
                                    //outtake.setPixelLatch(true);
                                })
                                .setReversed(true)
                                .lineToSplineHeading(new Pose2d(20, 12, Math.toRadians(180)))
                                .splineToConstantHeading(new Vector2d(42, 35), Math.toRadians(60.00))
                                .setReversed(false)
                                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                                    //outtake.depositPosition(300, 0);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                                    //outtake.setPixelLatch(false);
                                })
                                .waitSeconds(1.5)
                                .addTemporalMarker(()->{
                                    //outtake.transferPosition();
                                })
                                .lineTo(new Vector2d(30, 15))
                                .splineToConstantHeading(new Vector2d(12, 12), Math.toRadians(180))
                                .UNSTABLE_addDisplacementMarkerOffset(0, ()->{
                                    //intake.intakePosition2nd(900);
                                    //intake.setPower(1);
                                })
                                .lineToSplineHeading(new Pose2d(-29, 12, Math.toRadians(180)))
                                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                                    //intake.transferPosition();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(1.3, ()->{
                                    //intake.setPower(-0.2);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(2, ()->{
                                    //intake.setPower(0);
                                    //intake.intakePosition5th(60);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(2.5, ()->{
                                    //outtake.depositPosition(300, 0);
                                    //outtake.setPixelLatch(true);
                                })
                                .setReversed(true)
                                .lineToSplineHeading(new Pose2d(20, 12, Math.toRadians(180)))
                                .splineToConstantHeading(new Vector2d(42, 35), Math.toRadians(60.00))
                                .UNSTABLE_addTemporalMarkerOffset(0.5, ()->{
                                    //outtake.setPixelLatch(false);
                                })
                                .waitSeconds(1)
                                .lineTo(new Vector2d(30, 12))
                                .lineTo(new Vector2d(60, 12))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}