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
                        drive.trajectorySequenceBuilder(new Pose2d(11.83, -62.16, Math.toRadians(90.00)))
                                .splineTo(new Vector2d(12, -35), Math.toRadians(90))
                                .setReversed(true)
                                .UNSTABLE_addDisplacementMarkerOffset(10, ()->{
                                    //intake.stay(0);
                                })
                                .UNSTABLE_addDisplacementMarkerOffset(30, ()->{
                                    //outtake.depositPosition(0, 0);
                                    //outtake.setPixelLatch(true);
                                })
                                .splineTo(new Vector2d(48, -37), Math.toRadians(-2.00))
                                .setReversed(false)
                                .addTemporalMarker(()->{
                                    ///outtake.setPixelLatch(false);
                                })
                                .waitSeconds(0.5)
                                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                                    //outtake.transferPosition();
                                })
                                .splineTo(new Vector2d(12, -12), Math.toRadians(180))
                                .UNSTABLE_addDisplacementMarkerOffset(0, ()->{
                                    //intake.intakePosition4th(900);
                                    //intake.setPower(1);
                                })
                                .splineTo(new Vector2d(-27, -13), Math.toRadians(181))//first pickup
                                .waitSeconds(1)
                                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                                    //intake.transferPosition();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(1.1, ()->{
                                    //intake.setPower(-0.2);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(2, ()->{
                                    //intake.setPower(0);
                                    //intake.stay(60);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(2.5, ()->{
                                    //outtake.depositPosition(300, 0);
                                    //outtake.setPixelLatch(true);
                                })
                                .setReversed(true)
                                .lineToSplineHeading(new Pose2d(12, -12, Math.toRadians(180)))
                                .splineToConstantHeading(new Vector2d(43.5, -35), Math.toRadians(-30.00))
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
                                .splineTo(new Vector2d(12, -12), Math.toRadians(180))
                                .UNSTABLE_addDisplacementMarkerOffset(0, ()->{
                                    //intake.intakePosition2nd(900);
                                    //intake.setPower(1);
                                })
                                .splineTo(new Vector2d(-30, -14), Math.toRadians(190))//second pickup
                                .waitSeconds(1)
                                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                                    //intake.transferPosition();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(1.3, ()->{
                                    //intake.setPower(-0.2);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(2, ()->{
                                    //intake.setPower(0);
                                    //intake.stay(60);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(2.5, ()->{
                                    //outtake.depositPosition(300, 0);
                                    //outtake.setPixelLatch(true);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(3, ()->{
                                    //intake.setTarget(0);
                                })
                                .setReversed(true)
                                .lineToSplineHeading(new Pose2d(12, -12, Math.toRadians(180)))
                                .splineToConstantHeading(new Vector2d(43.5, -35), Math.toRadians(-30.00))
                                .UNSTABLE_addTemporalMarkerOffset(0.1, ()->{
                                    //outtake.setPixelLatch(false);
                                })
                                .waitSeconds(1)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}