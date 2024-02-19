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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36.11, -62.17, Math.toRadians(90)))
                                .splineTo(new Vector2d(-36.11, -34.51), Math.toRadians(90))
                                .setReversed(true)
                                .UNSTABLE_addDisplacementMarkerOffset(10, ()->{
                                })
                                .splineTo(new Vector2d(-35.08, -52.85), Math.toRadians(-60))
                                .splineTo(new Vector2d(-9, -58), Math.toRadians(0))
                                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                                })
                                .splineTo(new Vector2d(43, -43), Math.toRadians(0.00))
                                .setReversed(false)
                                .lineTo(new Vector2d(46, -34))
                                .waitSeconds(0.5)
                                .addTemporalMarker(()->{
                                })
                                .waitSeconds(1)
                                .addTemporalMarker(()->{
                                })
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}