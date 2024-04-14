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
                .setConstraints(50, 50, Math.toRadians(90), Math.toRadians(70), 10.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11.83, -62.16, Math.toRadians(-270.00)))


                                .lineTo(new Vector2d(25, -42))
                                .setReversed(true)


                                .back(4)
                                .lineToLinearHeading(new Pose2d(48, -42, Math.toRadians(-180.00)))
                                .setReversed(false)

                                .waitSeconds(0.4)
                                .lineToSplineHeading(new Pose2d(30, -12, Math.toRadians(-178)))
                                .splineToConstantHeading(new Vector2d(20, -12), Math.toRadians(-178))

                                .forward(42.5)//first pickup

                                .setReversed(true)


                                .lineTo(new Vector2d(30, -12))
                                .splineToConstantHeading(new Vector2d(43, -35), -90)


                                .lineToSplineHeading(new Pose2d(30, -12, Math.toRadians(-179)))
                                .splineToConstantHeading(new Vector2d(20, -12), Math.toRadians(-179))

                                .forward(44)//second pickup

                                .waitSeconds(1)
                                .setReversed(true)



                                .lineTo(new Vector2d(30, -12))
                                .splineToConstantHeading(new Vector2d(44, -35), -90)

                                .waitSeconds(0.8)
                                .setReversed(false)

                                .lineToSplineHeading(new Pose2d(30, -12, Math.toRadians(-178)))
                                .splineToConstantHeading(new Vector2d(20, -9), Math.toRadians(-178))
                                .splineTo(new Vector2d(-26, -9), Math.toRadians(-160))//intake for third time




                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-9, -9, Math.toRadians(-180)), 0)
                                .splineToSplineHeading(new Pose2d(30, -12, Math.toRadians(-180)), 0)
                                .splineToSplineHeading(new Pose2d(44.3, -35, Math.toRadians(-180)), Math.toRadians(80))

                                .waitSeconds(0.7)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}