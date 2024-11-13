package com.example.meepmeeptesting

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder

fun main() {
    val meepMeep = MeepMeep(800)
    val myBot =
        DefaultBotBuilder(meepMeep) // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 15.0)
            .build()

    //starting positions
    val blueRight = Pose2d(36.0, 60.0, -Math.PI/2)
    val blueLeft = Pose2d(-36.0, 60.0, -Math.PI/2)
    val redRight = Pose2d(36.0, -60.0, Math.PI/2)
    val redLeft = Pose2d(-36.0, -60.0, Math.PI/2)

    //any actions go here
    val action = myBot.drive.actionBuilder(redLeft) //this is your starting position
        .splineTo(Vector2d(0.0, 0.0), Math.PI/2)
        .splineToSplineHeading(Pose2d(10.0,10.0,0.0), -Math.PI/2)
        .build()

    myBot.runAction(action)

    meepMeep.setBackground(Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(myBot)
        .start()
}