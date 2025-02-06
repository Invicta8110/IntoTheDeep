package com.example.meepmeeptesting

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.SleepAction
import com.acmerobotics.roadrunner.Vector2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import kotlin.math.PI

fun main() {
    val meepMeep = MeepMeep(800)
    val myBot =
        DefaultBotBuilder(meepMeep) // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 15.0)
            .build()

    //starting positions
    val blueRight = Pose2d(36.0, 60.0, -Math.PI/2)
    val blueLeft = Pose2d(-26.0, 60.0, -Math.PI/2)
    val redRight = Pose2d(26.0, -60.0, Math.PI/2)
    val redLeft = Pose2d(-36.0, -60.0, Math.PI/2)

    //any actions go here
    val action = myBot.drive.actionBuilder(redRight)
        .splineToConstantHeading(Vector2d(46.0, -12.0), PI/2)
        .splineToConstantHeading(Vector2d(52.0, -60.0), PI/2)
        .splineToConstantHeading(Vector2d(52.0, -12.0), PI/2)
        .splineToConstantHeading(Vector2d(52.0, -60.0), PI/2)
        .splineToConstantHeading(Vector2d(68.0, -12.0), PI/2)
        .build()

    myBot.runAction(action)

    meepMeep.setBackground(Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(myBot)
        .start()
}