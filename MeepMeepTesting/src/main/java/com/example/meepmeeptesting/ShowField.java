package com.example.meepmeeptesting;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class ShowField {


    public static void showIt(MeepMeep meepMeep, RoadRunnerBotEntity myBot) {

        Image img = null;

        try {
            String filename = "MeepMeepTesting/Images/field-2024-juice-light.png";
            img = ImageIO.read(new File(filename));

        } catch (

                IOException e) {
        }

        if (img != null) meepMeep.setBackground(img)
                .setDarkMode(false)
                .setBackgroundAlpha(0.5f)
                .addEntity(myBot)
                .start();
        else {
            meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();
        }
    }
}
