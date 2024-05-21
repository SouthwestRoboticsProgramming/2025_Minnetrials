package com.swrobotics.robot.subsystems.music;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

public final class MusicSubsystem extends SubsystemBase {
    private static MusicSubsystem instance = new MusicSubsystem();

    public static MusicSubsystem getInstance() {
        return instance;
    }

    private static final String songsFolder = "music";

    public static List<String> getAvailableSongs() {
        File dir = new File(Filesystem.getDeployDirectory(), songsFolder);

        File[] files = dir.listFiles();
        List<String> out = new ArrayList<>();
        for (File file : files) {
            if (file.getName().endsWith(".chrp")) {
                out.add(file.getAbsolutePath());
            }
        }

        return out;
    }

    private final Orchestra orchestra;

    public MusicSubsystem() {
        orchestra = new Orchestra();
        instance = this;
    }

    public void addInstrument(TalonFX fx) {
        AudioConfigs conf = new AudioConfigs();
        conf.BeepOnBoot = true;
        conf.BeepOnConfig = true;
        conf.AllowMusicDurDisable = true;
        fx.getConfigurator().apply(conf);
        
        orchestra.addInstrument(fx);
    }

    public void beginSong(String file) {
        orchestra.loadMusic(file);
        orchestra.play();
    }

    public boolean isSongPlaying() {
        return orchestra.isPlaying();
    }

    public void endSong() {
        orchestra.stop();
    }
}
