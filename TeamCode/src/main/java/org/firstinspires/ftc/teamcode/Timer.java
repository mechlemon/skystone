package org.firstinspires.ftc.teamcode;

public class Timer {

    double starttime;

    Timer(){
        starttime = 1e-9 * System.nanoTime();
    }

    double getElapsed(){
        return 1e-9 * System.nanoTime() - starttime;
    }

    void reset(){
        starttime = 1e-9 * System.nanoTime();
    }

}
