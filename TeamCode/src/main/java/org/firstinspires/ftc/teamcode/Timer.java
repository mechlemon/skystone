package org.firstinspires.ftc.teamcode;

public class Timer {

    double starttime;
    double duration;

    void set(double seconds){
        starttime = 1e9 * System.nanoTime();
        duration = seconds;
    }

    double getElapsed(){
        return 1e9 * System.nanoTime() - starttime;
    }

    boolean isDone(){
        return getElapsed() > duration;
    }
}
